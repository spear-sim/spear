#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import cv2
import functools
import glob
import json
import msgpackrpc
import numpy as np
import os
import pandas as pd
import pathlib
import posixpath
import traceback
import unreal
import spear


#
# script() decorator and ScriptRunner
#

def script(func):
    @functools.wraps(func)
    def wrapper(*args, **kwargs):
        runner = ScriptRunner(generator=func(*args, **kwargs))
        runner.start()
        return runner
    return wrapper

class ScriptRunner:

    _initialized = False

    # Token state. Tokens are created by SPEAR before launching a script and used to track completion,
    # exceptions, and log messages. ScriptRunner.__init__ claims the pending token (if any) so that
    # stop() can mark it complete when the generator finishes or raises.
    _tokens = {}
    _exceptions = {}
    _log_messages = {}
    _active_token = None  # token of the ScriptRunner currently inside _advance(), used by _log_func
    _pending_token = None # token waiting to be claimed by the next ScriptRunner.__init__
    _next_token_id = 0

    # Delta time state, updated by tick callbacks.
    _current_delta_time = 0.0

    @staticmethod
    def initialize():
        if ScriptRunner._initialized:
            return
        ScriptRunner._initialized = True
        spear.register_log_func(func=ScriptRunner._log_func)

    def __init__(self, generator):
        ScriptRunner.initialize()
        assert generator is not None
        self._generator = generator
        self._running = False
        self._advancing = False
        self._token = ScriptRunner._pending_token  # claim the pending token (if any) for this instance
        ScriptRunner._pending_token = None
        self._pre_tick_handle = unreal.register_slate_pre_tick_callback(self._pre_tick)
        self._post_tick_handle = unreal.register_slate_post_tick_callback(self._post_tick)

    def start(self):
        spear.log("Executing editor script: ", self._generator)
        self._running = True

    def stop(self):
        self._running = False
        if self._token is not None:
            ScriptRunner._tokens[self._token] = True
        unreal.unregister_slate_pre_tick_callback(self._pre_tick_handle)
        unreal.unregister_slate_post_tick_callback(self._post_tick_handle)
        spear.log("Finished executing editor script.")

    def _pre_tick(self, delta_time):
        assert self._running
        ScriptRunner._current_delta_time = delta_time
        self._advance()

    def _post_tick(self, delta_time):
        assert self._running
        ScriptRunner._current_delta_time = delta_time
        self._advance()

    def _advance(self):
        if self._advancing:  # guard against re-entrancy from Unreal functions that pump the message loop
            return
        self._advancing = True
        prev_active_token = ScriptRunner._active_token
        ScriptRunner._active_token = self._token
        try:
            next(self._generator)
        except StopIteration:
            self.stop()
        except Exception:
            if self._token is not None:
                ScriptRunner._exceptions[self._token] = traceback.format_exc()
            self.stop()
            raise
        finally:
            ScriptRunner._active_token = prev_active_token
            self._advancing = False

    # Called by SPEAR before launching a script. Allocates a new token and initializes its state.
    @staticmethod
    def create_token():
        token = str(ScriptRunner._next_token_id)
        ScriptRunner._next_token_id = ScriptRunner._next_token_id + 1
        ScriptRunner._tokens[token] = False
        ScriptRunner._exceptions[token] = None
        ScriptRunner._log_messages[token] = []
        return token

    # Called by SPEAR before launching a script. Places the token in the pending slot so that the
    # next ScriptRunner.__init__ can claim it.
    @staticmethod
    def set_pending_token(token):
        ScriptRunner._pending_token = token

    # Called by SPEAR after launching a script. If the token is still in the pending slot, no
    # ScriptRunner was created and the script ran synchronously.
    @staticmethod
    def get_pending_token():
        return ScriptRunner._pending_token

    # Called by SPEAR after launching a script to clear the pending slot if the script ran
    # synchronously (i.e., get_pending_token() returned the same token).
    @staticmethod
    def clear_pending_token():
        ScriptRunner._pending_token = None

    # Called by SPEAR during the busy-wait loop to check if the script has finished.
    @staticmethod
    def is_complete(token):
        return ScriptRunner._tokens[token]

    # Called by SPEAR after the script has finished. Returns the exception traceback string
    # (or None) and clears the stored exception.
    @staticmethod
    def clear_exception(token):
        exception = ScriptRunner._exceptions[token]
        ScriptRunner._exceptions[token] = None
        return exception

    # Called by SPEAR during the busy-wait loop and after the script has finished. Returns
    # buffered log messages and clears the buffer.
    @staticmethod
    def clear_log_messages(token):
        log_messages = ScriptRunner._log_messages[token]
        ScriptRunner._log_messages[token] = []
        return log_messages

    # Called by SPEAR after the script has finished and its exception and log messages have been retrieved.
    # Destroys all state associated with the token. We don't assert that the token is complete, because if
    # the script ran synchronously, then no ScriptRunner claimed the token, so it was never marked complete.
    @staticmethod
    def destroy_token(token):
        del ScriptRunner._tokens[token]
        del ScriptRunner._exceptions[token]
        del ScriptRunner._log_messages[token]

    # Called by tick callbacks to get the most recent delta time.
    @staticmethod
    def get_delta_time():
        return ScriptRunner._current_delta_time

    @staticmethod
    def _log_func(message):
        if ScriptRunner._active_token is not None and ScriptRunner._active_token in ScriptRunner._log_messages:
            ScriptRunner._log_messages[ScriptRunner._active_token].append(message)

def get_script_delta_time():
    return ScriptRunner.get_delta_time()


#
# Module-level globals
#

asset_registry = unreal.AssetRegistryHelpers.get_asset_registry()
asset_tools = unreal.AssetToolsHelpers.get_asset_tools()
editor_actor_subsystem = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
unreal_editor_subsystem = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
subobject_data_subsystem = unreal.get_engine_subsystem(unreal.SubobjectDataSubsystem)

editor_properties_csv_file = spear.path.realpath(os.path.dirname(__file__), "..", "data", "editor_properties.csv")
df_editor_properties = pd.read_csv(editor_properties_csv_file, comment="#")

#
# Get actors and components
#

def find_actors(actor_class=unreal.Actor):
    actors = editor_actor_subsystem.get_all_level_actors()
    actors = sorted(actors, key=lambda actor: get_stable_name_for_actor(actor=actor))
    if actor_class is not None:
        actors = [ a for a in actors if isinstance(a, actor_class) ]
    return actors

def find_actor(stable_name):
    actors = [ actor for actor in find_actors() if get_stable_name_for_actor(actor=actor) == stable_name ]
    if len(actors) == 1:
        return actors[0]
    else:
        return None

def get_components(actor, component_class=unreal.ActorComponent):
    components = []
    component_names = []

    # add main component hierarchy
    if actor.root_component is not None:
        candidate_components = [actor.root_component] + list(actor.root_component.get_children_components(include_all_descendants=True))
        candidate_components = [ c for c in candidate_components if isinstance(c, component_class) ]
        components = components + candidate_components
        component_names = component_names + [ get_stable_name_for_component(component=c) for c in candidate_components ]

    # add any components that are not in the main hierarchy, use component_names to make sure we're not
    # adding any components redundantly
    candidate_components = actor.get_components_by_class(component_class=component_class)
    candidate_components = [ c for c in candidate_components if get_stable_name_for_component(component=c) not in component_names ]
    components = components + candidate_components
    component_names = component_names + [ get_stable_name_for_component(component=c) for c in candidate_components ]

    return components

def get_component(stable_name, actor=None):
    if actor is None:
        # Actor stable names can contain ":", but component names can't, so we split stable_name on the last ":".
        stable_actor_name, stable_component_name = stable_name.rsplit(":", 1)
        actor = find_actor(stable_name=stable_actor_name)
    else:
        stable_component_name = stable_name
    components = [ c for c in get_components(actor=actor) if get_stable_name_for_component(component=c).lower() == stable_component_name.lower() ] # case-insensitive comparison
    if len(components) == 1:
        return components[0]
    else:
        return None

def get_stable_name_for_actor(actor, include_unreal_id=False):
    folder_path = actor.get_folder_path()
    if folder_path.is_none():
        stable_name = actor.get_actor_label()
    else:
        stable_name = posixpath.join(str(folder_path), actor.get_actor_label())

    if include_unreal_id:
        assert actor.get_world() == unreal_editor_subsystem.get_editor_world() # get_path_name() would include a PIE prefix in a PIE game
        if stable_name != "":
            sep = ":"
        else:
            sep = ""
        stable_name = stable_name + sep + actor.get_path_name() # equivalent to Unreal::getUnrealIdString(...) in C++, because we're not in a PIE game

    return stable_name

def get_stable_name_for_component(component, include_actor_stable_name=False):
    if include_actor_stable_name:
        actor_name_str = f"{get_stable_name_for_actor(actor=component.get_owner())}:"
    else:
        actor_name_str = ""

    if "get_parent_components" in dir(component):
        component_name_str = ".".join([ c.get_name() for c in list(component.get_parent_components())[::-1] ] + [component.get_name()])
    else:
        component_name_str = component.get_name()

    return actor_name_str + component_name_str

#
# Create blueprint asset
#

def create_blueprint_asset(asset_name, package_dir, parent_class):

    blueprint_factory = unreal.BlueprintFactory()
    blueprint_factory.set_editor_property(name="parent_class", value=parent_class)

    # asset_class should be set to None when creating a new blueprint asset
    blueprint_asset = asset_tools.create_asset(asset_name=asset_name, package_path=package_dir, asset_class=None, factory=blueprint_factory)
    assert isinstance(blueprint_asset, unreal.Blueprint)

    return blueprint_asset

#
# Add new subobject
#

def add_new_subobject_to_instance(parent_data_handle, subobject_name, subobject_class):
    add_new_subobject_params = unreal.AddNewSubobjectParams(parent_handle=parent_data_handle, new_class=subobject_class, blueprint_context=None)
    return add_new_subobject_using_params(add_new_subobject_params=add_new_subobject_params, subobject_name=subobject_name, subobject_class=subobject_class)

def add_new_subobject_to_blueprint_asset(blueprint_asset, parent_data_handle, subobject_name, subobject_class):
    assert isinstance(blueprint_asset, unreal.Blueprint)
    add_new_subobject_params = unreal.AddNewSubobjectParams(parent_handle=parent_data_handle, new_class=subobject_class, blueprint_context=blueprint_asset)
    return add_new_subobject_using_params(add_new_subobject_params=add_new_subobject_params, subobject_name=subobject_name, subobject_class=subobject_class)

def add_new_subobject_using_params(add_new_subobject_params, subobject_name, subobject_class):
    subobject_data_handle, fail_reason = subobject_data_subsystem.add_new_subobject(params=add_new_subobject_params)
    assert fail_reason.is_empty()
    subobject_data = unreal.SubobjectDataBlueprintFunctionLibrary.get_data(data_handle=subobject_data_handle)
    assert unreal.SubobjectDataBlueprintFunctionLibrary.is_valid(data=subobject_data)
    subobject_object = unreal.SubobjectDataBlueprintFunctionLibrary.get_object(data=subobject_data)
    assert isinstance(subobject_object, subobject_class)
    success = subobject_data_subsystem.rename_subobject(handle=subobject_data_handle, new_name=unreal.Text(subobject_name))
    assert success
    return {"data_handle": subobject_data_handle, "data": subobject_data, "object": subobject_object}

#
# Get subobject descs
#

def get_subobject_descs_for_instance(instance):
    assert isinstance(instance, unreal.Object)
    subobject_data_handles = subobject_data_subsystem.k2_gather_subobject_data_for_instance(context=instance)
    return get_subobject_descs_for_data_handles(subobject_data_handles=subobject_data_handles)

def get_subobject_descs_for_blueprint_asset(blueprint_asset):
    assert isinstance(blueprint_asset, unreal.Blueprint)
    subobject_data_handles = subobject_data_subsystem.k2_gather_subobject_data_for_blueprint(context=blueprint_asset)
    return get_subobject_descs_for_data_handles(subobject_data_handles=subobject_data_handles)

def get_subobject_descs_for_data_handles(subobject_data_handles):
    assert isinstance(subobject_data_handles, unreal.Array)
    return [ get_subobject_desc_for_data_handle(subobject_data_handle=h) for h in subobject_data_handles ]

def get_subobject_desc_for_data_handle(subobject_data_handle):
    assert unreal.SubobjectDataBlueprintFunctionLibrary.is_handle_valid(data_handle=subobject_data_handle)
    subobject_data = unreal.SubobjectDataBlueprintFunctionLibrary.get_data(data_handle=subobject_data_handle)
    assert unreal.SubobjectDataBlueprintFunctionLibrary.is_valid(data=subobject_data)
    subobject_object = unreal.SubobjectDataBlueprintFunctionLibrary.get_object(data=subobject_data)
    return {"data_handle": subobject_data_handle, "data": subobject_data, "object": subobject_object, "name": subobject_object.get_name().removesuffix("_GEN_VARIABLE")}

#
# Get actor, component, asset, and editor property descs
#

def get_actor_desc(actor):
    actor_name = get_stable_name_for_actor(actor=actor, include_unreal_id=True)
    spear.log("Processing actor: ", actor_name)

    root_component = actor.get_editor_property(name="root_component")

    if root_component is not None:
        components_in_hierarchy = [root_component] + list(root_component.get_children_components(include_all_descendants=True))
    else:
        components_in_hierarchy = []
    components_in_hierarchy_names = [ get_stable_name_for_component(component=c) for c in components_in_hierarchy ]

    other_components = actor.get_components_by_class(component_class=unreal.ActorComponent)
    other_components = [ c for c in other_components if get_stable_name_for_component(component=c) not in components_in_hierarchy_names ]

    return {
        "class": actor.__class__.__name__,
        "debug_info": {"str": str(actor)},
        "editor_properties": get_editor_property_descs(uobject=actor, recurse_into_objects=True),
        "name": get_stable_name_for_actor(actor=actor, include_unreal_id=True),
        "other_components": { get_stable_name_for_component(component=c): get_component_desc(component=c) for c in other_components },
        "root_component": get_component_desc(component=root_component),
        "stable_name": get_stable_name_for_actor(actor=actor, include_unreal_id=False),
        "unreal_name": actor.get_name()}

def get_component_desc(component):

    if component is None:
        return None

    if "get_children_components" in dir(component):
        children_components = { get_stable_name_for_component(component=c): get_component_desc(component=c) for c in component.get_children_components(include_all_descendants=False) }
    else:
        children_components = None

    component_desc = {
        "attributes": {},
        "children_components": children_components,
        "class": component.__class__.__name__,
        "debug_info": {"str": str(component)},
        "editor_properties": get_editor_property_descs(uobject=component, recurse_into_objects=True),
        "stable_name": get_stable_name_for_component(component=component),
        "pipeline_info": {},
        "unreal_name": component.get_name()}

    if isinstance(component, unreal.SceneComponent):
        component_desc["attributes"]["world_transform"] = get_value_desc(value=component.get_world_transform(), recurse_into_objects=True)
        component_desc["attributes"]["world_transform_as_matrix"] = get_value_desc(value=component.get_world_transform().to_matrix(), recurse_into_objects=True)

    if isinstance(component, unreal.StaticMeshComponent):
        component_desc["attributes"]["local_bounds"] = get_value_desc(value=component.get_local_bounds(), recurse_into_objects=True)

    return component_desc

def get_asset_desc(asset_data):

    asset_path = f"{asset_data.package_name}.{asset_data.asset_name}"
    spear.log("Processing asset: " + asset_path)

    # Immediate (non-recursive) dependencies come from the Asset Registry's cached metadata, so we get the
    # dependency package names without loading the referenced assets.
    dependency_options = unreal.AssetRegistryDependencyOptions(include_soft_package_references=True, include_hard_package_references=True)
    dependencies = sorted( str(d) for d in asset_registry.get_dependencies(package_name=asset_data.package_name, dependency_options=dependency_options) )

    # We don't recurse into objects, since every referenced asset is already its own top-level entry, and
    # following references could cycle.
    asset = asset_data.get_asset()
    return {
        "class": str(asset_data.asset_class_path.asset_name),
        "dependencies": dependencies,
        "editor_properties": get_editor_property_descs(uobject=asset, recurse_into_objects=False) if asset is not None else None,
        "name": str(asset_data.asset_name),
        "package": str(asset_data.package_name),
        "path": asset_path}

def get_editor_property_descs(uobject, recurse_into_objects):

    assert "get_editor_property" in dir(uobject)

    # The Unreal Python interface does not provide a mechanism to iterate over editor properties directly,
    # so we use the fact that most editor properties are exposed as Python attributes, and iterate over
    # attributes instead. For each attribute, we use a guess-and-check strategy to see if it is an editor
    # property. We call uobject.get_editor_property(...) and if doing so doesn't throw an exception, we
    # assume that the attribute is a valid editor property. To obtain any remaining editor properties that
    # are not exposed as Python attributes, we maintain a CSV file with a list of editor properties for
    # various classes of interest. We populate the CSV file by manually copying and pasting from the Unreal
    # documentation.

    # Get all Python attributes, and all editor properties from our CSV file.
    uobject_class_names = [ c.__name__ for c in uobject.__class__.mro() ] # derived-to-base order
    editor_property_candidate_names = set(dir(uobject))
    for uobject_class_name in uobject_class_names:
        editor_property_candidate_names = editor_property_candidate_names | set(df_editor_properties.loc[df_editor_properties["class"] == uobject_class_name]["editor_property"])

    # Guess-and-check all editor property candidates.
    editor_property_descs = {}
    for editor_property_candidate_name in editor_property_candidate_names:
        try:
            editor_property = uobject.get_editor_property(name=editor_property_candidate_name)
            editor_property_descs[editor_property_candidate_name] = get_value_desc(value=editor_property, recurse_into_objects=recurse_into_objects)
        except Exception:
            pass

    return editor_property_descs

def get_value_desc(value, recurse_into_objects):

    # If the value is None, then return None, because we want it to be serialized as null rather than "None".
    if value is None:
        return None

    # Otherwise, if the value is an Unreal array, Unreal set, or tuple, then recurse via get_value_desc(...).
    elif isinstance(value, (unreal.Array, unreal.Set, tuple)):
        return [ get_value_desc(value=entry, recurse_into_objects=recurse_into_objects) for entry in value ]

    # Otherwise, if the value is an Unreal map, then recurse via get_value_desc(...).
    elif isinstance(value, unreal.Map):
        return { str(k): get_value_desc(value=v, recurse_into_objects=recurse_into_objects) for k, v in value.items() }

    # Otherwise, if the value is an Unreal struct, then recurse via get_editor_property_descs(...). Structs are
    # stored inline, so we always recurse into them.
    elif isinstance(value, unreal.StructBase):
        return {
            "class": value.__class__.__name__,
            "debug_info": {"str": str(value)},
            "editor_properties": get_editor_property_descs(uobject=value, recurse_into_objects=recurse_into_objects)}

    # Otherwise, if the value is an Unreal object and recurse_into_objects is False, then return a reference to
    # the object rather than recursing into it. This is useful when each referenced object will be described
    # separately anyway, and following references could cycle.
    elif isinstance(value, unreal.Object) and not recurse_into_objects:
        return {
            "class": value.__class__.__name__,
            "debug_info": {"str": str(value)},
            "path": value.get_path_name()}

    # Otherwise, if the value is an Actor or ActorComponent, then do not return any editor properties to avoid
    # an infinite recursion. If users want to obtain the editor properties for an Actor or ActorComponent, they
    # must call get_editor_property_descs(...).

    elif isinstance(value, unreal.Actor):
        return {
            "class": value.__class__.__name__,
            "debug_info": {"str": str(value)},
            "name": get_stable_name_for_actor(actor=value, include_unreal_id=True),
            "stable_name": get_stable_name_for_actor(actor=value, include_unreal_id=False),
            "unreal_name": value.get_name()}

    elif isinstance(value, unreal.ActorComponent):
        return {
            "class": value.__class__.__name__,
            "debug_info": {"str": str(value)},
            "stable_name": get_stable_name_for_component(component=value),
            "unreal_name": value.get_name()}

    # Otherwise, if the value is a StaticMesh, then recurse via get_editor_property_descs(...).
    elif isinstance(value, unreal.StaticMesh):
        return {
            "class": value.__class__.__name__,
            "debug_info": {"str": str(value)},
            "editor_properties": get_editor_property_descs(uobject=value, recurse_into_objects=recurse_into_objects),
            "path": value.get_path_name()}

    # Otherwise, if the value is an Unreal object, then recurse via get_editor_property_descs(...).
    elif isinstance(value, unreal.Object):
        return {
            "class": value.__class__.__name__,
            "debug_info": {"str": str(value)},
            "editor_properties": get_editor_property_descs(uobject=value, recurse_into_objects=recurse_into_objects)}

    # Otherwise, if the value is serializable as JSON, then return the value, otherwise return the string
    # representation of the value.
    else:
        try:
            json.dumps(value)
            return value
        except TypeError:
            return str(value)


#
# Math conversion functions. In the functions below, converting to or from an "unreal" type means the type
# in the unreal namespace that is only available in the editor Python environment.
#

# unreal.Matrix

def to_numpy_matrix_from_unreal_matrix(unreal_matrix, as_matrix=None):
    return spear.math.to_numpy_matrix_from_spear_matrix(spear_matrix=to_spear_matrix_from_unreal_matrix(unreal_matrix=unreal_matrix), as_matrix=as_matrix)

def to_unreal_matrix_from_numpy_matrix(numpy_matrix):
    return to_unreal_matrix_from_spear_matrix(spear_matrix=spear.math.to_spear_matrix_from_numpy_matrix(numpy_matrix=numpy_matrix))

# unreal.Plane

def to_numpy_array_from_unreal_plane(unreal_plane, as_matrix=None):
    return spear.math.to_numpy_array_from_spear_plane(spear_plane=to_spear_plane_from_unreal_plane(unreal_plane=unreal_plane), as_matrix=as_matrix)

def to_unreal_plane_from_numpy_array(numpy_array):
    return to_unreal_plane_from_spear_plane(spear_plane=spear.math.to_spear_plane_from_numpy_array(numpy_array=numpy_array))

# unreal.Quat

def to_numpy_array_from_unreal_quat(unreal_quat):
    return spear.math.to_numpy_array_from_spear_quat(spear_quat=to_spear_quat_from_unreal_quat(unreal_quat=unreal_quat))

def to_numpy_matrix_from_unreal_quat(unreal_quat, as_matrix=None):
    return spear.math.to_numpy_matrix_from_spear_quat(spear_quat=to_spear_quat_from_unreal_quat(unreal_quat=unreal_quat), as_matrix=as_matrix)

def to_unreal_quat_from_numpy_array(numpy_array_xyzw):
    return to_unreal_quat_from_spear_quat(spear_quat=spear.math.to_spear_quat_from_numpy_array(numpy_array_xyzw=numpy_array_xyzw))

def to_unreal_quat_from_numpy_matrix(numpy_matrix):
    return to_unreal_quat_from_spear_quat(spear_quat=spear.math.to_spear_quat_from_numpy_matrix(numpy_matrix=numpy_matrix))

# unreal.Rotator

def to_numpy_array_from_unreal_rotator(unreal_rotator):
    return spear.math.to_numpy_array_from_spear_rotator(spear_rotator=to_spear_rotator_from_unreal_rotator(unreal_rotator=unreal_rotator))

def to_numpy_matrix_from_unreal_rotator(unreal_rotator, as_matrix=None):
    return spear.math.to_numpy_matrix_from_spear_rotator(spear_rotator=to_spear_rotator_from_unreal_rotator(unreal_rotator=unreal_rotator), as_matrix=as_matrix)

def to_unreal_rotator_from_numpy_array(numpy_array_pyr):
    return to_unreal_rotator_from_spear_rotator(spear_rotator=spear.math.to_spear_rotator_from_numpy_array(numpy_array_pyr=numpy_array_pyr))

def to_unreal_rotator_from_numpy_matrix(numpy_matrix):
    return to_unreal_rotator_from_spear_rotator(spear_rotator=spear.math.to_spear_rotator_from_numpy_matrix(numpy_matrix=numpy_matrix))

# unreal.Transform

def to_numpy_transform_from_unreal_transform(unreal_transform, as_quat=None, as_array=None, as_matrix=None):
    return spear.math.to_numpy_transform_from_spear_transform(spear_transform=to_spear_transform_from_unreal_transform(unreal_transform=unreal_transform), as_quat=as_quat, as_array=as_array, as_matrix=as_matrix)

def to_numpy_matrix_from_unreal_transform(unreal_transform, as_matrix=None):
    return spear.math.to_numpy_matrix_from_spear_transform(spear_transform=to_spear_transform_from_unreal_transform(unreal_transform=unreal_transform), as_matrix=as_matrix)

def to_unreal_transform_from_numpy_transform(numpy_transform):
    return to_unreal_transform_from_spear_transform(spear_transform=spear.math.to_spear_transform_from_numpy_transform(numpy_transform=numpy_transform))

def to_unreal_transform_from_numpy_matrix(numpy_matrix):
    return to_unreal_transform_from_spear_transform(spear_transform=spear.math.to_spear_transform_from_numpy_matrix(numpy_matrix=numpy_matrix))

# unreal.Vector

def to_numpy_array_from_unreal_vector(unreal_vector, as_matrix=None):
    return spear.math.to_numpy_array_from_spear_vector(spear_vector=to_spear_vector_from_unreal_vector(unreal_vector=unreal_vector), as_matrix=as_matrix)

def to_unreal_vector_from_numpy_array(numpy_array):
    return to_unreal_vector_from_spear_vector(spear_vector=spear.math.to_spear_vector_from_numpy_array(numpy_array=numpy_array))


#
# Convert between transform representations.
#

def convert_transform(transform, as_numpy=None, as_spear=None, as_unreal=None, as_quat=None, as_array=None, as_matrix=None):
    _validate_transform_flags(as_numpy=as_numpy, as_spear=as_spear, as_unreal=as_unreal, as_quat=as_quat, as_array=as_array, as_matrix=as_matrix)
    spear_transform = _to_spear_transform(transform=transform)
    if as_numpy is not None:
        return spear.math.convert_transform(transform=spear_transform, as_numpy=True, as_quat=as_quat, as_array=as_array, as_matrix=as_matrix)
    elif as_spear is not None:
        return spear_transform
    elif as_unreal is not None:
        return to_unreal_transform_from_spear_transform(spear_transform=spear_transform)
    else:
        assert False


#
# Composition functions
#

def compose_transforms(transforms, as_numpy=None, as_spear=None, as_unreal=None, as_quat=None, as_array=None, as_matrix=None):
    _validate_transform_flags(as_numpy=as_numpy, as_spear=as_spear, as_unreal=as_unreal, as_quat=as_quat, as_array=as_array, as_matrix=as_matrix)
    spear_transforms = [ _to_spear_transform(transform=t) for t in transforms ]
    if as_numpy is not None:
        return spear.math.compose_transforms(transforms=spear_transforms, as_numpy=True, as_quat=as_quat, as_array=as_array, as_matrix=as_matrix)
    elif as_spear is not None:
        return spear.math.compose_transforms(transforms=spear_transforms, as_spear=True)
    elif as_unreal is not None:
        spear_transform = spear.math.compose_transforms(transforms=spear_transforms, as_spear=True)
        return to_unreal_transform_from_spear_transform(spear_transform=spear_transform)
    else:
        assert False

def compose_component_transforms(transforms, is_absolute_location=None, is_absolute_rotation=None, is_absolute_scale=None, as_numpy=None, as_spear=None, as_unreal=None, as_quat=None, as_array=None, as_matrix=None):
    _validate_transform_flags(as_numpy=as_numpy, as_spear=as_spear, as_unreal=as_unreal, as_quat=as_quat, as_array=as_array, as_matrix=as_matrix)
    spear_transforms = [ _to_spear_transform(transform=t) for t in transforms ]
    if as_numpy is not None:
        return spear.math.compose_component_transforms(
            transforms=spear_transforms,
            is_absolute_location=is_absolute_location, is_absolute_rotation=is_absolute_rotation, is_absolute_scale=is_absolute_scale,
            as_numpy=True, as_quat=as_quat, as_array=as_array, as_matrix=as_matrix)
    elif as_spear is not None:
        return spear.math.compose_component_transforms(
            transforms=spear_transforms,
            is_absolute_location=is_absolute_location, is_absolute_rotation=is_absolute_rotation, is_absolute_scale=is_absolute_scale,
            as_spear=True)
    elif as_unreal is not None:
        spear_transform = spear.math.compose_component_transforms(
            transforms=spear_transforms,
            is_absolute_location=is_absolute_location, is_absolute_rotation=is_absolute_rotation, is_absolute_scale=is_absolute_scale,
            as_spear=True)
        return to_unreal_transform_from_spear_transform(spear_transform=spear_transform)
    else:
        assert False


#
# Helper functions for converting to and from the dictionaries that are returned when calling a UFUNCTION or
# accessing a UPROPERTY via the SPEAR API.
#

# unreal.Matrix

def to_spear_matrix_from_unreal_matrix(unreal_matrix):
    return {
        "XPlane": to_spear_plane_from_unreal_plane(unreal_plane=unreal_matrix.get_editor_property(name="x_plane")),
        "YPlane": to_spear_plane_from_unreal_plane(unreal_plane=unreal_matrix.get_editor_property(name="y_plane")),
        "ZPlane": to_spear_plane_from_unreal_plane(unreal_plane=unreal_matrix.get_editor_property(name="z_plane")),
        "WPlane": to_spear_plane_from_unreal_plane(unreal_plane=unreal_matrix.get_editor_property(name="w_plane"))}

def to_unreal_matrix_from_spear_matrix(spear_matrix):
    return unreal.Matrix(
        x_plane=to_unreal_plane_from_spear_plane(spear_plane=spear_matrix["XPlane"]),
        y_plane=to_unreal_plane_from_spear_plane(spear_plane=spear_matrix["YPlane"]),
        z_plane=to_unreal_plane_from_spear_plane(spear_plane=spear_matrix["ZPlane"]),
        w_plane=to_unreal_plane_from_spear_plane(spear_plane=spear_matrix["WPlane"]))

# unreal.Plane

def to_spear_plane_from_unreal_plane(unreal_plane):
    return {"X": unreal_plane.get_editor_property(name="x"), "Y": unreal_plane.get_editor_property(name="y"), "Z": unreal_plane.get_editor_property(name="z"), "W": unreal_plane.get_editor_property(name="w")}

def to_unreal_plane_from_spear_plane(spear_plane):
    return unreal.Plane(x=spear_plane["X"], y=spear_plane["Y"], z=spear_plane["Z"], w=spear_plane["W"])

# unreal.Quat

def to_spear_quat_from_unreal_quat(unreal_quat):
    return {"X": unreal_quat.get_editor_property(name="x"), "Y": unreal_quat.get_editor_property(name="y"), "Z": unreal_quat.get_editor_property(name="z"), "W": unreal_quat.get_editor_property(name="w")}

def to_unreal_quat_from_spear_quat(spear_quat):
    return unreal.Quat(x=spear_quat["X"], y=spear_quat["Y"], z=spear_quat["Z"], w=spear_quat["W"])

# unreal.Rotator

def to_spear_rotator_from_unreal_rotator(unreal_rotator):
    return {"Roll": unreal_rotator.get_editor_property(name="roll"), "Pitch": unreal_rotator.get_editor_property(name="pitch"), "Yaw": unreal_rotator.get_editor_property(name="yaw")}

def to_unreal_rotator_from_spear_rotator(spear_rotator):
    return unreal.Rotator(roll=spear_rotator["Roll"], pitch=spear_rotator["Pitch"], yaw=spear_rotator["Yaw"])

# unreal.Transform

def to_spear_transform_from_unreal_transform(unreal_transform):
    return {
        "Translation": to_spear_vector_from_unreal_vector(unreal_vector=unreal_transform.get_editor_property(name="translation")),
        "Rotation": to_spear_quat_from_unreal_quat(unreal_quat=unreal_transform.get_editor_property(name="rotation")),
        "Scale3D": to_spear_vector_from_unreal_vector(unreal_vector=unreal_transform.get_editor_property(name="scale3d"))}

# The unreal.Transform(...) constructor expects an unreal.Rotator, so we set the unreal.Quat directly instead.
def to_unreal_transform_from_spear_transform(spear_transform):
    unreal_transform = unreal.Transform()
    unreal_transform.set_editor_property(name="translation", value=to_unreal_vector_from_spear_vector(spear_vector=spear_transform["Translation"]))
    unreal_transform.set_editor_property(name="rotation", value=to_unreal_quat_from_spear_quat(spear_quat=spear_transform["Rotation"]))
    unreal_transform.set_editor_property(name="scale3d", value=to_unreal_vector_from_spear_vector(spear_vector=spear_transform["Scale3D"]))
    return unreal_transform

# unreal.Vector

def to_spear_vector_from_unreal_vector(unreal_vector):
    return {"X": unreal_vector.get_editor_property(name="x"), "Y": unreal_vector.get_editor_property(name="y"), "Z": unreal_vector.get_editor_property(name="z")}

def to_unreal_vector_from_spear_vector(spear_vector):
    return unreal.Vector(x=spear_vector["X"], y=spear_vector["Y"], z=spear_vector["Z"])


#
# Private helper functions for converting between transform representations and composing transforms.
#

def _validate_transform_flags(as_numpy=None, as_spear=None, as_unreal=None, as_quat=None, as_array=None, as_matrix=None):
    if as_numpy is not None:
        assert as_numpy
        assert as_spear is None
        assert as_unreal is None
    elif as_spear is not None:
        assert as_spear
        assert as_unreal is None
        assert as_quat is None
        assert as_array is None
        assert as_matrix is None
    elif as_unreal is not None:
        assert as_unreal
        assert as_quat is None
        assert as_array is None
        assert as_matrix is None
    else:
        assert False

def _to_spear_transform(transform):
    if isinstance(transform, unreal.Transform):
        return to_spear_transform_from_unreal_transform(unreal_transform=transform)
    elif isinstance(transform, dict) and set(transform.keys()) == set(["translation", "rotation", "scale"]):
        return spear.math.to_spear_transform_from_numpy_transform(numpy_transform=transform)
    elif isinstance(transform, dict) and set(transform.keys()) == set(["Translation", "Rotation", "Scale3D"]):
        return transform
    else:
        assert False


#
# imshow
#

def imshow(name, image, temp_dir=None, temp_file=None):
    if temp_dir is None:
        temp_dir = spear.path.realpath(unreal.Paths.project_saved_dir(), "SPEAR")
    if temp_file is None:
        temp_file = f"{name}.png"
    os.makedirs(temp_dir, exist_ok=True)

    path = spear.path.realpath(temp_dir, temp_file)
    spear.log("Saving image: ", path)
    cv2.imwrite(path, image)

    python_bin = spear.path.realpath(unreal.get_interpreter_executable_path())

    run_imshow = spear.path.realpath(os.path.dirname(spear.__file__), "tools", "run_imshow.py")
    cmd = [python_bin, run_imshow, "--path", path, "--name", name]
    spear.tools.run_cmd(cmd=cmd)


#
# Get filesystem path from content path
#

def get_filesystem_path_from_content_path(content_path):

    content_path_tokens = pathlib.PurePosixPath(content_path).parts
    assert len(content_path_tokens) >= 2
    content_root = content_path_tokens[1]

    # The unreal.Paths functions can return paths that are relative to the editor's working directory (e.g.,
    # "../../../SpearSim/Content/"), so we convert filesystem_base_dir to an absolute path. But we use
    # os.path.join(...) instead of spear.path.realpath(...) for paths inside filesystem_base_dir, because content
    # directories can be symlinks (e.g., see tools/update_symlink_for_external_content.py), and we want the
    # paths we return to refer to the symlinked paths inside filesystem_base_dir.
    if content_root == "Game":
        filesystem_base_dir = spear.path.realpath(unreal.Paths.project_content_dir())
    elif content_root == "Engine":
        filesystem_base_dir = spear.path.realpath(unreal.Paths.engine_content_dir())
    else:
        plugin_content_dir = unreal.PluginBlueprintLibrary.get_plugin_content_dir(plugin_name=content_root)
        assert plugin_content_dir is not None
        filesystem_base_dir = spear.path.realpath(plugin_content_dir)

    if len(content_path_tokens) == 2:
        return filesystem_base_dir
    else:
        content_sub_path = os.path.join(*content_path_tokens[2:])
        filesystem_path = os.path.join(filesystem_base_dir, content_sub_path)
        if os.path.exists(filesystem_path) and os.path.isdir(filesystem_path):
            return filesystem_path
        else:
            content_file_tokens = content_path_tokens[-1].split(".")
            if len(content_file_tokens) == 1:
                filesystem_paths = glob.glob(os.path.join(filesystem_base_dir, *content_path_tokens[2:-1], f"{content_file_tokens[0]}.*"))
                if len(filesystem_paths) == 1:
                    return filesystem_paths[0]
                else:
                    return os.path.join(filesystem_base_dir, *content_path_tokens[2:])
            elif len(content_file_tokens) == 2:
                assert content_file_tokens[0] == content_file_tokens[1]
                filesystem_paths = glob.glob(os.path.join(filesystem_base_dir, *content_path_tokens[2:-1], f"{content_file_tokens[0]}.*"))
                if len(filesystem_paths) == 1:
                    return filesystem_paths[0]
                else:
                    return os.path.join(filesystem_base_dir, *content_path_tokens[2:])
            else:
                assert False


#
# Converters for interoperability between SPEAR Python and Editor Python
#

# For passing values from SPEAR Python to Editor Python. Decode a {type_string, value} script expression dict,
# produced in SPEAR Python by spear.to_script_expr(...), into an Editor Python object. Called when the expression
# strings returned by spear.to_script_expr(...) are evaluated in Editor Python.

def from_script_expr(script_expr_dict):
    type_string = script_expr_dict["type_string"]
    value = script_expr_dict["value"]
    if type_string == "tuple":
        return tuple( from_script_expr(script_expr_dict=elem) for elem in value )
    elif type_string == "list":
        return [ from_script_expr(script_expr_dict=elem) for elem in value ]
    elif type_string == "dict":
        return { k: from_script_expr(script_expr_dict=v) for k, v in value.items() }
    elif type_string == "numpy.matrix":
        return np.matrix(np.array(value["data"], dtype=np.dtype(value["dtype"])).reshape(value["shape"]))
    elif type_string == "numpy.ndarray":
        return np.array(value["data"], dtype=np.dtype(value["dtype"])).reshape(value["shape"])
    elif type_string == "UnrealStruct" or type_string == "UnrealClass" or type_string == "UnrealObject":
        return unreal.SpFuncUtils.to_object_from_string(string=value)
    elif type_string == "StructInstance":
        struct_type_string = script_expr_dict["struct_type_string"]
        assert struct_type_string.startswith("unreal.")
        struct_class = getattr(unreal, struct_type_string.split(".", 1)[1])
        decoded_value = { k: from_script_expr(script_expr_dict=v) for k, v in value.items() }
        export_text = unreal.SpFuncUtils.get_struct_properties_as_export_text_from_json_string(script_struct=struct_class.static_struct(), properties_string=json.dumps(decoded_value))
        result = struct_class()
        result.import_text(content=export_text)
        return result
    else:
        return value

# For passing values from Editor Python to SPEAR Python. Encode an Editor Python object into a {type_string, value}
# script result JSON string, which is decoded in SPEAR Python by spear.from_script_result(...). Used by
# PythonService.evaluate_expression(...), which wraps its expression in spear.editor.to_script_result(...).

def to_script_result(obj):
    result = _to_script_result_dict(obj=obj)
    return json.dumps(result)

def _to_script_result_dict(obj):
    return { "type_string": _to_script_result_type_string(obj=obj), "value": _to_script_result_value(obj=obj) }

def _to_script_result_type_string(obj):
    if isinstance(obj, tuple):
        return "tuple"
    elif isinstance(obj, list):
        return "list"
    elif isinstance(obj, dict):
        return "dict"
    elif isinstance(obj, np.matrix):
        return "numpy.matrix"
    elif isinstance(obj, np.ndarray):
        return "numpy.ndarray"
    elif isinstance(obj, unreal.ScriptStruct):
        return "UnrealStruct"
    elif isinstance(obj, unreal.Class):
        return "UnrealClass"
    elif isinstance(obj, unreal.Object):
        return "UnrealObject"
    elif isinstance(obj, unreal.StructBase):
        return "StructInstance"
    elif type(obj).__module__ == "builtins":
        return type(obj).__qualname__
    else:
        return f"{type(obj).__module__}.{type(obj).__qualname__}"

def _to_script_result_value(obj):
    if isinstance(obj, tuple):
        return [ _to_script_result_dict(obj=item) for item in obj ]
    elif isinstance(obj, list):
        return [ _to_script_result_dict(obj=item) for item in obj ]
    elif isinstance(obj, dict):
        return { k: _to_script_result_dict(obj=v) for k, v in obj.items() }
    elif isinstance(obj, np.ndarray):
        return { "shape": list(obj.shape), "dtype": str(obj.dtype), "data": obj.tolist() }
    elif isinstance(obj, unreal.StructBase):
        return json.loads(unreal.SpFuncUtils.get_struct_properties_as_json_string_from_export_text(script_struct=type(obj).static_struct(), export_text=obj.export_text()))
    elif isinstance(obj, unreal.Object):
        return unreal.SpFuncUtils.to_string_from_object(object=obj)
    else:
        return obj


#
# Client. Mirrors the spear_ext client (see python_ext/cpp/client.h), and is responsible for converting between Python
# types and the msgpack representation expected by the server. For args, the client encodes strings as bytes, converts
# ClientStruct types to dicts, and converts SpPackedArray types to dicts with raw byte data. For return values, the client
# decodes bytes as strings, converts dicts to ClientStruct types, and converts raw byte data to NumPy arrays. The client
# is not responsible for any higher-level conversions, e.g., JSON strings are returned as strings.
#

_to_data_type = {
    np.dtype("uint8"): "u1",
    np.dtype("int8"): "i1",
    np.dtype("uint16"): "u2",
    np.dtype("int16"): "i2",
    np.dtype("uint32"): "u4",
    np.dtype("int32"): "i4",
    np.dtype("uint64"): "u8",
    np.dtype("int64"): "i8",
    np.dtype("float16"): "f2",
    np.dtype("float32"): "f4",
    np.dtype("float64"): "f8"}

_to_dtype = {v: k for k, v in _to_data_type.items()}

class Client:
    def __init__(self, address, port, timeout, reconnect_limit):
        assert spear.__can_import_msgpackrpc__
        self.verbose_rpc_calls = False
        self._client = msgpackrpc.Client(msgpackrpc.Address(address, port), timeout=timeout, reconnect_limit=reconnect_limit)

        # Custom converters for args that can't be converted generically by _to_arg(...), keyed by Python type.
        self._arg_converters = {
            SpPackedArray: self._to_sp_packed_array_arg}

        # Converters for return values, keyed by return_as string. Must match the s_type_names_ values in
        # cpp/unreal_plugins/SpServices/Source/SpServices/FuncSignatureRegistry.h, in the same order, which is validated
        # in _validate_entry_point_signature_type_names(). A converter is None if no conversion is needed, or if the
        # return value is a container that _to_return_value(...) converts by recursively converting its elements.
        self._return_value_converters = {
            "void":                                         None,
            "bool":                                         None,
            "float":                                        None,
            "int32":                                        None,
            "int64":                                        None,
            "uint64":                                       None,
            "string":                                       self._to_str_return_value,
            "vector_of_string":                             None,
            "vector_of_sp_handle":                          None,
            "vector_of_sp_static_struct_desc":              None,
            "vector_of_sp_static_class_desc":               None,
            "map_of_string_to_string":                      None,
            "map_of_string_to_sp_array_shared_memory_view": None,
            "map_of_string_to_sp_packed_array":             None,
            "map_of_string_to_sp_handle":                   None,
            "map_of_string_to_sp_property_value":           None,
            "map_of_string_to_sp_func_signature_desc":      None,
            "map_of_string_to_sp_world_desc":               None,
            "sp_output_log_messages":                       self._to_sp_output_log_messages_return_value,
            "sp_shared_memory_view":                        self._to_sp_shared_memory_view_return_value,
            "sp_array_shared_memory_view":                  self._to_sp_array_shared_memory_view_return_value,
            "sp_packed_array":                              self._to_sp_packed_array_return_value,
            "sp_func_data_bundle":                          self._to_sp_func_data_bundle_return_value,
            "sp_handle":                                    self._to_sp_handle_return_value,
            "sp_property_desc":                             self._to_sp_property_desc_return_value,
            "sp_property_value":                            self._to_sp_property_value_return_value,
            "sp_func_signature_desc":                       self._to_sp_func_signature_desc_return_value,
            "sp_future":                                    self._to_sp_future_return_value,
            "sp_function_desc":                             self._to_sp_function_desc_return_value,
            "sp_static_struct_desc":                        self._to_sp_static_struct_desc_return_value,
            "sp_static_class_desc":                         self._to_sp_static_class_desc_return_value,
            "sp_world_desc":                                self._to_sp_world_desc_return_value}

    def initialize(self):
        result = self._client.call("engine_service.call_sync_on_worker_thread.get_entry_point_signature_type_names")
        self.entry_point_signature_type_names = self._to_return_value(return_value=result, return_as="vector_of_string")
        self._validate_entry_point_signature_type_names()

        result = self._client.call("engine_service.call_sync_on_worker_thread.get_entry_point_signature_descs")
        self.entry_point_signature_descs = self._to_return_value(return_value=result, return_as="map_of_string_to_sp_func_signature_desc")

    def terminate(self):
        self._client.close()
        self.entry_point_signature_descs = {}
        self.entry_point_signature_type_names = []

    def get_timeout(self):
        assert False

    def set_timeout(self, value):
        assert False

    def clear_timeout(self):
        assert False

    def ping(self):
        result = self._client.call("engine_globals_service.call_sync_on_worker_thread.ping")
        return self._to_return_value(return_value=result, return_as="string")

    def get_current_process_id(self):
        result = self._client.call("engine_globals_service.call_sync_on_worker_thread.get_current_process_id")
        return self._to_return_value(return_value=result, return_as="int64")

    def is_server_local(self):
        return True

    def get_entry_point_signature_descs(self):
        return self.entry_point_signature_descs

    def get_entry_point_signature_type_names(self):
        return self.entry_point_signature_type_names

    def call(self, func_name, *args):
        desc = self.entry_point_signature_descs[func_name]
        return_as = desc.type_names[0]
        if self.verbose_rpc_calls:
            spear.log(f"Calling: {func_name} : {args} -> {return_as}")

        # Unlike the spear_ext client (see Client::call(...) in python_ext/cpp/client.h), we don't call
        # SharedMemoryFence::fence() before sending the request and after receiving the response, because pure
        # Python doesn't provide an atomic read-modify-write operation. So the server-side fences in
        # RpcServer::bindImpl(...) are not paired with corresponding fences here, and it is not strictly
        # guaranteed that memory operations on shared memory are ordered across this client and the server.
        args = self._to_args(args=args)
        return_value = self._client.call(func_name, *args)
        return_value = self._to_return_value(return_value=return_value, return_as=return_as)
        if self.verbose_rpc_calls:
            spear.log(f"Obtained return value: {return_value}")
        return return_value

    def _validate_entry_point_signature_type_names(self):
        client_type_names = list(self._return_value_converters.keys())
        server_type_names = self.entry_point_signature_type_names
        success = True
        for i in range(max(len(client_type_names), len(server_type_names))):
            if i < len(client_type_names):
                client_type_name = f'"{client_type_names[i]}"'
            else:
                client_type_name = "<missing>"
            if i < len(server_type_names):
                server_type_name = f'"{server_type_names[i]}"'
            else:
                server_type_name = "<missing>"
            if client_type_name != server_type_name:
                spear.log(f"ERROR: Type ID/name mismatch at id={i} (server={server_type_name}, client={client_type_name})")
                success = False
        if not success:
            spear.log("ERROR: The editor client is out of sync with the server. Make sure the return value converters in spear/utils/editor_utils.py match the type names in SpServices/FuncSignatureRegistry.h.")
        assert success

    #
    # Args
    #

    def _to_args(self, args):
        return [ self._to_arg(arg=arg) for arg in args ]

    def _to_arg(self, arg):
        if isinstance(arg, str):
            return self._to_str_arg(arg=arg)
        elif type(arg) in self._arg_converters:
            return self._to_arg(arg=self._arg_converters[type(arg)](arg=arg))
        elif isinstance(arg, ClientStruct):
            return { self._to_arg(arg=attribute_name): self._to_arg(arg=attribute_value) for attribute_name, attribute_value in vars(arg).items() }
        elif isinstance(arg, list):
            return [ self._to_arg(arg=a) for a in arg ]
        elif isinstance(arg, dict):
            return { self._to_arg(arg=k): self._to_arg(arg=v) for k, v in arg.items() }
        else:
            return arg

    def _to_str_arg(self, arg):
        return arg.encode("utf-8")

    # Custom arg converters

    def _to_sp_packed_array_arg(self, arg):
        assert isinstance(arg.data, np.ndarray)
        data_type = _to_data_type[np.dtype(arg.data.dtype)]
        if arg.data_source == "Internal":
            data = arg.data.tobytes()
        elif arg.data_source == "Shared":
            data = b""
        else:
            assert False
        return {"data": data, "data_source": arg.data_source, "shape": arg.shape, "data_type": data_type, "shared_memory_name": arg.shared_memory_name}

    #
    # Return values. The return_as strings used in _to_return_value(...) need to match the s_type_names_ values in:
    #     cpp/unreal_plugins/SpServices/Source/SpServices/FuncSignatureRegistry.h
    #

    def _to_return_value(self, return_value, return_as):
        if return_as.startswith("vector_of_"):
            value_return_as = return_as.removeprefix("vector_of_")
            return [ self._to_return_value(return_value=v, return_as=value_return_as) for v in return_value ]
        elif return_as.startswith("map_of_"):
            key_return_as, value_return_as = return_as.removeprefix("map_of_").split("_to_", 1)
            return { self._to_return_value(return_value=k, return_as=key_return_as): self._to_return_value(return_value=v, return_as=value_return_as) for k, v in return_value.items() }
        else:
            converter = self._return_value_converters[return_as]
            if converter is None:
                return return_value
            elif isinstance(return_value, dict):
                return converter(return_value={ self._to_str_return_value(return_value=k): v for k, v in return_value.items() })
            else:
                return converter(return_value=return_value)

    # Custom return value converters

    def _to_sp_output_log_messages_return_value(self, return_value):
        return SpOutputLogMessages(
            messages=self._to_return_value(return_value=return_value["messages"], return_as="vector_of_string"),
            num_evicted=return_value["num_evicted"])

    def _to_sp_shared_memory_view_return_value(self, return_value):
        return SpSharedMemoryView(
            id=self._to_str_return_value(return_value=return_value["id"]),
            num_bytes=return_value["num_bytes"],
            offset_bytes=return_value["offset_bytes"])

    def _to_sp_array_shared_memory_view_return_value(self, return_value):
        return SpArraySharedMemoryView(
            id=self._to_str_return_value(return_value=return_value["id"]),
            num_bytes=return_value["num_bytes"],
            offset_bytes=return_value["offset_bytes"],
            usage_flags=self._to_return_value(return_value=return_value["usage_flags"], return_as="vector_of_string"),
            name=self._to_str_return_value(return_value=return_value["name"]))

    def _to_sp_packed_array_return_value(self, return_value):
        data_source = self._to_str_return_value(return_value=return_value["data_source"])
        dtype = _to_dtype[self._to_str_return_value(return_value=return_value["data_type"])]
        if data_source == "Internal":
            data = np.frombuffer(return_value["data"], dtype=dtype, count=-1).reshape(return_value["shape"])
        elif data_source == "Shared":
            data = np.array([], dtype=dtype)
        else:
            assert False
        return SpPackedArray(
            data=data,
            data_source=data_source,
            shape=return_value["shape"],
            shared_memory_name=self._to_str_return_value(return_value=return_value["shared_memory_name"]))

    def _to_sp_func_data_bundle_return_value(self, return_value):
        return SpFuncDataBundle(
            packed_arrays=self._to_return_value(return_value=return_value["packed_arrays"], return_as="map_of_string_to_sp_packed_array"),
            unreal_obj_strings=self._to_return_value(return_value=return_value["unreal_obj_strings"], return_as="map_of_string_to_string"),
            info=self._to_str_return_value(return_value=return_value["info"]))

    def _to_sp_handle_return_value(self, return_value):
        return SpHandle(
            ptr=return_value["ptr"],
            object_index=return_value["object_index"],
            object_serial_number=return_value["object_serial_number"])

    def _to_sp_property_desc_return_value(self, return_value):
        return SpPropertyDesc(
            property=return_value["property"],
            value_ptr=return_value["value_ptr"],
            type_id=self._to_str_return_value(return_value=return_value["type_id"]),
            notify_objects=self._to_return_value(return_value=return_value["notify_objects"], return_as="vector_of_sp_handle"),
            notify_member_properties=return_value["notify_member_properties"],
            notify_element_properties=return_value["notify_element_properties"],
            notify_array_indices=return_value["notify_array_indices"],
            notify_map_keys=self._to_return_value(return_value=return_value["notify_map_keys"], return_as="vector_of_string"))

    def _to_sp_property_value_return_value(self, return_value):
        return SpPropertyValue(
            value=self._to_str_return_value(return_value=return_value["value"]),
            type_id=self._to_str_return_value(return_value=return_value["type_id"]))

    def _to_sp_func_signature_desc_return_value(self, return_value):
        return SpFuncSignatureDesc(
            name=self._to_str_return_value(return_value=return_value["name"]),
            type_ids=return_value["type_ids"],
            type_names=self._to_return_value(return_value=return_value["type_names"], return_as="vector_of_string"))

    def _to_sp_future_return_value(self, return_value):
        return SpFuture(
            future_ptr=return_value["future_ptr"],
            type_id=self._to_str_return_value(return_value=return_value["type_id"]))

    def _to_sp_function_desc_return_value(self, return_value):
        return SpFunctionDesc(
            function=self._to_return_value(return_value=return_value["function"], return_as="sp_handle"),
            function_name=self._to_str_return_value(return_value=return_value["function_name"]),
            static_class=self._to_return_value(return_value=return_value["static_class"], return_as="sp_handle"),
            static_class_name=self._to_str_return_value(return_value=return_value["static_class_name"]))

    def _to_sp_static_struct_desc_return_value(self, return_value):
        return SpStaticStructDesc(
            static_struct=self._to_return_value(return_value=return_value["static_struct"], return_as="sp_handle"),
            name=self._to_str_return_value(return_value=return_value["name"]))

    def _to_sp_static_class_desc_return_value(self, return_value):
        return SpStaticClassDesc(
            static_class=self._to_return_value(return_value=return_value["static_class"], return_as="sp_handle"),
            name=self._to_str_return_value(return_value=return_value["name"]),
            derived_to_base_classes=self._to_return_value(return_value=return_value.get("derived_to_base_classes", []), return_as="vector_of_sp_handle"),
            derived_to_base_class_names=self._to_return_value(return_value=return_value.get("derived_to_base_class_names", []), return_as="vector_of_string"),
            function_descs=self._to_return_value(return_value=return_value.get("function_descs", {}), return_as="map_of_string_to_sp_function_desc"))

    def _to_sp_world_desc_return_value(self, return_value):
        return SpWorldDesc(
            world=self._to_return_value(return_value=return_value["world"], return_as="sp_handle"),
            world_id=return_value["world_id"],
            is_editor_world=return_value["is_editor_world"],
            is_game_world=return_value["is_game_world"])

    def _to_str_return_value(self, return_value):
        if isinstance(return_value, bytes):
            return return_value.decode("utf-8")
        else:
            return return_value


#
# ClientStruct and derived structs. Keep sorted in the same order as in cpp/unreal_plugins/SpServices/Source/SpServices/MsgpackAdaptors.h
#

class ClientStruct:
    pass

class SpOutputLogMessages(ClientStruct):
    def __init__(self, messages=None, num_evicted=0):
        messages = messages if messages is not None else []
        self.messages = messages
        self.num_evicted = num_evicted

    def __repr__(self):
        return f"spear.SpOutputLogMessages(messages={len(self.messages)}, num_evicted={self.num_evicted})"

class SpSharedMemoryView(ClientStruct):
    def __init__(self, id="", num_bytes=0, offset_bytes=0):
        self.id = id
        self.num_bytes = num_bytes
        self.offset_bytes = offset_bytes

    def __repr__(self):
        return f"spear.SpSharedMemoryView(id={self.id}, num_bytes={self.num_bytes}, offset_bytes={self.offset_bytes})"

class SpArraySharedMemoryView(ClientStruct):
    def __init__(self, id="", num_bytes=0, offset_bytes=0, usage_flags=None, name="smem:invalid"):
        usage_flags = usage_flags if usage_flags is not None else ["DoNotUse"]
        self.id = id
        self.num_bytes = num_bytes
        self.offset_bytes = offset_bytes
        self.usage_flags = usage_flags
        self.name = name

    def __repr__(self):
        return f"spear.SpArraySharedMemoryView(id={self.id}, num_bytes={self.num_bytes}, offset_bytes={self.offset_bytes}, usage_flags={self.usage_flags}, name={self.name})"

class SpPackedArray(ClientStruct):
    def __init__(self, data=None, data_source="Invalid", shape=None, shared_memory_name=""):
        shape = shape if shape is not None else []
        self.data = data
        self.data_source = data_source
        self.shape = shape
        self.shared_memory_name = shared_memory_name

    def __repr__(self):
        return f'spear.SpPackedArray(data={self.data}, data_source="{self.data_source}", shape={self.shape}, shared_memory_name="{self.shared_memory_name}")'

class SpFuncDataBundle(ClientStruct):
    def __init__(self, packed_arrays=None, unreal_obj_strings=None, info=""):
        packed_arrays = packed_arrays if packed_arrays is not None else {}
        unreal_obj_strings = unreal_obj_strings if unreal_obj_strings is not None else {}
        self.packed_arrays = packed_arrays
        self.unreal_obj_strings = unreal_obj_strings
        self.info = info

    def __repr__(self):
        return f"spear.SpFuncDataBundle(packed_arrays={len(self.packed_arrays)}, unreal_obj_strings={len(self.unreal_obj_strings)}, info={len(self.info)})"

class SpHandle(ClientStruct):
    def __init__(self, ptr=0, object_index=0, object_serial_number=0):
        self.ptr = ptr
        self.object_index = object_index
        self.object_serial_number = object_serial_number

    def __eq__(self, other):
        return isinstance(other, SpHandle) and (self.ptr, self.object_index, self.object_serial_number) == (other.ptr, other.object_index, other.object_serial_number)

    def __hash__(self):
        return hash((self.ptr, self.object_index, self.object_serial_number))

    def __repr__(self):
        return f"spear.SpHandle(ptr={self.ptr:#x}, object_index={self.object_index}, object_serial_number={self.object_serial_number})"

class SpPropertyDesc(ClientStruct):
    def __init__(self, property=0, value_ptr=0, type_id="", notify_objects=None, notify_member_properties=None, notify_element_properties=None, notify_array_indices=None, notify_map_keys=None):
        self.property = property
        self.value_ptr = value_ptr
        self.type_id = type_id
        self.notify_objects = notify_objects if notify_objects is not None else []
        self.notify_member_properties = notify_member_properties if notify_member_properties is not None else []
        self.notify_element_properties = notify_element_properties if notify_element_properties is not None else []
        self.notify_array_indices = notify_array_indices if notify_array_indices is not None else []
        self.notify_map_keys = notify_map_keys if notify_map_keys is not None else []

    def __repr__(self):
        return f"spear.SpPropertyDesc(property={self.property:#x}, value_ptr={self.value_ptr:#x})"

class SpPropertyValue(ClientStruct):
    def __init__(self, value="", type_id=""):
        self.value = value
        self.type_id = type_id

    def __repr__(self):
        return f"spear.SpPropertyValue(value={self.value}, type_id={self.type_id})"

class SpFuncSignatureDesc(ClientStruct):
    def __init__(self, name="", type_ids=None, type_names=None):
        type_ids = type_ids if type_ids is not None else []
        type_names = type_names if type_names is not None else []
        self.name = name
        self.type_ids = type_ids
        self.type_names = type_names

    def __repr__(self):
        return f"spear.SpFuncSignatureDesc(name='{self.name}', type_ids={self.type_ids}, type_names={self.type_names})"

class SpFuture(ClientStruct):
    def __init__(self, future_ptr=0, type_id=""):
        self.future_ptr = future_ptr
        self.type_id = type_id

    def __repr__(self):
        return f'spear.SpFuture(future_ptr={self.future_ptr:#x}, type_id="{self.type_id}")'

class SpFunctionDesc(ClientStruct):
    def __init__(self, function=None, function_name="", static_class=None, static_class_name=""):
        function = function if function is not None else SpHandle()
        static_class = static_class if static_class is not None else SpHandle()
        self.function = function
        self.function_name = function_name
        self.static_class = static_class
        self.static_class_name = static_class_name

    def __repr__(self):
        return f"spear.SpFunctionDesc(function_name='{self.function_name}', static_class_name='{self.static_class_name}')"

class SpStaticStructDesc(ClientStruct):
    def __init__(self, static_struct=None, name=""):
        static_struct = static_struct if static_struct is not None else SpHandle()
        self.static_struct = static_struct
        self.name = name

    def __repr__(self):
        return f"spear.SpStaticStructDesc(name='{self.name}', static_struct={self.static_struct.ptr:#x})"

class SpStaticClassDesc(ClientStruct):
    def __init__(self, static_class=None, name="", derived_to_base_classes=None, derived_to_base_class_names=None, function_descs=None):
        static_class = static_class if static_class is not None else SpHandle()
        derived_to_base_classes = derived_to_base_classes if derived_to_base_classes is not None else []
        derived_to_base_class_names = derived_to_base_class_names if derived_to_base_class_names is not None else []
        function_descs = function_descs if function_descs is not None else {}
        self.static_class = static_class
        self.name = name
        self.derived_to_base_classes = derived_to_base_classes
        self.derived_to_base_class_names = derived_to_base_class_names
        self.function_descs = function_descs

    def __repr__(self):
        return f"spear.SpStaticClassDesc(name='{self.name}', static_class={self.static_class.ptr:#x}, function_descs={len(self.function_descs)})"

class SpWorldDesc(ClientStruct):
    def __init__(self, world=None, world_id=-1, is_editor_world=False, is_game_world=False):
        world = world if world is not None else SpHandle()
        self.world = world
        self.world_id = world_id
        self.is_editor_world = is_editor_world
        self.is_game_world = is_game_world

    def __repr__(self):
        return f"spear.SpWorldDesc(world={self.world.ptr:#x}, world_id={self.world_id}, is_editor_world={self.is_editor_world}, is_game_world={self.is_game_world})"
