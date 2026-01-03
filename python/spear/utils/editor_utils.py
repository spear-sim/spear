#
# Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import glob
import numpy as np
import os
import pathlib
import posixpath
import unreal


asset_tools = unreal.AssetToolsHelpers.get_asset_tools()
editor_actor_subsystem = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
subobject_data_subsystem = unreal.get_engine_subsystem(unreal.SubobjectDataSubsystem)


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
        candidate_components = [ c for c in candidate_components if get_stable_name_for_component(component=c) not in component_names ]
        candidate_components = [ c for c in candidate_components if isinstance(c, component_class) ]
        components = components + candidate_components
        component_names = component_names + [ get_stable_name_for_component(component=c) for c in candidate_components ]

    # add any components that are not in the main hierarchy, use component_names to make sure we're not
    # adding any components redundantly
    candidate_components = actor.get_components_by_class(component_class=component_class)
    candidate_components = [ c for c in candidate_components if get_stable_name_for_component(component=c) not in component_names ]
    candidate_components = [ c for c in candidate_components if isinstance(c, component_class) ]
    components = components + candidate_components
    component_names = component_names + [ get_stable_name_for_component(component=c) for c in candidate_components ]

    return components

def get_component(stable_name, actor=None):
    if actor is None:
        stable_actor_name, stable_component_name = stable_name.split(":")
        actor = find_actor(stable_name=stable_actor_name)
    else:
        stable_component_name = stable_name
    components = [ c for c in get_components(actor=actor) if get_stable_name_for_component(component=c) == stable_component_name ]
    if len(components) == 1:
        return components[0]
    else:
        return None

def get_stable_name_for_actor(actor):
    folder_path = actor.get_folder_path()
    if folder_path.is_none():
        return actor.get_actor_label()
    else:
        return posixpath.join(str(folder_path), actor.get_actor_label())

def get_stable_name_for_component(component, include_stable_actor_name=False):
    if include_stable_actor_name:
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
    blueprint_factory.set_editor_property("parent_class", parent_class)

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
    subobject_data_handles = subobject_data_subsystem.k2_gather_subobject_data_for_instance(instance)
    return get_subobject_descs_for_data_handles(subobject_data_handles)

def get_subobject_descs_for_blueprint_asset(blueprint_asset):
    assert isinstance(blueprint_asset, unreal.Blueprint)
    subobject_data_handles = subobject_data_subsystem.k2_gather_subobject_data_for_blueprint(blueprint_asset)
    return get_subobject_descs_for_data_handles(subobject_data_handles)

def get_subobject_descs_for_data_handles(subobject_data_handles):
    assert isinstance(subobject_data_handles, unreal.Array)
    return [ get_subobject_desc_for_data_handle(h) for h in subobject_data_handles ]

def get_subobject_desc_for_data_handle(subobject_data_handle):
    assert unreal.SubobjectDataBlueprintFunctionLibrary.is_handle_valid(data_handle=subobject_data_handle)
    subobject_data = unreal.SubobjectDataBlueprintFunctionLibrary.get_data(data_handle=subobject_data_handle)
    assert unreal.SubobjectDataBlueprintFunctionLibrary.is_valid(data=subobject_data)
    subobject_object = unreal.SubobjectDataBlueprintFunctionLibrary.get_object(data=subobject_data)
    return {"data_handle": subobject_data_handle, "data": subobject_data, "object": subobject_object}


#
# Get filesystem path from content path
#

def get_filesystem_path_from_content_path(content_path):

    content_path_tokens = pathlib.PurePosixPath(content_path).parts
    assert len(content_path_tokens) >= 2
    content_root = content_path_tokens[1]

    if content_root == "Game":
        filesystem_base_dir = unreal.Paths.project_content_dir()
    elif content_root == "Engine":
        filesystem_base_dir = unreal.Paths.engine_content_dir()
    else:
        filesystem_base_dir = unreal.PluginBlueprintLibrary.get_plugin_content_dir(plugin_name=content_root)
        assert filesystem_base_dir is not None

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
# Conversion functions
#

# Convert to a NumPy array from an unreal vector.
def to_numpy_array_from_vector(vector, as_matrix=None):
    if as_matrix is None:
        return np.array([vector.get_editor_property("x"), vector.get_editor_property("y"), vector.get_editor_property("z")])
    else:
        assert as_matrix
        return np.matrix([vector.get_editor_property("x"), vector.get_editor_property("y"), vector.get_editor_property("z")]).T

# Convert to an Unreal vector from a NumPy array or matrix.
def to_vector_from_numpy_array(array):
    if isinstance(array, np.matrix):
        assert array.shape == (3, 1)
        array = array.A1
    elif isinstance(array, np.ndarray):
        assert array.shape == (3,)
    else:
        assert False
    return unreal.Vector(x=array[0], y=array[1], z=array[2])
