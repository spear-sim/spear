#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import glob
import os
import pathlib
import posixpath
import unreal


#
# Get actors and components
#

def find_actors(actor_class=None):
    actors = editor_actor_subsystem.get_all_level_actors()
    actors = sorted(actors, key=lambda actor: get_stable_name_for_actor(actor=actor))
    if actor_class is not None:
        actors = [ actor for actor in actors if actor.__class__.__name__ == actor_class ]
    return actors

def find_actor(name):
    actors = [ actor for actor in find_actors() if get_stable_name_for_actor(actor=actor) == name ]
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
    candidate_components = actor.get_components_by_class(component_class)
    candidate_components = [ c for c in candidate_components if get_stable_name_for_component(component=c) not in component_names ]
    candidate_components = [ c for c in candidate_components if isinstance(c, component_class) ]
    components = components + candidate_components
    component_names = component_names + [ get_stable_name_for_component(component=c) for c in candidate_components ]

    return components

def get_component(stable_name, actor=None):
    if actor is None:
        stable_actor_name, stable_component_name = stable_name.split(":")
        actor = find_actor(stable_actor_name)
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
        return str(folder_path) + posixpath.sep + actor.get_actor_label()

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
# Create new blueprint types
#

def get_subobject_desc(subobject_data_handle):
    assert unreal.SubobjectDataBlueprintFunctionLibrary.is_handle_valid(subobject_data_handle)
    subobject_data = unreal.SubobjectDataBlueprintFunctionLibrary.get_data(subobject_data_handle)
    assert unreal.SubobjectDataBlueprintFunctionLibrary.is_valid(subobject_data)
    subobject_object = unreal.SubobjectDataBlueprintFunctionLibrary.get_object(subobject_data)
    return {"data_handle": subobject_data_handle, "data": subobject_data, "object": subobject_object}

def get_subobject_descs(subobject_data_handles):
    if isinstance(subobject_data_handles, unreal.Array):
        return [ get_subobject_desc(h) for h in subobject_data_handles ]
    elif isinstance(subobject_data_handles, dict):
        return { k: get_subobject_desc(subobject_data_handle=v) for k, v in subobject_data_handles.items() }
    else:
        assert False

def add_new_subobject(blueprint_asset, parent_data_handle, subobject_name, subobject_class):

    add_new_subobject_params = unreal.AddNewSubobjectParams(
        parent_handle=parent_data_handle,
        new_class=subobject_class,
        blueprint_context=blueprint_asset)
    subobject_data_handle, fail_reason = subobject_data_subsystem.add_new_subobject(add_new_subobject_params)
    assert fail_reason.is_empty()
    subobject_data = unreal.SubobjectDataBlueprintFunctionLibrary.get_data(subobject_data_handle)
    assert unreal.SubobjectDataBlueprintFunctionLibrary.is_valid(subobject_data)
    subobject_object = unreal.SubobjectDataBlueprintFunctionLibrary.get_object(subobject_data)
    assert isinstance(subobject_object, subobject_class)
    subobject_data_subsystem.rename_subobject(subobject_data_handle, unreal.Text(subobject_name))

    return {"data_handle": subobject_data_handle, "data": subobject_data, "object": subobject_object}

def create_blueprint(asset_name, package_path, actor_class=unreal.Actor, root_component_class=None, root_component_name=None):

    blueprint_factory = unreal.BlueprintFactory()
    blueprint_factory.set_editor_property("parent_class", actor_class)

    # asset_class should be set to None when creating a new blueprint asset
    blueprint_asset = asset_tools.create_asset(asset_name=asset_name, package_path=package_path, asset_class=None, factory=blueprint_factory)
    assert isinstance(blueprint_asset, unreal.Blueprint)

    blueprint_subobject_data_handles = subobject_data_subsystem.k2_gather_subobject_data_for_blueprint(blueprint_asset)
    blueprint_subobject_descs = get_subobject_descs(blueprint_subobject_data_handles)
    assert len(blueprint_subobject_descs) == 2
    assert isinstance(blueprint_subobject_descs[0]["object"], actor_class)

    if root_component_class is not None:
        subobject_data_subsystem.detach_subobject(
            owner_handle=blueprint_subobject_descs[0]["data_handle"],
            child_to_remove=blueprint_subobject_descs[1]["data_handle"])

        num_deleted = subobject_data_subsystem.delete_subobject(
            context_handle=blueprint_subobject_descs[0]["data_handle"],
            subobject_to_delete=blueprint_subobject_descs[1]["data_handle"],
            bp_context=blueprint_asset)
        assert num_deleted == 1

        root_component_suboject_desc = add_new_subobject(
            blueprint_asset=blueprint_asset,
            parent_data_handle=blueprint_subobject_descs[0]["data_handle"],
            subobject_name=root_component_name,
            subobject_class=root_component_class)

        subobject_data_subsystem.attach_subobject(
            owner_handle=blueprint_subobject_descs[0]["data_handle"],
            child_to_add_handle=root_component_suboject_desc["data_handle"])

        blueprint_subobject_data_handles = subobject_data_subsystem.k2_gather_subobject_data_for_blueprint(blueprint_asset)
        blueprint_subobject_descs = get_subobject_descs(blueprint_subobject_data_handles)
        assert len(blueprint_subobject_descs) == 2
        assert isinstance(blueprint_subobject_descs[0]["object"], actor_class)
        assert isinstance(blueprint_subobject_descs[1]["object"], root_component_class)

    if root_component_name is not None:
        subobject_data_subsystem.rename_subobject(blueprint_subobject_descs[1]["data_handle"], unreal.Text(root_component_name))

    blueprint_subobject_descs = {"actor": blueprint_subobject_descs[0], "root_component": blueprint_subobject_descs[1]}

    return blueprint_asset, blueprint_subobject_descs


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
        plugin_manager = unreal.PluginManager.get()
        plugin = plugin_manager.find_plugin(content_root)
        assert plugin is not None
        filesystem_base_dir = plugin.get_content_dir()

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
