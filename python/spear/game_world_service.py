#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import json

class GameWorldService():
    def __init__(self, rpc_client):
        self._rpc_client = rpc_client

    def get_world_name(self):
        return self._rpc_client.call("game_world_service.get_world_name")

    def open_level(self, level_name):
        self._rpc_client.call("game_world_service.open_level", level_name)

    def set_game_paused(self, paused):
        self._rpc_client.call("game_world_service.set_game_paused", paused)

    #
    # Find actors unconditionally and return a list or dict
    #

    def find_actors(self):
        return self._rpc_client.call("game_world_service.find_actors")

    def find_actors_as_dict(self):
        return self._rpc_client.call("game_world_service.find_actors_as_map")    

    #
    # Get components unconditionally and return a list or dict
    #

    def get_components(self, actor):
        return self._rpc_client.call("game_world_service.get_components", actor)

    def get_components_as_dict(self, actor):
        return self._rpc_client.call("game_world_service.get_components_as_map", actor)

    #
    # Get children components unconditionally and return a list or dict
    #

    def get_children_components(self, actor, include_all_descendants):
        return self._rpc_client.call("game_world_service.get_children_components", actor, include_all_descendants)

    def get_children_components_as_dict(self, actor, include_all_descendants):
        return self._rpc_client.call("game_world_service.get_children_components_as_map", actor, include_all_descendants)

    #
    # Get and Set object properties
    #

    def get_object_properties_as_string_from_uobject(self, uobject):
        return self._rpc_client.call("game_world_service.get_object_properties_as_string_from_uobject", uobject)

    def get_object_properties_as_string_from_ustruct(self, value_ptr, ustruct):
        return self._rpc_client.call("game_world_service.get_object_properties_as_string_from_ustruct", value_ptr, ustruct)

    def set_object_properties_from_string_for_uobject(self, uobject, string):
        self._rpc_client.call("game_world_service.set_object_properties_from_string_for_uobject", uobject, string)

    def set_object_properties_from_string_for_ustruct(self, value_ptr, ustruct, string):
        self._rpc_client.call("game_world_service.set_object_properties_from_string_for_ustruct", value_ptr, ustruct, string)

    #
    # Find properties
    #

    def find_property_by_name_on_uobject(self, uobject, name):
        return self._rpc_client.call("game_world_service.find_property_by_name_on_uobject", uobject, name)

    def find_property_by_name_on_ustruct(self, value_ptr, ustruct, name):
        return self._rpc_client.call("game_world_service.find_property_by_name_on_ustruct", value_ptr, ustruct, name)

    #
    # Get property values
    #

    def get_property_value_as_string(self, property_desc):
        return self._rpc_client.call("game_world_service.get_property_value_as_string", property_desc)

    def set_property_value_from_string(self, property_desc, string):
        return self._rpc_client.call("game_world_service.set_property_value_from_string", property_desc, string)

    #
    # Find and Call functions
    #

    def find_function_by_name(self, uclass, name, include_super_flag):
        return self._rpc_client.call("game_world_service.find_function_by_name", uclass, name, {"IncludeSuperFlag": json.dumps({"Enum": include_super_flag})})

    def call_function(self, uobject, ufunction, **kwargs):
        args = {}
        for key, value in kwargs.items():
            if isinstance(value, zip):
                args[key] = json.dumps(dict(value))
            elif isinstance(value, (dict, list)):
                args[key] = json.dumps(value)
            elif isinstance(value, (bool, float, int)):
                args[key] = str(value)
            elif isinstance(value, str):
                args[key] = value
            else:
                assert False
        return self._rpc_client.call("game_world_service.call_function", uobject, ufunction, args)

    #
    # Find structs by name
    #

    def find_special_struct_by_name(self, name):
        return self._rpc_client.call("game_world_service.find_special_struct_by_name", name)

    #
    # Stable Name
    #

    def has_stable_name(self, actor):
        return self._rpc_client.call("game_world_service.has_stable_name", actor)

    def get_stable_name_for_actor(self, actor):
        return self._rpc_client.call("game_world_service.get_stable_name_for_actor", actor)

    def get_stable_name_for_actor_component(self, component, include_actor_name):
        return self._rpc_client.call("game_world_service.get_stable_name_for_actor_component", component, include_actor_name)

    def get_stable_name_for_scene_component(self, component, include_actor_name):
        return self._rpc_client.call("game_world_service.get_stable_name_for_scene_component", component, include_actor_name)

    #
    # Get Actor and Component tags
    #

    def get_actor_tags(self, actor):
        return self._rpc_client.call("game_world_service.get_actor_tags", actor)

    def get_component_tags(self, actor):
        return self._rpc_client.call("game_world_service.get_component_tags", actor)

    #
    # Get Class
    #

    def get_class(self, uobject):
        return self._rpc_client.call("game_world_service.get_class", uobject)

    #
    # Create Components
    #

    def create_component_on_actor(class_name, owner, name):
        return self._rpc_client.call("game_world_service.create_component_outside_owner_constructor", class_name, owner, name)

    def create_scene_component_on_actor(class_name, owner, name):
        return self._rpc_client.call("game_world_service.create_scene_component_outside_owner_constructor_from_actor", class_name, owner, name)

    def create_scene_component_on_component(class_name, owner, parent, name):
        return self._rpc_client.call("game_world_service.create_scene_component_outside_owner_constructor_from_object", class_name, owner, parent, name)

    def create_scene_component_on_owning_scene_component(class_name, owner, name):
        return self._rpc_client.call("game_world_service.create_scene_component_outside_owner_constructor_from_component", class_name, owner, name)

    #
    # Create NewObject
    #

    def new_object(class_name, outer, name, flags, uobject_template, copy_transients_from_class_defaults, in_instance_graph):
        return self._rpc_client.call("game_world_service.new_object", class_name, outer, name, {"ObjectFlags": json.dumps({"Enum": flags})}, uobject_template, copy_transients_from_class_defaults, in_instance_graph)

    #
    # Load Object
    #

    def load_object(class_name, outer, name, filename, load_flags, sandbox, instancing_context):
        return self._rpc_client.call("game_world_service.load_object", class_name, outer, name, filename, {"LoadFlags": json.dumps({"Enum": load_flags})}, sandbox, instancing_context)

    #
    # Get Default Object
    #

    def get_default_object(uclass, create_if_needed):
        return self._rpc_client.call("game_world_service.get_default_object", uclass, create_if_needed)

    #
    # Get Static Class
    #

    def get_static_class(self, class_name):
        return self._rpc_client.call("game_world_service.get_static_class", class_name)

    #
    # Spawn actor
    #

    def spawn_actor(self, class_name, location, rotation, **spawn_params):
        return self._rpc_client.call("game_world_service.spawn_actor", class_name, location, rotation, {"SpawnParams": json.dumps(spawn_params)})

    #
    # Find Actors conditionally and return a list or dict
    #

    def find_actors_by_name(class_name, names, return_null_if_not_found):
        return self._rpc_client.call("game_world_service.find_actors_by_name", class_name, names, return_null_if_not_found)

    def find_actors_by_tag(class_name, tag):
        return self._rpc_client.call("game_world_service.find_actors_by_tag", class_name, tag)

    def find_actors_by_tag_any(class_name, tags):
        return self._rpc_client.call("game_world_service.find_actors_by_tag_any", class_name, tags)

    def find_actors_by_tag_all(class_name, tags):
        return self._rpc_client.call("game_world_service.find_actors_by_tag_all", class_name, tags)

    def find_actors_by_type(class_name):
        return self._rpc_client.call("game_world_service.find_actors_by_type", class_name)

    def find_actors_by_name_as_dict(class_name, names, return_null_if_not_found):
        return self._rpc_client.call("game_world_service.find_actors_by_name_as_map", class_name, names, return_null_if_not_found)

    def find_actors_by_tag_as_dict(class_name, tag):
        return self._rpc_client.call("game_world_service.find_actors_by_tag_as_map", class_name, tag)

    def find_actors_by_tag_any_as_dict(class_name, tags):
        return self._rpc_client.call("game_world_service.find_actors_by_tag_any_as_map", class_name, tags)

    def find_actors_by_tag_all_as_dict(class_name, tags):
        return self._rpc_client.call("game_world_service.find_actors_by_tag_all_as_map", class_name, tags)

    def find_actors_by_type_as_dict(class_name):
        return self._rpc_client.call("game_world_service.find_actors_by_type_as_map", class_name)

    #
    # Find Actors conditionally
    #

    def find_actor_by_name(class_name, name, assert_if_not_found):
        return self._rpc_client.call("game_world_service.find_actor_by_name", class_name, name, assert_if_not_found)

    def find_actor_by_tag(class_name, tag, assert_if_not_found, assert_if_multiple_found):
        return self._rpc_client.call("game_world_service.find_actor_by_tag", class_name, tag, assert_if_not_found, assert_if_multiple_found)

    def find_actor_by_tag_any(class_name, tags, assert_if_not_found, assert_if_multiple_found):
        return self._rpc_client.call("game_world_service.find_actor_by_tag_any", class_name, tags, assert_if_not_found, assert_if_multiple_found)

    def find_actor_by_tag_all(class_name, tags, assert_if_not_found, assert_if_multiple_found):
        return self._rpc_client.call("game_world_service.find_actor_by_tag_all", class_name, tags, assert_if_not_found, assert_if_multiple_found)

    def find_actor_by_type(class_name, assert_if_not_found, assert_if_multiple_found):
        return self._rpc_client.call("game_world_service.find_actor_by_type", class_name, assert_if_not_found, assert_if_multiple_found)

    #
    # Get Components conditionally and return a list or dict
    #

    def get_components_by_name(class_name, actor, names, return_null_if_not_found):
        return self._rpc_client.call("game_world_service.get_components_by_name", class_name, actor, names, return_null_if_not_found)

    def get_components_by_tag(class_name, actor, tag):
        return self._rpc_client.call("game_world_service.get_components_by_tag", class_name, actor, tag)

    def get_components_by_tag_any(class_name, actor, tags):
        return self._rpc_client.call("game_world_service.get_components_by_tag_any", class_name, actor, tags)

    def get_components_by_tag_all(class_name, actor, tags):
        return self._rpc_client.call("game_world_service.get_components_by_tag_all", class_name, actor, tags)

    def get_components_by_type(class_name, actor):
        return self._rpc_client.call("game_world_service.get_components_by_type", class_name, actor)

    def get_components_by_name_as_dict(class_name, actor, names, return_null_if_not_found):
        return self._rpc_client.call("game_world_service.get_components_by_name_as_map", class_name, actor, names, return_null_if_not_found)

    def get_components_by_tag_as_dict(class_name, actor, tag):
        return self._rpc_client.call("game_world_service.get_components_by_tag_as_map", class_name, actor, tag)

    def get_components_by_tag_any_as_dict(class_name, actor, tags):
        return self._rpc_client.call("game_world_service.get_components_by_tag_any_as_map", class_name, actor, tags)

    def get_components_by_tag_all_as_dict(class_name, actor, tags):
        return self._rpc_client.call("game_world_service.get_components_by_tag_all_as_map", class_name, actor, tags)

    def get_components_by_type_as_dict(class_name, actor):
        return self._rpc_client.call("game_world_service.get_components_by_type_as_map", class_name, actor)

    #
    # Get Component conditionally
    #

    def get_component_by_name(class_name, actor, name, assert_if_not_found):
        return self._rpc_client.call("game_world_service.get_component_by_name", class_name, actor, name, assert_if_not_found)

    def get_component_by_tag(class_name, actor, tag, assert_if_not_found, assert_if_multiple_found):
        return self._rpc_client.call("game_world_service.get_component_by_tag", class_name, actor, tag, assert_if_not_found, assert_if_multiple_found)

    def get_component_by_tag_any(class_name, actor, tags, assert_if_not_found, assert_if_multiple_found):
        return self._rpc_client.call("game_world_service.get_component_by_tag_any", class_name, actor, tags, assert_if_not_found, assert_if_multiple_found)

    def get_component_by_tag_all(class_name, actor, tags, assert_if_not_found, assert_if_multiple_found):
        return self._rpc_client.call("game_world_service.get_component_by_tag_all", class_name, actor, tags, assert_if_not_found, assert_if_multiple_found)

    def get_component_by_type(class_name, actor, assert_if_not_found, assert_if_multiple_found):
        return self._rpc_client.call("game_world_service.get_component_by_type", class_name, actor, assert_if_not_found, assert_if_multiple_found)

    #
    # Get Children Components conditionally from an Actor and return a list or dict
    #

    def get_children_components_by_name_from_actor(class_name,  parent, names, include_all_descendants, return_null_if_not_found):
        return self._rpc_client.call("game_world_service.get_children_components_by_name_from_actor", class_name,  parent, names, include_all_descendants, return_null_if_not_found)

    def get_children_components_by_tag_from_actor(class_name, parent, tag, include_all_descendants):
        return self._rpc_client.call("game_world_service.get_children_components_by_tag_from_actor", class_name, parent, tag, include_all_descendants)

    def get_children_components_by_tag_any_from_actor(class_name, parent, tags, include_all_descendants):
        return self._rpc_client.call("game_world_service.get_children_components_by_tag_any_from_actor", class_name, parent, tags, include_all_descendants)

    def get_children_components_by_tag_all_from_actor(class_name, parent, tags, include_all_descendants):
        return self._rpc_client.call("game_world_service.get_children_components_by_tag_all_from_actor", class_name, parent, tags, include_all_descendants)

    def get_children_components_by_type_from_actor(class_name, parent, include_all_descendants):
        return self._rpc_client.call("game_world_service.get_children_components_by_type_from_actor", class_name, parent, include_all_descendants)

    def get_children_components_by_name_as_map_from_actor(class_name, parent, names, include_all_descendants, return_null_if_not_found):
        return self._rpc_client.call("game_world_service.get_children_components_by_name_as_map_from_actor", class_name, parent, names, include_all_descendants, return_null_if_not_found)

    def get_children_components_by_tag_as_map_from_actor(class_name, parent, tag, include_all_descendants):
        return self._rpc_client.call("game_world_service.get_children_components_by_tag_as_map_from_actor", class_name, parent, tag, include_all_descendants)

    def get_children_components_by_tag_any_as_map_from_actor(class_name, parent, tags, include_all_descendants):
        return self._rpc_client.call("game_world_service.get_children_components_by_tag_any_as_map_from_actor", class_name, parent, tags, include_all_descendants)

    def get_children_components_by_tag_all_as_map_from_actor(class_name, parent, tags, include_all_descendants):
        return self._rpc_client.call("game_world_service.get_children_components_by_tag_all_as_map_from_actor", class_name, parent, tags, include_all_descendants)

    def get_children_components_by_type_as_map_from_actor(class_name, parent, include_all_descendants):
        return self._rpc_client.call("game_world_service.get_children_components_by_type_as_map_from_actor", class_name, parent, include_all_descendants)

    #
    # Get Child Component conditionally from an Actor
    #

    def get_child_component_by_name_from_actor(class_name, parent, name, include_all_descendants, assert_if_not_found):
        return self._rpc_client.call("game_world_service.get_child_component_by_name_from_actor", class_name, parent, name, include_all_descendants, assert_if_not_found)

    def get_child_component_by_tag_from_actor(class_name, parent, tag, include_all_descendants, assert_if_not_found, assert_if_multiple_found):
        return self._rpc_client.call("game_world_service.get_child_component_by_tag_from_actor", class_name, parent, tag, include_all_descendants, assert_if_not_found, assert_if_multiple_found)

    def get_child_component_by_tag_any_from_actor(class_name, parent, tags, include_all_descendants, assert_if_not_found, assert_if_multiple_found):
        return self._rpc_client.call("game_world_service.get_child_component_by_tag_any_from_actor", class_name, parent, tags, include_all_descendants, assert_if_not_found, assert_if_multiple_found)

    def get_child_component_by_tag_all_from_actor(class_name, parent, tags, include_all_descendants, assert_if_not_found, assert_if_multiple_found):
        return self._rpc_client.call("game_world_service.get_child_component_by_tag_all_from_actor", class_name, parent, tags, include_all_descendants, assert_if_not_found, assert_if_multiple_found)

    def get_child_component_by_type_from_actor(class_name, parent, include_all_descendants, assert_if_not_found, assert_if_multiple_found):
        return self._rpc_client.call("game_world_service.get_child_component_by_type_from_actor", class_name, parent, include_all_descendants, assert_if_not_found, assert_if_multiple_found)

    #
    # Get Children Components conditionally from a Scene Component and return a list or dict
    #

    def get_children_components_by_name_from_scene_component(class_name, parent, names, include_all_descendants, return_null_if_not_found):
        return self._rpc_client.call("game_world_service.get_children_components_by_name_from_scene_component", class_name, parent, names, include_all_descendants, return_null_if_not_found)

    def get_children_components_by_tag_from_scene_component(class_name, parent, tag, include_all_descendants):
        return self._rpc_client.call("game_world_service.get_children_components_by_tag_from_scene_component", class_name, parent, tag, include_all_descendants)

    def get_children_components_by_tag_any_from_scene_component(class_name, parent, tags, include_all_descendants):
        return self._rpc_client.call("game_world_service.get_children_components_by_tag_any_from_scene_component", class_name, parent, tags, include_all_descendants)

    def get_children_components_by_tag_all_from_scene_component(class_name, parent, tags, include_all_descendants):
        return self._rpc_client.call("game_world_service.get_children_components_by_tag_all_from_scene_component", class_name, parent, tags, include_all_descendants)

    def get_children_components_by_type_from_scene_component(class_name, parent, include_all_descendants):
        return self._rpc_client.call("game_world_service.get_children_components_by_type_from_scene_component", class_name, parent, include_all_descendants)

    def get_children_components_by_name_as_map_from_scene_component(class_name, parent, names, include_all_descendants, return_null_if_not_found):
        return self._rpc_client.call("game_world_service.get_children_components_by_name_as_map_from_scene_component", class_name, parent, names, include_all_descendants, return_null_if_not_found)

    def get_children_components_by_tag_as_map_from_scene_component(class_name, parent, tag, include_all_descendants):
        return self._rpc_client.call("game_world_service.get_children_components_by_tag_as_map_from_scene_component", class_name, parent, tag, include_all_descendants)

    def get_children_components_by_tag_any_as_map_from_scene_component(class_name, parent, tags, include_all_descendants):
        return self._rpc_client.call("game_world_service.get_children_components_by_tag_any_as_map_from_scene_component", class_name, parent, tags, include_all_descendants)

    def get_children_components_by_tag_all_as_map_from_scene_component(class_name, parent, tags, include_all_descendants):
        return self._rpc_client.call("game_world_service.get_children_components_by_tag_all_as_map_from_scene_component", class_name, parent, tags, include_all_descendants)

    def get_children_components_by_type_as_map_from_scene_component(class_name, parent, include_all_descendants):
        return self._rpc_client.call("game_world_service.get_children_components_by_type_as_map_from_scene_component", class_name, parent, include_all_descendants)

    #
    # Get Child Component conditionally from a Scene Component
    #

    def get_child_component_by_name_from_scene_component(class_name, parent, name, include_all_descendants, assert_if_not_found):
        return self._rpc_client.call("game_world_service.get_child_component_by_name_from_scene_component", class_name, parent, name, include_all_descendants, assert_if_not_found)

    def get_child_component_by_tag_from_scene_component(class_name, parent, tag, include_all_descendants, assert_if_not_found, assert_if_multiple_found):
        return self._rpc_client.call("game_world_service.get_child_component_by_tag_from_scene_component", class_name, parent, tag, include_all_descendants, assert_if_not_found, assert_if_multiple_found)

    def get_child_component_by_tag_any_from_scene_component(class_name, parent, tags, include_all_descendants, assert_if_not_found, assert_if_multiple_found):
        return self._rpc_client.call("game_world_service.get_child_component_by_tag_any_from_scene_component", class_name, parent, tags, include_all_descendants, assert_if_not_found, assert_if_multiple_found)

    def get_child_component_by_tag_all_from_scene_component(class_name, parent, tags, include_all_descendants, assert_if_not_found, assert_if_multiple_found):
        return self._rpc_client.call("game_world_service.get_child_component_by_tag_all_from_scene_component", class_name, parent, tags, include_all_descendants, assert_if_not_found, assert_if_multiple_found)

    def get_child_component_by_type_from_scene_component(class_name, parent, include_all_descendants, assert_if_not_found, assert_if_multiple_found):
        return self._rpc_client.call("game_world_service.get_child_component_by_type_from_scene_component", class_name, parent, include_all_descendants, assert_if_not_found, assert_if_multiple_found)
