#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import spear

class UnrealService(spear.utils.func_utils.Service):
    def __init__(self, entry_point_caller, create_children=True):

        self._entry_point_caller = entry_point_caller
        super().__init__(entry_point_caller, create_children) # do this after initializing local state

    def create_child(self, entry_point_caller):
        return UnrealService(entry_point_caller=entry_point_caller, create_children=False)

    #
    # Get engine subsystem
    #

    def get_engine_subsystem_by_type(self, class_name):
        return self._entry_point_caller.call_on_game_thread("uint64_t", "get_engine_subsystem_by_type", None, class_name)

    def get_engine_subsystem_by_class(self, uclass):
        return self._entry_point_caller.call_on_game_thread("uint64_t", "get_engine_subsystem_by_class", None, uclass)

    #
    # Get editor subsystem
    #

    def get_editor_subsystem_by_type(self, class_name):
        return self._entry_point_caller.call_on_game_thread("uint64_t", "get_editor_subsystem_by_type", None, class_name)

    def get_editor_subsystem_by_class(self, uclass):
        return self._entry_point_caller.call_on_game_thread("uint64_t", "get_editor_subsystem_by_class", None, uclass)

    #
    # Get subsystem
    #

    def get_subsystem_by_type(self, class_name):
        return self._entry_point_caller.call_on_game_thread("uint64_t", "get_subsystem_by_type", None, class_name)

    def get_subsystem_by_class(self, class_name, uclass):
        return self._entry_point_caller.call_on_game_thread("uint64_t", "get_subsystem_by_class", None, class_name, uclass)

    #
    # Get uclass from class name, get default object from uclass, get uclass from object
    #

    def get_static_class(self, class_name):
        return self._entry_point_caller.call_on_game_thread("uint64_t", "get_static_class", None, class_name)

    def get_default_object(self, uclass, create_if_needed=True):
        return self._entry_point_caller.call_on_game_thread("uint64_t", "get_default_object", None, uclass, create_if_needed)

    def get_class(self, uobject):
        return self._entry_point_caller.call_on_game_thread("uint64_t", "get_class", None, uobject)

    #
    # Get static struct
    #

    def get_static_struct(self, struct_name):
        return self._entry_point_caller.call_on_game_thread("uint64_t", "get_static_struct", None, struct_name)

    #
    # Get and set object properties
    #

    def get_properties_from_object(self, uobject):
        return self._entry_point_caller.call_on_game_thread(
            "std::string",
            "get_properties_as_string_from_object",
            lambda result: spear.utils.func_utils.try_to_dict(json_string=result, default_value={}),
            uobject)

    def get_properties_from_struct(self, value_ptr, ustruct):
        return self._entry_point_caller.call_on_game_thread(
            "std::string",
            "get_properties_as_string_from_struct",
            lambda result: spear.utils.func_utils.try_to_dict(json_string=result, default_value={}),
            value_ptr,
            ustruct)

    def set_properties_for_object(self, uobject, properties):
        return self._entry_point_caller.call_on_game_thread("void", "set_properties_from_string_for_object", None, uobject, spear.utils.func_utils.to_json_string(obj=properties))

    def set_properties_for_struct(self, value_ptr, ustruct, properties):
        return self._entry_point_caller.call_on_game_thread("void", "set_properties_from_string_for_struct", None, value_ptr, ustruct, spear.utils.func_utils.to_json_string(obj=properties))

    #
    # Find property
    #

    def find_property_by_name_on_object(self, uobject, property_name):
        return self._entry_point_caller.call_on_game_thread("PropertyDesc", "find_property_by_name_on_object", None, uobject, property_name)

    def find_property_by_name_on_struct(self, value_ptr, ustruct, property_name):
        return self._entry_point_caller.call_on_game_thread("PropertyDesc", "find_property_by_name_on_struct", None, value_ptr, ustruct, property_name)

    #
    # Get and set property value
    #

    def get_property_value(self, property_desc):
        return self._entry_point_caller.call_on_game_thread(
            "std::string",
            "get_property_value_as_string",
            lambda result: spear.utils.func_utils.try_to_dict(json_string=result),
            property_desc)

    def set_property_value(self, property_desc, property_value):
        return self._entry_point_caller.call_on_game_thread("void", "set_property_value_from_string", None, property_desc, spear.utils.func_utils.to_json_string(obj=property_value))

    #
    # Find and call function
    #

    def find_function_by_name(self, uclass, function_name, include_super_flag="IncludeSuper"):
        return self._entry_point_caller.call_on_game_thread("uint64_t", "find_function_by_name", None, uclass, function_name, include_super_flag)

    #
    # Interface for calling functions. When using this interface, pointers must be handled specially. For
    # example, suppose you want to call a function that takes a pointer as input and returns a pointer as
    # output. Suppose that you already have a handle that you would like to pass as input to the function,
    # that you obtained from another UnrealService function, e.g.,
    #
    #     actor_handle = unreal_service.find_actor_by_name(...)
    #
    # You would invoke your desired function as follows. After executing this code, return_value_handle will
    # be in the correct form to pass into other functions in unreal_service.
    # 
    #     args = {"Actor": spear.to_ptr(handle=actor_handle)}
    #     return_values = unreal_service.call_function(uobject=uobject, ufunction=ufunction, args=args)
    #     return_value_handle = spear.to_handle(string=return_values["ReturnValue"])
    #

    # call an arbitrary function
    def call_function(self, uobject, ufunction, args={}, world_context="WorldContextObject"):
        return self._entry_point_caller.call_on_game_thread(
            "std::map<std::string, std::string>",
            "call_function",
            lambda result: spear.utils.func_utils.try_to_dicts(json_strings=result),
            uobject,
            ufunction,
            spear.utils.func_utils.to_json_strings(objs=args),
            world_context)

    #
    # Find actors unconditionally and return a list or dict
    #

    def find_actors(self):
        return self._entry_point_caller.call_on_game_thread("std::vector<uint64_t>", "find_actors", None)

    def find_actors_as_dict(self):
        return self._entry_point_caller.call_on_game_thread("std::map<std::string, uint64_t>", "find_actors_as_map", None)

    #
    # Get components unconditionally and return a list or dict
    #

    def get_components(self, actor):
        return self._entry_point_caller.call_on_game_thread("std::vector<uint64_t>", "get_components", None, actor)

    def get_components_as_dict(self, actor):
        return self._entry_point_caller.call_on_game_thread("std::map<std::string, uint64_t>", "get_components_as_map", None, actor)

    #
    # Get children components unconditionally and return a list or dict
    #

    def get_children_components(self, actor, include_all_descendants=True):
        return self._entry_point_caller.call_on_game_thread("std::vector<uint64_t>", "get_children_components", None, actor, include_all_descendants)

    def get_children_components_as_dict(self, actor, include_all_descendants=True):
        return self._entry_point_caller.call_on_game_thread("std::map<std::string, uint64_t>", "get_children_components_as_map", None, actor, include_all_descendants)

    #
    # Find actors conditionally and return a list
    #

    def find_actors_by_name(self, class_name, actor_names, return_null_if_not_found=True):
        return self._entry_point_caller.call_on_game_thread("std::vector<uint64_t>", "find_actors_by_name", None, class_name, actor_names, return_null_if_not_found)

    def find_actors_by_tag(self, class_name, tag):
        return self._entry_point_caller.call_on_game_thread("std::vector<uint64_t>", "find_actors_by_tag", None, class_name, tag)

    def find_actors_by_tag_any(self, class_name, tags):
        return self._entry_point_caller.call_on_game_thread("std::vector<uint64_t>", "find_actors_by_tag_any", None, class_name, tags)

    def find_actors_by_tag_all(self, class_name, tags):
        return self._entry_point_caller.call_on_game_thread("std::vector<uint64_t>", "find_actors_by_tag_all", None, class_name, tags)

    def find_actors_by_type(self, class_name):
        return self._entry_point_caller.call_on_game_thread("std::vector<uint64_t>", "find_actors_by_type", None, class_name)

    def find_actors_by_class(self, uclass):
        return self._entry_point_caller.call_on_game_thread("std::vector<uint64_t>", "find_actors_by_class", None, uclass)

    #
    # Find actors conditionally and return a dict
    #

    def find_actors_by_name_as_dict(self, class_name, actor_names, return_null_if_not_found=True):
        return self._entry_point_caller.call_on_game_thread("std::map<std::string, uint64_t>", "find_actors_by_name_as_map", None, class_name, actor_names, return_null_if_not_found)

    def find_actors_by_tag_as_dict(self, class_name, tag):
        return self._entry_point_caller.call_on_game_thread("std::map<std::string, uint64_t>", "find_actors_by_tag_as_map", None, class_name, tag)

    def find_actors_by_tag_any_as_dict(self, class_name, tags):
        return self._entry_point_caller.call_on_game_thread("std::map<std::string, uint64_t>", "find_actors_by_tag_any_as_map", None, class_name, tags)

    def find_actors_by_tag_all_as_dict(self, class_name, tags):
        return self._entry_point_caller.call_on_game_thread("std::map<std::string, uint64_t>", "find_actors_by_tag_all_as_map", None, class_name, tags)

    def find_actors_by_type_as_dict(self, class_name):
        return self._entry_point_caller.call_on_game_thread("std::map<std::string, uint64_t>", "find_actors_by_type_as_map", None, class_name)

    def find_actors_by_class_as_dict(self, uclass):
        return self._entry_point_caller.call_on_game_thread("std::map<std::string, uint64_t>", "find_actors_by_class_as_map", None, uclass)

    #
    # Find actor conditionally
    #

    def find_actor_by_name(self, class_name, actor_name):
        return self._entry_point_caller.call_on_game_thread("uint64_t", "find_actor_by_name", None, class_name, actor_name)

    def find_actor_by_tag(self, class_name, tag):
        return self._entry_point_caller.call_on_game_thread("uint64_t", "find_actor_by_tag", None, class_name, tag)

    def find_actor_by_tag_any(self, class_name, tags):
        return self._entry_point_caller.call_on_game_thread("uint64_t", "find_actor_by_tag_any", None, class_name, tags)

    def find_actor_by_tag_all(self, class_name, tags):
        return self._entry_point_caller.call_on_game_thread("uint64_t", "find_actor_by_tag_all", None, class_name, tags)

    def find_actor_by_type(self, class_name):
        return self._entry_point_caller.call_on_game_thread("uint64_t", "find_actor_by_type", None, class_name)

    def find_actor_by_class(self, uclass):
        return self._entry_point_caller.call_on_game_thread("uint64_t", "find_actor_by_class", None, uclass)

    #
    # Get components conditionally and return a list
    #

    def get_components_by_name(self, class_name, actor, component_names, include_from_child_actors=False, return_null_if_not_found=True):
        return self._entry_point_caller.call_on_game_thread("std::vector<uint64_t>", "get_components_by_name", None, class_name, actor, component_names, include_from_child_actors, return_null_if_not_found)

    def get_components_by_path(self, class_name, actor, component_paths, include_from_child_actors=False, return_null_if_not_found=True):
        return self._entry_point_caller.call_on_game_thread("std::vector<uint64_t>", "get_components_by_path", None, class_name, actor, component_paths, include_from_child_actors, return_null_if_not_found)

    def get_components_by_tag(self, class_name, actor, tag, include_from_child_actors=False):
        return self._entry_point_caller.call_on_game_thread("std::vector<uint64_t>", "get_components_by_tag", None, class_name, actor, tag, include_from_child_actors)

    def get_components_by_tag_any(self, class_name, actor, tags, include_from_child_actors=False):
        return self._entry_point_caller.call_on_game_thread("std::vector<uint64_t>", "get_components_by_tag_any", None, class_name, actor, tags, include_from_child_actors)

    def get_components_by_tag_all(self, class_name, actor, tags, include_from_child_actors=False):
        return self._entry_point_caller.call_on_game_thread("std::vector<uint64_t>", "get_components_by_tag_all", None, class_name, actor, tags, include_from_child_actors)

    def get_components_by_type(self, class_name, actor, include_from_child_actors=False):
        return self._entry_point_caller.call_on_game_thread("std::vector<uint64_t>", "get_components_by_type", None, class_name, actor, include_from_child_actors)

    def get_components_by_class(self, actor, uclass, include_from_child_actors=False):
        return self._entry_point_caller.call_on_game_thread("std::vector<uint64_t>", "get_components_by_class", None, actor, uclass, include_from_child_actors)

    #
    # Get components conditionally and return a dict
    #

    def get_components_by_name_as_dict(self, class_name, actor, component_names, include_from_child_actors=False, return_null_if_not_found=True):
        return self._entry_point_caller.call_on_game_thread("std::map<std::string, uint64_t>", "get_components_by_name_as_map", None, class_name, actor, component_names, include_from_child_actors, return_null_if_not_found)

    def get_components_by_path_as_dict(self, class_name, actor, component_paths, include_from_child_actors=False, return_null_if_not_found=True):
        return self._entry_point_caller.call_on_game_thread("std::map<std::string, uint64_t>", "get_components_by_path_as_map", None, class_name, actor, component_paths, include_from_child_actors, return_null_if_not_found)

    def get_components_by_tag_as_dict(self, class_name, actor, tag, include_from_child_actors=False):
        return self._entry_point_caller.call_on_game_thread("std::map<std::string, uint64_t>", "get_components_by_tag_as_map", None, class_name, actor, tag, include_from_child_actors)

    def get_components_by_tag_any_as_dict(self, class_name, actor, tags, include_from_child_actors=False):
        return self._entry_point_caller.call_on_game_thread("std::map<std::string, uint64_t>", "get_components_by_tag_any_as_map", None, class_name, actor, tags, include_from_child_actors)

    def get_components_by_tag_all_as_dict(self, class_name, actor, tags, include_from_child_actors=False):
        return self._entry_point_caller.call_on_game_thread("std::map<std::string, uint64_t>", "get_components_by_tag_all_as_map", None, class_name, actor, tags, include_from_child_actors)

    def get_components_by_type_as_dict(self, class_name, actor, include_from_child_actors=False):
        return self._entry_point_caller.call_on_game_thread("std::map<std::string, uint64_t>", "get_components_by_type_as_map", None, class_name, actor, include_from_child_actors)

    def get_components_by_class_as_dict(self, actor, uclass, include_from_child_actors=False):
        return self._entry_point_caller.call_on_game_thread("std::map<std::string, uint64_t>", "get_components_by_class_as_map", None, actor, uclass, include_from_child_actors)

    #
    # Get component conditionally
    #

    def get_component_by_name(self, class_name, actor, component_name, include_from_child_actors=False):
        return self._entry_point_caller.call_on_game_thread("uint64_t", "get_component_by_name", None, class_name, actor, component_name, include_from_child_actors)

    def get_component_by_path(self, class_name, actor, component_path, include_from_child_actors=False):
        return self._entry_point_caller.call_on_game_thread("uint64_t", "get_component_by_path", None, class_name, actor, component_path, include_from_child_actors)

    def get_component_by_tag(self, class_name, actor, tag, include_from_child_actors=False):
        return self._entry_point_caller.call_on_game_thread("uint64_t", "get_component_by_tag", None, class_name, actor, tag, include_from_child_actors)

    def get_component_by_tag_any(self, class_name, actor, tags, include_from_child_actors=False):
        return self._entry_point_caller.call_on_game_thread("uint64_t", "get_component_by_tag_any", None, class_name, actor, tags, include_from_child_actors)

    def get_component_by_tag_all(self, class_name, actor, tags, include_from_child_actors=False):
        return self._entry_point_caller.call_on_game_thread("uint64_t", "get_component_by_tag_all", None, class_name, actor, tags, include_from_child_actors)

    def get_component_by_type(self, class_name, actor, include_from_child_actors=False):
        return self._entry_point_caller.call_on_game_thread("uint64_t", "get_component_by_type", None, class_name, actor, include_from_child_actors)

    def get_component_by_class(self, actor, uclass, include_from_child_actors=False):
        return self._entry_point_caller.call_on_game_thread("uint64_t", "get_component_by_class", None, actor, uclass, include_from_child_actors)

    #
    # Get children components conditionally from an actor and return a list
    #

    def get_children_components_by_name_from_actor(self, class_name, parent, children_component_names, include_all_descendants=True, return_null_if_not_found=True):
        return self._entry_point_caller.call_on_game_thread("std::vector<uint64_t>", "get_children_components_by_name_from_actor", None, class_name,  parent, children_component_names, include_all_descendants, return_null_if_not_found)

    def get_children_components_by_tag_from_actor(self, class_name, parent, tag, include_all_descendants=True):
        return self._entry_point_caller.call_on_game_thread("std::vector<uint64_t>", "get_children_components_by_tag_from_actor", None, class_name, parent, tag, include_all_descendants)

    def get_children_components_by_tag_any_from_actor(self, class_name, parent, tags, include_all_descendants=True):
        return self._entry_point_caller.call_on_game_thread("std::vector<uint64_t>", "get_children_components_by_tag_any_from_actor", None, class_name, parent, tags, include_all_descendants)

    def get_children_components_by_tag_all_from_actor(self, class_name, parent, tags, include_all_descendants=True):
        return self._entry_point_caller.call_on_game_thread("std::vector<uint64_t>", "get_children_components_by_tag_all_from_actor", None, class_name, parent, tags, include_all_descendants)

    def get_children_components_by_type_from_actor(self, class_name, parent, include_all_descendants=True):
        return self._entry_point_caller.call_on_game_thread("std::vector<uint64_t>", "get_children_components_by_type_from_actor", None, class_name, parent, include_all_descendants)

    def get_children_components_by_class_from_actor(self, parent, uclass, include_all_descendants=True):
        return self._entry_point_caller.call_on_game_thread("std::vector<uint64_t>", "get_children_components_by_class_from_actor", None, parent, uclass, include_all_descendants)

    #
    # Get children components conditionally from an actor and return a dict
    #

    def get_children_components_by_name_as_dict_from_actor(self, class_name, parent, children_component_names, include_all_descendants=True, return_null_if_not_found=True):
        return self._entry_point_caller.call_on_game_thread("std::map<std::string, uint64_t>", "get_children_components_by_name_as_map_from_actor", None, class_name, parent, children_component_names, include_all_descendants, return_null_if_not_found)

    def get_children_components_by_tag_as_dict_from_actor(self, class_name, parent, tag, include_all_descendants=True):
        return self._entry_point_caller.call_on_game_thread("std::map<std::string, uint64_t>", "get_children_components_by_tag_as_map_from_actor", None, class_name, parent, tag, include_all_descendants)

    def get_children_components_by_tag_any_as_dict_from_actor(self, class_name, parent, tags, include_all_descendants=True):
        return self._entry_point_caller.call_on_game_thread("std::map<std::string, uint64_t>", "get_children_components_by_tag_any_as_map_from_actor", None, class_name, parent, tags, include_all_descendants)

    def get_children_components_by_tag_all_as_dict_from_actor(self, class_name, parent, tags, include_all_descendants=True):
        return self._entry_point_caller.call_on_game_thread("std::map<std::string, uint64_t>", "get_children_components_by_tag_all_as_map_from_actor", None, class_name, parent, tags, include_all_descendants)

    def get_children_components_by_type_as_dict_from_actor(self, class_name, parent, include_all_descendants=True):
        return self._entry_point_caller.call_on_game_thread("std::map<std::string, uint64_t>", "get_children_components_by_type_as_map_from_actor", None, class_name, parent, include_all_descendants)

    def get_children_components_by_class_as_dict_from_actor(self, parent, uclass, include_all_descendants=True):
        return self._entry_point_caller.call_on_game_thread("std::map<std::string, uint64_t>", "get_children_components_by_class_as_map_from_actor", None, parent, uclass, include_all_descendants)

    #
    # Get child component conditionally from an actor
    #

    def get_child_component_by_name_from_actor(self, class_name, parent, child_component_name, include_all_descendants=True):
        return self._entry_point_caller.call_on_game_thread("uint64_t", "get_child_component_by_name_from_actor", None, class_name, parent, child_component_name, include_all_descendants)

    def get_child_component_by_tag_from_actor(self, class_name, parent, tag, include_all_descendants=True):
        return self._entry_point_caller.call_on_game_thread("uint64_t", "get_child_component_by_tag_from_actor", None, class_name, parent, tag, include_all_descendants)

    def get_child_component_by_tag_any_from_actor(self, class_name, parent, tags, include_all_descendants=True):
        return self._entry_point_caller.call_on_game_thread("uint64_t", "get_child_component_by_tag_any_from_actor", None, class_name, parent, tags, include_all_descendants)

    def get_child_component_by_tag_all_from_actor(self, class_name, parent, tags, include_all_descendants=True):
        return self._entry_point_caller.call_on_game_thread("uint64_t", "get_child_component_by_tag_all_from_actor", None, class_name, parent, tags, include_all_descendants)

    def get_child_component_by_type_from_actor(self, class_name, parent, include_all_descendants=True):
        return self._entry_point_caller.call_on_game_thread("uint64_t", "get_child_component_by_type_from_actor", None, class_name, parent, include_all_descendants)

    def get_child_component_by_class_from_actor(self, parent, uclass, include_all_descendants=True):
        return self._entry_point_caller.call_on_game_thread("uint64_t", "get_child_component_by_class_from_actor", None, parent, uclass, include_all_descendants)

    #
    # Get children components conditionally from a scene component and return a list
    #

    def get_children_components_by_name_from_scene_component(self, class_name, parent, children_component_names, include_all_descendants=True, return_null_if_not_found=True):
        return self._entry_point_caller.call_on_game_thread("std::vector<uint64_t>", "get_children_components_by_name_from_scene_component", None, class_name, parent, children_component_names, include_all_descendants, return_null_if_not_found)

    def get_children_components_by_tag_from_scene_component(self, class_name, parent, tag, include_all_descendants=True):
        return self._entry_point_caller.call_on_game_thread("std::vector<uint64_t>", "get_children_components_by_tag_from_scene_component", None, class_name, parent, tag, include_all_descendants)

    def get_children_components_by_tag_any_from_scene_component(self, class_name, parent, tags, include_all_descendants=True):
        return self._entry_point_caller.call_on_game_thread("std::vector<uint64_t>", "get_children_components_by_tag_any_from_scene_component", None, class_name, parent, tags, include_all_descendants)

    def get_children_components_by_tag_all_from_scene_component(self, class_name, parent, tags, include_all_descendants=True):
        return self._entry_point_caller.call_on_game_thread("std::vector<uint64_t>", "get_children_components_by_tag_all_from_scene_component", None, class_name, parent, tags, include_all_descendants)

    def get_children_components_by_type_from_scene_component(self, class_name, parent, include_all_descendants=True):
        return self._entry_point_caller.call_on_game_thread("std::vector<uint64_t>", "get_children_components_by_type_from_scene_component", None, class_name, parent, include_all_descendants)

    def get_children_components_by_type_from_scene_component(self, class_name, parent, include_all_descendants=True):
        return self._entry_point_caller.call_on_game_thread("std::vector<uint64_t>", "get_children_components_by_type_from_scene_component", None, class_name, parent, include_all_descendants)

    def get_children_components_by_class_from_scene_component(self, parent, uclass, include_all_descendants=True):
        return self._entry_point_caller.call_on_game_thread("std::vector<uint64_t>", "get_children_components_by_class_from_scene_component", None, parent, uclass, include_all_descendants)

    #
    # Get children components conditionally from a scene component and return a dict
    #

    def get_children_components_by_name_as_dict_from_scene_component(self, class_name, parent, children_component_names, include_all_descendants=True, return_null_if_not_found=True):
        return self._entry_point_caller.call_on_game_thread("std::map<std::string, uint64_t>", "get_children_components_by_name_as_map_from_scene_component", None, class_name, parent, children_component_names, include_all_descendants, return_null_if_not_found)

    def get_children_components_by_tag_as_dict_from_scene_component(self, class_name, parent, tag, include_all_descendants=True):
        return self._entry_point_caller.call_on_game_thread("std::map<std::string, uint64_t>", "get_children_components_by_tag_as_map_from_scene_component", None, class_name, parent, tag, include_all_descendants)

    def get_children_components_by_tag_any_as_dict_from_scene_component(self, class_name, parent, tags, include_all_descendants=True):
        return self._entry_point_caller.call_on_game_thread("std::map<std::string, uint64_t>", "get_children_components_by_tag_any_as_map_from_scene_component", None, class_name, parent, tags, include_all_descendants)

    def get_children_components_by_tag_all_as_dict_from_scene_component(self, class_name, parent, tags, include_all_descendants=True):
        return self._entry_point_caller.call_on_game_thread("std::map<std::string, uint64_t>", "get_children_components_by_tag_all_as_map_from_scene_component", None, class_name, parent, tags, include_all_descendants)

    def get_children_components_by_type_as_dict_from_scene_component(self, class_name, parent, include_all_descendants=True):
        return self._entry_point_caller.call_on_game_thread("std::map<std::string, uint64_t>", "get_children_components_by_type_as_map_from_scene_component", None, class_name, parent, include_all_descendants)

    def get_children_components_by_class_as_dict_from_scene_component(self, parent, uclass, include_all_descendants=True):
        return self._entry_point_caller.call_on_game_thread("std::map<std::string, uint64_t>", "get_children_components_by_class_as_map_from_scene_component", None, parent, uclass, include_all_descendants)

    #
    # Get child component conditionally from a scene component
    #

    def get_child_component_by_name_from_scene_component(self, class_name, parent, child_component_name, include_all_descendants=True):
        return self._entry_point_caller.call_on_game_thread("uint64_t", "get_child_component_by_name_from_scene_component", None, class_name, parent, child_component_name, include_all_descendants)

    def get_child_component_by_tag_from_scene_component(self, class_name, parent, tag, include_all_descendants=True):
        return self._entry_point_caller.call_on_game_thread("uint64_t", "get_child_component_by_tag_from_scene_component", None, class_name, parent, tag, include_all_descendants)

    def get_child_component_by_tag_any_from_scene_component(self, class_name, parent, tags, include_all_descendants=True):
        return self._entry_point_caller.call_on_game_thread("uint64_t", "get_child_component_by_tag_any_from_scene_component", None, class_name, parent, tags, include_all_descendants)

    def get_child_component_by_tag_all_from_scene_component(self, class_name, parent, tags, include_all_descendants=True):
        return self._entry_point_caller.call_on_game_thread("uint64_t", "get_child_component_by_tag_all_from_scene_component", None, class_name, parent, tags, include_all_descendants)

    def get_child_component_by_type_from_scene_component(self, class_name, parent, include_all_descendants=True):
        return self._entry_point_caller.call_on_game_thread("uint64_t", "get_child_component_by_type_from_scene_component", None, class_name, parent, include_all_descendants)

    def get_child_component_by_class_from_scene_component(self, parent, uclass, include_all_descendants=True):
        return self._entry_point_caller.call_on_game_thread("uint64_t", "get_child_component_by_class_from_scene_component", None, parent, uclass, include_all_descendants)

    #
    # Spawn actor
    #

    def spawn_actor(self, class_name, location={"X": 0.0, "Y": 0.0, "Z": 0.0}, rotation={"Pitch": 0.0, "Yaw": 0.0, "Roll": 0.0}, spawn_parameters={}):

        if "TransformScaleMethod" not in spawn_parameters:
            spawn_parameters["TransformScaleMethod"] = "MultiplyWithRoot" # see Engine/Source/Runtime/Engine/Classes/Engine/World.h

        if "ObjectFlags" in spawn_parameters:
            object_flags = spawn_parameters.pop("ObjectFlags")
        else:
            object_flags = ["RF_Transactional"] # see Engine/Source/Runtime/Engine/Private/World.cpp

        return self._entry_point_caller.call_on_game_thread(
            "uint64_t",
            "spawn_actor",
            None,
            class_name,
            spear.utils.func_utils.to_json_string(obj=location),
            spear.utils.func_utils.to_json_string(obj=rotation),
            spear.utils.func_utils.to_json_string(obj=spawn_parameters),
            object_flags)

    def spawn_actor_from_class(self, uclass, location={"X": 0.0, "Y": 0.0, "Z": 0.0}, rotation={"Pitch": 0.0, "Yaw": 0.0, "Roll": 0.0}, spawn_parameters={}):

        if "TransformScaleMethod" not in spawn_parameters:
            spawn_parameters["TransformScaleMethod"] = "MultiplyWithRoot" # see Engine/Source/Runtime/Engine/Classes/Engine/World.h

        if "ObjectFlags" in spawn_parameters:
            object_flags = spawn_parameters.pop("ObjectFlags")
        else:
            object_flags = ["RF_Transactional"] # see Engine/Source/Runtime/Engine/Private/World.cpp

        return self._entry_point_caller.call_on_game_thread(
            "uint64_t",
            "spawn_actor_from_class",
            None,
            uclass,
            spear.utils.func_utils.to_json_string(obj=location),
            spear.utils.func_utils.to_json_string(obj=rotation),
            spear.utils.func_utils.to_json_string(obj=spawn_parameters),
            object_flags)

    #
    # Destroy actor
    #

    def destroy_actor(self, actor, net_force=False, should_modify_level=True):
        return self._entry_point_caller.call_on_game_thread("bool", "destroy_actor", None, actor, net_force, should_modify_level)

    #
    # Create component
    #

    def create_component(self, class_name, owner, component_name):
        return self._entry_point_caller.call_on_game_thread("uint64_t", "create_component_outside_owner_constructor", None, class_name, owner, component_name)

    def create_scene_component_from_actor(self, class_name, owner, scene_component_name):
        return self._entry_point_caller.call_on_game_thread("uint64_t", "create_scene_component_outside_owner_constructor_from_actor", None, class_name, owner, scene_component_name)

    def create_scene_component_from_object(self, class_name, owner, parent, scene_component_name):
        return self._entry_point_caller.call_on_game_thread("uint64_t", "create_scene_component_outside_owner_constructor_from_object", None, class_name, owner, parent, scene_component_name)

    def create_scene_component_from_component(self, class_name, owner, scene_component_name):
        return self._entry_point_caller.call_on_game_thread("uint64_t", "create_scene_component_outside_owner_constructor_from_component", None, class_name, owner, scene_component_name)

    def create_component_by_class(self, component_class, owner, component_name):
        return self._entry_point_caller.call_on_game_thread("uint64_t", "create_component_outside_owner_constructor_by_class", None, component_class, owner, component_name)

    def create_scene_component_by_class_from_actor(self, scene_component_class, owner, scene_component_name):
        return self._entry_point_caller.call_on_game_thread("uint64_t", "create_scene_component_outside_owner_constructor_by_class_from_actor", None, scene_component_class, owner, scene_component_name)

    def create_scene_component_by_class_from_object(self, scene_component_class, owner, parent, scene_component_name):
        return self._entry_point_caller.call_on_game_thread("uint64_t", "create_scene_component_outside_owner_constructor_by_class_from_object", None, scene_component_class, owner, parent, scene_component_name)

    def create_scene_component_by_class_from_component(self, scene_component_class, owner, scene_component_name):
        return self._entry_point_caller.call_on_game_thread("uint64_t", "create_scene_component_outside_owner_constructor_by_class_from_component", None, scene_component_class, owner, scene_component_name)

    #
    # Destroy component
    #

    def destroy_component(self, component, promote_children=False):
        return self._entry_point_caller.call_on_game_thread("void", "destroy_component", None, component, promote_children)

    #
    # Create new object
    #

    def new_object(self, class_name, outer=0, name="", object_flags=["RF_NoFlags"], template=0, copy_transients_from_class_defaults=False, in_instance_graph=0, external_package=0):
        return self._entry_point_caller.call_on_game_thread("uint64_t", "new_object", None, class_name, outer, name, object_flags, template, copy_transients_from_class_defaults, in_instance_graph, external_package)

    #
    # Load object and class
    #

    def load_object(self, class_name, outer, name="", filename="", load_flags=["LOAD_None"], sandbox=0, instancing_context=0):
        return self._entry_point_caller.call_on_game_thread("uint64_t", "load_object", None, class_name, outer, name, filename, load_flags, sandbox, instancing_context)

    def load_class(self, class_name, outer, name="", filename="", load_flags=["LOAD_None"], sandbox=0):
        return self._entry_point_caller.call_on_game_thread("uint64_t", "load_class", None, class_name, outer, name, filename, load_flags, sandbox)

    def static_load_object(self, uclass, in_outer, name="", filename="", load_flags=["LOAD_None"], sandbox=0, allow_object_reconciliation=True, instancing_context=0):
        return self._entry_point_caller.call_on_game_thread("uint64_t", "static_load_object", None, uclass, in_outer, name, filename, load_flags, sandbox, allow_object_reconciliation, instancing_context)

    def static_load_class(self, uclass, in_outer, name="", filename="", load_flags=["LOAD_None"], sandbox=0):
        return self._entry_point_caller.call_on_game_thread("uint64_t", "static_load_class", None, uclass, in_outer, name, filename, load_flags, sandbox)

    #
    # Enable and disable garbage collection for uobject
    #

    def add_object_to_root(self, uobject):
        return self._entry_point_caller.call_on_game_thread("void", "add_object_to_root", None, uobject)

    def remove_object_from_root(self, uobject):
        return self._entry_point_caller.call_on_game_thread("void", "remove_object_from_root", None, uobject)

    #
    # Find, get, and set console variable
    #

    def find_console_variable_by_name(self, console_variable_name):
        return self._entry_point_caller.call_on_game_thread("uint64_t", "find_console_variable_by_name", None, console_variable_name)

    def get_console_variable_value_as_bool(self, cvar):
        return self._entry_point_caller.call_on_game_thread("bool", "get_console_variable_value_as_bool", None, cvar)

    def get_console_variable_value_as_int(self, cvar):
        return self._entry_point_caller.call_on_game_thread("int64_t", "get_console_variable_value_as_int", None, cvar)

    def get_console_variable_value_as_float(self, cvar):
        return self._entry_point_caller.call_on_game_thread("float", "get_console_variable_value_as_float", None, cvar)

    def get_console_variable_value_as_string(self, cvar):
        return self._entry_point_caller.call_on_game_thread("std::string", "get_console_variable_value_as_string", None, cvar)

    def set_console_variable_value(self, cvar, value, set_by_flags=["ECVF_SetByCode"]):
        if isinstance(value, bool):
            return self._entry_point_caller.call_on_game_thread("void", "set_console_variable_value_from_bool", None, cvar, value, set_by_flags)
        elif isinstance(value, int):
            return self._entry_point_caller.call_on_game_thread("void", "set_console_variable_value_from_int", None, cvar, value, set_by_flags)
        elif isinstance(value, float):
            return self._entry_point_caller.call_on_game_thread("void", "set_console_variable_value_from_float", None, cvar, value, set_by_flags)
        elif isinstance(value, str):
            return self._entry_point_caller.call_on_game_thread("void", "set_console_variable_value_from_string", None, cvar, value, set_by_flags)
        else:
            assert False

    #
    # Execute console command
    #

    def execute_console_command(self, command):
        return self._entry_point_caller.call_on_game_thread("void", "execute_console_command", None, command)

    #
    # Stable name helper functions
    #

    def has_stable_name(self, actor):
        return self._entry_point_caller.call_on_game_thread("bool", "has_stable_name", None, actor)

    def get_stable_name_for_actor(self, actor):
        return self._entry_point_caller.call_on_game_thread("std::string", "get_stable_name_for_actor", None, actor)

    def get_stable_name_for_component(self, component, include_actor_name=False):
        return self._entry_point_caller.call_on_game_thread("std::string", "get_stable_name_for_component", None, component, include_actor_name)

    #
    # Get actor and component tags
    #

    def get_actor_tags(self, actor):
        return self._entry_point_caller.call_on_game_thread("std::vector<std::string>", "get_actor_tags", None, actor)

    def get_component_tags(self, actor):
        return self._entry_point_caller.call_on_game_thread("std::vector<std::string>", "get_component_tags", None, actor)
