#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import spear

class UnrealService():
    def __init__(self, entry_point_caller):
        self._entry_point_caller = entry_point_caller

    #
    # Get subsystems
    #

    def get_subsystem_by_type(self, class_name):
        return self._entry_point_caller.call("unreal_service.get_subsystem_by_type", class_name)

    def get_subsystem_by_class(self, class_name, uclass):
        return self._entry_point_caller.call("unreal_service.get_subsystem_by_class", class_name, uclass)

    #
    # Get uclass from class name, get default object from uclass, get uclass from object
    #

    def get_static_class(self, class_name):
        return self._entry_point_caller.call("unreal_service.get_static_class", class_name)

    def get_default_object(self, uclass, create_if_needed=True):
        return self._entry_point_caller.call("unreal_service.get_default_object", uclass, create_if_needed)

    def get_class(self, uobject):
        return self._entry_point_caller.call("unreal_service.get_class", uobject)

    #
    # Get static struct
    #

    def get_static_struct(self, struct_name):
        return self._entry_point_caller.call("unreal_service.get_static_struct", struct_name)

    #
    # Get and set object properties
    #

    def get_object_properties_from_uobject(self, uobject):
        return spear.try_to_dict(json_string=self._entry_point_caller.call("unreal_service.get_object_properties_as_string_from_uobject", uobject), default_value={})

    def get_object_properties_from_ustruct(self, value_ptr, ustruct):
        return spear.try_to_dict(json_string=self._entry_point_caller.call("unreal_service.get_object_properties_as_string_from_ustruct", value_ptr, ustruct), default_value={})

    def set_object_properties_for_uobject(self, uobject, properties):
        self._entry_point_caller.call("unreal_service.set_object_properties_from_string_for_uobject", uobject, spear.to_json_string(obj=properties))

    def set_object_properties_for_ustruct(self, value_ptr, ustruct, properties):
        self._entry_point_caller.call("unreal_service.set_object_properties_from_string_for_ustruct", value_ptr, ustruct, spear.to_json_string(obj=properties))

    #
    # Find properties
    #

    def find_property_by_name_on_uobject(self, uobject, property_name):
        return self._entry_point_caller.call("unreal_service.find_property_by_name_on_uobject", uobject, property_name)

    def find_property_by_name_on_ustruct(self, value_ptr, ustruct, property_name):
        return self._entry_point_caller.call("unreal_service.find_property_by_name_on_ustruct", value_ptr, ustruct, property_name)

    #
    # Get property values
    #

    def get_property_value(self, property_desc):
        return spear.try_to_dict(json_string=self._entry_point_caller.call("unreal_service.get_property_value_as_string", property_desc))

    def set_property_value(self, property_desc, property_value):
        self._entry_point_caller.call("unreal_service.set_property_value_from_string", property_desc, spear.to_json_string(obj=property_value))

    #
    # Find and call functions
    #

    def find_function_by_name(self, uclass, function_name, include_super_flag="IncludeSuper"):
        return self._entry_point_caller.call("unreal_service.find_function_by_name", uclass, function_name, include_super_flag)

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
        return spear.try_to_dicts(json_strings=self._entry_point_caller.call("unreal_service.call_function", uobject, ufunction, spear.to_json_strings(objs=args), world_context))

    #
    # Find actors unconditionally and return a list or dict
    #

    def find_actors(self):
        return self._entry_point_caller.call("unreal_service.find_actors")

    def find_actors_as_dict(self):
        return self._entry_point_caller.call("unreal_service.find_actors_as_map")

    #
    # Get components unconditionally and return a list or dict
    #

    def get_components(self, actor):
        return self._entry_point_caller.call("unreal_service.get_components", actor)

    def get_components_as_dict(self, actor):
        return self._entry_point_caller.call("unreal_service.get_components_as_map", actor)

    #
    # Get children components unconditionally and return a list or dict
    #

    def get_children_components(self, actor, include_all_descendants=True):
        return self._entry_point_caller.call("unreal_service.get_children_components", actor, include_all_descendants)

    def get_children_components_as_dict(self, actor, include_all_descendants=True):
        return self._entry_point_caller.call("unreal_service.get_children_components_as_map", actor, include_all_descendants)

    #
    # Find actors conditionally and return a list
    #

    def find_actors_by_name(self, class_name, actor_names, return_null_if_not_found=True):
        return self._entry_point_caller.call("unreal_service.find_actors_by_name", class_name, actor_names, return_null_if_not_found)

    def find_actors_by_tag(self, class_name, tag):
        return self._entry_point_caller.call("unreal_service.find_actors_by_tag", class_name, tag)

    def find_actors_by_tag_any(self, class_name, tags):
        return self._entry_point_caller.call("unreal_service.find_actors_by_tag_any", class_name, tags)

    def find_actors_by_tag_all(self, class_name, tags):
        return self._entry_point_caller.call("unreal_service.find_actors_by_tag_all", class_name, tags)

    def find_actors_by_type(self, class_name):
        return self._entry_point_caller.call("unreal_service.find_actors_by_type", class_name)

    def find_actors_by_class(self, uclass):
        return self._entry_point_caller.call("unreal_service.find_actors_by_class", uclass)

    #
    # Find actors conditionally and return a dict
    #

    def find_actors_by_name_as_dict(self, class_name, actor_names, return_null_if_not_found=True):
        return self._entry_point_caller.call("unreal_service.find_actors_by_name_as_map", class_name, actor_names, return_null_if_not_found)

    def find_actors_by_tag_as_dict(self, class_name, tag):
        return self._entry_point_caller.call("unreal_service.find_actors_by_tag_as_map", class_name, tag)

    def find_actors_by_tag_any_as_dict(self, class_name, tags):
        return self._entry_point_caller.call("unreal_service.find_actors_by_tag_any_as_map", class_name, tags)

    def find_actors_by_tag_all_as_dict(self, class_name, tags):
        return self._entry_point_caller.call("unreal_service.find_actors_by_tag_all_as_map", class_name, tags)

    def find_actors_by_type_as_dict(self, class_name):
        return self._entry_point_caller.call("unreal_service.find_actors_by_type_as_map", class_name)

    def find_actors_by_class_as_dict(self, uclass):
        return self._entry_point_caller.call("unreal_service.find_actors_by_class_as_map", uclass)

    #
    # Find actor conditionally
    #

    def find_actor_by_name(self, class_name, actor_name):
        return self._entry_point_caller.call("unreal_service.find_actor_by_name", class_name, actor_name)

    def find_actor_by_tag(self, class_name, tag):
        return self._entry_point_caller.call("unreal_service.find_actor_by_tag", class_name, tag)

    def find_actor_by_tag_any(self, class_name, tags):
        return self._entry_point_caller.call("unreal_service.find_actor_by_tag_any", class_name, tags)

    def find_actor_by_tag_all(self, class_name, tags):
        return self._entry_point_caller.call("unreal_service.find_actor_by_tag_all", class_name, tags)

    def find_actor_by_type(self, class_name):
        return self._entry_point_caller.call("unreal_service.find_actor_by_type", class_name)

    def find_actor_by_class(self, uclass):
        return self._entry_point_caller.call("unreal_service.find_actor_by_class", uclass)

    #
    # Get components conditionally and return a list
    #

    def get_components_by_name(self, class_name, actor, component_names, include_from_child_actors=False, return_null_if_not_found=True):
        return self._entry_point_caller.call("unreal_service.get_components_by_name", class_name, actor, component_names, include_from_child_actors, return_null_if_not_found)

    def get_components_by_tag(self, class_name, actor, tag, include_from_child_actors=False):
        return self._entry_point_caller.call("unreal_service.get_components_by_tag", class_name, actor, tag, include_from_child_actors)

    def get_components_by_tag_any(self, class_name, actor, tags, include_from_child_actors=False):
        return self._entry_point_caller.call("unreal_service.get_components_by_tag_any", class_name, actor, tags, include_from_child_actors)

    def get_components_by_tag_all(self, class_name, actor, tags, include_from_child_actors=False):
        return self._entry_point_caller.call("unreal_service.get_components_by_tag_all", class_name, actor, tags, include_from_child_actors)

    def get_components_by_type(self, class_name, actor, include_from_child_actors=False):
        return self._entry_point_caller.call("unreal_service.get_components_by_type", class_name, actor, include_from_child_actors)

    def get_components_by_class(self, actor, uclass, include_from_child_actors=False):
        return self._entry_point_caller.call("unreal_service.get_components_by_class", actor, uclass, include_from_child_actors)

    #
    # Get components conditionally and return a dict
    #

    def get_components_by_name_as_dict(self, class_name, actor, component_names, include_from_child_actors=False, return_null_if_not_found=True):
        return self._entry_point_caller.call("unreal_service.get_components_by_name_as_map", class_name, actor, component_names, include_from_child_actors, return_null_if_not_found)

    def get_components_by_tag_as_dict(self, class_name, actor, tag, include_from_child_actors=False):
        return self._entry_point_caller.call("unreal_service.get_components_by_tag_as_map", class_name, actor, tag, include_from_child_actors)

    def get_components_by_tag_any_as_dict(self, class_name, actor, tags, include_from_child_actors=False):
        return self._entry_point_caller.call("unreal_service.get_components_by_tag_any_as_map", class_name, actor, tags, include_from_child_actors)

    def get_components_by_tag_all_as_dict(self, class_name, actor, tags, include_from_child_actors=False):
        return self._entry_point_caller.call("unreal_service.get_components_by_tag_all_as_map", class_name, actor, tags, include_from_child_actors)

    def get_components_by_type_as_dict(self, class_name, actor, include_from_child_actors=False):
        return self._entry_point_caller.call("unreal_service.get_components_by_type_as_map", class_name, actor, include_from_child_actors)

    def get_components_by_class_as_dict(self, actor, uclass, include_from_child_actors=False):
        return self._entry_point_caller.call("unreal_service.get_components_by_class_as_map", actor, uclass, include_from_child_actors)

    #
    # Get component conditionally
    #

    def get_component_by_name(self, class_name, actor, component_name, include_from_child_actors=False):
        return self._entry_point_caller.call("unreal_service.get_component_by_name", class_name, actor, component_name, include_from_child_actors)

    def get_component_by_tag(self, class_name, actor, tag, include_from_child_actors=False):
        return self._entry_point_caller.call("unreal_service.get_component_by_tag", class_name, actor, tag, include_from_child_actors)

    def get_component_by_tag_any(self, class_name, actor, tags, include_from_child_actors=False):
        return self._entry_point_caller.call("unreal_service.get_component_by_tag_any", class_name, actor, tags, include_from_child_actors)

    def get_component_by_tag_all(self, class_name, actor, tags, include_from_child_actors=False):
        return self._entry_point_caller.call("unreal_service.get_component_by_tag_all", class_name, actor, tags, include_from_child_actors)

    def get_component_by_type(self, class_name, actor, include_from_child_actors=False):
        return self._entry_point_caller.call("unreal_service.get_component_by_type", class_name, actor, include_from_child_actors)

    def get_component_by_class(self, actor, uclass, include_from_child_actors=False):
        return self._entry_point_caller.call("unreal_service.get_component_by_class", actor, uclass, include_from_child_actors)

    #
    # Get children components conditionally from an actor and return a list
    #

    def get_children_components_by_name_from_actor(self, class_name,  parent, children_component_names, include_all_descendants=True, return_null_if_not_found=True):
        return self._entry_point_caller.call("unreal_service.get_children_components_by_name_from_actor", class_name,  parent, children_component_names, include_all_descendants, return_null_if_not_found)

    def get_children_components_by_tag_from_actor(self, class_name, parent, tag, include_all_descendants=True):
        return self._entry_point_caller.call("unreal_service.get_children_components_by_tag_from_actor", class_name, parent, tag, include_all_descendants)

    def get_children_components_by_tag_any_from_actor(self, class_name, parent, tags, include_all_descendants=True):
        return self._entry_point_caller.call("unreal_service.get_children_components_by_tag_any_from_actor", class_name, parent, tags, include_all_descendants)

    def get_children_components_by_tag_all_from_actor(self, class_name, parent, tags, include_all_descendants=True):
        return self._entry_point_caller.call("unreal_service.get_children_components_by_tag_all_from_actor", class_name, parent, tags, include_all_descendants)

    def get_children_components_by_type_from_actor(self, class_name, parent, include_all_descendants=True):
        return self._entry_point_caller.call("unreal_service.get_children_components_by_type_from_actor", class_name, parent, include_all_descendants)

    def get_children_components_by_class_from_actor(self, parent, uclass, include_all_descendants=True):
        return self._entry_point_caller.call("unreal_service.get_children_components_by_class_from_actor", parent, uclass, include_all_descendants)

    #
    # Get children components conditionally from an actor and return a dict
    #

    def get_children_components_by_name_as_dict_from_actor(self, class_name, parent, children_component_names, include_all_descendants=True, return_null_if_not_found=True):
        return self._entry_point_caller.call("unreal_service.get_children_components_by_name_as_map_from_actor", class_name, parent, children_component_names, include_all_descendants, return_null_if_not_found)

    def get_children_components_by_tag_as_dict_from_actor(self, class_name, parent, tag, include_all_descendants=True):
        return self._entry_point_caller.call("unreal_service.get_children_components_by_tag_as_map_from_actor", class_name, parent, tag, include_all_descendants)

    def get_children_components_by_tag_any_as_dict_from_actor(self, class_name, parent, tags, include_all_descendants=True):
        return self._entry_point_caller.call("unreal_service.get_children_components_by_tag_any_as_map_from_actor", class_name, parent, tags, include_all_descendants)

    def get_children_components_by_tag_all_as_dict_from_actor(self, class_name, parent, tags, include_all_descendants=True):
        return self._entry_point_caller.call("unreal_service.get_children_components_by_tag_all_as_map_from_actor", class_name, parent, tags, include_all_descendants)

    def get_children_components_by_type_as_dict_from_actor(self, class_name, parent, include_all_descendants=True):
        return self._entry_point_caller.call("unreal_service.get_children_components_by_type_as_map_from_actor", class_name, parent, include_all_descendants)

    def get_children_components_by_class_as_dict_from_actor(self, parent, uclass, include_all_descendants=True):
        return self._entry_point_caller.call("unreal_service.get_children_components_by_class_as_map_from_actor", parent, uclass, include_all_descendants)

    #
    # Get child component conditionally from an actor
    #

    def get_child_component_by_name_from_actor(self, class_name, parent, child_component_name, include_all_descendants=True):
        return self._entry_point_caller.call("unreal_service.get_child_component_by_name_from_actor", class_name, parent, child_component_name, include_all_descendants)

    def get_child_component_by_tag_from_actor(self, class_name, parent, tag, include_all_descendants=True):
        return self._entry_point_caller.call("unreal_service.get_child_component_by_tag_from_actor", class_name, parent, tag, include_all_descendants)

    def get_child_component_by_tag_any_from_actor(self, class_name, parent, tags, include_all_descendants=True):
        return self._entry_point_caller.call("unreal_service.get_child_component_by_tag_any_from_actor", class_name, parent, tags, include_all_descendants)

    def get_child_component_by_tag_all_from_actor(self, class_name, parent, tags, include_all_descendants=True):
        return self._entry_point_caller.call("unreal_service.get_child_component_by_tag_all_from_actor", class_name, parent, tags, include_all_descendants)

    def get_child_component_by_type_from_actor(self, class_name, parent, include_all_descendants=True):
        return self._entry_point_caller.call("unreal_service.get_child_component_by_type_from_actor", class_name, parent, include_all_descendants)

    def get_child_component_by_class_from_actor(self, parent, uclass, include_all_descendants=True):
        return self._entry_point_caller.call("unreal_service.get_child_component_by_class_from_actor", parent, uclass, include_all_descendants)

    #
    # Get children components conditionally from a scene component and return a list
    #

    def get_children_components_by_name_from_scene_component(self, class_name, parent, children_component_names, include_all_descendants=True, return_null_if_not_found=True):
        return self._entry_point_caller.call("unreal_service.get_children_components_by_name_from_scene_component", class_name, parent, children_component_names, include_all_descendants, return_null_if_not_found)

    def get_children_components_by_tag_from_scene_component(self, class_name, parent, tag, include_all_descendants=True):
        return self._entry_point_caller.call("unreal_service.get_children_components_by_tag_from_scene_component", class_name, parent, tag, include_all_descendants)

    def get_children_components_by_tag_any_from_scene_component(self, class_name, parent, tags, include_all_descendants=True):
        return self._entry_point_caller.call("unreal_service.get_children_components_by_tag_any_from_scene_component", class_name, parent, tags, include_all_descendants)

    def get_children_components_by_tag_all_from_scene_component(self, class_name, parent, tags, include_all_descendants=True):
        return self._entry_point_caller.call("unreal_service.get_children_components_by_tag_all_from_scene_component", class_name, parent, tags, include_all_descendants)

    def get_children_components_by_type_from_scene_component(self, class_name, parent, include_all_descendants=True):
        return self._entry_point_caller.call("unreal_service.get_children_components_by_type_from_scene_component", class_name, parent, include_all_descendants)

    def get_children_components_by_type_from_scene_component(self, class_name, parent, include_all_descendants=True):
        return self._entry_point_caller.call("unreal_service.get_children_components_by_type_from_scene_component", class_name, parent, include_all_descendants)

    def get_children_components_by_class_from_scene_component(self, parent, uclass, include_all_descendants=True):
        return self._entry_point_caller.call("unreal_service.get_children_components_by_class_from_scene_component", parent, uclass, include_all_descendants)

    #
    # Get children components conditionally from a scene component and return a dict
    #

    def get_children_components_by_name_as_map_from_scene_component(self, class_name, parent, children_component_names, include_all_descendants=True, return_null_if_not_found=True):
        return self._entry_point_caller.call("unreal_service.get_children_components_by_name_as_map_from_scene_component", class_name, parent, children_component_names, include_all_descendants, return_null_if_not_found)

    def get_children_components_by_tag_as_map_from_scene_component(self, class_name, parent, tag, include_all_descendants=True):
        return self._entry_point_caller.call("unreal_service.get_children_components_by_tag_as_map_from_scene_component", class_name, parent, tag, include_all_descendants)

    def get_children_components_by_tag_any_as_map_from_scene_component(self, class_name, parent, tags, include_all_descendants=True):
        return self._entry_point_caller.call("unreal_service.get_children_components_by_tag_any_as_map_from_scene_component", class_name, parent, tags, include_all_descendants)

    def get_children_components_by_tag_all_as_map_from_scene_component(self, class_name, parent, tags, include_all_descendants=True):
        return self._entry_point_caller.call("unreal_service.get_children_components_by_tag_all_as_map_from_scene_component", class_name, parent, tags, include_all_descendants)

    def get_children_components_by_type_as_map_from_scene_component(self, class_name, parent, include_all_descendants=True):
        return self._entry_point_caller.call("unreal_service.get_children_components_by_type_as_map_from_scene_component", class_name, parent, include_all_descendants)

    def get_children_components_by_class_as_map_from_scene_component(self, parent, uclass, include_all_descendants=True):
        return self._entry_point_caller.call("unreal_service.get_children_components_by_class_as_map_from_scene_component", parent, uclass, include_all_descendants)

    #
    # Get child component conditionally from a scene component
    #

    def get_child_component_by_name_from_scene_component(self, class_name, parent, child_component_name, include_all_descendants=True):
        return self._entry_point_caller.call("unreal_service.get_child_component_by_name_from_scene_component", class_name, parent, child_component_name, include_all_descendants)

    def get_child_component_by_tag_from_scene_component(self, class_name, parent, tag, include_all_descendants=True):
        return self._entry_point_caller.call("unreal_service.get_child_component_by_tag_from_scene_component", class_name, parent, tag, include_all_descendants)

    def get_child_component_by_tag_any_from_scene_component(self, class_name, parent, tags, include_all_descendants=True):
        return self._entry_point_caller.call("unreal_service.get_child_component_by_tag_any_from_scene_component", class_name, parent, tags, include_all_descendants)

    def get_child_component_by_tag_all_from_scene_component(self, class_name, parent, tags, include_all_descendants=True):
        return self._entry_point_caller.call("unreal_service.get_child_component_by_tag_all_from_scene_component", class_name, parent, tags, include_all_descendants)

    def get_child_component_by_type_from_scene_component(self, class_name, parent, include_all_descendants=True):
        return self._entry_point_caller.call("unreal_service.get_child_component_by_type_from_scene_component", class_name, parent, include_all_descendants)

    def get_child_component_by_class_from_scene_component(self, parent, uclass, include_all_descendants=True):
        return self._entry_point_caller.call("unreal_service.get_child_component_by_class_from_scene_component", parent, uclass, include_all_descendants)

    #
    # Spawn actor
    #

    def spawn_actor(self, class_name, location={}, rotation={}, spawn_parameters={}):

        if "TransformScaleMethod" not in spawn_parameters:
            spawn_parameters["TransformScaleMethod"] = "MultiplyWithRoot" # see Engine/Source/Runtime/Engine/Classes/Engine/World.h

        if "ObjectFlags" in spawn_parameters:
            object_flags = spawn_parameters.pop("ObjectFlags")
        else:
            object_flags = ["RF_Transactional"] # see Engine/Source/Runtime/Engine/Private/World.cpp

        return self._entry_point_caller.call(
            "unreal_service.spawn_actor", class_name, spear.to_json_string(obj=location), spear.to_json_string(obj=rotation), spear.to_json_string(obj=spawn_parameters), object_flags)

    def spawn_actor_from_uclass(self, uclass, location={}, rotation={}, spawn_parameters={}):

        if "TransformScaleMethod" not in spawn_parameters:
            spawn_parameters["TransformScaleMethod"] = "MultiplyWithRoot" # see Engine/Source/Runtime/Engine/Classes/Engine/World.h

        if "ObjectFlags" in spawn_parameters:
            object_flags = spawn_parameters.pop("ObjectFlags")
        else:
            object_flags = ["RF_Transactional"] # see Engine/Source/Runtime/Engine/Private/World.cpp

        return self._entry_point_caller.call(
            "unreal_service.spawn_actor_from_uclass", uclass, spear.to_json_string(obj=location), spear.to_json_string(obj=rotation), spear.to_json_string(obj=spawn_parameters), object_flags)

    #
    # Destroy actor
    #

    def destroy_actor(self, actor, net_force=False, should_modify_level=True):
        return self._entry_point_caller.call("unreal_service.destroy_actor", actor, net_force, should_modify_level)

    #
    # Create component
    #

    def create_component_on_actor(self, class_name, owner, component_name):
        return self._entry_point_caller.call("unreal_service.create_component_outside_owner_constructor", class_name, owner, component_name)

    def create_scene_component_on_actor(self, class_name, owner, scene_component_name):
        return self._entry_point_caller.call("unreal_service.create_scene_component_outside_owner_constructor_from_actor", class_name, owner, scene_component_name)

    def create_scene_component_on_component(self, class_name, owner, parent, scene_component_name):
        return self._entry_point_caller.call("unreal_service.create_scene_component_outside_owner_constructor_from_object", class_name, owner, parent, scene_component_name)

    def create_scene_component_on_owning_scene_component(self, class_name, owner, scene_component_name):
        return self._entry_point_caller.call("unreal_service.create_scene_component_outside_owner_constructor_from_component", class_name, owner, scene_component_name)

    #
    # Destroy component
    #

    def destroy_component(self, component, promote_children=False):
        self._entry_point_caller.call("unreal_service.destroy_component", component, promote_children)

    #
    # Create new object
    #

    def new_object(self, class_name, outer=0, name="", object_flags=["RF_NoFlags"], template=0, copy_transients_from_class_defaults=False, in_instance_graph=0, external_package=0):
        return self._entry_point_caller.call("unreal_service.new_object", class_name, outer, name, object_flags, template, copy_transients_from_class_defaults, in_instance_graph, external_package)

    #
    # Load objects and classes
    #

    def load_object(self, class_name, outer, name="", filename="", load_flags=["LOAD_None"], sandbox=0, instancing_context=0):
        return self._entry_point_caller.call("unreal_service.load_object", class_name, outer, name, filename, load_flags, sandbox, instancing_context)

    def load_class(self, class_name, outer, name="", filename="", load_flags=["LOAD_None"], sandbox=0,):
        return self._entry_point_caller.call("unreal_service.load_class", class_name, outer, name, filename, load_flags, sandbox)

    def static_load_object(self, uclass, in_outer, name="", filename="", load_flags=["LOAD_None"], sandbox=0, allow_object_reconciliation=True, instancing_context=0):
        return self._entry_point_caller.call("unreal_service.static_load_object", uclass, in_outer, name, filename, load_flags, sandbox, allow_object_reconciliation, instancing_context)

    def static_load_class(self, base_uclass, in_outer, name="", filename="", load_flags=["LOAD_None"], sandbox=0):
        return self._entry_point_caller.call("unreal_service.static_load_class", base_uclass, in_outer, name, filename, load_flags, sandbox)

    #
    # Enable and disable garbage collection for uobjects
    #

    def add_uobject_to_root(self, uobject):
        return self._entry_point_caller.call("unreal_service.add_uobject_to_root", uobject)

    def remove_uobject_from_root(self, uobject):
        return self._entry_point_caller.call("unreal_service.remove_uobject_from_root", uobject)

    #
    # Find, get, and set console variables
    #

    def find_console_variable_by_name(self, console_variable_name):
        return self._entry_point_caller.call("unreal_service.find_console_variable_by_name", cvar_name)

    def get_console_variable_value_as_bool(self, cvar):
        return self._entry_point_caller.call("unreal_service.get_console_variable_value_as_bool", cvar)

    def get_console_variable_value_as_int(self, cvar):
        return self._entry_point_caller.call("unreal_service.get_console_variable_value_as_int", cvar)

    def get_console_variable_value_as_float(self, cvar):
        return self._entry_point_caller.call("unreal_service.get_console_variable_value_as_float", cvar)

    def get_console_variable_value_as_string(self, cvar):
        return self._entry_point_caller.call("unreal_service.get_console_variable_value_as_string", cvar)

    def set_console_variable_value(self, cvar, val, set_by_flags=["ECVF_SetByCode"]):
        if isinstance(val, bool):
            return self._entry_point_caller.call("unreal_service.set_console_variable_value_from_bool", cvar, val, set_by_flags)
        elif isinstance(val, int):
            return self._entry_point_caller.call("unreal_service.set_console_variable_value_from_int", cvar, val, set_by_flags)
        elif isinstance(val, float):
            return self._entry_point_caller.call("unreal_service.set_console_variable_value_from_float", cvar, val, set_by_flags)
        elif isinstance(val, str):
            return self._entry_point_caller.call("unreal_service.set_console_variable_value_from_string", cvar, val, set_by_flags)
        else:
            assert False

    #
    # Execute console commands
    #

    def execute_console_command(self, command):
        return self._entry_point_caller.call("unreal_service.execute_console_command", command)

    #
    # Stable name helper functions
    #

    def has_stable_name(self, actor):
        return self._entry_point_caller.call("unreal_service.has_stable_name", actor)

    def get_stable_name_for_actor(self, actor):
        return self._entry_point_caller.call("unreal_service.get_stable_name_for_actor", actor)

    def get_stable_name_for_component(self, component, include_actor_name=False):
        return self._entry_point_caller.call("unreal_service.get_stable_name_for_component", component, include_actor_name)

    #
    # Get actor and component tags
    #

    def get_actor_tags(self, actor):
        return self._entry_point_caller.call("unreal_service.get_actor_tags", actor)

    def get_component_tags(self, actor):
        return self._entry_point_caller.call("unreal_service.get_component_tags", actor)
