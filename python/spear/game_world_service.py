#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

class GameWorldService():

    def __init__(self, rpc_client):
        self._rpc_client = rpc_client

    def get_world_name(self):
        return self._rpc_client.call("game_world_service.get_world_name")

    def open_level(self, level_name):
        self._rpc_client.call("game_world_service.open_level", level_name)

    def set_game_paused(self, paused):
        self._rpc_client.call("game_world_service.set_game_paused", paused)

    def find_actors(self):
        return self._rpc_client.call("game_world_service.find_actors")

    def find_actors_as_map(self):
        return self._rpc_client.call("game_world_service.find_actors_as_map")    

    def get_components(self, actor):
        return self._rpc_client.call("game_world_service.get_components", actor)

    def get_components_as_map(self, actor):
        return self._rpc_client.call("game_world_service.get_components_as_map", actor)

    def get_children_components(self, actor, include_all_descendants):
        return self._rpc_client.call("game_world_service.get_children_components", actor, include_all_descendants)

    def get_children_components_as_map(self, actor, include_all_descendants):
        return self._rpc_client.call("game_world_service.get_children_components_as_map", actor, include_all_descendants)

    def get_object_properties_as_string_from_uobject(self, uobject):
        return self._rpc_client.call("game_world_service.get_object_properties_as_string_from_uobject", uobject)

    def get_object_properties_as_string_from_ustruct(self, value_ptr, ustruct):
        return self._rpc_client.call("game_world_service.get_object_properties_as_string_from_ustruct", value_ptr, ustruct)

    def set_object_properties_from_string_for_uobject(self, uobject, string):
        self._rpc_client.call("game_world_service.set_object_properties_from_string_for_uobject", uobject, string)

    def set_object_properties_from_string_for_ustruct(self, value_ptr, ustruct, string):
        self._rpc_client.call("game_world_service.set_object_properties_from_string_for_ustruct", value_ptr, ustruct, string)

    def find_property_by_name_on_object(self, uobject, name):
        return self._rpc_client.call("game_world_service.find_property_by_name_on_uobject", uobject, name)

    def find_property_by_name_on_struct(self, value_ptr, ustruct, name):
        return self._rpc_client.call("game_world_service.find_property_by_name_on_ustruct", value_ptr, ustruct, name)

    def get_property_value_as_string(self, property_desc):
        return self._rpc_client.call("game_world_service.get_property_value_as_string", property_desc)

    def set_property_value_from_string(self, property_desc, string):
        return self._rpc_client.call("game_world_service.set_property_value_from_string", property_desc, string)

    def find_function_by_name(self, uclass, name, include_super_flag):
        return self._rpc_client.call("game_world_service.find_function_by_name", uclass, name, include_super_flag)

    def call_function(self, uobject, ufunction, **kwargs):
        return self._rpc_client.call("game_world_service.call_function", uobject, ufunction, kwargs)

    def find_special_struct_by_name(self, name):
        return self._rpc_client.call("game_world_service.find_special_struct_by_name", name)

    def has_stable_name(self, actor):
        return self._rpc_client.call("game_world_service.has_stable_name", actor)

    def get_stable_name_for_actor(self, actor):
        return self._rpc_client.call("game_world_service.get_stable_name_for_actor", actor)

    def get_stable_name_for_actor_component(self, component, include_actor_name):
        return self._rpc_client.call("game_world_service.get_stable_name_for_actor_component", component, include_actor_name)

    def get_stable_name_for_scene_component(self, component, include_actor_name):
        return self._rpc_client.call("game_world_service.get_stable_name_for_scene_component", component, include_actor_name)

    def get_actor_tags(self, actor):
        return self._rpc_client.call("game_world_service.get_actor_tags", actor)

    def get_component_tags(self, actor):
        return self._rpc_client.call("game_world_service.get_component_tags", actor)

    def get_class_from_instance(self, instance):
        return self._rpc_client.call("game_world_service.get_class_from_instance", instance)
