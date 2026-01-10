#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import spear

class InputService(spear.Service):
    def __init__(self, entry_point_caller, parent_service=None, create_children_services=True):

        # do this after initializing local state
        super().__init__(entry_point_caller=entry_point_caller, parent_service=parent_service, create_children_services=create_children_services)

    def create_child_service(self, entry_point_caller, sp_func_service=None, unreal_service=None, config=None):
        assert self.is_top_level_service() # this function should only be called from the top-level service
        return InputService(entry_point_caller=entry_point_caller, parent_service=self, create_children_services=False)

    def setup_player_input_component(self, actor, input_component):
        actor = spear.to_handle(obj=actor)
        input_component = spear.to_handle(obj=input_component)
        return self.entry_point_caller.call_on_game_thread("setup_player_input_component", None, actor, input_component)

    def inject_key_for_actor(self, actor, chord, key_event):
        actor = spear.to_handle(obj=actor)
        return self.entry_point_caller.call_on_game_thread("inject_key_for_actor", None, actor, spear.to_json_string(obj=chord), key_event)

    def inject_touch_for_actor(self, actor, key_event, finger_index, location):
        actor = spear.to_handle(obj=actor)
        return self.entry_point_caller.call_on_game_thread("inject_touch_for_actor", None, actor, key_event, finger_index, spear.to_json_string(obj=location))

    def inject_axis_for_actor(self, actor, axis_name, axis_value):
        actor = spear.to_handle(obj=actor)
        return self.entry_point_caller.call_on_game_thread("inject_axis_for_actor", None, actor, axis_name, axis_value)

    def inject_axis_key_for_actor(self, actor, axis_key_name, axis_key_value):
        actor = spear.to_handle(obj=actor)
        return self.entry_point_caller.call_on_game_thread("inject_axis_key_for_actor", None, actor, axis_key_name, axis_key_value)

    def inject_vector_axis_for_actor(self, actor, vector_axis_name, vector_axis_value):
        actor = spear.to_handle(obj=actor)
        return self.entry_point_caller.call_on_game_thread("inject_vector_axis_for_actor", None, actor, vector_axis_name, spear.to_json_string(obj=vector_axis_value))

    def inject_gesture_for_actor(self, actor, gesture_name, gesture_value):
        actor = spear.to_handle(obj=actor)
        return self.entry_point_caller.call_on_game_thread("inject_gesture_for_actor", None, actor, gesture_name, gesture_value)

    def inject_action_for_actor(self, actor, action_name, key_event, key_name):
        actor = spear.to_handle(obj=actor)
        return self.entry_point_caller.call_on_game_thread("inject_action_for_actor", None, actor, action_name, key_event, key_name)
