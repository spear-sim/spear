#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import spear.func_utils

class InputService():
    def __init__(self, entry_point_caller):
        self._entry_point_caller = entry_point_caller

    def setup_player_input_component(self, actor, input_component):
        self._entry_point_caller.call_on_game_thread("input_service.setup_player_input_component", actor, input_component)

    def inject_key_for_actor(self, actor, chord, key_event):
        self._entry_point_caller.call_on_game_thread("input_service.inject_key_for_actor", actor, spear.func_utils.to_json_string(obj=chord), key_event)

    def inject_touch_for_actor(self, actor, key_event, finger_index, location):
        self._entry_point_caller.call_on_game_thread("input_service.inject_touch_for_actor", actor, key_event, finger_index, spear.func_utils.to_json_string(obj=location))

    def inject_axis_for_actor(self, actor, axis_name, axis_value):
        self._entry_point_caller.call_on_game_thread("input_service.inject_axis_for_actor", actor, axis_name, axis_value)

    def inject_axis_key_for_actor(self, actor, axis_key_name, axis_key_value):
        self._entry_point_caller.call_on_game_thread("input_service.inject_axis_key_for_actor", actor, axis_key_name, axis_key_value)

    def inject_vector_axis_for_actor(self, actor, vector_axis_name, vector_axis_value):
        self._entry_point_caller.call_on_game_thread("input_service.inject_vector_axis_for_actor", actor, vector_axis_name, spear.func_utils.to_json_string(obj=vector_axis_value))

    def inject_gesture_for_actor(self, actor, gesture_name, gesture_value):
        self._entry_point_caller.call_on_game_thread("input_service.inject_gesture_for_actor", actor, gesture_name, gesture_value)

    def inject_action_for_actor(self, actor, action_name, key_event, key_name):
        self._entry_point_caller.call_on_game_thread("input_service.inject_action_for_actor", actor, action_name, key_event, key_name)
