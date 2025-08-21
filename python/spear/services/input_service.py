#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import spear

class InputService(spear.utils.func_utils.Service):
    def __init__(self, entry_point_caller, create_children=True):

        self._entry_point_caller = entry_point_caller
        super().__init__(entry_point_caller, create_children) # do this after initializing local state

    def create_child(self, entry_point_caller):
        return InputService(entry_point_caller=entry_point_caller, create_children=False)

    def setup_player_input_component(self, actor, input_component):
        return self._entry_point_caller.call_on_game_thread("void", "setup_player_input_component", None, actor, input_component)

    def inject_key_for_actor(self, actor, chord, key_event):
        return self._entry_point_caller.call_on_game_thread("void", "inject_key_for_actor", None, actor, spear.utils.func_utils.to_json_string(obj=chord), key_event)

    def inject_touch_for_actor(self, actor, key_event, finger_index, location):
        return self._entry_point_caller.call_on_game_thread("void", "inject_touch_for_actor", None, actor, key_event, finger_index, spear.utils.func_utils.to_json_string(obj=location))

    def inject_axis_for_actor(self, actor, axis_name, axis_value):
        return self._entry_point_caller.call_on_game_thread("void", "inject_axis_for_actor", None, actor, axis_name, axis_value)

    def inject_axis_key_for_actor(self, actor, axis_key_name, axis_key_value):
        return self._entry_point_caller.call_on_game_thread("void", "inject_axis_key_for_actor", None, actor, axis_key_name, axis_key_value)

    def inject_vector_axis_for_actor(self, actor, vector_axis_name, vector_axis_value):
        return self._entry_point_caller.call_on_game_thread("void", "inject_vector_axis_for_actor", None, actor, vector_axis_name, spear.utils.func_utils.to_json_string(obj=vector_axis_value))

    def inject_gesture_for_actor(self, actor, gesture_name, gesture_value):
        return self._entry_point_caller.call_on_game_thread("void", "inject_gesture_for_actor", None, actor, gesture_name, gesture_value)

    def inject_action_for_actor(self, actor, action_name, key_event, key_name):
        return self._entry_point_caller.call_on_game_thread("void", "inject_action_for_actor", None, actor, action_name, key_event, key_name)
