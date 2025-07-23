#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import spear

class InputService():
    def __init__(self, entry_point_caller, create_children=True):
        self._entry_point_caller = entry_point_caller

        if create_children:
            call_async_service_name = self._entry_point_caller.service_name + ".call_async"
            call_async_entry_point_caller = spear.services.entry_point_caller.CallAsyncEntryPointCaller(service_name=call_async_service_name, engine_service=self._entry_point_caller.engine_service)
            self.call_async = InputService(entry_point_caller=call_async_entry_point_caller, create_children=False)

            send_async_service_name = self._entry_point_caller.service_name + ".send_async"
            send_async_entry_point_caller = spear.services.entry_point_caller.SendAsyncEntryPointCaller(service_name=send_async_service_name, engine_service=self._entry_point_caller.engine_service)
            self.send_async = InputService(entry_point_caller=send_async_entry_point_caller, create_children=False)

            call_async_fast_service_name = self._entry_point_caller.service_name + ".call_async"
            call_async_fast_entry_point_caller = spear.services.entry_point_caller.CallAsyncFastEntryPointCaller(service_name=call_async_fast_service_name, engine_service=self._entry_point_caller.engine_service)
            self.call_async_fast = InputService(entry_point_caller=call_async_fast_entry_point_caller, create_children=False)

            send_async_fast_service_name = self._entry_point_caller.service_name + ".send_async"
            send_async_fast_entry_point_caller = spear.services.entry_point_caller.SendAsyncFastEntryPointCaller(service_name=send_async_service_name, engine_service=self._entry_point_caller.engine_service)
            self.send_async_fast = InputService(entry_point_caller=send_async_fast_entry_point_caller, create_children=False)


    def setup_player_input_component(self, actor, input_component):
        self._entry_point_caller.call_on_game_thread("void", "input_service.setup_player_input_component", None, actor, input_component)

    def inject_key_for_actor(self, actor, chord, key_event):
        self._entry_point_caller.call_on_game_thread("void", "input_service.inject_key_for_actor", None, actor, spear.utils.func_utils.to_json_string(obj=chord), key_event)

    def inject_touch_for_actor(self, actor, key_event, finger_index, location):
        self._entry_point_caller.call_on_game_thread("void", "input_service.inject_touch_for_actor", None, actor, key_event, finger_index, spear.utils.func_utils.to_json_string(obj=location))

    def inject_axis_for_actor(self, actor, axis_name, axis_value):
        self._entry_point_caller.call_on_game_thread("void", "input_service.inject_axis_for_actor", None, actor, axis_name, axis_value)

    def inject_axis_key_for_actor(self, actor, axis_key_name, axis_key_value):
        self._entry_point_caller.call_on_game_thread("void", "input_service.inject_axis_key_for_actor", None, actor, axis_key_name, axis_key_value)

    def inject_vector_axis_for_actor(self, actor, vector_axis_name, vector_axis_value):
        self._entry_point_caller.call_on_game_thread("void", "input_service.inject_vector_axis_for_actor", None, actor, vector_axis_name, spear.utils.func_utils.to_json_string(obj=vector_axis_value))

    def inject_gesture_for_actor(self, actor, gesture_name, gesture_value):
        self._entry_point_caller.call_on_game_thread("void", "input_service.inject_gesture_for_actor", None, actor, gesture_name, gesture_value)

    def inject_action_for_actor(self, actor, action_name, key_event, key_name):
        self._entry_point_caller.call_on_game_thread("void", "input_service.inject_action_for_actor", None, actor, action_name, key_event, key_name)
