#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import spear


class EnhancedInputService():
    def __init__(self, entry_point_caller, create_children=True):
        self._entry_point_caller = entry_point_caller

        if create_children:
            call_async_service_name = self._entry_point_caller.service_name + ".call_async"
            call_async_entry_point_caller = spear.services.entry_point_caller.CallAsyncEntryPointCaller(service_name=call_async_service_name, engine_service=self._entry_point_caller.engine_service)
            self.call_async = EnhancedInputService(entry_point_caller=call_async_entry_point_caller, create_children=False)

            send_async_service_name = self._entry_point_caller.service_name + ".send_async"
            send_async_entry_point_caller = spear.services.entry_point_caller.SendAsyncEntryPointCaller(service_name=send_async_service_name, engine_service=self._entry_point_caller.engine_service)
            self.send_async = EnhancedInputService(entry_point_caller=send_async_entry_point_caller, create_children=False)

            call_async_fast_service_name = self._entry_point_caller.service_name + ".call_async"
            call_async_fast_entry_point_caller = spear.services.entry_point_caller.CallAsyncFastEntryPointCaller(service_name=call_async_fast_service_name, engine_service=self._entry_point_caller.engine_service)
            self.call_async_fast = EnhancedInputService(entry_point_caller=call_async_fast_entry_point_caller, create_children=False)

            send_async_fast_service_name = self._entry_point_caller.service_name + ".send_async"
            send_async_fast_entry_point_caller = spear.services.entry_point_caller.SendAsyncFastEntryPointCaller(service_name=send_async_service_name, engine_service=self._entry_point_caller.engine_service)
            self.send_async_fast = EnhancedInputService(entry_point_caller=send_async_fast_entry_point_caller, create_children=False)


    def inject_input(self, enhanced_input_subsystem, input_action, input_action_value, modifiers=[], triggers=[]):
        self._entry_point_caller.call_on_game_thread(
            "void",
            "enhanced_input_service.inject_input",
            None,
            enhanced_input_subsystem,
            input_action,
            spear.utils.func_utils.to_json_string(obj=input_action_value),
            modifiers,
            triggers)


    def inject_input_for_actor(self, actor, input_action_name, trigger_event, input_action_value, input_action_instance, modifiers=[], triggers=[]):
        self._entry_point_caller.call_on_game_thread(
            "void",
            "enhanced_input_service.inject_input_for_actor",
            None,
            actor,
            input_action_name,
            trigger_event,
            spear.utils.func_utils.to_json_string(obj=input_action_value),
            spear.utils.func_utils.to_json_string(obj=input_action_instance),
            modifiers,
            triggers)


    def inject_debug_key_for_actor(self, actor, chord, key_event, input_action_value):
        self._entry_point_caller.call_on_game_thread(
            "void",
            "enhanced_input_service.inject_debug_key_for_actor",
            None,
            actor,
            spear.utils.func_utils.to_json_string(obj=chord),
            key_event,
            spear.utils.func_utils.to_json_string(obj=input_action_value))
