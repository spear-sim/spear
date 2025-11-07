#
# Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import spear

class EnhancedInputService(spear.utils.func_utils.Service):
    def __init__(self, entry_point_caller, is_top_level_service=True, create_children_services=True):
        self._entry_point_caller = entry_point_caller

        super().__init__(
            is_top_level_service=is_top_level_service,
            create_children_services=create_children_services,
            entry_point_caller=entry_point_caller) # do this after initializing local state


    def create_child_service(self, entry_point_caller):
        assert self.is_top_level_service() # this function should only be called from the top-level service
        return EnhancedInputService(entry_point_caller=entry_point_caller, is_top_level_service=False, create_children_services=False)


    def inject_input(self, enhanced_input_subsystem, input_action, input_action_value, modifiers=[], triggers=[]):
        return self._entry_point_caller.call_on_game_thread(
            "inject_input",
            None,
            enhanced_input_subsystem,
            input_action,
            spear.utils.func_utils.to_json_string(obj=input_action_value),
            modifiers,
            triggers)

    def inject_input_for_actor(self, actor, input_action_name, trigger_event, input_action_value, input_action_instance, modifiers=[], triggers=[]):
        return self._entry_point_caller.call_on_game_thread(
            "inject_input_for_actor",
            None,
            actor,
            input_action_name,
            trigger_event,
            spear.utils.func_utils.to_json_string(obj=input_action_value),
            spear.utils.func_utils.to_json_string(obj=input_action_instance),
            modifiers,
            triggers)

    def inject_debug_key_for_actor(self, actor, chord, key_event, input_action_value):
        return self._entry_point_caller.call_on_game_thread(
            "inject_debug_key_for_actor",
            None,
            actor,
            spear.utils.func_utils.to_json_string(obj=chord),
            key_event,
            spear.utils.func_utils.to_json_string(obj=input_action_value))
