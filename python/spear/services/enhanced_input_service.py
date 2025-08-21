#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import spear

class EnhancedInputService(spear.utils.func_utils.Service):
    def __init__(self, entry_point_caller, create_children=True):

        self._entry_point_caller = entry_point_caller
        super().__init__(entry_point_caller, create_children) # do this after initializing local state

    def create_child(self, entry_point_caller):
        return EnhancedInputService(entry_point_caller=entry_point_caller, create_children=False)

    def inject_input(self, enhanced_input_subsystem, input_action, input_action_value, modifiers=[], triggers=[]):
        return self._entry_point_caller.call_on_game_thread(
            "void",
            "inject_input",
            None,
            enhanced_input_subsystem,
            input_action,
            spear.utils.func_utils.to_json_string(obj=input_action_value),
            modifiers,
            triggers)

    def inject_input_for_actor(self, actor, input_action_name, trigger_event, input_action_value, input_action_instance, modifiers=[], triggers=[]):
        return self._entry_point_caller.call_on_game_thread(
            "void",
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
            "void",
            "inject_debug_key_for_actor",
            None,
            actor,
            spear.utils.func_utils.to_json_string(obj=chord),
            key_event,
            spear.utils.func_utils.to_json_string(obj=input_action_value))
