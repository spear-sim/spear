#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import spear

class EnhancedInputService():
    def __init__(self, entry_point_caller):
        self._entry_point_caller = entry_point_caller

    def inject_input(self, enhanced_input_subsystem, input_action, input_action_value, modifiers=[], triggers=[]):
        self._entry_point_caller.call("enhanced_input_service.inject_input",
            enhanced_input_subsystem, input_action, spear.to_json_string(obj=input_action_value), modifiers, triggers)

    def inject_input_for_actor(self, actor, input_action_name, trigger_event, input_action_value, input_action_instance, modifiers=[], triggers=[]):
        self._entry_point_caller.call("enhanced_input_service.inject_input_for_actor",
            actor, input_action_name, trigger_event, spear.to_json_string(obj=input_action_value), spear.to_json_string(obj=input_action_instance), modifiers, triggers)

    def inject_debug_key_for_actor(self, actor, chord, key_event, input_action_value):
        self._entry_point_caller.call("enhanced_input_service.inject_debug_key_for_actor",
            actor, spear.to_json_string(obj=chord), key_event, spear.to_json_string(obj=input_action_value))
