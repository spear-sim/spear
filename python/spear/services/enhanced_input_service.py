#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import spear

class EnhancedInputService(spear.Service):
    def __init__(self, entry_point_caller, parent_service=None, create_children_services=True):

        # do this after initializing local state
        super().__init__(entry_point_caller=entry_point_caller, parent_service=parent_service, create_children_services=create_children_services)

    def create_child_service(self, entry_point_caller, sp_func_service=None, unreal_service=None, config=None):
        assert self.is_top_level_service() # this function should only be called from the top-level service
        return EnhancedInputService(entry_point_caller=entry_point_caller, parent_service=self, create_children_services=False)

    def inject_input(self, enhanced_input_subsystem, input_action, input_action_value, modifiers=None, triggers=None):
        modifiers = modifiers if modifiers is not None else []
        triggers = triggers if triggers is not None else []
        return self.entry_point_caller.call_on_game_thread(
            "inject_input",
            None,
            spear.to_handle(obj=enhanced_input_subsystem),
            spear.to_handle(obj=input_action),
            spear.to_json_string(obj=input_action_value),
            spear.to_handle(obj=modifiers),
            spear.to_handle(obj=triggers))

    def inject_input_for_actor(self, actor, input_action_name, trigger_event, input_action_value, input_action_instance, modifiers=None, triggers=None):
        modifiers = modifiers if modifiers is not None else []
        triggers = triggers if triggers is not None else []
        return self.entry_point_caller.call_on_game_thread(
            "inject_input_for_actor",
            None,
            spear.to_handle(obj=actor),
            input_action_name,
            trigger_event,
            spear.to_json_string(obj=input_action_value),
            spear.to_json_string(obj=input_action_instance),
            spear.to_handle(obj=modifiers),
            spear.to_handle(obj=triggers))

    def inject_debug_key_for_actor(self, actor, chord, key_event, input_action_value):
        return self.entry_point_caller.call_on_game_thread(
            "inject_debug_key_for_actor",
            None,
            spear.to_handle(obj=actor),
            spear.to_json_string(obj=chord),
            key_event,
            spear.to_json_string(obj=input_action_value))
