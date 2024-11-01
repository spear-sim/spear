#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import json
import spear

class EnhancedInputService():
    def __init__(self, entry_point_caller):
        self._entry_point_caller = entry_point_caller

    def inject_input_for_action(self, enhanced_input_subsystem, action, raw_value, modifiers=[], triggers=[]):
        self._entry_point_caller.call("enhanced_input_service.inject_input_for_action",
            enhanced_input_subsystem, action, json.dumps(raw_value), modifiers, triggers)
