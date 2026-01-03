#
# Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import spear

class InitializeWorldService(spear.Service):
    def __init__(self, entry_point_caller):

        # do this after initializing local state
        super().__init__(entry_point_caller=entry_point_caller, parent_service=None, create_children_services=False)

    def is_initialized(self):
        return self.entry_point_caller.call_on_worker_thread("is_initialized", None)
