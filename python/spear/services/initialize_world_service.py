#
# Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import spear

class InitializeWorldService(spear.utils.func_utils.Service):
    def __init__(self, entry_point_caller, is_top_level_service=True, create_children_services=True):
        self._entry_point_caller = entry_point_caller

        super().__init__(
            is_top_level_service=is_top_level_service,
            create_children_services=create_children_services,
            entry_point_caller=entry_point_caller) # do this after initializing local state


    def create_child_service(self, entry_point_caller):
        assert self.is_top_level_service() # this function should only be called from the top-level service
        return InitializeWorldService(entry_point_caller=entry_point_caller, is_top_level_service=False, create_children_services=False)


    def is_initialized(self):
        return self._entry_point_caller.call_on_worker_thread("is_initialized", None)
