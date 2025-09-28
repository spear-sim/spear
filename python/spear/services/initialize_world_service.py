#
# Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

class InitializeWorldService():
    def __init__(self, entry_point_caller):
        self._entry_point_caller = entry_point_caller

    def is_initialized(self):
        return self._entry_point_caller.call_on_worker_thread("bool", "is_initialized")
