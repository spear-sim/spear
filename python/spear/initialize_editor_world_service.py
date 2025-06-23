#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

class InitializeEditorWorldService():
    def __init__(self, namespace, entry_point_caller):
        self._entry_point_caller = entry_point_caller
        self._service_name = namespace + ".initialize_editor_world_service"

    def is_initialized(self):
        return self._entry_point_caller.call_on_worker_thread(self._service_name + ".is_initialized")
