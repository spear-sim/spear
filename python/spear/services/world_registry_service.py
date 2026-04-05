#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import spear

class WorldRegistryService(spear.Service):
    def __init__(self, entry_point_caller):

        # do this after initializing local state
        super().__init__(entry_point_caller=entry_point_caller)

    def get_world_descs(self):
        return self.entry_point_caller.call_on_worker_thread("get_world_descs", None)

    def remove_world(self, world):
        return self.entry_point_caller.call_on_game_thread("remove_world", None, world)
