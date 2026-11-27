#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import spear

class WorldRegistryService(spear.Service):
    def __init__(self, entry_point_caller):

        # do this after initializing local state
        super().__init__(entry_point_caller=entry_point_caller)

    def register_worlds(self):
        return self.entry_point_caller.call_on_game_thread("register_worlds", None)

    def unregister_world(self, world):
        world = spear.to_handle(obj=world)
        return self.entry_point_caller.call_on_game_thread("unregister_world", None, world)

    def get_world_descs(self):
        return self.entry_point_caller.call_on_game_thread("get_world_descs", None)
