#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import spear

class WorldService():
    def __init__(self, entry_point_caller):
        self._entry_point_caller = entry_point_caller

    def get_first_player_controller(self):
        return self._entry_point_caller.call("world_service.get_first_player_controller")
