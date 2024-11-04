#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import spear

class WorldService():
    def __init__(self, entry_point_caller):
        self._entry_point_caller = entry_point_caller

    # Entry points for miscellaneous functions that are accessible via getWorld(), but are not accessible via
    # the property system.

    def get_first_player_controller(self):
        return self._entry_point_caller.call("world_service.get_first_player_controller")
