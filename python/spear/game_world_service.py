#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

class GameWorldService():
    def __init__(self, rpc_client):
        self._rpc_client = rpc_client

    def get_world_name(self):
        return self._rpc_client.call("game_world_service.get_world_name")

    def open_level(self, level_name):
        self._rpc_client.call("game_world_service.open_level", level_name)

    def set_game_paused(self, paused):
        self._rpc_client.call("game_world_service.set_game_paused", paused)
