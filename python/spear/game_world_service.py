#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

class GameWorldService():
    def __init__(self, rpc_client):
        self._rpc_client = rpc_client

    def get_current_level(self):
        return self._rpc_client.call("game_world_service.get_current_level_name")

    def open_level(self, desired_level_name):
        self._rpc_client.call("game_world_service.open_level", desired_level_name)

    def pause_game(self):
        self._rpc_client.call("game_world_service.pause_game")

    def unpause_game(self):
        self._rpc_client.call("game_world_service.unpause_game")
