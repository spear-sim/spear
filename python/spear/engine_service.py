#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import spear
import sys

# TODO (RP): Separate EngineService and GameWorldService function calls from this class.
class EngineService():
    def __init__(self, rpc_client):
        self._rpc_client = rpc_client

    def call(self, service_name, func_name, *args):
        return self._rpc_client.call(service_name + "." + func_name, *args)

    def begin_tick(self):
        self._rpc_client.call("engine_service.begin_tick")
        self._rpc_client.call("game_world_service.unpause_game")

    def tick(self):
        self._rpc_client.call("engine_service.tick")

    def end_tick(self):
        self._rpc_client.call("game_world_service.pause_game")
        self._rpc_client.call("engine_service.end_tick")

    def get_current_level(self):
        return self._rpc_client.call("game_world_service.get_current_level_name")

    def open_level(self, scene_id, map_id=""):
        desired_level_name = ""
        if scene_id != "":
            if map_id == "":
                map_id = scene_id
            else:
                map_id = map_id
            desired_level_name = "/Game/Scenes/" + scene_id + "/Maps/" + map_id

        spear.log("scene_id:           ", scene_id)
        spear.log("map_id:             ", map_id)
        spear.log("desired_level_name: ", desired_level_name)

        self.begin_tick()
        self._rpc_client.call("game_world_service.open_level", desired_level_name)
        self.tick()
        self.end_tick()

    def get_byte_order(self):
        unreal_instance_byte_order = self._rpc_client.call("engine_service.get_byte_order")
        rpc_client_byte_order = sys.byteorder
        if unreal_instance_byte_order == rpc_client_byte_order:
            return None
        elif unreal_instance_byte_order == "little":
            return "<"
        elif unreal_instance_byte_order == "big":
            return ">"
        else:
            assert False
