#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import sys

class EngineService():
    def __init__(self, rpc_client):
        self._rpc_client = rpc_client

    def ping(self):
        return self._rpc_client.call("engine_service.ping")

    def begin_tick(self):
        self._rpc_client.call("engine_service.begin_tick")

    def tick(self):
        self._rpc_client.call("engine_service.tick")

    def end_tick(self):
        self._rpc_client.call("engine_service.end_tick")

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
