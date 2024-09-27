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

    # TODO: Move to sp_func_service.py, because this is the only place where we need to concern ourselves
    # the endian-ness of the Unreal instance. All other services send and receive std::vector<T> where T is
    # not uint8_t, and therefore the endian-ness of the Unreal instance is handled implicitly at the msgpack
    # layer. Since SpFuncService sends and receives std::vector<uint8_t>, the msgpack layer does not attempt
    # to perform any endian conversions, so sp_func_service.py needs to perform explicit conversions. On the
    # Unreal instance, we should use Boost to detect endian-ness.
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
