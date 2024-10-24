#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import contextlib
import spear
import sys

class EngineService():
    def __init__(self, rpc_client):
        self._rpc_client = rpc_client
        self._frame_state = "idle"

    # These context managers are intended as exception-safe wrappers around the {begin_tick, tick, end_tick}
    # entry points in EngineService.

    @contextlib.contextmanager
    def begin_frame(self):
        try:
            assert self._frame_state == "idle"
            self._frame_state = "request_pre_tick"
            self._rpc_client.call("engine_service.begin_tick")
            self._frame_state = "executing_pre_tick"
            yield None
        except Exception as e:
            spear.log("Exception: ", e)
            if self._frame_state == "request_pre_tick" or self._frame_state == "executing_pre_tick":
                spear.log("Attempting to exit critical section...")
                self._rpc_client.call("engine_service.tick")
                self._rpc_client.call("engine_service.end_tick")
                self._frame_state = "idle"
            raise e

    @contextlib.contextmanager
    def end_frame(self):
        try:
            assert self._frame_state == "executing_pre_tick"
            self._rpc_client.call("engine_service.tick")
            self._frame_state = "executing_post_tick"
            yield None
        except Exception as e:
            spear.log("Exception: ", e)
            if self._frame_state == "executing_pre_tick" or self._frame_state == "executing_post_tick":
                spear.log("Attempting to exit critical section...")
                self._rpc_client.call("engine_service.end_tick")
                self._frame_state = "idle"
            raise e
        else:
            self._rpc_client.call("engine_service.end_tick")
            self._frame_state = "idle"

    # TODO: Move to sp_func_service.py, because this is the only place where we need to concern ourselves
    # the endian-ness of the Unreal instance. All other services send and receive std::vector<T> where T is
    # not uint8_t, and therefore the endian-ness of the Unreal instance is handled implicitly at the msgpack
    # layer. Since SpFuncService sends and receives std::vector<uint8_t>, the msgpack layer does not attempt
    # to perform any endian conversions, so sp_func_service.py needs to perform any required conversions
    # explicitly.
    def get_byte_order(self):
        return self._rpc_client.call("engine_service.get_byte_order")

    # This function is used in Instance.is_running() to determine if there is a valid world pointer, i.e., if
    # there is a valid simulation running. The RPC client can be successfully connected, but the world
    # pointer can be invald, e.g., if the editor is running but play-in-editor mode isn't.
    def get_world(self):
        return self._rpc_client.call("engine_service.get_world")

    # This function is not currently used, but it will be necessary to implement a wait() function that can
    # handle situations where the engine takes a long time to execute a command, e.g., OpenLevel(...).
    def get_frame_state(self):
        return self._rpc_client.call("engine_service.get_frame_state")

    # This function is used in Instance.close() to close the Unreal application.
    def request_exit(self):
        return self._rpc_client.call("engine_service.request_exit")
