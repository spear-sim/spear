#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import contextlib
import spear
import sys

class EngineService():
    def __init__(self, rpc_client, config):
        self._rpc_client = rpc_client
        self._config = config
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
            yield
        except Exception as e:
            spear.log("Exception: ", e)
            spear.log("Attempting to exit critical section...")
            self._rpc_client.call("engine_service.tick")
            self._rpc_client.call("engine_service.end_tick")
            assert self._frame_state == "request_pre_tick" or self._frame_state == "executing_pre_tick"
            self._frame_state = "idle"
            raise e
        else:
            self._frame_state = "finished_executing_pre_tick"

    @contextlib.contextmanager
    def end_frame(self):
        try:
            assert self._frame_state == "finished_executing_pre_tick"
            self._frame_state = "executing_tick"
            self._rpc_client.call("engine_service.tick")
            self._frame_state = "executing_post_tick"
            yield
        except Exception as e:
            spear.log("Exception: ", e)
            spear.log("Attempting to exit critical section...")
            self._rpc_client.call("engine_service.end_tick")
            assert self._frame_state == "executing_tick" or self._frame_state == "executing_post_tick"
            self._frame_state = "idle"
            raise e
        else:
            self._rpc_client.call("engine_service.end_tick")
            self._frame_state = "idle"

    # This function is called by all other services.
    def call(self, func_name, *args):
        if self._frame_state == "finished_executing_pre_tick":
            spear.log('ERROR: Calling SPEAR functions in between "with begin_frame()" and "with end_frame()" code blocks is not supported.')
            spear.log("Attempting to exit critical section...")
            self._rpc_client.call("engine_service.tick")
            self._rpc_client.call("engine_service.end_tick")
            self._frame_state = "idle"
            assert False

        if self._config.SPEAR.ENGINE_SERVICE.PRINT_ENTRY_POINT_CALLS:
            spear.log("Calling:               ", func_name, args)

        return_value = self._rpc_client.call(func_name, *args)

        if self._config.SPEAR.ENGINE_SERVICE.PRINT_ENTRY_POINT_CALLS:
            spear.log("Obtained return value: ", return_value)

        return return_value

    # These functions are used in Instance.is_running() to determine if there is a valid simulation running.

    def get_world(self):
        return self.call("engine_service.get_world")

    def get_frame_state(self):
        return self.call("engine_service.get_frame_state")

    # This function is used in Instance.close() to close the Unreal application.
    def request_exit(self):
        return self.call("engine_service.request_exit")
