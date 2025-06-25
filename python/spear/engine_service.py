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

    #
    # These context managers are intended as exception-safe wrappers around the {begin_frame, execute_frame,
    # end_frame} entry points in EngineService.
    #

    @contextlib.contextmanager
    def begin_frame(self):

        assert self._frame_state == "idle"
        self._frame_state = "request_begin_frame"

        # try calling begin_frame()
        try:
            self.call_on_worker_thread("engine_service.begin_frame")
        except Exception as e:
            spear.log("Exception: ", e)
            spear.log("We might or might not be in a critical section, but there is nothing we can do to get out of it from here, so we need to give up...")
            self._frame_state = "error"
            raise e

        assert self._frame_state == "request_begin_frame"
        self._frame_state = "executing_begin_frame"

        # try executing pre-frame work
        try:
            yield
        except Exception as e:
            spear.log("Exception: ", e)
            spear.log("Attempting to exit critical section...")
            self.call_on_worker_thread("engine_service.execute_frame")
            self.call_on_worker_thread("engine_service.end_frame")
            self._frame_state = "idle"
            raise e

        assert self._frame_state == "executing_begin_frame"
        self._frame_state = "finished_executing_begin_frame"


    @contextlib.contextmanager
    def end_frame(self):

        assert self._frame_state == "finished_executing_begin_frame"
        self._frame_state = "executing_frame"

        # try executing execute_frame()
        try:
            self.call_on_worker_thread("engine_service.execute_frame")
        except Exception as e:
            spear.log("Exception: ", e)
            spear.log("We're currently in a critical section, but there is nothing we can do to get out of it from here, so we need to give up...")
            self._frame_state = "error"
            raise e

        assert self._frame_state == "executing_frame"
        self._frame_state = "executing_end_frame"

        # try executing post-frame work
        try:
            yield
        except Exception as e:
            spear.log("Exception: ", e)
            spear.log("Attempting to exit critical section...")
            self._rpc_client.call_on_worker_thread("engine_service.end_frame")
            self._frame_state = "idle"
            raise e

        assert self._frame_state == "executing_end_frame"
        self._frame_state = "request_end_frame"

        # try executing end_frame()
        try:
            self.call_on_worker_thread("engine_service.end_frame")
        except Exception as e:
            spear.log("Exception: ", e)
            spear.log("We're currently in a critical section, but there is nothing we can do to get out of it from here, so we need to give up...")
            self._frame_state = "error"
            raise e

        assert self._frame_state == "request_end_frame"
        self._frame_state = "idle"

    #
    # Call functions
    #

    def call_on_game_thread(self, entry_point_name, *args):
        if self._frame_state not in ["executing_begin_frame", "executing_end_frame"]:
            spear.log('ERROR: Calling entry points that execute on the game thread is only allowed in "with begin_frame()" and "with end_frame()" code blocks.')
            spear.log('ERROR: self._frame_state == "' + self._frame_state + '").')

            if self._frame_state == "finished_executing_begin_frame":
                spear.log("ERROR: Attempting to exit critical section...")
                self._call("engine_service.execute_frame")
                self._call("engine_service.end_frame")
                self._frame_state = "idle"

            assert False

        return self._call(entry_point_name, *args)

    def call_on_worker_thread(self, entry_point_name, *args):
        return self._call(entry_point_name, *args)

    def _call(self, entry_point_name, *args):
        if self._config.SPEAR.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO:
            spear.log("Calling:               ", entry_point_name, args)

        return_value = self._rpc_client.call(entry_point_name, *args)

        if self._config.SPEAR.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO:
            spear.log("Obtained return value: ", return_value)

        return return_value

    # Miscellaneous low-level entry points.

    def ping(self):
        return self.call_on_worker_thread("engine_service.ping")

    def initialize(self):
        return self.call_on_worker_thread("engine_service.initialize")

    def request_exit(self):
        return self.call_on_worker_thread("engine_service.request_exit")

    def with_editor(self):
        return self.call_on_worker_thread("engine_service.with_editor")

    # Entry points for miscellaneous functions that are accessible via GEngine.

    def get_viewport_size(self):
        return self.call_on_game_thread("engine_service.get_viewport_size")
