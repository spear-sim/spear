#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import contextlib
import spear
import sys


class EngineService():
    def __init__(self, client, config):

        self._client = client
        self._config = config

        self._frame_state = None
        self._byte_order = None
        self._server_signature_descs = None

    def initialize(self):
        assert self._frame_state is None

        # we use _call_impl(...) here because call_sync_on_worker_thread(...) assumes we're already initialized

        self._frame_state = "idle"

        # explicitly initialize before calling begin_frame() for the first time

        self._call_impl(
            "void",
            "_call_sync_on_worker_thread_as_void",
            "engine_service.call_sync_on_worker_thread.initialize")

        # cache data that will be constant for the life of the server

        server_signature_type_descs = self._call_impl(
            "vector_of_func_signature_type_desc",
            "_call_sync_on_worker_thread_as_vector_of_func_signature_type_desc",
            "engine_service.call_sync_on_worker_thread.get_entry_point_signature_type_descs")

        self._server_signature_type_descs = server_signature_type_descs

        server_signature_descs = self._call_impl(
            "map_of_string_to_vector_of_func_signature_desc",
            "_call_sync_on_worker_thread_as_map_of_string_to_vector_of_func_signature_desc",
            "engine_service.call_sync_on_worker_thread.get_entry_point_signature_descs")

        self._server_signature_descs = { registry_name: { desc.name: desc for desc in descs } for registry_name, descs in server_signature_descs.items() }

        byte_order = self._call_impl(
            "string",
            "_call_sync_on_worker_thread_as_string",
            "engine_globals_service.call_sync_on_worker_thread.get_byte_order")

        if byte_order == sys.byteorder:
            self._byte_order = "native"
        else:
            self._byte_order = byte_order

    def terminate(self):
        assert self._frame_state is not None

        # we use _call_impl(...) here for symmetry with initialize()

        self._frame_state = None

        self._call_impl(
            "void",
            "_call_sync_on_worker_thread_as_void",
            "engine_service.call_sync_on_worker_thread.terminate")

    #
    # Helper functions to support initializing a spear.Instance
    #

    def get_byte_order(self):
        assert self._byte_order is not None
        return self._byte_order

    def get_server_signature_type_descs(self):
        assert self._server_signature_type_descs is not None
        return self._server_signature_type_descs

    def get_server_signature_descs(self):
        assert self._server_signature_descs is not None
        return self._server_signature_descs

    #
    # These context managers are intended as exception-safe wrappers for managing the server's frame state.
    #

    @contextlib.contextmanager
    def begin_frame(self):
        assert self._frame_state == "idle" or self._frame_state == "request_begin_next_frame"

        success = False

        # if we're currently in the idle state, we need to call begin_frame_impl()
        if self._frame_state == "idle":        
            self._frame_state = "request_begin_frame"

            # try calling begin_frame_impl()
            try:
                success = self._begin_frame_impl()
            except Exception as e:
                spear.log("Exception: ", e)
                spear.log("ERROR: We might or might not be in a critical section, but we don't know how to get out of it from here. Giving up...")
                self._frame_state = "error"
                raise e

            if not success:
                spear.log("ERROR: Server error state detected when calling _begin_frame_impl(), but we do not appear to be in a critical section.")
                self._frame_state = "error"
                assert False

        # if we're currently in the request_begin_next_frame state, it means that the user previously called
        # end_frame(single_step=True), in which case we have effectively already called begin_frame_impl() as
        # part of our previous call to end_frame_impl_single_step(), so we do not call it again here
        elif self._frame_state == "request_begin_next_frame":
            self._frame_state = "request_begin_frame"

        else:
            assert False

        # try executing pre-frame work
        try:
            assert self._frame_state == "request_begin_frame"
            self._frame_state = "executing_begin_frame"
            yield
            assert self._frame_state == "executing_begin_frame"
            self._frame_state = "executing_frame"
        except Exception as e:
            spear.log("Exception: ", e)
            spear.log("ERROR: Attempting to exit critical section by calling _execute_frame_impl() and _end_frame_impl()...")
            self._execute_frame_impl() # ignore return value because we already know we need to call _execute_frame_impl() and _end_frame_impl()
            self._end_frame_impl()
            self._frame_state = "error"
            raise e

    @contextlib.contextmanager
    def end_frame(self, single_step=False):

        # try calling execute_frame_impl()
        try:
            success = self._execute_frame_impl()
        except Exception as e:
            spear.log("Exception: ", e)
            spear.log("ERROR: We are in a critical section, but we don't know how to get out of it from here. Attempting to call _end_frame_impl()...")
            self._end_frame_impl()
            self._frame_state = "error"
            raise e

        if not success:
            spear.log("ERROR: Server error state detected when calling _execute_frame_impl(). Attempting to exit critical section by calling _end_frame_impl()...")
            self._end_frame_impl()
            self._frame_state = "error"
            assert False

        # try executing post-frame work
        try:
            assert self._frame_state == "executing_frame"
            self._frame_state = "executing_end_frame"
            yield
            assert self._frame_state == "executing_end_frame"
            self._frame_state = "request_end_frame"
        except Exception as e:
            spear.log("Exception: ", e)
            spear.log("ERROR: Attempting to exit critical section by calling _end_frame_impl()...")
            self._end_frame_impl()
            self._frame_state = "error"
            raise e

        success = True

        # try calling end_frame_impl()
        try:
            success = self._end_frame_impl(single_step=single_step)
        except Exception as e:
            spear.log("Exception: ", e)
            spear.log("ERROR: We might or might not be in a critical section, but we don't know how to get out of it from here. Giving up...")
            self._frame_state = "error"
            raise e

        assert self._frame_state == "request_end_frame"

        if not success:
            spear.log("ERROR: Server error state detected when calling _end_frame_impl_single_step(), but we do not appear to be in a critical section.")
            self._frame_state = "error"
            assert False

        if single_step:
            self._frame_state = "request_begin_next_frame"
        else:
            self._frame_state = "idle"

    #
    # Helper functions for managing the server's frame state.
    #

    def _begin_frame_impl(self):
        return self.call_sync_on_worker_thread("engine_service.call_sync_on_worker_thread.begin_frame")

    def _execute_frame_impl(self):
        return self.call_sync_on_worker_thread("engine_service.call_sync_on_worker_thread.execute_frame")

    def _end_frame_impl(self, single_step=False):
        return self.call_sync_on_worker_thread("engine_service.call_sync_on_worker_thread.end_frame", single_step)

    #
    # Functions for calling entry points on the server.
    #

    # worker thread

    def call_sync_on_worker_thread(self, func_name, *args):
        self._validate_frame_state_for_worker_thread_work()
        return_as = self._server_signature_descs["call_sync_on_worker_thread"][func_name].func_signature[0].type_names["entry_point"]
        call_func_name = "_call_sync_on_worker_thread_as_" + return_as
        return self._call_impl(return_as, call_func_name, func_name, *args)

    def call_async_fast_on_worker_thread(self, func_name, *args):
        self._validate_frame_state_for_worker_thread_work()
        return_as = "uint64"
        call_func_name = "_call_async_fast_on_worker_thread"
        return self._call_impl(return_as, call_func_name, func_name, *args)

    def send_async_fast_on_worker_thread(self, func_name, *args):
        self._validate_frame_state_for_worker_thread_work()
        return_as = "void"
        call_func_name = "_send_async_fast_on_worker_thread"
        self._call_impl(return_as, call_func_name, func_name, *args)

    def get_future_result_fast_from_worker_thread(self, future):
        self._validate_frame_state_for_worker_thread_work()
        return_as = future._return_as
        func_name = future._func_name
        get_future_result_func_name = "_get_future_result_fast_from_worker_thread_as_" + return_as
        return self._get_future_result_impl(return_as, get_future_result_func_name, future.future, func_name)

    # game thread

    def call_sync_on_game_thread(self, func_name, *args):
        self._validate_frame_state_for_game_thread_work()
        return_as = self._server_signature_descs["call_sync_on_game_thread"][func_name].func_signature[0].type_names["entry_point"]
        call_func_name = "_call_sync_on_game_thread_as_" + return_as
        return self._call_impl(return_as, call_func_name, func_name, *args)

    def call_async_on_game_thread(self, func_name, *args):
        self._validate_frame_state_for_game_thread_work() # need the server's current work queue to be in a well-defined state
        return_as = self._server_signature_descs["call_async_on_game_thread"][func_name].func_signature[0].type_names["entry_point"]
        assert return_as == "future"
        call_func_name = "_call_async_on_game_thread"
        return self._call_impl(return_as, call_func_name, func_name, *args)

    def send_async_on_game_thread(self, func_name, *args):
        self._validate_frame_state_for_game_thread_work() # need the server's current work queue to be in a well-defined state
        return_as = self._server_signature_descs["send_async_on_game_thread"][func_name].func_signature[0].type_names["entry_point"]
        assert return_as == "void"
        call_func_name = "_send_async_on_game_thread"
        self._call_impl(return_as, call_func_name, func_name, *args)

    def get_future_result_from_game_thread(self, future):
        self._validate_frame_state_for_worker_thread_work() # only need to execute work on a worker thread
        return_as = future._return_as
        func_name = future._func_name
        get_future_result_func_name = "_get_future_result_from_game_thread_as_" + return_as
        return self._get_future_result_impl(return_as, get_future_result_func_name, future.future, func_name)

    def call_async_fast_on_game_thread(self, func_name, *args):
        self._validate_frame_state_for_game_thread_work() # need the server's current work queue to be in a well-defined state
        return_as = "uint64"
        call_func_name = "_call_async_fast_on_game_thread"
        return self._call_impl(return_as, call_func_name, func_name, *args)

    def send_async_fast_on_game_thread(self, func_name, *args):
        self._validate_frame_state_for_game_thread_work() # need the server's current work queue to be in a well-defined state
        return_as = "void"
        call_func_name = "_send_async_fast_on_game_thread"
        self._call_impl(return_as, call_func_name, func_name, *args)

    def get_future_result_fast_from_game_thread(self, future):
        self._validate_frame_state_for_worker_thread_work() # only need to execute work on a worker thread
        return_as = future._return_as
        func_name = future._func_name
        get_future_result_func_name = "_get_future_result_fast_from_game_thread_as_" + return_as
        return self._get_future_result_impl(return_as, get_future_result_func_name, future.future, func_name)

    #
    # Helper functions for calling entry points on the server
    #

    def _validate_frame_state_for_worker_thread_work(self):
        if self._frame_state not in ["idle", "request_begin_frame", "executing_begin_frame", "executing_frame", "executing_end_frame", "request_end_frame", "error"]:
            spear.log("ERROR: Unexpected frame state: ", self._frame_state)
            assert False

    def _validate_frame_state_for_game_thread_work(self):
        if self._frame_state not in ["executing_begin_frame", "executing_end_frame"]:
            spear.log('ERROR: Calling entry points that execute on the game thread is only allowed in "with begin_frame()" and "with end_frame()" code blocks.')
            spear.log(f'ERROR: self._frame_state == "{self._frame_state}"')

            if self._frame_state == "executing_frame":
                spear.log("ERROR: Attempting to exit critical section by calling _end_frame_impl()...")
                self._end_frame_impl()
                self._frame_state = "error"

            assert False

    def _call_impl(self, return_as, call_func_name, func_name, *args):

        if self._config.SPEAR.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO:
            spear.log(f"Calling: {func_name} : {args} -> {return_as} via {call_func_name}")

        call_func = getattr(self._client, call_func_name, None)
        if call_func is None:
            spear.log(f"ERROR: Couldn't find client entry point {call_func_name} when attempting to call {func_name} : {args} -> {return_as}")
            assert False
        if not callable(call_func):
            spear.log(f"ERROR: Attribute {call_func_name} is not callable when attempting to call {func_name} : {args} -> {return_as}")
            assert False

        try:
            return_value = call_func(func_name, *args)
        except Exception:
            spear.log(f"ERROR: When calling {func_name} : {args} -> {return_as} via {call_func_name}")
            raise

        if self._config.SPEAR.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO:
            spear.log("Obtained return value: ", return_value)

        return return_value

    def _get_future_result_impl(self, return_as, get_future_result_func_name, future, func_name):

        if self._config.SPEAR.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO:
            spear.log(f"Calling: {get_future_result_func_name} : () -> {return_as} for {func_name}")

        get_future_result_func = getattr(self._client, get_future_result_func_name, None)
        if get_future_result_func is None:
            spear.log(f"ERROR: Couldn't find client entry point {get_future_result_func_name} when attempting to get future result {return_as} for {func_name}")
            assert False
        if not callable(get_future_result_func):
            spear.log(f"ERROR: Attribute {get_future_result_func_name} is not callable when attempting to get future result {return_as} for {func_name}")
            assert False

        try:
            return_value = get_future_result_func(future)
        except Exception:
            spear.log(f"ERROR: When calling {get_future_result_func_name} : () -> {return_as} for {func_name}")
            raise

        if self._config.SPEAR.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO:
            spear.log("Obtained return value: ", return_value)

        return return_value
