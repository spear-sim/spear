#
# Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import contextlib
import spear
import sys

class EngineService(spear.utils.func_utils.Service):
    def __init__(self, entry_point_caller, client, config, is_top_level_service=True, create_children_services=True):
        self._entry_point_caller = entry_point_caller
        entry_point_caller.engine_service = self # need to assign the entry_point_caller.engine_service once we have a valid self reference

        self._client = client
        self._config = config

        self._frame_state = None
        self._byte_order = None

        super().__init__(
            is_top_level_service=is_top_level_service,
            create_children_services=create_children_services,
            entry_point_caller=entry_point_caller) # do this after initializing local state


    def create_child_service(self, entry_point_caller):
        assert self.is_top_level_service() # this function should only be called from the top-level service
        return EngineService(entry_point_caller=entry_point_caller, client=self._client, config=self._config, is_top_level_service=False, create_children_services=False)

    #
    # Functions for managing the server's frame state.
    #

    def initialize(self):
        assert self.is_top_level_service() # this function should only be called from the top-level service

        self._frame_state = "idle"

        # explicitly initialize before calling begin_frame() for the first time
        self._call_impl(
            "void",
            "_call_sync_on_worker_thread_as_void",
            "engine_service.call_sync_on_worker_thread.initialize")

        # cache byte order because it will be constant for the life of the server
        byte_order = self._call_impl(
            "string",
            "_call_sync_on_worker_thread_as_string",
            "engine_service.call_sync_on_worker_thread.get_byte_order")
        if byte_order == sys.byteorder:
            self._byte_order = "native"
        else:
            self._byte_order = byte_order

        # cache function signatures because they will be constant for the life of the server
        server_signature_descs = self._call_impl(
            "map_of_string_to_vector_of_func_signature_desc",
            "_call_sync_on_worker_thread_as_map_of_string_to_vector_of_func_signature_desc",
            "engine_service.call_sync_on_worker_thread.get_entry_point_signature_descs")
        self._server_signature_descs = { registry_name: { desc.name: desc for desc in descs } for registry_name, descs in server_signature_descs.items() }

    def terminate(self):
        assert self.is_top_level_service() # this function should only be called from the top-level service

        self._frame_state = None

        self._call_impl(
            "void",
            "_call_sync_on_worker_thread_as_void",
            "engine_service.call_sync_on_worker_thread.terminate")

    #
    # These context managers are intended as exception-safe wrappers for managing the server's frame state.
    #

    @contextlib.contextmanager
    def begin_frame(self):
        assert self.is_top_level_service() # user should only call this function on the top-level service
        assert self._frame_state == "idle"
        self._frame_state = "request_begin_frame"

        # try calling begin_frame_impl()
        try:
            self._begin_frame_impl()
        except Exception as e:
            spear.log("Exception: ", e)
            spear.log("ERROR: We might or might not be in a critical section, but we don't know how to get out of it from here. Giving up...")
            self._frame_state = "error"
            raise e

        # try executing pre-frame work
        try:
            assert self._frame_state == "request_begin_frame"
            self._frame_state = "executing_begin_frame"
            yield
            assert self._frame_state == "executing_begin_frame"
            self._frame_state = "executing_frame"
        except Exception as e:
            spear.log("Exception: ", e)
            spear.log("ERROR: Attempting to exit critical section by calling engine_service.execute_frame and engine_service.end_frame...")
            self._execute_frame_impl()
            self._end_frame_impl()
            self._frame_state = "error"
            raise e

        # try calling execute_frame_impl()
        try:
            self._execute_frame_impl()
        except Exception as e:
            spear.log("Exception: ", e)
            spear.log("ERROR: We are in a critical section, but we don't know how to get out of it from here. Attempting to call engine_service.end_frame...")
            self._end_frame_impl()
            self._frame_state = "error"
            raise e

    @contextlib.contextmanager
    def end_frame(self):

        # try executing post-frame work
        try:
            assert self.is_top_level_service() # user should only call this function on the top-level service
            assert self._frame_state == "executing_frame"
            self._frame_state = "executing_end_frame"
            yield
            assert self._frame_state == "executing_end_frame"
            self._frame_state = "request_end_frame"
        except Exception as e:
            spear.log("Exception: ", e)
            spear.log("ERROR: Attempting to exit critical section by calling engine_service.end_frame...")
            self._end_frame_impl()
            self._frame_state = "error"
            raise e

        # try calling end_frame_impl()
        try:
            self._end_frame_impl()
        except Exception as e:
            spear.log("Exception: ", e)
            spear.log("ERROR: We might or might not be in a critical section, but we don't know how to get out of it from here. Giving up...")
            self._frame_state = "error"
            raise e

        assert self._frame_state == "request_end_frame"
        self._frame_state = "idle"

    #
    # Helper functions to support initializing a spear.Instance
    #

    def get_byte_order(self):
        assert self.is_top_level_service()
        assert self._byte_order is not None
        return self._byte_order

    #
    # Miscellaneous low-level entry points to support initializing a spear.Instance
    #

    def ping(self):
        return self._entry_point_caller.call_on_worker_thread("ping", None)

    def get_id(self):
        return self._entry_point_caller.call_on_worker_thread("get_id", None)

    #
    # Miscellaneous low-level entry points that interact with Unreal globals
    #

    def get_engine(self):
        return self._entry_point_caller.call_on_worker_thread("get_engine", None)

    def is_with_editor(self):
        return self._entry_point_caller.call_on_worker_thread("is_with_editor", None)

    def is_running_commandlet(self):
        return self._entry_point_caller.call_on_worker_thread("is_running_commandlet", None)

    def is_async_loading(self):
        return self._entry_point_caller.call_on_game_thread("is_async_loading", None)

    #
    # Miscellaneous low-level entry points that interact with Unreal singleton structs and are needed in the
    # implementation of spear.Instance. We could implement UCLASSES and UFUNCTIONS to access the low-level
    # Unreal structs, but then we would be limited to accessing them on the game thread. Providing access
    # through these entry points enables access on the worker thread, which simplifies the implmeentation of
    # spear.Instance.
    #

    def get_command_line(self):
        return self._entry_point_caller.call_on_worker_thread("get_command_line", None)

    def request_exit(self, immediate_shutdown):
        self._entry_point_caller.call_on_worker_thread("request_exit", None, immediate_shutdown)

    #
    # Miscellaneous low-level entry points that interact with GEngine
    #

    def get_viewport_size(self):
        return self._entry_point_caller.call_on_worker_thread("get_viewport_size", None)

    #
    # Helper functions for managing the server's frame state. Note that _begin_frame_impl() and _execute_frame_impl()
    # both need to block to ensure that the client thread doesn't get too far ahead of the game thread, but _end_frame_impl()
    # does not. So we can call the server using send_async_fast(...) in _end_frame_impl(), which avoids all
    # blocking on the client thread.
    #

    def _begin_frame_impl(self):
        assert self.is_top_level_service() # this function should only be called from the top-level service
        self.call_sync_on_worker_thread("engine_service.call_sync_on_worker_thread.begin_frame")

    def _execute_frame_impl(self):
        assert self.is_top_level_service() # this function should only be called from the top-level service
        self.call_sync_on_worker_thread("engine_service.call_sync_on_worker_thread.execute_frame")

    def _end_frame_impl(self):
        assert self.is_top_level_service() # this function should only be called from the top-level service
        self.send_async_fast_on_worker_thread("engine_service.call_sync_on_worker_thread.end_frame")

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
        return_as = self._server_signature_descs["call_sync_on_worker_thread"][func_name].func_signature[0].type_names["entry_point"]
        assert return_as == "future"
        call_func_name = "_call_async_fast_on_worker_thread"
        return self._call_impl(return_as, call_func_name, func_name, *args)

    def send_async_fast_on_worker_thread(self, func_name, *args):
        self._validate_frame_state_for_worker_thread_work()
        return_as = self._server_signature_descs["call_sync_on_worker_thread"][func_name].func_signature[0].type_names["entry_point"]
        assert return_as == "void"
        call_func_name = "_send_async_fast_on_worker_thread"
        return self._call_impl(return_as, call_func_name, func_name, *args)

    def get_future_result_fast_on_worker_thread(self, future):
        self._validate_frame_state_for_worker_thread_work()
        return_as = future._return_as
        func_name = future._func_name
        get_future_result_func_name = "_get_future_result_fast_on_worker_thread_as_" + return_as
        return self._get_future_result_impl(return_as, get_future_result_func_name, future._future, func_name)

    # game thread

    def call_sync_on_game_thread(self, func_name, *args):
        self._validate_frame_state_for_game_thread_work()
        return_as = self._server_signature_descs["call_sync_on_game_thread"][func_name].func_signature[0].type_names["entry_point"]
        call_func_name = "_call_sync_on_game_thread_as_" + return_as
        return self._call_impl(return_as, call_func_name, func_name, *args)

    def call_async_on_game_thread(self, func_name, *args):
        self._validate_frame_state_for_game_thread_work()
        return_as = self._server_signature_descs["call_async_on_game_thread"][func_name].func_signature[0].type_names["entry_point"]
        assert return_as == "future"
        call_func_name = "_call_async_on_game_thread"
        return self._call_impl(return_as, call_func_name, func_name, *args)

    def send_async_on_game_thread(self, func_name, *args):
        self._validate_frame_state_for_game_thread_work()
        return_as = self._server_signature_descs["send_sync_on_game_thread"][func_name].func_signature[0].type_names["entry_point"]
        assert return_as == "void"
        call_func_name = "_send_async_on_game_thread"
        return self._call_impl(return_as, call_func_name, func_name, *args)

    def get_future_result_on_game_thread(self, future):
        self._validate_frame_state_for_worker_thread_work()
        return_as = future._return_as
        func_name = future._func_name
        get_future_result_func_name = "_get_future_result_on_game_thread_as_" + return_as
        return self._get_future_result_impl(return_as, get_future_result_func_name, future._future, func_name)

    def call_async_fast_on_game_thread(self, func_name, *args):
        self._validate_frame_state_for_game_thread_work()
        return_as = self._server_signature_descs["call_async_on_game_thread"][func_name].func_signature[0].type_names["entry_point"]
        assert return_as == "future"
        call_func_name = "_call_async_fast_on_game_thread"
        return self._call_impl(return_as, call_func_name, func_name, *args)

    def send_async_fast_on_game_thread(self, func_name, *args):
        self._validate_frame_state_for_game_thread_work()
        return_as = self._server_signature_descs["send_sync_on_game_thread"][func_name].func_signature[0].type_names["entry_point"]
        assert return_as == "void"
        call_func_name = "_send_async_fast_on_game_thread"
        return self._call_impl(return_as, call_func_name, func_name, *args)

    def get_future_result_fast_on_game_thread(self, future):
        self._validate_frame_state_for_worker_thread_work()
        return_as = future._return_as
        func_name = future._func_name
        get_future_result_func_name = "_get_future_result_fast_on_game_thread_as_" + return_as
        return self._get_future_result_impl(return_as, get_future_result_func_name, future._future, func_name)

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
                spear.log("ERROR: Attempting to exit critical section by calling engine_service.end_frame...")
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
