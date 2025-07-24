#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import contextlib
import spear
import sys

class EngineService():
    def __init__(self, client, config):
        self._client = client
        self._config = config
        self._byte_order = None

        self.get_byte_order() # cache byte order because it will be constant for the life of the client
        self.initialize() # explicitly initialize before calling begin_frame() for the first time

    #
    # Functions for managing the server's frame state.
    #

    def initialize(self):
        self._frame_state = "idle"
        self.call_on_worker_thread("void", "engine_service.initialize")

    def close(self):
        self.call_on_worker_thread("void", "engine_service.close")

    #
    # These context managers are intended as exception-safe wrappers for managing the server's frame state.
    #

    @contextlib.contextmanager
    def begin_frame(self):

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
    # Miscellaneous low-level entry points to support initializing a spear.Instance
    #

    def ping(self):
        return self.call_on_worker_thread("std::string", "engine_service.ping")

    def get_id(self):
        return self.call_on_worker_thread("int64_t", "engine_service.get_id")

    #
    # Miscellaneous low-level entry points to support initializing a spear.services.EngineService
    #

    def get_byte_order(self):
        if self._byte_order is None:
            byte_order = self.call_on_worker_thread("std::string", "engine_service.get_byte_order")
            if byte_order == sys.byteorder:
                self._byte_order = "native"
            else:
                self._byte_order = unreal_instance_byte_order
        return self._byte_order

    #
    # Miscellaneous low-level entry points that interact with Unreal globals
    #

    def is_with_editor(self):
        return self.call_on_worker_thread("bool", "engine_service.is_with_editor")

    def is_running_commandlet(self):
        return self.call_on_worker_thread("bool", "engine_service.is_running_commandlet")

    def get_command_line(self):
        return self.call_on_worker_thread("std::string", "engine_service.get_command_line")

    def request_exit(self, immediate_shutdown):
        self.call_on_worker_thread("void", "engine_service.request_exit", immediate_shutdown)

    # Miscellaneous low-level entry points that interact with GEngine

    def get_viewport_size(self):
        return self.call_on_worker_thread("std::vector<double>", "engine_service.get_viewport_size")

    #
    # Functions for calling entry points on the server.
    #

    def call_on_game_thread(self, return_as, func_name, *args):
        self._validate_frame_state_for_game_thread_work()
        return self._call(return_as, func_name, *args)

    def call_on_worker_thread(self, return_as, func_name, *args):
        return self._call(return_as, func_name, *args)

    def call_async_on_game_thread(self, func_name, *args):
        self._validate_frame_state_for_game_thread_work()
        return self._call_async(func_name, *args)

    def call_async_on_worker_thread(self, func_name, *args):
        return self._call_async(func_name, *args)

    def send_async_on_game_thread(self, func_name, *args):
        self._validate_frame_state_for_game_thread_work()
        self._send_async(func_name, *args)

    def send_async_on_worker_thread(self, func_name, *args):
        self._send_async(func_name, *args)

    def get_future_result(self, return_as, future):
        return self._get_future_result(return_as=return_as, future=future)

    def call_async_fast_on_game_thread(self, func_name, *args):
        self._validate_frame_state_for_game_thread_work()
        return self._call_async_fast(func_name, *args)

    def call_async_fast_on_worker_thread(self, func_name, *args):
        return self._call_async_fast(func_name, *args)

    def send_async_fast_on_game_thread(self, func_name, *args):
        self._validate_frame_state_for_game_thread_work()
        self._send_async_fast(func_name, *args)

    def send_async_fast_on_worker_thread(self, func_name, *args):
        self._send_async_fast(func_name, *args)

    def get_future_result_fast(self, return_as, future):
        return self._get_future_result_fast(return_as=return_as, future=future)

    #
    # Helper functions for managing the server's frame state. Note that _begin_frame_impl() and _execute_frame_impl()
    # both need to block to ensure that the client thread doesn't get too far ahead of the game thread, but _end_frame_impl()
    # does not. So we can call the server using send_async_fast(...) in _end_frame_impl(), which avoids all
    # blocking on the client thread.
    #

    def _begin_frame_impl(self):
        self.call_on_worker_thread("void", "engine_service.begin_frame")

    def _execute_frame_impl(self):
        self.call_on_worker_thread("void", "engine_service.execute_frame")

    def _end_frame_impl(self):
        self.send_async_fast_on_worker_thread("engine_service.end_frame")

    #
    # Helper functions for calling entry points on the server
    #

    def _validate_frame_state_for_game_thread_work(self):
        if self._frame_state not in ["executing_begin_frame", "executing_end_frame"]:
            spear.log('ERROR: Calling entry points that execute on the game thread is only allowed in "with begin_frame()" and "with end_frame()" code blocks.')
            spear.log(f'ERROR: self._frame_state == "{self._frame_state}"')

            if self._frame_state == "executing_frame":
                spear.log("ERROR: Attempting to exit critical section by calling engine_service.end_frame...")
                self._end_frame_impl()
                self._frame_state = "error"

            assert False

    def _call(self, return_as, func_name, *args):
        if self._config.SPEAR.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO:
            spear.log("Calling: ", func_name, args, " -> ", return_as)

        if return_as == "void":
            return_value = self._client.call_as_void(func_name, *args)
        elif return_as == "bool":
            return_value = self._client.call_as_bool(func_name, *args)
        elif return_as == "float":
            return_value = self._client.call_as_float(func_name, *args)
        elif return_as == "int32_t":
            return_value = self._client.call_as_int32(func_name, *args)
        elif return_as == "int64_t":
            return_value = self._client.call_as_int64(func_name, *args)
        elif return_as == "uint64_t":
            return_value = self._client.call_as_uint64(func_name, *args)
        elif return_as == "std::string":
            return_value = self._client.call_as_string(func_name, *args)
        elif return_as == "std::vector<uint64_t>":
            return_value = self._client.call_as_vector_of_uint64(func_name, *args)
        elif return_as == "std::vector<double>":
            return_value = self._client.call_as_vector_of_double(func_name, *args)
        elif return_as == "std::vector<std::string>":
            return_value = self._client.call_as_vector_of_string(func_name, *args)
        elif return_as == "std::map<std::string, uint64_t>":
            return_value = self._client.call_as_map_of_string_to_uint64(func_name, *args)
        elif return_as == "std::map<std::string, std::string>":
            return_value = self._client.call_as_map_of_string_to_string(func_name, *args)
        elif return_as == "std::map<std::string, SharedMemoryView>":
            return_value = self._client.call_as_map_of_string_to_shared_memory_view(func_name, *args)
        elif return_as == "std::map<std::string, PackedArray>":
            return_value = self._client.call_as_map_of_string_to_packed_array(func_name, *args)
        elif return_as == "PropertyDesc":
            return_value = self._client.call_as_property_desc(func_name, *args)
        elif return_as == "SharedMemoryView":
            return_value = self._client.call_as_shared_memory_view(func_name, *args)
        elif return_as == "PackedArray":
            return_value = self._client.call_as_packed_array(func_name, *args)
        elif return_as == "DataBundle":
            return_value = self._client.call_as_data_bundle(func_name, *args)
        else:
            spear.log("ERROR: Unrecognized return type when calling: ", func_name, args, " -> ", return_as)
            assert False

        if self._config.SPEAR.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO:
            spear.log("Obtained return value: ", return_value)

        return return_value

    def _call_async(self, func_name, *args):
        if self._config.SPEAR.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO:
            spear.log("Calling asynchronously: ", func_name, args, " -> Future")

        future = self._client.call_async(func_name, *args)

        if self._config.SPEAR.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO:
            spear.log("Obtained future: ", future)

        return future

    def _send_async(self, func_name, *args):
        if self._config.SPEAR.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO:
            spear.log("Sending message asynchronously: ", func_name, args)

        self._client.send_async(func_name, *args)

        if self._config.SPEAR.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO:
            spear.log("No return value.")

    def _get_future_result(self, return_as, future):
        if self._config.SPEAR.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO:
            spear.log("Getting result for future: ", future, " -> ", return_as)

        if return_as == "void":
            return_value = self._client.get_future_result_as_void(future)
        if return_as == "bool":
            return_value = self._client.get_future_result_as_bool(future)
        elif return_as == "float":
            return_value = self._client.get_future_result_as_float(future)
        elif return_as == "int32_t":
            return_value = self._client.get_future_result_as_int32(future)
        elif return_as == "int64_t":
            return_value = self._client.get_future_result_as_int64(future)
        elif return_as == "uint64_t":
            return_value = self._client.get_future_result_as_uint64(future)
        elif return_as == "std::string":
            return_value = self._client.get_future_result_as_string(future)
        elif return_as == "std::vector<uint64_t>":
            return_value = self._client.get_future_result_as_vector_of_uint64(future)
        elif return_as == "std::vector<double>":
            return_value = self._client.get_future_result_as_vector_of_double(future)
        elif return_as == "std::vector<std::string>":
            return_value = self._client.get_future_result_as_vector_of_string(future)
        elif return_as == "std::map<std::string, uint64_t>":
            return_value = self._client.get_future_result_as_map_of_string_to_uint64(future)
        elif return_as == "std::map<std::string, std::string>":
            return_value = self._client.get_future_result_as_map_of_string_to_string(future)
        elif return_as == "std::map<std::string, SharedMemoryView>":
            return_value = self._client.get_future_result_as_map_of_string_to_shared_memory_view(future)
        elif return_as == "std::map<std::string, PackedArray>":
            return_value = self._client.get_future_result_as_map_of_string_to_packed_array(future)
        elif return_as == "PropertyDesc":
            return_value = self._client.get_future_result_as_property_desc(future)
        elif return_as == "SharedMemoryView":
            return_value = self._client.get_future_result_as_shared_memory_view(future)
        elif return_as == "PackedArray":
            return_value = self._client.get_future_result_as_packed_array(future)
        elif return_as == "DataBundle":
            return_value = self._client.get_future_result_as_data_bundle(future)
        else:
            spear.log("ERROR: Unrecognized return type when getting result for future: ", future, " -> ", return_as)
            assert False

        if self._config.SPEAR.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO:
            spear.log("Obtained return value: ", return_value)

        return return_value

    def _call_async_fast(self, func_name, *args):
        if self._config.SPEAR.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO:
            spear.log("Calling asynchronously (fast): ", func_name, args, " -> Future")

        future = self._client.call_async_fast(func_name, *args)

        if self._config.SPEAR.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO:
            spear.log("Obtained future: ", future)

        return future

    def _send_async_fast(self, func_name, *args):
        if self._config.SPEAR.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO:
            spear.log("Sending message asynchronously (fast): ", func_name, args)

        self._client.send_async_fast(func_name, *args)

        if self._config.SPEAR.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO:
            spear.log("No return value.")

    def _get_future_result_fast(self, return_as, future):
        if self._config.SPEAR.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO:
            spear.log("Getting result for future (fast): ", future, " -> ", return_as)

        if return_as == "void":
            return_value = self._client.get_future_result_fast_as_void(future)
        if return_as == "bool":
            return_value = self._client.get_future_result_fast_as_bool(future)
        elif return_as == "float":
            return_value = self._client.get_future_result_fast_as_float(future)
        elif return_as == "int32_t":
            return_value = self._client.get_future_result_fast_as_int32(future)
        elif return_as == "int64_t":
            return_value = self._client.get_future_result_fast_as_int64(future)
        elif return_as == "uint64_t":
            return_value = self._client.get_future_result_fast_as_uint64(future)
        elif return_as == "std::string":
            return_value = self._client.get_future_result_fast_as_string(future)
        elif return_as == "std::vector<uint64_t>":
            return_value = self._client.get_future_result_fast_as_vector_of_uint64(future)
        elif return_as == "std::vector<double>":
            return_value = self._client.get_future_result_fast_as_vector_of_double(future)
        elif return_as == "std::vector<std::string>":
            return_value = self._client.get_future_result_fast_as_vector_of_string(future)
        elif return_as == "std::map<std::string, uint64_t>":
            return_value = self._client.get_future_result_fast_as_map_of_string_to_uint64(future)
        elif return_as == "std::map<std::string, std::string>":
            return_value = self._client.get_future_result_fast_as_map_of_string_to_string(future)
        elif return_as == "std::map<std::string, SharedMemoryView>":
            return_value = self._client.get_future_result_fast_as_map_of_string_to_shared_memory_view(future)
        elif return_as == "std::map<std::string, PackedArray>":
            return_value = self._client.get_future_result_fast_as_map_of_string_to_packed_array(future)
        elif return_as == "PropertyDesc":
            return_value = self._client.get_future_result_fast_as_property_desc(future)
        elif return_as == "SharedMemoryView":
            return_value = self._client.get_future_result_fast_as_shared_memory_view(future)
        elif return_as == "PackedArray":
            return_value = self._client.get_future_result_fast_as_packed_array(future)
        elif return_as == "DataBundle":
            return_value = self._client.get_future_result_fast_as_data_bundle(future)
        else:
            spear.log("ERROR: Unrecognized return type when getting result for future: ", future, " -> ", return_as)
            assert False

        if self._config.SPEAR.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO:
            spear.log("Obtained return value: ", return_value)

        return return_value
