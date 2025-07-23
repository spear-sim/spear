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
        self._frame_state = "idle"
        self._byte_order = None

        self.initialize() # explicitly initialize before calling begin_frame() for the first time
        self.get_byte_order() # pre-cache byte order because it will be constant for the life of the client

    #
    # Functions for managing the server's frame state.
    #

    def initialize(self):
        self._call("void", "engine_service.initialize")

    def close(self):
        self._call("void", "engine_service.close")

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
            self._call("void", "engine_service.begin_frame")
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
            self._call("void", "engine_service.execute_frame")
            self._call("void", "engine_service.end_frame")
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
            self._call("void", "engine_service.execute_frame")
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
            self._call("void", "engine_service.end_frame")
            self._frame_state = "idle"
            raise e

        assert self._frame_state == "executing_end_frame"
        self._frame_state = "request_end_frame"

        # try executing end_frame()
        try:
            self._call("void", "engine_service.end_frame")
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
    # Miscellaneous low-level entry points to support initializing a spear.Instance
    #

    def ping(self):
        return self.call_on_worker_thread("std::string", "engine_service.ping")

    def get_id(self):
        return self.call_on_worker_thread("int64_t", "engine_service.get_id")

    # Miscellaneous low-level entry points to support initializing a spear.services.EngineService

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
    # Helper functions to call server functions
    #

    def _validate_frame_state_for_game_thread_work(self):
        if self._frame_state not in ["executing_begin_frame", "executing_end_frame"]:
            spear.log('ERROR: Calling entry points that execute on the game thread is only allowed in "with begin_frame()" and "with end_frame()" code blocks.')
            spear.log(f'ERROR: self._frame_state == "{self._frame_state}"')

            if self._frame_state == "finished_executing_begin_frame":
                spear.log("ERROR: Attempting to exit critical section...")
                self._call("engine_service.execute_frame")
                self._call("engine_service.end_frame")
                self._frame_state = "idle"

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
