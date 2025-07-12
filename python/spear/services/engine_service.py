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

        self.get_byte_order() # pre-cache byte order because it will be constant for the life of the client

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
            self.call_on_worker_thread("engine_service.end_frame")
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

    def call_on_game_thread(self, func_name, *args):
        self._validate_frame_state()
        return self._call(func_name, *args)

    def call_on_worker_thread(self, func_name, *args):
        return self._call(func_name, *args)

    def call_on_game_thread_and_get_return_value(self, return_as, func_name, *args):
        self._validate_frame_state()
        return self._call_and_get_return_value(return_as, func_name, *args)

    def call_on_worker_thread_and_get_return_value(self, return_as, func_name, *args):
        return self._call_and_get_return_value(return_as, func_name, *args)

    def _validate_frame_state(self):
        if self._frame_state not in ["executing_begin_frame", "executing_end_frame"]:
            spear.log('ERROR: Calling entry points that execute on the game thread is only allowed in "with begin_frame()" and "with end_frame()" code blocks.')
            spear.log(f'ERROR: self._frame_state == "{self._frame_state}"')

            if self._frame_state == "finished_executing_begin_frame":
                spear.log("ERROR: Attempting to exit critical section...")
                self._call("engine_service.execute_frame")
                self._call("engine_service.end_frame")
                self._frame_state = "idle"

            assert False

    def _call(self, func_name, *args):
        if self._config.SPEAR.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO:
            spear.log("Calling:               ", func_name, args)

        self._client.call(func_name, *args)

        if self._config.SPEAR.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO:
            spear.log("No return value.")

    def _call_and_get_return_value(self, return_as, func_name, *args):
        if self._config.SPEAR.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO:
            spear.log("Calling:               ", func_name, args, " -> ", return_as)

        if return_as == "bool":
            return_value = self._client.call_and_get_return_value_as_bool(func_name, *args)
        elif return_as == "float":
            return_value = self._client.call_and_get_return_value_as_float(func_name, *args)
        elif return_as == "int32_t":
            return_value = self._client.call_and_get_return_value_as_int32(func_name, *args)
        elif return_as == "uint32_t":
            return_value = self._client.call_and_get_return_value_as_uint32(func_name, *args)
        elif return_as == "uint64_t":
            return_value = self._client.call_and_get_return_value_as_uint64(func_name, *args)
        elif return_as == "std::string":
            return_value = self._client.call_and_get_return_value_as_string(func_name, *args)
        elif return_as == "std::vector<uint64_t>":
            return_value = self._client.call_and_get_return_value_as_vector_of_uint64(func_name, *args)
        elif return_as == "std::vector<double>":
            return_value = self._client.call_and_get_return_value_as_vector_of_double(func_name, *args)
        elif return_as == "std::vector<std::string>":
            return_value = self._client.call_and_get_return_value_as_vector_of_string(func_name, *args)
        elif return_as == "std::map<std::string, uint64_t>":
            return_value = self._client.call_and_get_return_value_as_map_of_string_to_uint64(func_name, *args)
        elif return_as == "std::map<std::string, std::string>":
            return_value = self._client.call_and_get_return_value_as_map_of_string_to_string(func_name, *args)
        elif return_as == "std::map<std::string, SharedMemoryView>":
            return_value = self._client.call_and_get_return_value_as_map_of_string_to_shared_memory_view(func_name, *args)
        elif return_as == "std::map<std::string, PackedArray>":
            return_value = self._client.call_and_get_return_value_as_map_of_string_to_packed_array(func_name, *args)
        elif return_as == "SharedMemoryView":
            return_value = self._client.call_and_get_return_value_as_shared_memory_view(func_name, *args)
        elif return_as == "PackedArray":
            return_value = self._client.call_and_get_return_value_as_packed_array(func_name, *args)
        elif return_as == "DataBundle":
            return_value = self._client.call_and_get_return_value_as_data_bundle(func_name, *args)
        elif return_as == "PropertyDesc":
            return_value = self._client.call_and_get_return_value_as_property_desc(func_name, *args)
        else:
            spear.log("ERROR: Unrecognized return type when calling: ", func_name, args, " -> ", return_as)
            assert False

        if self._config.SPEAR.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO:
            spear.log("Obtained return value: ", return_value)

        return return_value

    # Miscellaneous low-level entry points.

    def ping(self):
        return self.call_on_worker_thread_and_get_return_value("std::string", "engine_service.ping")

    def initialize(self):
        self.call_on_worker_thread("engine_service.initialize")

    def request_exit(self):
        self.call_on_worker_thread("engine_service.request_exit")

    def with_editor(self):
        return self.call_on_worker_thread_and_get_return_value("bool", "engine_service.with_editor")

    def get_byte_order(self):
        if self._byte_order is None:
            byte_order = self.call_on_worker_thread_and_get_return_value("std::string", "engine_service.get_byte_order")
            if byte_order == sys.byteorder:
                self._byte_order = "native"
            else:
                self._byte_order = unreal_instance_byte_order
        return self._byte_order

    # Entry points for miscellaneous functions that are accessible via GEngine.

    def get_viewport_size(self):
        return self.call_on_worker_thread_and_get_return_value("std::vector<double>", "engine_service.get_viewport_size")
