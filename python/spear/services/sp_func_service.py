#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import copy
import mmap
import multiprocessing.shared_memory
import numpy as np
import spear
import sys

try:
    import spear_ext # can't be installed in the UE Python environment because UE doesn't ship with CPython headers
except:
    pass


class SpFuncService():
    def __init__(self, entry_point_caller, shared_memory_service, create_children=True):

        self._entry_point_caller = entry_point_caller
        self._shared_memory_service = shared_memory_service

        if create_children:
            call_async_service_name = self._entry_point_caller.service_name + ".call_async"
            call_async_entry_point_caller = spear.services.entry_point_caller.CallAsyncEntryPointCaller(service_name=call_async_service_name, engine_service=self._entry_point_caller.engine_service)
            self.call_async = SpFuncService(entry_point_caller=call_async_entry_point_caller, shared_memory_service=shared_memory_service, create_children=False)

            send_async_service_name = self._entry_point_caller.service_name + ".send_async"
            send_async_entry_point_caller = spear.services.entry_point_caller.SendAsyncEntryPointCaller(service_name=send_async_service_name, engine_service=self._entry_point_caller.engine_service)
            self.send_async = SpFuncService(entry_point_caller=send_async_entry_point_caller, shared_memory_service=shared_memory_service, create_children=False)

            call_async_fast_service_name = self._entry_point_caller.service_name + ".call_async"
            call_async_fast_entry_point_caller = spear.services.entry_point_caller.CallAsyncFastEntryPointCaller(service_name=call_async_fast_service_name, engine_service=self._entry_point_caller.engine_service)
            self.call_async_fast = SpFuncService(entry_point_caller=call_async_fast_entry_point_caller, shared_memory_service=shared_memory_service, create_children=False)

            send_async_fast_service_name = self._entry_point_caller.service_name + ".send_async"
            send_async_fast_entry_point_caller = spear.services.entry_point_caller.SendAsyncFastEntryPointCaller(service_name=send_async_service_name, engine_service=self._entry_point_caller.engine_service)
            self.send_async_fast = SpFuncService(entry_point_caller=send_async_fast_entry_point_caller, shared_memory_service=shared_memory_service, create_children=False)


    def create_shared_memory_handles_for_object(self, uobject):
        views = self._entry_point_caller.call_on_game_thread("std::map<std::string, SharedMemoryView>", "get_shared_memory_views", None, uobject)
        return self._shared_memory_service.create_shared_memory_handles(shared_memory_views=views)


    def destroy_shared_memory_handles_for_object(self, shared_memory_handles):
        self._shared_memory_service.destroy_shared_memory_handles(shared_memory_handles=shared_memory_handles)


    def call_function(self, uobject, function_name, arrays={}, unreal_objs={}, info="", uobject_shared_memory_handles={}):

        # convert args to data bundle
        args_data_bundle = spear_ext.DataBundle()
        args_data_bundle.packed_arrays = spear.utils.func_utils.to_packed_arrays(
            arrays=arrays,
            dest_byte_order=self._entry_point_caller.engine_service.get_byte_order(),
            usage_flags=["Arg"])
        args_data_bundle.unreal_obj_strings = spear.utils.func_utils.to_json_strings(
            objs=unreal_objs)
        args_data_bundle.info = info

        # get the shared memory handle for each arg that uses shared memory and could be used as return value
        arg_shared_memory_handles = self._shared_memory_service.get_shared_memory_handles_from_arrays(
            arrays=arrays,
            usage_flags=["ReturnValue"])

        # arg_shared_memory_handles and uobject_shared_memory_handles are both dicts that map from a string
        # to a handle, where a handle is itself a dict that maps from a string to various low-level buffer
        # metadata objects. The caller must ensure that all of the underlying handles corresponding to arrays
        # and uobject_shared_memory_handles remain valid until future.get() has been called for the future
        # that might be returned by this function.

        # define convert func
        def convert_func(
            result_data_bundle,
            arg_shared_memory_handles=arg_shared_memory_handles,
            uobject_shared_memory_handles=uobject_shared_memory_handles):

            result_shared_memory_names = self._shared_memory_service.get_shared_memory_names_from_packed_arrays(
                packed_arrays=result_data_bundle.packed_arrays)
            result_shared_memory_handles = self._shared_memory_service.get_shared_memory_handles_from_dicts(
                shared_memory_names=result_shared_memory_names,
                shared_memory_handle_dicts=[arg_shared_memory_handles, uobject_shared_memory_handles])

            # convert data bundle to result
            result = {
                "arrays": spear.utils.func_utils.to_arrays(
                    packed_arrays=result_data_bundle.packed_arrays,
                    src_byte_order=self._entry_point_caller.engine_service.get_byte_order(),
                    usage_flags=["ReturnValue"],
                    shared_memory_handles=result_shared_memory_handles),
                "unreal_objs": spear.utils.func_utils.try_to_dicts(
                    json_strings=result_data_bundle.unreal_obj_strings),
                "info": result_data_bundle.info}

            return result

        return self._entry_point_caller.call_on_game_thread(
            "DataBundle",
            "call_function",
            convert_func,
            uobject,
            function_name,
            args_data_bundle)
