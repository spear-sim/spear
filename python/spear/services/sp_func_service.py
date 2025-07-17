#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

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
    def __init__(self, entry_point_caller, shared_memory_service):
        self._entry_point_caller = entry_point_caller
        self._shared_memory_service = shared_memory_service


    def create_shared_memory_handles_for_object(self, uobject):
        views = self._entry_point_caller.call_on_game_thread_and_get_return_value("std::map<std::string, SharedMemoryView>", "sp_func_service.get_shared_memory_views", uobject)
        return self._shared_memory_service.create_shared_memory_handles(shared_memory_views=views)

    def destroy_shared_memory_handles_for_object(self, shared_memory_handles):
        self._shared_memory_service.destroy_shared_memory_handles(shared_memory_handles=shared_memory_handles)


    def call_function(self, uobject, function_name, arrays={}, unreal_objs={}, info="", uobject_shared_memory_handles={}):

        # convert args to data bundle
        args_data_bundle = spear_ext.DataBundle()
        args_data_bundle.packed_arrays = spear.utils.func_utils.to_packed_arrays(
            arrays=arrays,
            dest_byte_order=self._entry_point_caller.get_byte_order(),
            usage_flags=["Arg"])
        args_data_bundle.unreal_obj_strings = spear.utils.func_utils.to_json_strings(
            objs=unreal_objs)
        args_data_bundle.info = info

        # call function
        return_values_data_bundle = self._entry_point_caller.call_on_game_thread_and_get_converted_return_value("DataBundle", "sp_func_service.call_function", uobject, function_name, args_data_bundle)

        # get the shared memory handle for each return value
        arg_shared_memory_handles = self._shared_memory_service.get_shared_memory_handles_from_arrays(
            arrays=arrays,
            usage_flags=["ReturnValue"])
        return_value_shared_memory_names = self._shared_memory_service.get_shared_memory_names_from_packed_arrays(
            packed_arrays=return_values_data_bundle.packed_arrays)
        return_value_shared_memory_handles = self._shared_memory_service.get_shared_memory_handles_from_dicts(
            shared_memory_names=return_value_shared_memory_names,
            shared_memory_handle_dicts=[arg_shared_memory_handles, uobject_shared_memory_handles])

        # convert data bundle to return values
        return_values = {
            "arrays": spear.utils.func_utils.to_arrays(
                packed_arrays=return_values_data_bundle.packed_arrays,
                src_byte_order=self._entry_point_caller.get_byte_order(),
                usage_flags=["ReturnValue"],
                shared_memory_handles=return_value_shared_memory_handles),
            "unreal_objs": spear.utils.func_utils.try_to_dicts(
                json_strings=return_values_data_bundle.unreal_obj_strings),
            "info": return_values_data_bundle.info}

        return return_values
