#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import spear

class SpFuncService(spear.utils.func_utils.Service):
    def __init__(self, entry_point_caller, shared_memory_service, create_children=True):

        self._entry_point_caller = entry_point_caller
        self._shared_memory_service = shared_memory_service
        super().__init__(entry_point_caller, create_children) # do this after initializing local state

    def create_child(self, entry_point_caller):
        return SpFuncService(entry_point_caller=entry_point_caller, shared_memory_service=self._shared_memory_service, create_children=False)

    def create_shared_memory_handles_for_object(self, uobject):
        views = self._entry_point_caller.call_on_game_thread("std::map<std::string, SharedMemoryView>", "get_shared_memory_views", None, uobject)
        return self._shared_memory_service.create_shared_memory_handles(shared_memory_views=views)

    def destroy_shared_memory_handles_for_object(self, shared_memory_handles):
        self._shared_memory_service.destroy_shared_memory_handles(shared_memory_handles=shared_memory_handles)

    # The caller must ensure that arrays and uobject_shared_memory_handles remain valid until future.get()
    # has been called for the future that might be returned by this function.

    def call_function(self, uobject, function_name, arrays={}, unreal_objs={}, info="", uobject_shared_memory_handles={}):

        # convert args to data bundle
        args_data_bundle = spear.utils.func_utils.to_data_bundle(
            dest_byte_order=self._entry_point_caller.engine_service.get_byte_order(),
            usage_flags=["Arg"],
            arrays=arrays,
            unreal_objs=unreal_objs,
            info=info)

        # define convert func
        def convert_func(
            result_data_bundle,
            arrays=arrays,
            uobject_shared_memory_handles=uobject_shared_memory_handles):

            # get the shared memory handle for each arg that uses shared memory and could be used as return value
            arg_shared_memory_handles = self._shared_memory_service.get_shared_memory_handles_from_arrays(
                arrays=arrays, usage_flags=["ReturnValue"])

            # get the shared memory names for all of the return values
            result_shared_memory_names = self._shared_memory_service.get_shared_memory_names_from_packed_arrays(
                packed_arrays=result_data_bundle.packed_arrays)

            # get the shared memory handles for all return values
            result_shared_memory_handles = self._shared_memory_service.get_shared_memory_handles_from_dicts(
                shared_memory_names=result_shared_memory_names,
                shared_memory_handle_dicts=[arg_shared_memory_handles, uobject_shared_memory_handles])

            # convert data bundle to result
            result = spear.utils.func_utils.to_data_bundle_dict(
                data_bundle=result_data_bundle,
                src_byte_order=self._entry_point_caller.engine_service.get_byte_order(),
                usage_flags=["ReturnValue"],
                shared_memory_handles=result_shared_memory_handles)

            return result

        return self._entry_point_caller.call_on_game_thread(
            "DataBundle",
            "call_function",
            convert_func,
            uobject,
            function_name,
            args_data_bundle)
