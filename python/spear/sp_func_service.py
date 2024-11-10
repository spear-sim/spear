#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import mmap
import multiprocessing.shared_memory
import numpy as np
import spear
import sys

class SpFuncService():
    def __init__(self, entry_point_caller):
        self._entry_point_caller = entry_point_caller
        self._shared_memory_handles = {}

        unreal_instance_byte_order = self._entry_point_caller.call("sp_func_service.get_byte_order")
        if unreal_instance_byte_order == sys.byteorder:
            self._unreal_instance_byte_order = "native"
        else:
            self._unreal_instance_byte_order = unreal_instance_byte_order

    #
    # High-level functions for interacting with shared memory.
    #

    def create_shared_memory_region(self, num_bytes, shared_memory_name):
        assert shared_memory_name not in self._shared_memory_handles.keys()
        view = self._entry_point_caller.call("sp_func_service.create_shared_memory_region", num_bytes, shared_memory_name)
        handle = self.create_shared_memory_handle(shared_memory_view=view)
        self._shared_memory_handles[shared_memory_name] = handle
        return handle

    def destroy_shared_memory_region(self, shared_memory_name):
        assert shared_memory_name in self._shared_memory_handles.keys()
        self.destroy_shared_memory_handle(shared_memory_handle=self._shared_memory_handles[shared_memory_name])
        self._shared_memory_handles.pop(shared_memory_name)
        self._entry_point_caller.call("sp_func_service.destroy_shared_memory_region", shared_memory_name)

    def create_shared_memory_handles_for_uobject(self, uobject):
        views = self._entry_point_caller.call("sp_func_service.get_shared_memory_views", uobject)
        handles = {}
        for name, view in views.items():
            handles[name] = self.create_shared_memory_handle(shared_memory_view=view)
        return handles

    def destroy_shared_memory_handles_for_uobject(self, shared_memory_handles):
        for name, handle in shared_memory_handles.items():
            self.destroy_shared_memory_handle(shared_memory_handle=handle)

    #
    # Call function interface.
    #

    def call_function(self, uobject, function_name, arrays={}, unreal_objs={}, info="", uobject_shared_memory_handles={}):

        # If an arg is a numpy array, then convert it to a packed array that uses Internal storage with the
        # Unreal instance's native byte order. If an arg is a spear.Shared object, then convert it to a
        # packed array that uses Shared storage.
        packed_arrays = {}
        for array_name, array in arrays.items():
            if isinstance(array, np.ndarray):
                packed_array = {
                    "data": np.array(array, dtype=array.dtype.newbyteorder(self._unreal_instance_byte_order)).data,
                    "data_source": "Internal",
                    "shape": array.shape,
                    "data_type": array.dtype.str.replace("<", "").replace(">", ""),
                    "shared_memory_name": ""}
            elif isinstance(array, spear.Shared):
                # assume that the handle for the array is in self._shared_memory_handles
                assert "Arg" in self._shared_memory_handles[array.shared_memory_name]["view"]["usage_flags"]
                packed_array = {
                    "data": np.array([]).data,
                    "data_source": "Shared",
                    "shape": array.array.shape,
                    "data_type": array.array.dtype.str.replace("<", "").replace(">", ""),
                    "shared_memory_name": array.shared_memory_name}
            else:
                assert False
            packed_arrays[array_name] = packed_array

        # Call function.
        args = {"packed_arrays": packed_arrays, "unreal_obj_strings": spear.to_json_strings(objs=unreal_objs), "info": info}
        return_values = self._entry_point_caller.call("sp_func_service.call_function", uobject, function_name, args)

        # If a return value is backed by Internal storage, then convert to a numpy array using the packed
        # array's data. If a return value is backed by Shared storage, then convert to a numpy array using
        # shared memory.
        arrays = {}
        for packed_array_name, packed_array in return_values["packed_arrays"].items():            
            if packed_array["data_source"] == "Internal":
                dtype = np.dtype(packed_array["data_type"]).newbyteorder(self._unreal_instance_byte_order)
                array = np.frombuffer(packed_array["data"], dtype=dtype, count=-1).reshape(packed_array["shape"])
            elif packed_array["data_source"] == "Shared":
                # assume that the handle for the array is uobject_shared_memory_handles
                assert packed_array["shared_memory_name"] in uobject_shared_memory_handles
                assert "ReturnValue" in uobject_shared_memory_handles[packed_array["shared_memory_name"]]["view"]["usage_flags"]
                buffer = uobject_shared_memory_handles[packed_array["shared_memory_name"]]["buffer"]
                array = np.ndarray(shape=packed_array["shape"], dtype=np.dtype(packed_array["data_type"]), buffer=buffer)
            else:
                assert False
            arrays[packed_array_name] = array

        # Return all converted data.
        return {"arrays": arrays, "unreal_objs": spear.try_to_dicts(json_strings=return_values["unreal_obj_strings"]), "info": return_values["info"]}

    #
    # Low-level helper functions for interacting with shared memory. Most users will not need to call these
    # functions directly.
    #

    def create_shared_memory_handle(self, shared_memory_view):
        if sys.platform == "win32":
            handle = mmap.mmap(fileno=-1, length=shared_memory_view["num_bytes"], tagname=shared_memory_view["id"])
            return {"handle": handle, "buffer": handle, "view": shared_memory_view}
        elif sys.platform in ["darwin", "linux"]:
            handle = multiprocessing.shared_memory.SharedMemory(name=shared_memory_view["id"])
            multiprocessing.resource_tracker.unregister(handle._name, "shared_memory") # prevent Python from destroying on exit
            return {"handle": handle, "buffer": handle.buf, "view": shared_memory_view}
        else:
            assert False

    def destroy_shared_memory_handle(self, shared_memory_handle):
        if sys.platform == "win32":
            shared_memory_handle["handle"].close()
        elif sys.platform in ["darwin", "linux"]:
            shared_memory_handle["handle"].close()
        else:
            assert False

    def get_shared_memory_views(self, uobject):
        return self._entry_point_caller.call("sp_func_service.get_shared_memory_views", uobject)
