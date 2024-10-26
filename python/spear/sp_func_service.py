#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import json
import mmap
import multiprocessing.shared_memory
import numpy as np
import spear
import sys

class SpFuncService():
    def __init__(self, entry_point_caller):
        self._entry_point_caller = entry_point_caller

        unreal_instance_byte_order = self._entry_point_caller.call("sp_func_service.get_byte_order")
        if unreal_instance_byte_order == sys.byteorder:
            self._unreal_instance_byte_order = "native"
        else:
            self._unreal_instance_byte_order = unreal_instance_byte_order

    #
    # Call function interface.
    #

    def call_function(self, uobject, function_name, arrays={}, unreal_objs={}, info="", shared_memory_handles={}):

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
                assert "Arg" in shared_memory_handles[array.shared_memory_name]["view"]["usage_flags"]
                packed_array = {
                    "data": np.array([]).data,
                    "data_source": "Shared",
                    "shape": array.array.shape,
                    "data_type": array.array.dtype.str.replace("<", "").replace(">", ""),
                    "shared_memory_name": array.shared_memory_name}
            else:
                assert False
            packed_arrays[array_name] = packed_array

        # If an arg is a string, then don't convert. If an arg is a spear.Ptr, then use spear.Ptr.to_string()
        # to convert. If an arg is any other type, then assume it is valid JSON and use json.dumps(...) to
        # convert.
        unreal_obj_strings = {}
        for unreal_obj_name, unreal_obj in unreal_objs.items():
            if isinstance(unreal_obj, str):
                unreal_obj_string = unreal_obj
            elif isinstance(unreal_obj, spear.Ptr):
                unreal_obj_string = unreal_obj.to_string()
            else:
                unreal_obj_string = json.dumps(unreal_obj)
            unreal_obj_strings[unreal_obj_name] = unreal_obj_string

        # Call function.
        args = {"packed_arrays": packed_arrays, "unreal_obj_strings": unreal_obj_strings, "info": info}
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
                assert "ReturnValue" in shared_memory_handles[packed_array["shared_memory_name"]]["view"]["usage_flags"]
                buffer = shared_memory_handles[packed_array["shared_memory_name"]]["buffer"]
                array = np.ndarray(shape=packed_array["shape"], dtype=np.dtype(packed_array["data_type"]), buffer=buffer)
            else:
                assert False
            arrays[packed_array_name] = array

        # Try to parse each return value string as JSON, and if that doesn't work, then return the string
        # directly. If the returned string is intended to be a handle, then the user can get it as a handle
        # by calling spear.to_handle(...).
        unreal_objs = {}
        for unreal_obj_name, unreal_obj_string in return_values["unreal_obj_strings"].items():
            try:
                unreal_obj = json.loads(unreal_obj_string)
            except:
                unreal_obj = unreal_obj_string
            unreal_objs[unreal_obj_name] = unreal_obj

        # Return all converted data.
        return {"arrays": arrays, "unreal_objs": unreal_objs, "info": return_values["info"]}

    #
    # Helper functions for interacting with shared memory.
    #

    def create_shared_memory_handles(self, uobject):
        views = self._entry_point_caller.call("sp_func_service.get_shared_memory_views", uobject)
        handles = {}
        for name, view in views.items():
            handles[name] = self.create_shared_memory_handle(view)
        return handles

    def destroy_shared_memory_handles(self, shared_memory_handles):
        for name, handle in shared_memory_handles.items():
            self.destroy_shared_memory_handle(handle)

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

    #
    # Low-level helper function for getting shared memory views. Most users will not need to call this
    # function directly.
    #

    def get_shared_memory_views(self, uobject):
        return self._entry_point_caller.call("sp_func_service.get_shared_memory_views", uobject)
