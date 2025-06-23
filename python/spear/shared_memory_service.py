#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import mmap
import multiprocessing.shared_memory
import spear
import spear.func_utils
import sys

class SharedMemoryService():
    def __init__(self, entry_point_caller):
        self._entry_point_caller = entry_point_caller
        self._shared_memory_handles = {}

        unreal_instance_byte_order = self._entry_point_caller.call_on_worker_thread("shared_memory_service.get_byte_order")
        if unreal_instance_byte_order == sys.byteorder:
            self.unreal_instance_byte_order = "native"
        else:
            self.unreal_instance_byte_order = unreal_instance_byte_order

    #
    # High-level functions for managing shared memory regions.
    #

    def create_shared_memory_region(self, shared_memory_name, num_bytes, usage_flags):
        assert shared_memory_name not in self._shared_memory_handles.keys()
        view = self._entry_point_caller.call_on_worker_thread("shared_memory_service.create_shared_memory_region", shared_memory_name, num_bytes, usage_flags)
        handle = self._create_shared_memory_handle(shared_memory_name=shared_memory_name, shared_memory_view=view)
        self._shared_memory_handles[shared_memory_name] = handle
        return handle

    def destroy_shared_memory_region(self, shared_memory_handle):
        assert shared_memory_handle["name"] in self._shared_memory_handles.keys()
        self._destroy_shared_memory_handle(shared_memory_handle=shared_memory_handle)
        self._shared_memory_handles.pop(shared_memory_handle["name"])
        self._entry_point_caller.call_on_worker_thread("shared_memory_service.destroy_shared_memory_region", shared_memory_handle["name"])

    #
    # High-level functions for interacting with shared memory that is managed internally in C++.
    #

    def create_shared_memory_handles(self, shared_memory_views):
        handles = {}
        for shared_memory_name, shared_memory_view in shared_memory_views.items():
            handles[shared_memory_name] = self._create_shared_memory_handle(shared_memory_name=shared_memory_name, shared_memory_view=shared_memory_view)
        return handles

    def destroy_shared_memory_handles(self, shared_memory_handles):
        for shared_memory_name, shared_memory_handle in shared_memory_handles.items():
            self._destroy_shared_memory_handle(shared_memory_handle=shared_memory_handle)

    #
    # High-level helper functions for interacting with shared memory handles
    #

    def get_shared_memory_handles_from_arrays(self, arrays, usage_flags=[]):
        return { a.shared_memory_handle["name"]: a.shared_memory_handle for a in arrays if isinstance(a, spear.func_utils.Shared) and set(usage_flags) <= set(a.shared_memory_handle["view"]["usage_flags"]) }

    def get_shared_memory_names_from_packed_arrays(self, packed_arrays):
        return [ v["shared_memory_name"] for k, v in packed_arrays.items() if v["data_source"] == "Shared" ]

    def get_shared_memory_handles_from_dicts(self, shared_memory_names, shared_memory_handle_dicts):
        shared_memory_handles = {}
        for shared_memory_name in shared_memory_names:
            shared_memory_handle = None
            for shared_memory_handles in shared_memory_handle_dicts:
                if shared_memory_name in shared_memory_handles.keys():
                    assert shared_memory_handle is None
                    shared_memory_handle = shared_memory_handles[shared_memory_name]
            shared_memory_handles[shared_memory_name] = shared_memory_handle
        return shared_memory_handles

    #
    # Low-level helper functions for interacting with shared memory. Most users will not need to call these
    # functions directly.
    #

    def _create_shared_memory_handle(self, shared_memory_name, shared_memory_view):
        if sys.platform == "win32":
            handle = mmap.mmap(fileno=-1, length=shared_memory_view["num_bytes"], tagname=shared_memory_view["id"])
            return {"name": shared_memory_name, "handle": handle, "buffer": handle, "view": shared_memory_view}
        elif sys.platform in ["darwin", "linux"]:
            handle = multiprocessing.shared_memory.SharedMemory(name=shared_memory_view["id"])
            multiprocessing.resource_tracker.unregister(handle._name, "shared_memory") # prevent Python from destroying on exit
            return {"name": shared_memory_name, "handle": handle, "buffer": handle.buf, "view": shared_memory_view}
        else:
            assert False

    def _destroy_shared_memory_handle(self, shared_memory_handle):
        if sys.platform == "win32":
            shared_memory_handle["handle"].close()
        elif sys.platform in ["darwin", "linux"]:
            shared_memory_handle["handle"].close()
        else:
            assert False
