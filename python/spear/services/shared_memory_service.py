#
# Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import mmap
import multiprocessing.shared_memory
import spear
import sys

class SharedMemoryService(spear.Service):
    def __init__(self, entry_point_caller):

        self._shared_memory_handles = {}

        # do this after initializing local state
        super().__init__(entry_point_caller=entry_point_caller, parent_service=None, create_children_services=False)

    #
    # High-level functions for managing shared memory regions.
    #

    def create_shared_memory_region(self, shared_memory_name, num_bytes, usage_flags):
        assert shared_memory_name not in self._shared_memory_handles.keys()
        view = self.entry_point_caller.call_on_worker_thread("create_shared_memory_region", None, shared_memory_name, num_bytes, usage_flags)
        handle = self._create_shared_memory_handle(shared_memory_name=shared_memory_name, shared_memory_view=view)
        self._shared_memory_handles[shared_memory_name] = handle
        return handle

    def destroy_shared_memory_region(self, shared_memory_handle):
        assert self.is_top_level_service() # user should only call this function on the top-level service
        assert shared_memory_handle["name"] in self._shared_memory_handles.keys()
        self._destroy_shared_memory_handle(shared_memory_handle=shared_memory_handle)
        self._shared_memory_handles.pop(shared_memory_handle["name"])
        self.entry_point_caller.call_on_worker_thread("destroy_shared_memory_region", None, shared_memory_handle["name"])

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

    def get_shared_memory_handles_from_arrays(self, arrays, usage_flags):
        return { a._shared_memory_handle["name"]: a._shared_memory_handle for a in arrays if isinstance(a, spear.Shared) and set(usage_flags) <= set(a._shared_memory_handle["view"].usage_flags) }

    def get_shared_memory_names_from_packed_arrays(self, packed_arrays):
        return [ v.shared_memory_name for k, v in packed_arrays.items() if v.data_source == "Shared" ]

    def get_shared_memory_handles_from_dicts(self, shared_memory_names, shared_memory_handle_dicts):
        shared_memory_handles = {}
        for shared_memory_name in shared_memory_names:
            shared_memory_handle = None
            for shared_memory_handle_dict in shared_memory_handle_dicts:
                if shared_memory_name in shared_memory_handle_dict.keys():
                    assert shared_memory_handle is None
                    shared_memory_handle = shared_memory_handle_dict[shared_memory_name]
            assert shared_memory_handle is not None
            shared_memory_handles[shared_memory_name] = shared_memory_handle
        return shared_memory_handles

    #
    # Low-level helper functions for interacting with shared memory. Most users will not need to call these
    # functions directly.
    #

    def _create_shared_memory_handle(self, shared_memory_name, shared_memory_view):
        if sys.platform == "win32":
            handle = mmap.mmap(fileno=-1, length=shared_memory_view.num_bytes + shared_memory_view.offset_bytes, tagname=shared_memory_view.id)
            buffer = memoryview(handle)[shared_memory_view.offset_bytes:]
            return {"name": shared_memory_name, "handle": handle, "buffer": buffer, "view": shared_memory_view}
        elif sys.platform in ["darwin", "linux"]:
            handle = multiprocessing.shared_memory.SharedMemory(name=shared_memory_view.id)
            buffer = memoryview(handle.buf)[shared_memory_view.offset_bytes:]
            multiprocessing.resource_tracker.unregister(handle._name, "shared_memory") # prevent Python from destroying on exit
            return {"name": shared_memory_name, "handle": handle, "buffer": buffer, "view": shared_memory_view}
        else:
            assert False

    def _destroy_shared_memory_handle(self, shared_memory_handle):
        if sys.platform == "win32":
            shared_memory_handle["buffer"].release()
            shared_memory_handle["handle"].close()
        elif sys.platform in ["darwin", "linux"]:
            shared_memory_handle["buffer"].release()
            shared_memory_handle["handle"].close()
        else:
            assert False
