#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

#
# These classes are for internal use, and do not need to be instantiated directly by users.
#

class Ptr:
    def __init__(self, handle):
        self._handle = handle

    def to_string(self):
        return f"{self._handle:#0{18}x}"

class Shared:
    def __init__(self, array, shared_memory_name):
        self.array = array
        self.shared_memory_name = shared_memory_name

#
# Helper functions to convert arguments and return values that need to be handled specially.
#

# Convert a handle obtained from a service function into a Ptr object that can be passed as an argument to
# unreal_service.call_function(...) and sp_func_service.call_function(...).
def to_ptr(handle):
    return Ptr(handle)

# Convert a string returned by unreal_service.call_function(...) or sp_func_service.call_function(...) into a
# handle that can be passed as an argument to other service functions.
def to_handle(string):
    return int(string, 0)

# Convert a numpy array backed by shared memory to a Shared object that can be passed as an argument to
# sp_func_service.call_function(...).
def to_shared(array, shared_memory_name):
    return Shared(array, shared_memory_name)
