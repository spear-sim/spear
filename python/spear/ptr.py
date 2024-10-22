#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# The Ptr class is for internal use, and does not need to be instantiated directly by users.
class Ptr:
    def __init__(self, handle):
        self._handle = handle

    def to_string(self):
        return f"{self._handle:#0{18}x}"

# Convert a handle obtained from a service function into a ptr that can be passed as an argument to
# unreal_service.call_function(...) and sp_func_service.call_function(...).
def to_ptr(handle):
    return Ptr(handle)

# Convert a ptr returned by unreal_service.call_function(...) or sp_func_service.call_function(...)
# into a handle that can be passed as an argument to other service functions.
def to_handle(string):
    return int(string, 0)
