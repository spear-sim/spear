#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import json
import numbers
import numpy as np
import scipy

try:
    import spear_ext # can't be installed in the UE Python environment because UE doesn't ship with CPython headers
except:
    pass


# These classes are for internal use, and do not need to be instantiated directly by most users.

class Ptr:
    def __init__(self, handle):
        self._handle = handle

    def to_string(self):
        return f"{self._handle:#0{18}x}"

class Shared:
    def __init__(self, array, shared_memory_handle):
        self.array = array
        self.shared_memory_handle = shared_memory_handle

class Future:
    def __init__(self, future, return_as, get_future_result_func, convert_func):
        self._future = future
        self._return_as = return_as
        self._get_future_result_func = get_future_result_func
        self._convert_func = convert_func

    def get(self):
        result = self._get_future_result_func(return_as=self._return_as, future=self._future)
        if self._convert_func is not None:
            return self._convert_func(result)
        else:
            return result

# EntryPointCaller and each of its derived types encapsulates a strategy for calling EngineService functions.

class EntryPointCaller():
    def __init__(self, service_name, engine_service):
        self.service_name = service_name
        self.engine_service = engine_service

    def call_on_game_thread(self, return_as, func_name, convert_func, *args):
        long_func_name = self.service_name + "." + func_name
        return_value = self.engine_service.call_on_game_thread(return_as, long_func_name, *args)
        if convert_func is not None:
            return convert_func(return_value)
        else:
            return return_value

    def call_on_worker_thread(self, return_as, func_name, *args):
        long_func_name = self.service_name + "." + func_name
        return self.engine_service.call_on_worker_thread(return_as, long_func_name, *args)

class CallAsyncEntryPointCaller(EntryPointCaller):
    def call_on_game_thread(self, return_as, func_name, convert_func, *args):
        long_func_name = self.service_name + "." + func_name
        future = self.engine_service.call_async_on_game_thread(long_func_name, *args)
        return Future(future=future, return_as=return_as, get_future_result_func=self.engine_service.get_future_result, convert_func=convert_func)

class SendAsyncEntryPointCaller(EntryPointCaller):
    def call_on_game_thread(self, return_as, func_name, convert_func, *args):
        long_func_name = self.service_name + "." + func_name
        self.engine_service.send_async_on_game_thread(long_func_name, *args)

class CallAsyncFastEntryPointCaller(EntryPointCaller):
    def call_on_game_thread(self, return_as, func_name, convert_func, *args):
        long_func_name = self.service_name + "." + func_name
        future = self.engine_service.call_async_fast_on_game_thread(long_func_name, *args)
        return Future(future=future, return_as=return_as, get_future_result_func=self.engine_service.get_future_result_fast, convert_func=convert_func)

class SendAsyncFastEntryPointCaller(EntryPointCaller):
    def call_on_game_thread(self, return_as, func_name, convert_func, *args):
        long_func_name = self.service_name + "." + func_name
        self.engine_service.send_async_fast_on_game_thread(long_func_name, *args)

# Service is a base class for services that define their own child services for calling EntryPointCaller functions.

class Service():
    def __init__(self, entry_point_caller, create_children=False):

        if create_children:
            call_async_service_name = entry_point_caller.service_name + ".call_async"
            call_async_entry_point_caller = CallAsyncEntryPointCaller(service_name=call_async_service_name, engine_service=entry_point_caller.engine_service)
            self.call_async = self.create_child(entry_point_caller=call_async_entry_point_caller)

            send_async_service_name = entry_point_caller.service_name + ".send_async"
            send_async_entry_point_caller = SendAsyncEntryPointCaller(service_name=send_async_service_name, engine_service=entry_point_caller.engine_service)
            self.send_async = self.create_child(entry_point_caller=send_async_entry_point_caller)

            call_async_fast_service_name = entry_point_caller.service_name + ".call_async"
            call_async_fast_entry_point_caller = CallAsyncFastEntryPointCaller(service_name=call_async_fast_service_name, engine_service=entry_point_caller.engine_service)
            self.call_async_fast = self.create_child(entry_point_caller=call_async_fast_entry_point_caller)

            send_async_fast_service_name = entry_point_caller.service_name + ".send_async"
            send_async_fast_entry_point_caller = SendAsyncFastEntryPointCaller(service_name=send_async_service_name, engine_service=entry_point_caller.engine_service)
            self.send_async_fast = self.create_child(entry_point_caller=send_async_fast_entry_point_caller)

    def create_child(self, entry_point_caller):
        return None


# Convert a collection of objects to a collection of JSON strings so they can be passed to a service.
def to_json_strings(objs):
    if isinstance(objs, list):
        return [ to_json_string(obj=o) for o in objs ]
    elif isinstance(objs, dict):
        return { k: to_json_string(obj=v) for k, v in objs.items() }
    else:
        assert False

# Convert an object to a JSON string so it can be passed to a service.
def to_json_string(obj, stringify=True):
    if isinstance(obj, str):
        return obj
    elif isinstance(obj, Ptr):
        return obj.to_string()
    elif isinstance(obj, numbers.Number) or isinstance(obj, list) or isinstance(obj, dict):
        if stringify:
            if isinstance(obj, numbers.Number):
                return json.dumps(obj)
            if isinstance(obj, list):
                return json.dumps([ to_json_string(obj=o, stringify=False) for o in obj ])
            elif isinstance(obj, dict):
                return json.dumps({ to_json_string(obj=k, stringify=True): to_json_string(obj=v, stringify=False) for k, v in obj.items() })
            else:
                assert False
        else:
            if isinstance(obj, numbers.Number):
                return obj
            elif isinstance(obj, list):
                return [ to_json_string(obj=o, stringify=False) for o in obj ]
            elif isinstance(obj, dict):
                return { to_json_string(obj=k, stringify=True): to_json_string(obj=v, stringify=False) for k, v in obj.items() }
            else:
                assert False
    else:
        return json.dumps(obj)


# Convert a collection of JSON strings returned by a service to a collection of dicts.
def try_to_dicts(json_strings, default_value=None):
    if isinstance(json_strings, list):
        return [ try_to_dict(json_string=s, default_value=default_value) for s in json_strings ]
    elif isinstance(json_strings, dict):
        return { k: try_to_dict(json_string=v, default_value=default_value) for k, v in json_strings.items() }
    else:
        assert False

# Convert a JSON string returned by a service to a dict.
def try_to_dict(json_string, default_value=None):
    try:
        return json.loads(json_string)
    except:
        if default_value is None:
            return json_string
        else:
            return default_value


# Convert a handle into a Ptr object that can be passed to a service.
def to_ptr(handle):
    return Ptr(handle)

# Convert a string returned by a service into a handle that can be passed a service.
def to_handle(string):
    assert string.startswith("0x")
    return int(string, 16)


# Convert a NumPy array backed by shared memory to a Shared object that can be passed to a service.
def to_shared(array, shared_memory_handle):
    return Shared(array, shared_memory_handle)


# Convert a NumPy array to a packed array.
def to_packed_array(array, dest_byte_order, usage_flags):
    if isinstance(array, np.ndarray):
        packed_array = spear_ext.PackedArray()
        if dest_byte_order == "native":
            packed_array.data = array
        elif dest_byte_order in ["big", "little"]:
            packed_array.data = np.array(array, dtype=array.dtype.newbyteorder(dest_byte_order))
        else:
            assert False
        packed_array.data_source = "Internal"
        packed_array.shape = array.shape
        packed_array.shared_memory_name = ""
        return packed_array
    elif isinstance(array, Shared):
        assert usage_flags is not None
        assert set(usage_flags) <= set(array.shared_memory_handle["view"].usage_flags)
        packed_array = spear_ext.PackedArray()
        packed_array.data = np.array([], dtype=array.array.dtype)
        packed_array.data_source = "Shared"
        packed_array.shape = array.array.shape
        packed_array.shared_memory_name = array.shared_memory_handle["name"]
        return packed_array
    else:
        assert False

# Convert a packed array to a NumPy array.
def to_array(packed_array, src_byte_order, usage_flags, shared_memory_handles):
    if packed_array.data_source == "Internal":
        assert packed_array.data.shape == tuple(packed_array.shape)
        if src_byte_order == "native":
            return packed_array.data
        elif src_byte_order in ["big", "little"]:
            return np.frombuffer(packed_array.data.data, dtype=packed_array.data.dtype.newbyteorder(src_byte_order), count=-1).reshape(packed_array.shape)
        else:
            assert False
    elif packed_array.data_source == "Shared":
        assert shared_memory_handles is not None
        assert packed_array.shared_memory_name in shared_memory_handles
        assert usage_flags is not None
        assert set(usage_flags) <= set(shared_memory_handles[packed_array.shared_memory_name]["view"].usage_flags)
        return np.ndarray(shape=packed_array.shape, dtype=packed_array.data.dtype, buffer=shared_memory_handles[packed_array.shared_memory_name]["buffer"])
    else:
        assert False


# Convert a collection of NumPy arrays to a collection of packed arrays.
def to_packed_arrays(arrays, dest_byte_order, usage_flags):
    if isinstance(arrays, list):
        return [ to_packed_array(array=a, dest_byte_order=dest_byte_order, usage_flags=usage_flags) for a in arrays ]
    elif isinstance(arrays, dict):
        return { k: to_packed_array(array=v, dest_byte_order=dest_byte_order, usage_flags=usage_flags) for k, v in arrays.items() }
    else:
        assert False

# Convert a collection of packed arrays to a collection of NumPy arrays.
def to_arrays(packed_arrays, src_byte_order, usage_flags, shared_memory_handles):
    if isinstance(packed_arrays, list):
        return [ to_array(packed_array=p, src_byte_order=src_byte_order, usage_flags=usage_flags, shared_memory_handles=shared_memory_handles) for p in packed_arrays ]
    elif isinstance(packed_arrays, dict):
        return { k: to_array(packed_array=v, src_byte_order=src_byte_order, usage_flags=usage_flags, shared_memory_handles=shared_memory_handles) for k, v in packed_arrays.items() }
    else:
        assert False


# Convert a collection of arrays, unreal objects, and an info string to a data bundle.
def to_data_bundle(dest_byte_order, usage_flags, arrays={}, unreal_objs={}, info=""):
    assert dest_byte_order is not None
    assert usage_flags is not None
    data_bundle = spear_ext.DataBundle()
    data_bundle.packed_arrays = to_packed_arrays(arrays=arrays, dest_byte_order=dest_byte_order, usage_flags=["Arg"])
    data_bundle.unreal_obj_strings = to_json_strings(objs=unreal_objs)
    data_bundle.info = info
    return data_bundle

# Convert a data bundle to a collection of arrays, unreal objects, and an info string.
def to_data_bundle_dict(data_bundle, src_byte_order, usage_flags, shared_memory_handles):
    return {
        "arrays": to_arrays(packed_arrays=data_bundle.packed_arrays, src_byte_order=src_byte_order, usage_flags=["ReturnValue"], shared_memory_handles=shared_memory_handles),
        "unreal_objs": try_to_dicts(json_strings=data_bundle.unreal_obj_strings),
        "info": data_bundle.info}


# Convert to a NumPy matrix from an Unreal rotator. See pipeline.py for more details on Unreal's Euler angle conventions.
def to_matrix_from_rotator(rotator):
    assert isinstance(rotator, dict)
    assert set(["roll", "pitch", "yaw"]) == set(rotator.keys())
    roll  = np.deg2rad(-rotator["roll"])
    pitch = np.deg2rad(-rotator["pitch"])
    yaw   = np.deg2rad(rotator["yaw"])
    return np.matrix(scipy.spatial.transform.Rotation.from_euler("xyz", [roll, pitch, yaw]).as_matrix())

# Convert from a NumPy array or matrix to an Unreal vector.
def to_vector_from_array(array):
    if isinstance(array, np.matrix):
        assert array.shape == (3, 1)
        array = array.A1
    elif isinstance(array, np.ndarray):
        assert array.shape == (3,)
    else:
        assert False
    return {"X": array[0], "Y": array[1], "Z": array[2]}
