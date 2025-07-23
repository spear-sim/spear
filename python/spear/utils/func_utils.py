#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import json
import numbers
import numpy as np
import scipy
import spear

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


# Convert a collection of NumPy arrays to a collection of packed arrays.
def to_packed_arrays(arrays, dest_byte_order, usage_flags=None):
    if isinstance(arrays, list):
        return [ to_packed_array(array=a, dest_byte_order=dest_byte_order, usage_flags=usage_flags) for a in arrays ]
    elif isinstance(arrays, dict):
        return { k: to_packed_array(array=v, dest_byte_order=dest_byte_order, usage_flags=usage_flags) for k, v in arrays.items() }
    else:
        assert False

# Convert a NumPy array to a packed array.
def to_packed_array(array, dest_byte_order, usage_flags=None):
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


# Convert a collection of packed arrays to a collection of NumPy arrays.
def to_arrays(packed_arrays, src_byte_order, usage_flags=None, shared_memory_handles=None):
    if isinstance(packed_arrays, list):
        return [ to_array(packed_array=p, src_byte_order=src_byte_order, usage_flags=usage_flags, shared_memory_handles=shared_memory_handles) for p in packed_arrays ]
    elif isinstance(packed_arrays, dict):
        return { k: to_array(packed_array=v, src_byte_order=src_byte_order, usage_flags=usage_flags, shared_memory_handles=shared_memory_handles) for k, v in packed_arrays.items() }
    else:
        assert False

# Convert a packed array to a NumPy array.
def to_array(packed_array, src_byte_order, usage_flags=None, shared_memory_handles=None):
    if packed_array.data_source == "Internal":
        assert packed_array.data.shape == tuple(packed_array.shape)
        if src_byte_order == "native":
            return packed_array.data
        elif src_byte_order in ["big", "little"]:
            return np.frombuffer(packed_array.data.data, dtype=packed_array.data.dtype.newbyteorder(src_byte_order), count=-1).reshape(packed_array.shape)
        else:
            assert False
    elif packed_array.data_source == "Shared":
        assert packed_array.shared_memory_name in shared_memory_handles
        assert usage_flags is not None
        assert set(usage_flags) <= set(shared_memory_handles[packed_array.shared_memory_name]["view"].usage_flags)
        return np.ndarray(shape=packed_array.shape, dtype=packed_array.data.dtype, buffer=shared_memory_handles[packed_array.shared_memory_name]["buffer"])
    else:
        assert False


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
