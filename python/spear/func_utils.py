#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import json
import numbers
import numpy as np
import scipy
import spear


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
    elif isinstance(obj, spear.Ptr):
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

# Convert a numpy array backed by shared memory to a Shared object that can be passed to sp_func_service.
def to_shared(array, shared_memory_name):
    return Shared(array, shared_memory_name)


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


# The Ptr and Shared classes are for internal use, and do not need to be instantiated directly by most users.

class Ptr:
    def __init__(self, handle):
        self._handle = handle

    def to_string(self):
        return f"{self._handle:#0{18}x}"

class Shared:
    def __init__(self, array, shared_memory_name):
        self.array = array
        self.shared_memory_name = shared_memory_name
