#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import json
import numbers
import numpy as np
import scipy
import spear

if spear.__can_import_spear_ext__:
    import spear_ext


#
# ServiceBase is a low-level base class for Services and ServiceWrappers
#

class ServiceBase():
    def __init__(self, sp_func_service=None, unreal_service=None, config=None):

        # need to be extra careful to avoid naming conflicts here

        if sp_func_service is not None:
            self._private_sp_func_service = sp_func_service.get_top_level_service()
        else:
            self._private_sp_func_service = None

        if unreal_service is not None:
            self._private_unreal_service = unreal_service.get_top_level_service()
        else:
            self._private_unreal_service = None

        if config is not None:
            self._private_config = config
        else:
            self._private_config = None

    def to_handle_or_unreal_struct(self, obj, as_handle=None, as_unreal_struct=None):
        assert self._private_unreal_service is not None
        return spear.to_handle_or_unreal_struct(
            obj=obj,
            unreal_service=self._private_unreal_service,
            as_handle=as_handle,
            as_unreal_struct=as_unreal_struct)

    def to_handle_or_unreal_class(self, obj, as_handle=None, as_unreal_class=None):
        assert self._private_unreal_service is not None
        return spear.to_handle_or_unreal_class(
            obj=obj,
            unreal_service=self._private_unreal_service,
            as_handle=as_handle,
            as_unreal_class=as_unreal_class)

    def to_handle_or_unreal_object(self, obj, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        assert self._private_sp_func_service is not None
        assert self._private_unreal_service is not None
        assert self._private_config is not None
        return spear.to_handle_or_unreal_object(
            obj=obj,
            unreal_service=self._private_unreal_service,
            sp_func_service=self._private_sp_func_service,
            config=self._private_config,
            as_handle=as_handle,
            as_unreal_object=as_unreal_object,
            with_sp_funcs=with_sp_funcs)

#
# A Service represents a collection of RPC entry points.
#

class Service(ServiceBase):
    def __init__(self, entry_point_caller, sp_func_service=None, unreal_service=None, config=None, parent_service=None, create_children_services=False):
        super().__init__(sp_func_service=sp_func_service, unreal_service=unreal_service, config=config)

        if parent_service is not None:
            assert not create_children_services # it is only allowed to create children services from the top-level service

        self.entry_point_caller = entry_point_caller
        self._parent_service = parent_service

        if create_children_services:
            assert entry_point_caller is not None

            call_async_entry_point_caller = CallAsyncEntryPointCaller(service_name=entry_point_caller._service_name, engine_service=entry_point_caller.engine_service)
            self.call_async = self.create_child_service(entry_point_caller=call_async_entry_point_caller, sp_func_service=sp_func_service, unreal_service=unreal_service, config=config)

            send_async_entry_point_caller = SendAsyncEntryPointCaller(service_name=entry_point_caller._service_name, engine_service=entry_point_caller.engine_service)
            self.send_async = self.create_child_service(entry_point_caller=send_async_entry_point_caller, sp_func_service=sp_func_service, unreal_service=unreal_service, config=config)

            call_async_fast_entry_point_caller = CallAsyncFastEntryPointCaller(service_name=entry_point_caller._service_name, engine_service=entry_point_caller.engine_service)
            self.call_async_fast = self.create_child_service(entry_point_caller=call_async_fast_entry_point_caller, sp_func_service=sp_func_service, unreal_service=unreal_service, config=config)

            send_async_fast_entry_point_caller = SendAsyncFastEntryPointCaller(service_name=entry_point_caller._service_name, engine_service=entry_point_caller.engine_service)
            self.send_async_fast = self.create_child_service(entry_point_caller=send_async_fast_entry_point_caller, sp_func_service=sp_func_service, unreal_service=unreal_service, config=config)

    def create_child_service(self, entry_point_caller, sp_func_service=None, unreal_service=None, config=None):
        assert False # if a derived class passes create_children_services=True into the base constructor, then the derived class must override create_child_service(...)

    def is_top_level_service(self):
        return self._parent_service is None

    def get_top_level_service(self):
        service = self
        while service._parent_service is not None:
            service = service._parent_service
        return service

#
# A ServiceWrapper wraps a low-level service and provides additional game-or-editor-scoped functionality,
# especially the ability to return UnrealObjects.
#

class ServiceWrapper(ServiceBase):
    def __init__(self, service, sp_func_service, unreal_service, config, parent_service_wrapper=None, create_children_service_wrappers=True):
        super().__init__(sp_func_service=sp_func_service, unreal_service=unreal_service, config=config)

        assert sp_func_service.is_top_level_service()
        assert unreal_service.is_top_level_service()

        if parent_service_wrapper is None:
            assert service.is_top_level_service() # the top-level service should be passed in when creating the top-level wrapper
        if parent_service_wrapper is not None:
            assert not create_children_service_wrappers # it is only allowed to create children services from the top-level service

        self.service = service
        self.sp_func_service = sp_func_service
        self.unreal_service = unreal_service
        self.config = config
        self._parent_service_wrapper = parent_service_wrapper

        if create_children_service_wrappers:
            assert hasattr(service, "call_async")
            assert hasattr(service, "send_async")
            assert hasattr(service, "call_async_fast")
            assert hasattr(service, "send_async_fast")

            self.call_async = self.create_child_service_wrapper(service=service.call_async)
            self.send_async = self.create_child_service_wrapper(service=service.send_async)
            self.call_async_fast = self.create_child_service_wrapper(service=service.call_async_fast)
            self.send_async_fast = self.create_child_service_wrapper(service=service.send_async_fast)

    def create_child_service_wrapper(self, service):
        assert False

    def is_top_level_service_wrapper(self):
        return self._parent_service_wrapper is None


#
# Each type that derives from EntryPointCaller encapsulates a particular strategy for calling EngineService functions.
#

class EntryPointCaller():
    def __init__(self, service_name, engine_service):
        self.engine_service = engine_service
        self._service_name = service_name

    def call_on_worker_thread(self, func_name, convert_func, *args):
        assert False

    def call_on_game_thread(self, func_name, convert_func, *args):
        assert False

    def get(self, obj):
        assert False

    def _get_return_as_string_for_worker_thread(self, func_name):
        long_func_name = f"{self._service_name}.call_sync_on_worker_thread.{func_name}"
        return self.engine_service.get_server_signature_descs()["call_sync_on_worker_thread"][long_func_name].func_signature[0].type_names["entry_point"]

    def _get_return_as_string_for_game_thread(self, func_name):
        long_func_name = f"{self._service_name}.call_sync_on_game_thread.{func_name}"
        return self.engine_service.get_server_signature_descs()["call_sync_on_game_thread"][long_func_name].func_signature[0].type_names["entry_point"]

    def __repr__(self):
        return f"{self.__class__.__name__}(_service_name={self._service_name})"

class CallSyncEntryPointCaller(EntryPointCaller):
    def call_on_worker_thread(self, func_name, convert_func, *args):
        long_func_name = f"{self._service_name}.call_sync_on_worker_thread.{func_name}"
        return_value = self.engine_service.call_sync_on_worker_thread(long_func_name, *args)
        if convert_func is not None:
            return convert_func(return_value)
        else:
            return return_value

    def call_on_game_thread(self, func_name, convert_func, *args):
        long_func_name = f"{self._service_name}.call_sync_on_game_thread.{func_name}"
        return_value = self.engine_service.call_sync_on_game_thread(long_func_name, *args)
        if convert_func is not None:
            return convert_func(return_value)
        else:
            return return_value

    def get(self, obj):
        return obj

class CallAsyncEntryPointCaller(EntryPointCaller):
    def call_on_worker_thread(self, func_name, convert_func, *args):
        assert False # worker thread entry points always execute synchronously unless we're using the fast path, so we should never be here

    def call_on_game_thread(self, func_name, convert_func, *args):
        long_func_name = f"{self._service_name}.call_async_on_game_thread.{func_name}"
        return_as = self._get_return_as_string_for_game_thread(func_name)
        future = self.engine_service.call_async_on_game_thread(long_func_name, *args) # non-fast path returns spear_ext.Future
        get_future_result_func = self.engine_service.get_future_result_from_game_thread
        return Future(future=future, get_future_result_func=get_future_result_func, convert_func=convert_func, return_as=return_as, func_name=long_func_name)

    def get(self, obj):
        spear.log_current_function()
        return Future(future=None, get_future_result_func=lambda future: obj, convert_func=None, return_as=None, func_name=None)

class SendAsyncEntryPointCaller(EntryPointCaller):
    def call_on_worker_thread(self, func_name, convert_func, *args):
        assert False # worker thread entry points always execute synchronously unless we're using the fast path, so we should never be here

    def call_on_game_thread(self, func_name, convert_func, *args):
        long_func_name = f"{self._service_name}.send_async_on_game_thread.{func_name}"
        self.engine_service.send_async_on_game_thread(long_func_name, *args)

class CallAsyncFastEntryPointCaller(EntryPointCaller):
    def call_on_worker_thread(self, func_name, convert_func, *args):
        long_func_name = f"{self._service_name}.call_sync_on_worker_thread.{func_name}" # fast path calls call_sync variant for worker thread entry points
        return_as = self._get_return_as_string_for_worker_thread(func_name)
        future = self.engine_service.call_async_fast_on_worker_thread(long_func_name, *args) # fast path returns integer
        get_future_result_func = self.engine_service.get_future_result_fast_from_worker_thread
        return Future(future=future, get_future_result_func=get_future_result_func, convert_func=convert_func, return_as=return_as, func_name=long_func_name)

    def call_on_game_thread(self, func_name, convert_func, *args):
        long_func_name = f"{self._service_name}.call_async_on_game_thread.{func_name}" # fast path calls call_async variant for game thread entry points
        return_as = self._get_return_as_string_for_game_thread(func_name)
        future = self.engine_service.call_async_fast_on_game_thread(long_func_name, *args) # fast path returns integer
        get_future_result_func = self.engine_service.get_future_result_fast_from_game_thread
        return Future(future=future, get_future_result_func=get_future_result_func, convert_func=convert_func, return_as=return_as, func_name=long_func_name)

    def get(self, obj):
        spear.log_current_function()
        return Future(future=None, get_future_result_func=lambda future: obj, convert_func=None, return_as=None, func_name=None)

class SendAsyncFastEntryPointCaller(EntryPointCaller):
    def call_on_worker_thread(self, func_name, convert_func, *args):
        long_func_name = f"{self._service_name}.call_sync_on_worker_thread.{func_name}" # fast path calls call_sync variant for worker thread entry points
        self.engine_service.send_async_fast_on_worker_thread(long_func_name, *args)

    def call_on_game_thread(self, func_name, convert_func, *args):
        long_func_name = f"{self._service_name}.send_async_on_game_thread.{func_name}" # fast path calls send_async variant for game thread entry points
        self.engine_service.send_async_fast_on_game_thread(long_func_name, *args)


#
# These classes are for internal use, and do not need to be instantiated directly by most users.
#

class Ptr:
    def __init__(self, handle):
        self._handle = handle

    def to_string(self):
        return f"{self._handle:#0{18}x}"

    def __repr__(self):
        return f"Ptr(_handle={self._handle})"

class Shared:
    def __init__(self, array, shared_memory_handle):
        self._array = array
        self._shared_memory_handle = shared_memory_handle

    def __repr__(self):
        return f"Shared(_array={self._array}, _shared_memory_handle={self._shared_memory_handle})"

class PropertyValue:
    def __init__(self, value, type_id):
        self.value = value
        self.type_id = type_id

    def __repr__(self):
        return f"PropertyValue(value={self.value}, type_id={self.type_id})"

class Future:
    def __init__(self, future, get_future_result_func, convert_func, return_as, func_name):
        self.future = future
        self.convert_func = convert_func
        self._get_future_result_func = get_future_result_func
        self._return_as = return_as
        self._func_name = func_name

    def get(self):
        result = self._get_future_result_func(future=self)
        if self.convert_func is not None:
            return self.convert_func(result)
        else:
            return result

    def __repr__(self):
        return f'Future(future={self.future}, convert_func={self.convert_func is not None}, _return_as="{self._return_as}", _func_name="{self._func_name}")'


#
# Conversion functions
#

# Convert to handle or UnrealStruct
def to_handle_or_unreal_struct(obj, unreal_service, as_handle=None, as_unreal_struct=None):
    if as_handle is not None:
        assert as_unreal_struct is None
        return to_handle(obj=obj)
    else:
        return to_unreal_struct(obj=obj, unreal_service=unreal_service)

# Convert to handle or UnrealClass
def to_handle_or_unreal_class(obj, unreal_service, as_handle=None, as_unreal_class=None):
    if as_handle is not None:
        assert as_unreal_class is None
        return to_handle(obj=obj)
    else:
        return to_unreal_class(obj=obj, unreal_service=unreal_service)

# Convert to handle or UnrealObject
def to_handle_or_unreal_object(obj, unreal_service, sp_func_service, config, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
    if as_handle is not None:
        assert as_unreal_object is None
        assert with_sp_funcs is None
        return to_handle(obj=obj)
    else:
        return to_unreal_object(obj=obj, unreal_service=unreal_service, sp_func_service=sp_func_service, config=config, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

# Convert to UnrealStruct
def to_unreal_struct(obj, unreal_service):
    return to_unreal_type(obj=obj, create_func=lambda obj: spear.UnrealStruct(unreal_service=unreal_service, ustruct=obj))

# Convert to UnrealClass
def to_unreal_class(obj, unreal_service):
    return to_unreal_type(obj=obj, create_func=lambda obj: spear.UnrealClass(unreal_service=unreal_service, uclass=obj))

# Convert to UnrealObject
def to_unreal_object(obj, unreal_service, sp_func_service, config, as_unreal_object=None, with_sp_funcs=None):
    if as_unreal_object is None:
        as_unreal_object = True
    if with_sp_funcs is None:
        with_sp_funcs = False
    if isinstance(as_unreal_object, bool):
        assert as_unreal_object
        uclass = None
    elif isinstance(as_unreal_object, numbers.Integral):
        assert as_unreal_object != 0
        uclass = as_unreal_object
    elif isinstance(as_unreal_object, str):
        assert as_unreal_object != ""
        uclass = as_unreal_object
    else:
        assert False
    return to_unreal_type(obj=obj, create_func=lambda obj: spear.UnrealObject(unreal_service=unreal_service, sp_func_service=sp_func_service, config=config, uobject=obj, uclass=uclass, with_sp_funcs=with_sp_funcs))

# Convert to UnrealStruct or UnrealClass
def to_unreal_type(obj, create_func):
    if isinstance(obj, bool):
        assert False
    elif isinstance(obj, numbers.Integral):
        return create_func(obj=obj)
    elif isinstance(obj, list):
        return [ to_unreal_type(obj=o, create_func=create_func) for o in obj ]
    elif isinstance(obj, dict):
        return { k: to_unreal_type(obj=v, create_func=create_func) for k, v in obj.items() }
    elif isinstance(obj, spear.Future):
        inner_convert_func = obj.convert_func
        def convert_func(o):
            if inner_convert_func is not None:
                o = inner_convert_func(o)
            return to_unreal_type(obj=o, create_func=create_func)
        obj.convert_func = convert_func
        return obj
    else:
        assert False

# Convert to handle
def to_handle(obj):
    if isinstance(obj, bool):
        assert False
    elif isinstance(obj, numbers.Integral):
        return obj
    elif isinstance(obj, str):
        assert obj.startswith("0x")
        return int(obj, 16)
    elif isinstance(obj, spear.UnrealStruct):
        return obj.ustruct
    elif isinstance(obj, spear.UnrealClass):
        return obj.uclass
    elif isinstance(obj, spear.UnrealObject):
        return obj.uobject
    elif isinstance(obj, np.ndarray):
        assert obj.dtype == np.uint64
        return obj
    elif isinstance(obj, list):
        return [ to_handle(obj=o) for o in obj ]
    elif isinstance(obj, dict):
        return { k: to_handle(obj=v) for k, v in obj.items() }
    elif isinstance(obj, spear.Future):
        inner_convert_func = obj.convert_func
        def convert_func(o):
            if inner_convert_func is not None:
                o = inner_convert_func(o)
            return to_handle(obj=o)
        obj.convert_func = convert_func
        return obj
    else:
        assert False


# Convert to a collection of JSON strings from a collection of objects so they can be passed to a service.
def to_json_strings(objs):
    if isinstance(objs, list):
        return [ to_json_string(obj=o) for o in objs ]
    elif isinstance(objs, dict):
        return { k: to_json_string(obj=v) for k, v in objs.items() }
    else:
        assert False

# Convert to a JSON string from an object so it can be passed to a service.
def to_json_string(obj, stringify=True):
    if isinstance(obj, str):
        return obj
    elif isinstance(obj, Ptr):
        return obj.to_string()
    elif isinstance(obj, spear.UnrealStruct):
        return to_ptr(handle=obj.ustruct).to_string()
    elif isinstance(obj, spear.UnrealClass):
        return to_ptr(handle=obj.uclass).to_string()
    elif isinstance(obj, spear.UnrealObject):
        return to_ptr(handle=obj.uobject).to_string()
    elif isinstance(obj, bool) or isinstance(obj, numbers.Number) or isinstance(obj, list) or isinstance(obj, dict):
        if stringify:
            if isinstance(obj, bool):
                return json.dumps(obj)
            elif isinstance(obj, numbers.Number):
                return json.dumps(obj)
            elif isinstance(obj, list):
                return json.dumps([ to_json_string(obj=o, stringify=False) for o in obj ])
            elif isinstance(obj, dict):
                return json.dumps({ to_json_string(obj=k, stringify=True): to_json_string(obj=v, stringify=False) for k, v in obj.items() })
            else:
                assert False
        else:
            if isinstance(obj, bool):
                return obj
            elif isinstance(obj, numbers.Number):
                return obj
            elif isinstance(obj, list):
                return [ to_json_string(obj=o, stringify=False) for o in obj ]
            elif isinstance(obj, dict):
                return { to_json_string(obj=k, stringify=True): to_json_string(obj=v, stringify=False) for k, v in obj.items() }
            else:
                assert False
    else:
        return json.dumps(obj)


# Convert to a collection of dicts from a collection of JSON strings returned by a service.
def try_to_dicts(json_strings, default_value=None):
    if isinstance(json_strings, list):
        return [ try_to_dict(json_string=s, default_value=default_value) for s in json_strings ]
    elif isinstance(json_strings, dict):
        return { k: try_to_dict(json_string=v, default_value=default_value) for k, v in json_strings.items() }
    else:
        assert False

# Convert to a dict from a JSON string returned by a service.
def try_to_dict(json_string, default_value=None):
    try:
        return json.loads(json_string)
    except:
        if default_value is None:
            return json_string
        else:
            return default_value


# Convert to a Ptr object that can be passed to a service from a handle.
def to_ptr(handle):
    return Ptr(handle=handle)


# Convert to a Shared object that can be passed to a service from a NumPy array backed by shared memory.
def to_shared(array, shared_memory_handle):
    return Shared(array=array, shared_memory_handle=shared_memory_handle)


# Convert to a packed array from a NumPy array.
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
        assert set(usage_flags) <= set(array._shared_memory_handle["view"].usage_flags)
        packed_array = spear_ext.PackedArray()
        packed_array.data = np.array([], dtype=array._array.dtype)
        packed_array.data_source = "Shared"
        packed_array.shape = array._array.shape
        packed_array.shared_memory_name = array._shared_memory_handle["name"]
        return packed_array
    else:
        assert False

# Convert to a NumPy array from a packed array.
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


# Convert to a collection of packed arrays from a collection of NumPy arrays.
def to_packed_arrays(arrays, dest_byte_order, usage_flags):
    if isinstance(arrays, list):
        return [ to_packed_array(array=a, dest_byte_order=dest_byte_order, usage_flags=usage_flags) for a in arrays ]
    elif isinstance(arrays, dict):
        return { k: to_packed_array(array=v, dest_byte_order=dest_byte_order, usage_flags=usage_flags) for k, v in arrays.items() }
    else:
        assert False

# Convert to a collection of NumPy arrays from a collection of packed arrays.
def to_arrays(packed_arrays, src_byte_order, usage_flags, shared_memory_handles):
    if isinstance(packed_arrays, list):
        return [ to_array(packed_array=p, src_byte_order=src_byte_order, usage_flags=usage_flags, shared_memory_handles=shared_memory_handles) for p in packed_arrays ]
    elif isinstance(packed_arrays, dict):
        return { k: to_array(packed_array=v, src_byte_order=src_byte_order, usage_flags=usage_flags, shared_memory_handles=shared_memory_handles) for k, v in packed_arrays.items() }
    else:
        assert False


# Convert to a data bundle from {a collection of arrays, a collection of unreal objects, an info string}.
def to_data_bundle(dest_byte_order, usage_flags, arrays=None, unreal_objs=None, info=""):
    if arrays is None:
        arrays = {}
    if unreal_objs is None:
        unreal_objs = {}
    assert dest_byte_order is not None
    assert usage_flags is not None
    data_bundle = spear_ext.DataBundle()
    data_bundle.packed_arrays = to_packed_arrays(arrays=arrays, dest_byte_order=dest_byte_order, usage_flags=["Arg"])
    data_bundle.unreal_obj_strings = to_json_strings(objs=unreal_objs)
    data_bundle.info = info
    return data_bundle

# Convert to {a collection of arrays, a collection of unreal objects, an info string} from a data bundle.
def to_data_bundle_dict(data_bundle, src_byte_order, usage_flags, shared_memory_handles):
    return {
        "arrays": to_arrays(packed_arrays=data_bundle.packed_arrays, src_byte_order=src_byte_order, usage_flags=["ReturnValue"], shared_memory_handles=shared_memory_handles),
        "unreal_objs": try_to_dicts(json_strings=data_bundle.unreal_obj_strings),
        "info": data_bundle.info}


# Convert to a NumPy matrix from an Unreal rotator. See utils/pipeline_utils.py for more details on Unreal's Euler angle conventions.
def to_numpy_matrix_from_rotator(rotator, as_matrix=None):
    assert isinstance(rotator, dict)
    rotator = { k.lower(): v for k, v in rotator.items() }
    assert set(["roll", "pitch", "yaw"]) == set(rotator.keys())
    roll  = np.deg2rad(-rotator["roll"])
    pitch = np.deg2rad(-rotator["pitch"])
    yaw   = np.deg2rad(rotator["yaw"])
    if as_matrix is None:
        return np.array(scipy.spatial.transform.Rotation.from_euler("xyz", [roll, pitch, yaw]).as_matrix())
    else:
        assert as_matrix
        return np.matrix(scipy.spatial.transform.Rotation.from_euler("xyz", [roll, pitch, yaw]).as_matrix())

# Convert to an Unreal rotator from a NumPy matrix.
def to_rotator_from_numpy_matrix(matrix):
    if isinstance(matrix, np.ndarray) or isinstance(matrix, np.matrix):
        assert matrix.shape == (3,3)
    else:
        assert False
    scipy_roll, scipy_pitch, scipy_yaw = scipy.spatial.transform.Rotation.from_matrix(matrix).as_euler("xyz")
    roll  = np.rad2deg(-scipy_roll)
    pitch = np.rad2deg(-scipy_pitch)
    yaw   = np.rad2deg(scipy_yaw)
    return {"Roll": roll, "Pitch": pitch, "Yaw": yaw}

# Convert to an Unreal rotator from a NumPy array.
def to_numpy_array_from_rotator(rotator):
    assert isinstance(rotator, dict)
    rotator = { k.lower(): v for k, v in rotator.items() }
    assert set(["roll", "pitch", "yaw"]) == set(rotator.keys())
    return np.array([rotator["pitch"], rotator["yaw"], rotator["roll"]])

# Convert to an Unreal rotator from a NumPy array.
def to_rotator_from_numpy_array(array_pyr):
    if isinstance(array_pyr, np.ndarray):
        assert array_pyr.shape == (3,)
    else:
        assert False
    return {"Pitch": array_pyr[0], "Yaw": array_pyr[1], "Roll": array_pyr[2]}

# Convert to an Unreal vector from a NumPy array or matrix and vice versa.
def to_vector_from_numpy_array(array):
    if isinstance(array, np.matrix):
        assert array.shape == (3, 1)
        array = array.A1
    elif isinstance(array, np.ndarray):
        assert array.shape == (3,)
    else:
        assert False
    return {"X": array[0], "Y": array[1], "Z": array[2]}

# Convert to a NumPy array from an Unreal vector.
def to_numpy_array_from_vector(vector, as_matrix=None):
    assert isinstance(vector, dict)
    vector = { k.lower(): v for k, v in vector.items() }
    assert set(["x", "y", "z"]) == set(vector.keys())
    if as_matrix is None:
        return np.array([vector["x"], vector["y"], vector["z"]])
    else:
        assert as_matrix
        return np.matrix([vector["x"], vector["y"], vector["z"]]).T
