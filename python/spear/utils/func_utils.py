#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import collections.abc
import json
import numbers
import numpy as np
import pprint as pp # imported as pp because spear.pprint(...) is defined below
import spear

if spear.__can_import_unreal__:
    import unreal


#
# A Service represents a collection of RPC entry points.
#

class Service():
    def __init__(self, entry_point_caller, sp_func_service=None, unreal_service=None, cdo=None, config=None, parent_service=None, create_children_services=False):

        self._world = None

        self.entry_point_caller = entry_point_caller

        if sp_func_service is not None:
            self.sp_func_service = sp_func_service.get_top_level_service()
        else:
            self.sp_func_service = None

        if unreal_service is not None:
            self.unreal_service = unreal_service.get_top_level_service()
        else:
            self.unreal_service = None

        if parent_service is None:
            self.cdo = cdo
        else:
            assert cdo is None
            assert not create_children_services # it is only allowed to create children services from the top-level service

        self.config = config

        self._parent_service = parent_service

        if create_children_services:
            assert entry_point_caller is not None

            if spear.__can_import_unreal__:
                call_async_entry_point_caller_type = EditorCallAsyncEntryPointCaller
                send_async_entry_point_caller_type = EditorSendAsyncEntryPointCaller
            else:
                call_async_entry_point_caller_type = CallAsyncEntryPointCaller
                send_async_entry_point_caller_type = SendAsyncEntryPointCaller

            call_async_entry_point_caller = call_async_entry_point_caller_type(service_name=entry_point_caller._service_name, engine_service=entry_point_caller.engine_service)
            self.call_async = self.create_child_service(entry_point_caller=call_async_entry_point_caller, sp_func_service=sp_func_service, unreal_service=unreal_service, config=config)

            send_async_entry_point_caller = send_async_entry_point_caller_type(service_name=entry_point_caller._service_name, engine_service=entry_point_caller.engine_service)
            self.send_async = self.create_child_service(entry_point_caller=send_async_entry_point_caller, sp_func_service=sp_func_service, unreal_service=unreal_service, config=config)

    def create_child_service(self, entry_point_caller, sp_func_service=None, unreal_service=None, config=None):
        assert False # if a derived class passes create_children_services=True into the base constructor, then the derived class must override create_child_service(...)

    def is_top_level_service(self):
        return self._parent_service is None

    def get_top_level_service(self):
        service = self
        while service._parent_service is not None:
            service = service._parent_service
        return service

    def get_world(self):
        return self.get_top_level_service()._world

    def set_world(self, world):
        assert self.is_top_level_service()
        self._world = world

    def get_unreal_object(self, uobject=None, uclass=None, with_sp_funcs=False):
        return spear.get_unreal_object(
            unreal_service=self.unreal_service,
            sp_func_service=self.sp_func_service,
            config=self.config,
            uobject=uobject,
            uclass=uclass,
            with_sp_funcs=with_sp_funcs)

    def to_handle_or_unreal_struct(self, obj, as_handle=None, as_unreal_struct=None):
        assert self.unreal_service is not None
        return spear.to_handle_or_unreal_struct(
            obj=obj,
            unreal_service=self.unreal_service,
            as_handle=as_handle,
            as_unreal_struct=as_unreal_struct)

    def to_handle_or_unreal_class(self, obj, as_handle=None, as_unreal_class=None):
        assert self.unreal_service is not None
        return spear.to_handle_or_unreal_class(
            obj=obj,
            unreal_service=self.unreal_service,
            as_handle=as_handle,
            as_unreal_class=as_unreal_class)

    def to_handle_or_unreal_object(self, obj, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        assert self.sp_func_service is not None
        assert self.unreal_service is not None
        assert self.config is not None
        return spear.to_handle_or_unreal_object(
            obj=obj,
            unreal_service=self.unreal_service,
            sp_func_service=self.sp_func_service,
            config=self.config,
            as_handle=as_handle,
            as_unreal_object=as_unreal_object,
            with_sp_funcs=with_sp_funcs)


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

    def to_result(self, obj):
        assert False

    def __repr__(self):
        return f"{self.__class__.__name__}(_service_name={self._service_name})"

class CallSyncEntryPointCaller(EntryPointCaller):
    def call_on_worker_thread(self, func_name, convert_func, *args):
        long_func_name = f"{self._service_name}.call_sync_on_worker_thread.{func_name}"
        return_value = self.engine_service.call_on_worker_thread(long_func_name, *args)
        if convert_func is not None:
            return convert_func(return_value)
        else:
            return return_value

    def call_on_game_thread(self, func_name, convert_func, *args):
        long_func_name = f"{self._service_name}.call_sync_on_game_thread.{func_name}"
        return_value = self.engine_service.call_on_game_thread(long_func_name, *args)
        if convert_func is not None:
            return convert_func(return_value)
        else:
            return return_value

    def to_result(self, obj):
        return obj

class CallAsyncEntryPointCaller(EntryPointCaller):
    def call_on_worker_thread(self, func_name, convert_func, *args):
        assert False # worker thread entry points always execute synchronously, so we should never be here

    def call_on_game_thread(self, func_name, convert_func, *args):
        long_func_name = f"{self._service_name}.call_async_on_game_thread.{func_name}"
        sync_func_name = f"{self._service_name}.call_sync_on_game_thread.{func_name}"
        return_as = self.engine_service.entry_point_signature_descs[sync_func_name].type_names[0]
        future = self.engine_service.call_on_game_thread(long_func_name, *args)
        get_future_result_func = lambda future: self.engine_service.get_future_result_from_game_thread(future=future.future, return_as=return_as)
        return Future(future=future, get_future_result_func=get_future_result_func, convert_func=convert_func, return_as=return_as)

    def to_result(self, obj):
        return Future(future=None, get_future_result_func=lambda future: obj, convert_func=None, return_as=None)

class SendAsyncEntryPointCaller(EntryPointCaller):
    def call_on_worker_thread(self, func_name, convert_func, *args):
        assert False # worker thread entry points always execute synchronously, so we should never be here

    def call_on_game_thread(self, func_name, convert_func, *args):
        long_func_name = f"{self._service_name}.send_async_on_game_thread.{func_name}"
        self.engine_service.call_on_game_thread(long_func_name, *args)

    def to_result(self, obj):
        return None

class EditorCallSyncEntryPointCaller(EntryPointCaller):
    def call_on_worker_thread(self, func_name, convert_func, *args):
        long_func_name = f"{self._service_name}.call_sync_on_worker_thread.{func_name}"
        return_value = self.engine_service.call_on_worker_thread(long_func_name, *args)
        if convert_func is not None:
            return convert_func(return_value)
        else:
            return return_value

    def call_on_game_thread(self, func_name, convert_func, *args):
        long_func_name = f"{self._service_name}.call_async_on_game_thread.{func_name}"
        sync_func_name = f"{self._service_name}.call_sync_on_game_thread.{func_name}"
        return_as = self.engine_service.entry_point_signature_descs[sync_func_name].type_names[0]

        with self.engine_service.editor_transaction():
            future = self.engine_service.call_on_game_thread(long_func_name, *args)

        return_value = self.engine_service.get_future_result_from_game_thread(future=future, return_as=return_as)
        if convert_func is not None:
            return convert_func(return_value)
        else:
            return return_value

    def to_result(self, obj):
        return obj

class EditorCallAsyncEntryPointCaller(EditorCallSyncEntryPointCaller):
    def call_on_worker_thread(self, func_name, convert_func, *args):
        assert False # worker thread entry points always execute synchronously, so we should never be here

    # In editor Python, we execute the entry point eagerly and store the result in a Future, so get() has the same
    # interface inside and outside the editor. We can't defer execution until get() is called, because then the
    # entry point would execute at get() time rather than at queuing time, which would violate the ordering
    # guarantees of our programming model.
    def call_on_game_thread(self, func_name, convert_func, *args):
        result = super().call_on_game_thread(func_name, convert_func, *args)
        return Future(future=None, get_future_result_func=lambda future: result, convert_func=None, return_as=None)

    def to_result(self, obj):
        return Future(future=None, get_future_result_func=lambda future: obj, convert_func=None, return_as=None)

class EditorSendAsyncEntryPointCaller(EditorCallSyncEntryPointCaller):
    def call_on_worker_thread(self, func_name, convert_func, *args):
        assert False # worker thread entry points always execute synchronously, so we should never be here

    def call_on_game_thread(self, func_name, convert_func, *args):
        super().call_on_game_thread(func_name, None, *args)

    def to_result(self, obj):
        return None


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

    def __eq__(self, other):
        if isinstance(other, Ptr):
            return self._handle == other._handle
        else:
            return NotImplemented

    def __hash__(self):
        return hash(self._handle)

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
    def __init__(self, future, get_future_result_func, convert_func, return_as):
        self.future = future
        self.convert_func = convert_func
        self._get_future_result_func = get_future_result_func
        self._return_as = return_as

    def get(self):
        result = self._get_future_result_func(future=self)
        if self.convert_func is not None:
            return self.convert_func(result)
        else:
            return result

    def __repr__(self):
        return f'Future(future={self.future}, convert_func={self.convert_func is not None}, _return_as="{self._return_as}")'

class ScriptStruct:
    def __init__(self, value, type_string):
        self.value = value
        self.type_string = type_string

    def __repr__(self):
        return f'ScriptStruct(value={self.value}, type_string="{self.type_string}")'

#
# CaseInsensitiveDict is a dict with string keys that are looked up case-insensitively. Each key is stored with the
# spelling it was first inserted with, so iterating over, printing, or serializing a CaseInsensitiveDict shows those
# spellings. Constructing a CaseInsensitiveDict asserts if two keys differ only by case. Only the top level is
# case-insensitive; nested dicts are not converted. This is useful for dicts whose keys are FName strings returned by
# Unreal (e.g., function parameter names), because outside the editor an FName only stores the first spelling
# registered in the process, so a parameter declared as, e.g., "Result" might be returned as "result" if some other
# code registered FName("result") first. Lookups, membership tests (including "key in d.keys()" and "(key, value) in
# d.items()"), "d == other", and "d | other" all compare keys case-insensitively. Operations that construct a plain
# dict from a CaseInsensitiveDict, e.g., "dict(d)" and "{**d}", return a dict that compares keys exactly, so use
# "d.copy()" if a case-insensitive copy is needed.
#

class CaseInsensitiveDict(dict):
    def __init__(self, dict_=None):
        dict_ = dict_ if dict_ is not None else {}
        super().__init__()
        self._stored_keys = {} # map from lowercase key to stored key
        for key, value in dict_.items():
            assert isinstance(key, str)
            assert key.lower() not in self._stored_keys # keys must not differ only by case
            self[key] = value

    def __getitem__(self, key):
        return super().__getitem__(self._get_stored_key(key=key))

    def __setitem__(self, key, value):
        assert isinstance(key, str)
        stored_key = self._stored_keys.setdefault(key.lower(), key)
        super().__setitem__(stored_key, value)

    def __delitem__(self, key):
        stored_key = self._get_stored_key(key=key)
        super().__delitem__(stored_key)
        del self._stored_keys[stored_key.lower()]

    def __contains__(self, key):
        return super().__contains__(self._get_stored_key(key=key))

    def __eq__(self, other):
        assert isinstance(other, collections.abc.Mapping)
        # each key in other must match a distinct key in self, ignoring case, with an equal value
        stored_keys = { self._get_stored_key(key=key) for key in other.keys() }
        return len(other) == len(self) and len(stored_keys) == len(self) and all(key in self and self[key] == value for key, value in other.items())

    def __ne__(self, other):
        return not self.__eq__(other)

    def __or__(self, other):
        assert isinstance(other, collections.abc.Mapping)
        result = self.copy()
        result.update(other)
        return result

    def __ror__(self, other):
        assert isinstance(other, collections.abc.Mapping)
        result = CaseInsensitiveDict()
        result.update(other)
        result.update(self)
        return result

    def __ior__(self, other):
        self.update(other)
        return self

    def __reduce__(self):
        # needed for pickle, because pickle restores items before restoring self._stored_keys, and pickle passes args
        # positionally, so dict(self) is passed as dict_
        return (CaseInsensitiveDict, (dict(self),))

    def __repr__(self):
        return f"CaseInsensitiveDict(dict_={super().__repr__()})"

    def get(self, key, default=None):
        return super().get(self._get_stored_key(key=key), default)

    def keys(self):
        return collections.abc.KeysView(self) # uses self.__contains__(...) for membership tests

    def items(self):
        return collections.abc.ItemsView(self) # uses self.__getitem__(...) for membership tests

    def pop(self, key, *args):
        stored_key = self._get_stored_key(key=key)
        if super().__contains__(stored_key):
            del self._stored_keys[stored_key.lower()]
        return super().pop(stored_key, *args)

    def popitem(self):
        key, value = super().popitem()
        del self._stored_keys[key.lower()]
        return key, value

    def setdefault(self, key, default=None):
        if key not in self:
            self[key] = default
        return self[key]

    def update(self, *args, **kwargs):
        for key, value in dict(*args, **kwargs).items():
            self[key] = value

    def clear(self):
        super().clear()
        self._stored_keys.clear()

    def copy(self):
        return CaseInsensitiveDict(dict_=self)

    def _get_stored_key(self, key):
        if isinstance(key, str) and key.lower() in self._stored_keys:
            return self._stored_keys[key.lower()]
        else:
            return key


#
# Factories for constructing Unreal proxy types that correctly return None in cases where the input uobjects
# and uclasses resolve to 0, since uobjects and uclasses that resolve to 0 are considered invalid inputs to
# the constructors of Unreal proxy types.
#

def get_unreal_struct(unreal_service, ustruct):
    if ustruct is None:
        return None
    else:
        ustruct = unreal_service.to_ustruct(ustruct=ustruct)
        if ustruct == 0:
            return None
        else:
            return spear.UnrealStruct(unreal_service=unreal_service, ustruct=ustruct)

def get_unreal_class(unreal_service, uclass):
    if uclass is None:
        return None
    else:
        uclass = unreal_service.to_uclass(uclass=uclass)
        if uclass == 0:
            return None
        else:
            return spear.UnrealClass(unreal_service=unreal_service, uclass=uclass)

def get_unreal_object(unreal_service, sp_func_service, config, uobject, uclass, with_sp_funcs):
    if uobject is None and uclass is None:
        return None
    elif uobject is None and uclass is not None:
        uclass = unreal_service.to_uclass(uclass=uclass) # needed to handle the case where uclass is a string like "AActor", doesn't handle the case where uclass is a string like "0xabcdabcd"
        if uclass == 0:
            return None
        uobject = unreal_service.get_default_object(uclass=uclass, create_if_needed=False, as_handle=True)
        assert uobject != 0
    elif uobject is not None and uclass is None:
        uobject = to_handle(obj=uobject)
        if uobject == 0:
            return None
        uclass = unreal_service.get_class(uobject=uobject, as_handle=True)
        assert uclass != 0
    else:
        uobject = spear.to_handle(obj=uobject)
        if uobject == 0:
            return None
        uclass = unreal_service.to_uclass(uclass=uclass) # needed to handle the case where uclass is a string like "AActor", doesn't handle the case where uclass is a string like "0xabcdabcd"
        if uclass == 0:
            return None
    return spear.UnrealObject(
        unreal_service=unreal_service,
        sp_func_service=sp_func_service,
        config=config,
        uobject=uobject,
        uclass=uclass,
        with_sp_funcs=with_sp_funcs)

#
# Converters for handling Unreal types as return values.
#

# Convert to handle or UnrealStruct
def to_handle_or_unreal_struct(obj, unreal_service, as_handle=None, as_unreal_struct=None):
    if as_handle is not None:
        assert as_handle
        assert as_unreal_struct is None
        return to_handle(obj=obj)
    elif as_unreal_struct is not None:
        assert as_unreal_struct
        return to_unreal_struct(obj=obj, unreal_service=unreal_service)
    else:
        return to_unreal_struct(obj=obj, unreal_service=unreal_service)

# Convert to handle or UnrealClass
def to_handle_or_unreal_class(obj, unreal_service, as_handle=None, as_unreal_class=None):
    if as_handle is not None:
        assert as_handle
        assert as_unreal_class is None
        return to_handle(obj=obj)
    elif as_unreal_class is not None:
        assert as_unreal_class
        return to_unreal_class(obj=obj, unreal_service=unreal_service)
    else:
        return to_unreal_class(obj=obj, unreal_service=unreal_service)

# Convert to handle or UnrealObject
def to_handle_or_unreal_object(obj, unreal_service, sp_func_service, config, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
    if as_handle is not None:
        assert as_handle
        assert as_unreal_object is None
        assert with_sp_funcs is None
        return to_handle(obj=obj)
    else:
        return to_unreal_object(obj=obj, unreal_service=unreal_service, sp_func_service=sp_func_service, config=config, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

# Convert to UnrealStruct
def to_unreal_struct(obj, unreal_service):
    return to_unreal_type(obj=obj, create_func=lambda obj: get_unreal_struct(unreal_service=unreal_service, ustruct=obj))

# Convert to UnrealClass
def to_unreal_class(obj, unreal_service):
    return to_unreal_type(obj=obj, create_func=lambda obj: get_unreal_class(unreal_service=unreal_service, uclass=obj))

# Convert to UnrealObject (as_unreal_object=True means return an UnrealObject and discover its class; as_unreal_object=handle means return an UnrealObject and treat handle as its class)
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
    return to_unreal_type(obj=obj, create_func=lambda obj: get_unreal_object(unreal_service=unreal_service, sp_func_service=sp_func_service, config=config, uobject=obj, uclass=uclass, with_sp_funcs=with_sp_funcs))

# Convert to UnrealStruct or UnrealClass
def to_unreal_type(obj, create_func):
    if isinstance(obj, bool):
        assert False
    elif isinstance(obj, numbers.Integral):
        return create_func(obj=obj)
    elif isinstance(obj, str):
        # We need to convert to a handle first, because create_func might be get_unreal_struct or get_unreal_class,
        # and neither handles the case when obj is a pointer represented as string. This conversion isn't
        # necessary for get_unreal_object, because get_unreal_object does handle the case when obj is a
        # pointer represented as a string.
        obj = to_handle(obj=obj)
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
    elif spear.__can_import_unreal__ and isinstance(obj, unreal.Object):
        return unreal.SpFuncUtils.to_handle_from_object(object=obj)
    else:
        assert False


#
# Converters for calling functions
#

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
        # Only a top-level UnrealObject without a property name can be converted, because the conversion only
        # preserves the UObject handle.
        assert obj.is_top_level()
        assert obj.property_name == ""
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
                return json.dumps(_to_json_validated_dict(dict_=obj))
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
                return _to_json_validated_dict(dict_=obj)
            else:
                assert False
    elif spear.__can_import_unreal__ and isinstance(obj, unreal.Object):
        return unreal.SpFuncUtils.to_string_from_object(object=obj)
    else:
        return json.dumps(obj)

# Unreal collapses JSON object keys that differ only by case (e.g., "a" and "A"), so we assert that the JSON keys are
# still unique, ignoring case, after converting them to strings. This also detects distinct keys that convert to the
# same string (e.g., a Ptr and an UnrealObject that refer to the same UObject). This assert is the set-side equivalent
# of the assert in UnrealUtils::getPropertyValueAsString(...) for TMap keys.
def _to_json_validated_dict(dict_):
    assert all(isinstance(k, str) for k in dict_.keys()) or not any(isinstance(k, str) for k in dict_.keys()) # keys must not mix strings and non-strings
    json_dict = { to_json_string(obj=k, stringify=True): to_json_string(obj=v, stringify=False) for k, v in dict_.items() }
    assert len(json_dict) == len(dict_)
    assert len({ k.lower() for k in json_dict.keys() }) == len(json_dict)
    return json_dict

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
    except json.JSONDecodeError:
        if default_value is None:
            return json_string
        else:
            return default_value

# Convert each dict in obj to a CaseInsensitiveDict if all of its keys are strings. Dicts whose keys are all non-strings
# (e.g., ints) are recursed into but the keys are left unchanged, and dicts that mix string and non-string keys are not
# allowed. Every dict returned by a service has string keys, because JSON object keys are always strings, so every dict
# returned by a service is converted. If recursive is True, then nested dicts inside dicts and lists are also converted;
# otherwise only obj itself is converted.
def to_case_insensitive(obj, recursive=True):
    if isinstance(obj, dict):
        if recursive:
            dict_ = { k: to_case_insensitive(obj=v) for k, v in obj.items() }
        else:
            dict_ = dict(obj)
        if all(isinstance(k, str) for k in dict_.keys()):
            return CaseInsensitiveDict(dict_=dict_)
        else:
            assert not any(isinstance(k, str) for k in dict_.keys()) # keys must not mix strings and non-strings
            return dict_
    elif isinstance(obj, list):
        if recursive:
            return [ to_case_insensitive(obj=o) for o in obj ]
        else:
            return obj
    else:
        return obj

# Convert each CaseInsensitiveDict in obj to a dict. If recursive is True, then nested CaseInsensitiveDicts inside dicts
# and lists are also converted; otherwise only obj itself is converted.
def to_case_sensitive(obj, recursive=True):
    if isinstance(obj, dict):
        if recursive:
            return { k: to_case_sensitive(obj=v) for k, v in obj.items() }
        else:
            return dict(obj)
    elif isinstance(obj, list):
        if recursive:
            return [ to_case_sensitive(obj=o) for o in obj ]
        else:
            return obj
    else:
        return obj

# Pretty-print obj using the standard pprint module. The standard pprint module doesn't pretty-print CaseInsensitiveDicts,
# because pprint only pretty-prints dicts that don't override __repr__, so we convert each CaseInsensitiveDict in obj to
# a dict before printing.
def pprint(obj, **kwargs):
    pp.pprint(to_case_sensitive(obj=obj), **kwargs)

# Convert to a Ptr object that can be passed to a service from a handle.
def to_ptr(handle):
    return Ptr(handle=handle)

# Convert to a Shared object that can be passed to a service from a NumPy array backed by shared memory.
def to_shared(array, shared_memory_handle):
    return Shared(array=array, shared_memory_handle=shared_memory_handle)

# Convert to a packed array from a NumPy array.
def to_packed_array(array, dest_byte_order, usage_flags):
    if isinstance(array, np.ndarray):
        packed_array = spear.SpPackedArray()
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
        packed_array = spear.SpPackedArray()
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
    data_bundle = spear.SpFuncDataBundle()
    data_bundle.packed_arrays = to_packed_arrays(arrays=arrays, dest_byte_order=dest_byte_order, usage_flags=usage_flags)
    data_bundle.unreal_obj_strings = to_json_strings(objs=unreal_objs)
    data_bundle.info = info
    return data_bundle

# Convert to {a collection of arrays, a collection of unreal objects, an info string} from a data bundle.
def to_data_bundle_dict(data_bundle, src_byte_order, usage_flags, shared_memory_handles):
    return {
        "arrays": to_arrays(packed_arrays=data_bundle.packed_arrays, src_byte_order=src_byte_order, usage_flags=usage_flags, shared_memory_handles=shared_memory_handles),
        "unreal_objs": try_to_dicts(json_strings=data_bundle.unreal_obj_strings),
        "info": data_bundle.info}


#
# Converters for interoperability between SPEAR Python and Editor Python
#

# Encode a SPEAR Python object into a script expression string.
def to_script_expr(obj):
    return f"spear.editor.from_script_expr(script_expr_dict={_to_script_expr_dict(obj=obj)})"

def _to_script_expr_dict(obj):
    if isinstance(obj, ScriptStruct):
        return { "type_string": "StructInstance", "value": { k: _to_script_expr_dict(obj=v) for k, v in obj.value.items() }, "struct_type_string": obj.type_string }
    else:
        return { "type_string": _to_script_expr_type_string(obj=obj), "value": _to_script_expr_value(obj=obj) }

def _to_script_expr_type_string(obj):
    if isinstance(obj, tuple):
        return "tuple"
    elif isinstance(obj, list):
        return "list"
    elif isinstance(obj, dict):
        return "dict"
    elif isinstance(obj, np.matrix):
        return "numpy.matrix"
    elif isinstance(obj, np.ndarray):
        return "numpy.ndarray"
    elif isinstance(obj, spear.UnrealStruct):
        return "UnrealStruct"
    elif isinstance(obj, spear.UnrealClass):
        return "UnrealClass"
    elif isinstance(obj, spear.UnrealObject):
        return "UnrealObject"
    elif type(obj).__module__ == "builtins":
        return type(obj).__qualname__
    else:
        return f"{type(obj).__module__}.{type(obj).__qualname__}"

def _to_script_expr_value(obj):
    if isinstance(obj, tuple):
        return [ _to_script_expr_dict(obj=item) for item in obj ]
    elif isinstance(obj, list):
        return [ _to_script_expr_dict(obj=item) for item in obj ]
    elif isinstance(obj, dict):
        return { k: _to_script_expr_dict(obj=v) for k, v in obj.items() }
    elif isinstance(obj, np.ndarray):
        return { "shape": list(obj.shape), "dtype": str(obj.dtype), "data": obj.tolist() }
    elif isinstance(obj, spear.UnrealStruct):
        return to_handle(obj=obj)
    elif isinstance(obj, spear.UnrealClass):
        return to_handle(obj=obj)
    elif isinstance(obj, spear.UnrealObject):
        return to_handle(obj=obj)
    else:
        return obj

# needed to encode structs when passing them to editor Python
def to_script_struct_expr(value, type_string):
    assert isinstance(value, dict)
    return to_script_expr(obj=ScriptStruct(value=value, type_string=type_string))

# Decode a script result dict into a SPEAR Python object.
def from_script_result(script_result, unreal_service=None, sp_func_service=None, config=None, as_handle=None, as_unreal_struct=None, as_unreal_class=None, as_unreal_object=None, with_sp_funcs=None):
    type_string = script_result["type_string"]
    value = script_result["value"]
    if type_string == "tuple":
        assert as_handle is None
        assert as_unreal_struct is None
        assert as_unreal_class is None
        assert as_unreal_object is None
        assert with_sp_funcs is None
        return tuple( from_script_result(script_result=elem, unreal_service=unreal_service, sp_func_service=sp_func_service, config=config) for elem in value )
    elif type_string == "list":
        assert as_handle is None
        assert as_unreal_struct is None
        assert as_unreal_class is None
        assert as_unreal_object is None
        assert with_sp_funcs is None
        return [ from_script_result(script_result=elem, unreal_service=unreal_service, sp_func_service=sp_func_service, config=config) for elem in value ]
    elif type_string == "dict":
        assert as_handle is None
        assert as_unreal_struct is None
        assert as_unreal_class is None
        assert as_unreal_object is None
        assert with_sp_funcs is None
        return { k: from_script_result(script_result=v, unreal_service=unreal_service, sp_func_service=sp_func_service, config=config) for k, v in value.items() }
    elif type_string == "numpy.matrix":
        assert as_handle is None
        assert as_unreal_struct is None
        assert as_unreal_class is None
        assert as_unreal_object is None
        assert with_sp_funcs is None
        return np.matrix(np.array(value["data"], dtype=np.dtype(value["dtype"])).reshape(value["shape"]))
    elif type_string == "numpy.ndarray":
        assert as_handle is None
        assert as_unreal_struct is None
        assert as_unreal_class is None
        assert as_unreal_object is None
        assert with_sp_funcs is None
        return np.array(value["data"], dtype=np.dtype(value["dtype"])).reshape(value["shape"])
    elif type_string == "UnrealStruct":
        assert as_unreal_class is None
        assert as_unreal_object is None
        assert with_sp_funcs is None
        return to_handle_or_unreal_struct(obj=value, unreal_service=unreal_service, as_handle=as_handle, as_unreal_struct=as_unreal_struct)
    elif type_string == "UnrealClass":
        assert as_unreal_struct is None
        assert as_unreal_object is None
        assert with_sp_funcs is None
        return to_handle_or_unreal_class(obj=value, unreal_service=unreal_service, as_handle=as_handle, as_unreal_class=as_unreal_class)
    elif type_string == "UnrealObject":
        assert as_unreal_struct is None
        assert as_unreal_class is None
        return to_handle_or_unreal_object(obj=value, unreal_service=unreal_service, sp_func_service=sp_func_service, config=config, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)
    elif type_string == "StructInstance":
        assert as_handle is None
        assert as_unreal_struct is None
        assert as_unreal_class is None
        assert as_unreal_object is None
        assert with_sp_funcs is None
        return value
    else:
        assert as_handle is None
        assert as_unreal_struct is None
        assert as_unreal_class is None
        assert as_unreal_object is None
        assert with_sp_funcs is None
        return value
