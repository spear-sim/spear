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

    #
    # Helper function for derived service types to construct an UnrealObject directly from a uobject and/or uclass,
    # e.g., when a service needs to wrap an object it already has a handle to. For example, ClassDefaultObjectService
    # calls self.get_unreal_object(uclass=uclass) to get an UnrealObject for the default object of uclass.
    #

    def get_unreal_object(self, uobject=None, uclass=None, with_sp_funcs=False):
        return spear.get_unreal_object(
            unreal_service=self.unreal_service,
            sp_func_service=self.sp_func_service,
            config=self.config,
            uobject=uobject,
            uclass=uclass,
            with_sp_funcs=with_sp_funcs)

    #
    # Helper functions for converting entry point return values to values, handles, or proxy types, according to the
    # as_value, as_handle, and as_unreal_* arguments that derived services expose to users. to_unreal_proxy(...)
    # converts a spear.PropertyValue, and uses its type_id to decide whether to convert it to a proxy type if none of
    # these args are specified. to_unreal_struct(...), to_unreal_class(...), and to_unreal_object(...) convert handles,
    # and recurse into containers. These functions expect concrete values rather than Futures, so derived services call
    # them from the convert_func that they pass to the EntryPointCaller.
    #

    def to_unreal_proxy(self, property_value, as_value=None, as_handle=None, as_unreal_struct=None, as_unreal_class=None, as_unreal_object=None, with_sp_funcs=None):
        assert self.unreal_service is not None
        return spear.to_unreal_proxy(
            property_value=property_value,
            unreal_service=self.unreal_service,
            as_value=as_value,
            as_handle=as_handle,
            as_unreal_struct=as_unreal_struct,
            as_unreal_class=as_unreal_class,
            as_unreal_object=as_unreal_object,
            with_sp_funcs=with_sp_funcs)

    def to_unreal_struct(self, obj, as_handle=None, as_unreal_struct=None):
        assert self.unreal_service is not None
        return spear.to_unreal_struct(
            obj=obj,
            unreal_service=self.unreal_service,
            as_handle=as_handle,
            as_unreal_struct=as_unreal_struct)

    def to_unreal_class(self, obj, as_handle=None, as_unreal_class=None):
        assert self.unreal_service is not None
        return spear.to_unreal_class(
            obj=obj,
            unreal_service=self.unreal_service,
            as_handle=as_handle,
            as_unreal_class=as_unreal_class)

    def to_unreal_object(self, obj, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        assert self.sp_func_service is not None
        assert self.unreal_service is not None
        assert self.config is not None
        return spear.to_unreal_object(
            obj=obj,
            unreal_service=self.unreal_service,
            sp_func_service=self.sp_func_service,
            config=self.config,
            as_handle=as_handle,
            as_unreal_object=as_unreal_object,
            with_sp_funcs=with_sp_funcs)


#
# Each type that derives from EntryPointCaller encapsulates a particular strategy for calling EngineService functions.
# EntryPointCaller types sit above the client and below the services, and EngineService passes args and return values
# between them unchanged. For args, an EntryPointCaller converts NumPy scalars to Python scalars. For return values, an
# EntryPointCaller applies the convert_func provided by the calling service, either immediately or when Future.get() is
# called, depending on the strategy. Higher-level conversions, e.g., converting JSON strings to dicts, are not handled
# here, and are instead the responsibility of each service's convert_func.
#

class EntryPointCaller():
    def __init__(self, service_name, engine_service):
        self.engine_service = engine_service
        self._service_name = service_name

    def call_on_worker_thread(self, func_name, convert_func, *args):
        assert False

    def call_on_game_thread(self, func_name, convert_func, *args):
        assert False

    def to_result(self, convert_func, obj):
        assert False

    def __repr__(self):
        return f"{self.__class__.__name__}(_service_name={self._service_name})"

    def _to_args(self, args):
        return [ self._to_arg(arg=arg) for arg in args ]

    # Convert NumPy scalars to Python scalars, because the client only accepts Python scalars for scalar arguments.
    def _to_arg(self, arg):
        return to_scalar(obj=arg)

    def _to_return_value(self, return_value):
        return return_value # included for symmetry with _to_arg(...)

    def _to_return_value_call_sync(self, return_value, convert_func=None):
        return_value = self._to_return_value(return_value=return_value)
        if convert_func is not None:
            return convert_func(return_value)
        else:
            return return_value

    def _to_future_call_async(self, future, return_as, convert_func=None):
        get_future_result_func = lambda future: self.engine_service.get_future_result_from_game_thread(future=future.future, return_as=return_as)
        result = Future(future=future, get_future_result_func=get_future_result_func, convert_func=lambda result: self._to_return_value(return_value=result), return_as=return_as)
        if convert_func is not None:
            result.apply_last(convert_func=convert_func)
        return result

class CallSyncEntryPointCaller(EntryPointCaller):
    def call_on_worker_thread(self, func_name, convert_func, *args):
        long_func_name = f"{self._service_name}.call_sync_on_worker_thread.{func_name}"
        args = self._to_args(args=args)
        return_value = self.engine_service.call_on_worker_thread(long_func_name, *args)
        return_value = self._to_return_value_call_sync(return_value=return_value, convert_func=convert_func)
        return return_value

    def call_on_game_thread(self, func_name, convert_func, *args):
        long_func_name = f"{self._service_name}.call_sync_on_game_thread.{func_name}"
        args = self._to_args(args=args)
        return_value = self.engine_service.call_on_game_thread(long_func_name, *args)
        return_value = self._to_return_value_call_sync(return_value=return_value, convert_func=convert_func)
        return return_value

    def to_result(self, convert_func, obj):
        if convert_func is not None:
            return convert_func(obj)
        else:
            return obj

class CallAsyncEntryPointCaller(EntryPointCaller):
    def call_on_worker_thread(self, func_name, convert_func, *args):
        assert False # worker thread entry points always execute synchronously, so we should never be here

    def call_on_game_thread(self, func_name, convert_func, *args):
        sync_func_name = f"{self._service_name}.call_sync_on_game_thread.{func_name}" # use sync name to look up return_as
        return_as = self.engine_service.entry_point_signature_descs[sync_func_name].type_names[0]
        long_func_name = f"{self._service_name}.call_async_on_game_thread.{func_name}"
        args = self._to_args(args=args)
        future = self.engine_service.call_on_game_thread(long_func_name, *args)
        future = self._to_future_call_async(future=future, return_as=return_as, convert_func=convert_func)
        return future

    def to_result(self, convert_func, obj):
        return Future(future=None, get_future_result_func=lambda future: obj, convert_func=convert_func, return_as=None)

class SendAsyncEntryPointCaller(EntryPointCaller):
    def call_on_worker_thread(self, func_name, convert_func, *args):
        assert False # worker thread entry points always execute synchronously, so we should never be here

    def call_on_game_thread(self, func_name, convert_func, *args):
        long_func_name = f"{self._service_name}.send_async_on_game_thread.{func_name}"
        args = self._to_args(args=args)
        self.engine_service.call_on_game_thread(long_func_name, *args)

    def to_result(self, convert_func, obj):
        return None

class EditorCallSyncEntryPointCaller(EntryPointCaller):
    def call_on_worker_thread(self, func_name, convert_func, *args):
        long_func_name = f"{self._service_name}.call_sync_on_worker_thread.{func_name}"
        args = self._to_args(args=args)
        return_value = self.engine_service.call_on_worker_thread(long_func_name, *args)
        return_value = self._to_return_value_call_sync(return_value=return_value, convert_func=convert_func)
        return return_value

    def call_on_game_thread(self, func_name, convert_func, *args):
        sync_func_name = f"{self._service_name}.call_sync_on_game_thread.{func_name}" # use sync name to look up return_as
        return_as = self.engine_service.entry_point_signature_descs[sync_func_name].type_names[0]
        long_func_name = f"{self._service_name}.call_async_on_game_thread.{func_name}"
        args = self._to_args(args=args)
        with self.engine_service.editor_transaction():
            future = self.engine_service.call_on_game_thread(long_func_name, *args)
        return_value = self.engine_service.get_future_result_from_game_thread(future=future, return_as=return_as)
        return_value = self._to_return_value_call_sync(return_value=return_value, convert_func=convert_func)
        return return_value

    def to_result(self, convert_func, obj):
        if convert_func is not None:
            return convert_func(obj)
        else:
            return obj

class EditorCallAsyncEntryPointCaller(EditorCallSyncEntryPointCaller):
    def call_on_worker_thread(self, func_name, convert_func, *args):
        assert False # worker thread entry points always execute synchronously, so we should never be here

    # In editor Python, we execute the entry point eagerly and store the result in a Future, so get() has the same
    # interface inside and outside the editor. We can't defer execution until get() is called, because then the
    # entry point would execute at get() time rather than at queuing time, which would violate the ordering
    # guarantees of our programming model. However, we defer convert_func until get() is called, so all
    # post-processing happens at get() time, consistent with CallAsyncEntryPointCaller.
    def call_on_game_thread(self, func_name, convert_func, *args):
        result = super().call_on_game_thread(func_name, None, *args)
        return Future(future=None, get_future_result_func=lambda future: result, convert_func=convert_func, return_as=None)

    def to_result(self, convert_func, obj):
        return Future(future=None, get_future_result_func=lambda future: obj, convert_func=convert_func, return_as=None)

class EditorSendAsyncEntryPointCaller(EditorCallSyncEntryPointCaller):
    def call_on_worker_thread(self, func_name, convert_func, *args):
        assert False # worker thread entry points always execute synchronously, so we should never be here

    def call_on_game_thread(self, func_name, convert_func, *args):
        super().call_on_game_thread(func_name, None, *args)

    def to_result(self, convert_func, obj):
        return None


#
# These classes are for internal use, and do not need to be instantiated directly by most users.
#

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
        self._convert_func = convert_func
        self._get_future_result_func = get_future_result_func
        self._return_as = return_as
        self._result_obtained = False

    # get() can only be called once, because getting the result of an RPC-backed future destroys the server's
    # corresponding std::future object. For consistency, we enforce this rule for all futures, including futures
    # that already store their result (e.g., in editor Python).
    def get(self):
        assert not self._result_obtained
        self._result_obtained = True
        result = self._get_future_result_func(future=self)
        if self._convert_func is not None:
            return self._convert_func(result)
        else:
            return result

    # Append convert_func so it is applied after any existing convert_func when get() is called.
    def apply_last(self, convert_func):
        assert not self._result_obtained
        if self._convert_func is None:
            self._convert_func = convert_func
        else:
            prev_convert_func = self._convert_func
            self._convert_func = lambda result: convert_func(prev_convert_func(result))

    def __repr__(self):
        return f'Future(future={self.future}, _convert_func={self._convert_func is not None}, _return_as="{self._return_as}")'

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
# By default, two keys are considered equal if they are equal after calling str.lower(...) on both. The optional
# get_key_func arg overrides this behavior, i.e., two keys are considered equal if get_key_func(...) returns the same
# string for both. For example, get_key_func can be used to compare part of each key case-sensitively and the rest
# case-insensitively. Two CaseInsensitiveDicts with different get_key_func objects never compare equal. When merging
# (e.g., "d | other"), the result uses the get_key_func of the left operand if it is a CaseInsensitiveDict, and of the
# right operand otherwise.
#

class CaseInsensitiveDict(dict):
    def __init__(self, dict_=None, get_key_func=None):
        dict_ = dict_ if dict_ is not None else {}
        super().__init__()
        self._get_key_func = get_key_func if get_key_func is not None else str.lower
        self._stored_keys = {} # map from get_key_func(key) to stored key
        for key, value in dict_.items():
            assert isinstance(key, str)
            assert self._get_key_func(key) not in self._stored_keys # keys must not collide according to get_key_func
            self[key] = value

    def __getitem__(self, key):
        return super().__getitem__(self._get_stored_key(key=key))

    def __setitem__(self, key, value):
        assert isinstance(key, str)
        stored_key = self._stored_keys.setdefault(self._get_key_func(key), key)
        super().__setitem__(stored_key, value)

    def __delitem__(self, key):
        stored_key = self._get_stored_key(key=key)
        super().__delitem__(stored_key)
        del self._stored_keys[self._get_key_func(stored_key)]

    def __contains__(self, key):
        return super().__contains__(self._get_stored_key(key=key))

    def __eq__(self, other):
        assert isinstance(other, collections.abc.Mapping)
        if isinstance(other, CaseInsensitiveDict) and other._get_key_func != self._get_key_func:
            return False
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
        result = CaseInsensitiveDict(get_key_func=self._get_key_func)
        result.update(other)
        result.update(self)
        return result

    def __ior__(self, other):
        self.update(other)
        return self

    def __reduce__(self):
        # needed for copy.copy(...), copy.deepcopy(...), and pickle, because by default they restore items before
        # restoring self._stored_keys, and they pass args positionally, so dict(self) is passed as dict_ and
        # self._get_key_func is passed as get_key_func
        return (CaseInsensitiveDict, (dict(self), self._get_key_func))

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
            del self._stored_keys[self._get_key_func(stored_key)]
        return super().pop(stored_key, *args)

    def popitem(self):
        key, value = super().popitem()
        del self._stored_keys[self._get_key_func(key)]
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
        return CaseInsensitiveDict(dict_=self, get_key_func=self._get_key_func)

    def _get_stored_key(self, key):
        if isinstance(key, str) and self._get_key_func(key) in self._stored_keys:
            return self._stored_keys[self._get_key_func(key)]
        else:
            return key

# Convert each dict in obj to a CaseInsensitiveDict if all of its keys are strings. Dicts whose keys are all non-strings
# (e.g., ints) are recursed into but the keys are left unchanged, and dicts that mix string and non-string keys are not
# allowed. Every dict decoded from JSON has string keys, because JSON object keys are always strings, so every dict
# decoded from JSON is converted. If recursive is True, then nested dicts inside dicts and lists are also converted;
# otherwise only obj itself is converted. get_key_func is passed to each CaseInsensitiveDict constructed for obj itself
# (or for each dict in obj, if obj is a list). If apply_get_key_func_recursively is True, then get_key_func is also
# passed to each CaseInsensitiveDict constructed for nested dicts; otherwise nested CaseInsensitiveDicts use the default
# behavior.
def to_case_insensitive(obj, recursive=True, get_key_func=None, apply_get_key_func_recursively=False):
    if apply_get_key_func_recursively:
        assert recursive
        assert get_key_func is not None

    if apply_get_key_func_recursively:
        nested_get_key_func = get_key_func
    else:
        nested_get_key_func = None

    if isinstance(obj, dict):
        if recursive:
            dict_ = { k: to_case_insensitive(obj=v, get_key_func=nested_get_key_func, apply_get_key_func_recursively=apply_get_key_func_recursively) for k, v in obj.items() }
        else:
            dict_ = dict(obj)
        if all(isinstance(k, str) for k in dict_.keys()):
            return CaseInsensitiveDict(dict_=dict_, get_key_func=get_key_func)
        else:
            assert not any(isinstance(k, str) for k in dict_.keys()) # keys must not mix strings and non-strings
            return dict_
    elif isinstance(obj, list):
        if recursive:
            return [ to_case_insensitive(obj=o, get_key_func=get_key_func, apply_get_key_func_recursively=apply_get_key_func_recursively) for o in obj ]
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


#
# Factories for constructing Unreal proxy types that correctly return None in cases where the input uobjects
# and uclasses resolve to 0, since uobjects and uclasses that resolve to 0 are considered invalid inputs to
# the constructors of Unreal proxy types. Null pointers arise whenever an entry point or property returns
# nullptr and the caller has requested a proxy type, e.g., load_object(...) and load_class(...) when loading
# fails, get_default_object(...) with create_if_needed=False, get_super_class(...) on a class with no super
# class, null pointer properties read through UnrealObject (e.g., an unset RootComponent), and null elements
# in containers of pointers, since _to_unreal_proxy(...) calls these factories once per element.
#

def get_unreal_struct(unreal_service, ustruct):
    if ustruct is None:
        return None # never reached when called from _to_unreal_proxy(...), included for symmetry with get_unreal_object(...)
    else:
        ustruct = unreal_service.to_ustruct(ustruct=ustruct)
        if ustruct.ptr == 0:
            return None
        else:
            return spear.UnrealStruct(unreal_service=unreal_service, ustruct=ustruct)

def get_unreal_class(unreal_service, uclass):
    if uclass is None:
        return None # never reached when called from _to_unreal_proxy(...), included for symmetry with get_unreal_object(...)
    else:
        uclass = unreal_service.to_uclass(uclass=uclass)
        if uclass.ptr == 0:
            return None
        else:
            return spear.UnrealClass(unreal_service=unreal_service, uclass=uclass)

def get_unreal_object(unreal_service, sp_func_service, config, uobject, uclass, with_sp_funcs):
    if uobject is None and uclass is None:
        return None
    elif uobject is None and uclass is not None:
        uclass = unreal_service.to_uclass(uclass=uclass)
        if uclass.ptr == 0:
            return None
        uobject = unreal_service.get_default_object(uclass=uclass, create_if_needed=False, as_handle=True)
        assert uobject.ptr != 0
    elif uobject is not None and uclass is None:
        uobject = to_handle(obj=uobject)
        if uobject.ptr == 0:
            return None
        uclass = unreal_service.get_class(uobject=uobject, as_handle=True)
        assert uclass.ptr != 0
    else:
        uobject = to_handle(obj=uobject)
        if uobject.ptr == 0:
            return None
        uclass = unreal_service.to_uclass(uclass=uclass)
        if uclass.ptr == 0:
            return None
    return spear.UnrealObject(
        unreal_service=unreal_service,
        sp_func_service=sp_func_service,
        config=config,
        uobject=uobject,
        uclass=uclass,
        with_sp_funcs=with_sp_funcs)


#
# Converters for handling Unreal types, used only for return values.
#

# Convert a spear.PropertyValue to a value, handle, or proxy type, according to the as_value, as_handle, and as_unreal_*
# args. If none of these args are specified, then the spear.PropertyValue's type_id determines whether the value is
# converted to a proxy type.
def to_unreal_proxy(property_value, unreal_service, as_value=None, as_handle=None, as_unreal_struct=None, as_unreal_class=None, as_unreal_object=None, with_sp_funcs=None):
    assert isinstance(property_value, spear.PropertyValue)
    if as_value is not None:
        assert as_value
        assert as_handle is None
        assert as_unreal_struct is None
        assert as_unreal_class is None
        assert as_unreal_object is None
        assert with_sp_funcs is None
        return property_value.value
    elif as_handle is not None:
        assert as_handle
        assert as_unreal_struct is None
        assert as_unreal_class is None
        assert as_unreal_object is None
        assert with_sp_funcs is None
        return to_handle(obj=property_value.value)
    elif as_unreal_struct is not None:
        assert as_unreal_struct
        assert as_unreal_class is None
        assert as_unreal_object is None
        assert with_sp_funcs is None
        return unreal_service.to_unreal_struct(obj=to_handle(obj=property_value.value))
    elif as_unreal_class is not None:
        assert as_unreal_class
        assert as_unreal_object is None
        assert with_sp_funcs is None
        return unreal_service.to_unreal_class(obj=to_handle(obj=property_value.value))
    elif as_unreal_object is not None:
        return unreal_service.to_unreal_object(obj=to_handle(obj=property_value.value), as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)
    elif _is_ustruct_pointer_or_container(type_id=property_value.type_id):
        return unreal_service.to_unreal_struct(obj=to_handle(obj=property_value.value))
    elif _is_uclass_pointer_or_container(type_id=property_value.type_id):
        return unreal_service.to_unreal_class(obj=to_handle(obj=property_value.value))
    elif _is_uobject_pointer_or_container(type_id=property_value.type_id):
        return unreal_service.to_unreal_object(obj=to_handle(obj=property_value.value), with_sp_funcs=with_sp_funcs)
    else:
        return property_value.value

def _is_ustruct_pointer_or_container(type_id):
    assert not type_id.startswith(" ") and not type_id.endswith(" ")
    pointee_type_id = _get_pointee_type_id(type_id=type_id)
    if pointee_type_id is not None:
        assert pointee_type_id != "UStruct" # unexpected
        return pointee_type_id == "UScriptStruct" or pointee_type_id == "UUserDefinedStruct"
    elif type_id.startswith("TArray<") and type_id.endswith(">"):
        value_type_id = type_id.removeprefix("TArray<").removesuffix(">")
        return _is_ustruct_pointer_or_container(type_id=value_type_id)
    elif type_id.startswith("TMap<") and type_id.endswith(">"):
        value_type_id = type_id.removeprefix("TMap<").removesuffix(">").split(",", 1)[1] # split on the first comma only
        return _is_ustruct_pointer_or_container(type_id=value_type_id)
    elif type_id.startswith("TSet<") and type_id.endswith(">"):
        value_type_id = type_id.removeprefix("TSet<").removesuffix(">")
        return _is_ustruct_pointer_or_container(type_id=value_type_id)
    elif type_id.endswith("]"): # fixed-size array, e.g., AActor*[4]
        value_type_id = type_id[:type_id.rindex("[")]
        return _is_ustruct_pointer_or_container(type_id=value_type_id)
    else:
        return False

def _is_uclass_pointer_or_container(type_id):
    assert not type_id.startswith(" ") and not type_id.endswith(" ")
    pointee_type_id = _get_pointee_type_id(type_id=type_id)
    if pointee_type_id is not None:
        assert pointee_type_id != "UStruct" # unexpected
        return pointee_type_id == "UClass"
    elif type_id.startswith("TSubclassOf<") and type_id.endswith(">"):
        return True
    elif type_id.startswith("TArray<") and type_id.endswith(">"):
        value_type_id = type_id.removeprefix("TArray<").removesuffix(">")
        return _is_uclass_pointer_or_container(type_id=value_type_id)
    elif type_id.startswith("TMap<") and type_id.endswith(">"):
        value_type_id = type_id.removeprefix("TMap<").removesuffix(">").split(",", 1)[1] # split on the first comma only
        return _is_uclass_pointer_or_container(type_id=value_type_id)
    elif type_id.startswith("TSet<") and type_id.endswith(">"):
        value_type_id = type_id.removeprefix("TSet<").removesuffix(">")
        return _is_uclass_pointer_or_container(type_id=value_type_id)
    elif type_id.endswith("]"): # fixed-size array, e.g., AActor*[4]
        value_type_id = type_id[:type_id.rindex("[")]
        return _is_uclass_pointer_or_container(type_id=value_type_id)
    else:
        return False

def _is_uobject_pointer_or_container(type_id):
    assert not type_id.startswith(" ") and not type_id.endswith(" ")
    pointee_type_id = _get_pointee_type_id(type_id=type_id)
    if pointee_type_id is not None:
        assert pointee_type_id != "UStruct" # unexpected
        return pointee_type_id != "UScriptStruct" and pointee_type_id != "UUserDefinedStruct" and pointee_type_id != "UClass"
    elif type_id == "TScriptInterface":
        return True
    elif type_id.startswith("TLazyObjectPtr<") and type_id.endswith(">"):
        return True
    elif type_id.startswith("TWeakObjectPtr<") and type_id.endswith(">"):
        return True
    elif type_id.startswith("TScriptInterface<") and type_id.endswith(">"):
        return True
    elif type_id.startswith("TArray<") and type_id.endswith(">"):
        value_type_id = type_id.removeprefix("TArray<").removesuffix(">")
        return _is_uobject_pointer_or_container(type_id=value_type_id)
    elif type_id.startswith("TMap<") and type_id.endswith(">"):
        value_type_id = type_id.removeprefix("TMap<").removesuffix(">").split(",", 1)[1] # split on the first comma only
        return _is_uobject_pointer_or_container(type_id=value_type_id)
    elif type_id.startswith("TSet<") and type_id.endswith(">"):
        value_type_id = type_id.removeprefix("TSet<").removesuffix(">")
        return _is_uobject_pointer_or_container(type_id=value_type_id)
    elif type_id.endswith("]"): # fixed-size array, e.g., AActor*[4]
        value_type_id = type_id[:type_id.rindex("[")]
        return _is_uobject_pointer_or_container(type_id=value_type_id)
    else:
        return False

# TObjectPtr<T> is a drop-in replacement for T*, so we treat both as pointers to T and return T. We don't treat
# TWeakObjectPtr<T> or TLazyObjectPtr<T> as pointers because they have different semantics.
def _get_pointee_type_id(type_id):
    if type_id.endswith("*"):
        return type_id.removesuffix("*")
    elif type_id.startswith("TObjectPtr<") and type_id.endswith(">"):
        return type_id.removeprefix("TObjectPtr<").removesuffix(">")
    else:
        return None

# Convert to UnrealStruct (or handle if as_handle=True)
def to_unreal_struct(obj, unreal_service, as_handle=None, as_unreal_struct=None):
    if as_handle is not None:
        assert as_handle
        assert as_unreal_struct is None
        return to_handle(obj=obj)
    elif as_unreal_struct is not None:
        assert as_unreal_struct
        return _to_unreal_struct_as_proxy(obj=obj, unreal_service=unreal_service)
    else:
        return _to_unreal_struct_as_proxy(obj=obj, unreal_service=unreal_service)

# Convert to UnrealClass (or handle if as_handle=True)
def to_unreal_class(obj, unreal_service, as_handle=None, as_unreal_class=None):
    if as_handle is not None:
        assert as_handle
        assert as_unreal_class is None
        return to_handle(obj=obj)
    elif as_unreal_class is not None:
        assert as_unreal_class
        return _to_unreal_class_as_proxy(obj=obj, unreal_service=unreal_service)
    else:
        return _to_unreal_class_as_proxy(obj=obj, unreal_service=unreal_service)

# Convert to UnrealObject (or handle if as_handle=True)
def to_unreal_object(obj, unreal_service, sp_func_service, config, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
    if as_handle is not None:
        assert as_handle
        assert as_unreal_object is None
        assert with_sp_funcs is None
        return to_handle(obj=obj)
    elif as_unreal_object is not None:
        return _to_unreal_object_as_proxy(obj=obj, unreal_service=unreal_service, sp_func_service=sp_func_service, config=config, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)
    else:
        return _to_unreal_object_as_proxy(obj=obj, unreal_service=unreal_service, sp_func_service=sp_func_service, config=config, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

# Convert to UnrealStruct
def _to_unreal_struct_as_proxy(obj, unreal_service):
    return _to_unreal_proxy(obj=obj, create_func=lambda obj: get_unreal_struct(unreal_service=unreal_service, ustruct=obj))

# Convert to UnrealClass
def _to_unreal_class_as_proxy(obj, unreal_service):
    return _to_unreal_proxy(obj=obj, create_func=lambda obj: get_unreal_class(unreal_service=unreal_service, uclass=obj))

# Convert to UnrealObject (as_unreal_object=True means return an UnrealObject and discover its class; as_unreal_object=handle means return an UnrealObject and treat handle as its class)
def _to_unreal_object_as_proxy(obj, unreal_service, sp_func_service, config, as_unreal_object=None, with_sp_funcs=None):
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
    elif isinstance(as_unreal_object, spear.SpHandle):
        assert as_unreal_object.ptr != 0
        uclass = as_unreal_object
    else:
        assert False
    return _to_unreal_proxy(obj=obj, create_func=lambda obj: get_unreal_object(unreal_service=unreal_service, sp_func_service=sp_func_service, config=config, uobject=obj, uclass=uclass, with_sp_funcs=with_sp_funcs))

# Convert to a proxy type (UnrealStruct, UnrealClass, or UnrealObject) using create_func
def _to_unreal_proxy(obj, create_func):
    if isinstance(obj, bool):
        assert False
    elif isinstance(obj, numbers.Integral):
        return create_func(obj=obj)
    elif isinstance(obj, str):
        return create_func(obj=obj)
    elif isinstance(obj, np.ndarray):
        assert obj.dtype == np.uint64
        return _to_unreal_proxy(obj=obj.tolist(), create_func=create_func)
    elif isinstance(obj, spear.SpHandle):
        return create_func(obj=obj)
    elif isinstance(obj, list):
        return [ _to_unreal_proxy(obj=o, create_func=create_func) for o in obj ]
    elif isinstance(obj, CaseInsensitiveDict): # must be before dict, because CaseInsensitiveDict inherits from dict
        return to_case_insensitive(obj={ k: _to_unreal_proxy(obj=v, create_func=create_func) for k, v in obj.items() }, recursive=False, get_key_func=obj._get_key_func)
    elif isinstance(obj, dict):
        return { k: _to_unreal_proxy(obj=v, create_func=create_func) for k, v in obj.items() }
    else:
        assert False


#
# Converters for handles, used for both args and return values.
#

# Convert to handle
def to_handle(obj):
    if isinstance(obj, bool):
        assert False
    elif isinstance(obj, numbers.Integral):
        handle = spear.SpHandle()
        handle.ptr = int(obj) # obj might be a NumPy integer
        return handle
    elif isinstance(obj, str):
        return to_handle_from_string(string=obj)
    elif isinstance(obj, np.ndarray):
        assert obj.dtype == np.uint64
        return obj
    elif isinstance(obj, spear.SpHandle):
        return obj
    elif isinstance(obj, spear.UnrealStruct):
        return obj.ustruct
    elif isinstance(obj, spear.UnrealClass):
        return obj.uclass
    elif isinstance(obj, spear.UnrealObject):
        return obj.uobject
    elif isinstance(obj, list):
        return [ to_handle(obj=o) for o in obj ]
    elif isinstance(obj, CaseInsensitiveDict): # must be before dict, because CaseInsensitiveDict inherits from dict
        return to_case_insensitive(obj={ k: to_handle(obj=v) for k, v in obj.items() }, recursive=False, get_key_func=obj._get_key_func)
    elif isinstance(obj, dict):
        return { k: to_handle(obj=v) for k, v in obj.items() }
    elif spear.__can_import_unreal__ and isinstance(obj, unreal.Object):
        return to_handle_from_string(string=unreal.SpFuncUtils.to_string_from_object(object=obj))
    else:
        assert False

# Convert to a handle from a long or short string, see to_long_string_from_handle(...) and to_short_string_from_handle(...)
def to_handle_from_string(string):
    assert isinstance(string, str)
    tokens = string.split(":")
    assert len(tokens) == 1 or len(tokens) == 3
    assert tokens[0].startswith("0x")
    handle = spear.SpHandle()
    handle.ptr = int(tokens[0], 16)
    if len(tokens) == 3:
        handle.object_index = int(tokens[1])
        handle.object_serial_number = int(tokens[2])
    return handle


#
# Converters for handles, used only for args
#

# Convert to a long string of the form "0x<ptr>:<object_index>:<object_serial_number>" from a handle. This is the
# form expected by UnrealUtils::toPtrFromString(...), because it can be validated in the same way as an SpHandle.
def to_long_string_from_handle(handle):
    assert isinstance(handle, spear.SpHandle)
    return f"{to_short_string_from_handle(handle=handle)}:{handle.object_index}:{handle.object_serial_number}"

# Convert to a short string of the form "0x<ptr>" from a handle. UnrealUtils::toPtrFromString(...) also accepts this
# form, but it can't be validated as strongly.
def to_short_string_from_handle(handle):
    assert isinstance(handle, spear.SpHandle)
    return f"{handle.ptr:#0{18}x}"


#
# Converters for args
#

# Convert to a Shared object that can be passed to a service from a NumPy array backed by shared memory.
def to_shared(array, shared_memory_handle):
    return Shared(array=array, shared_memory_handle=shared_memory_handle)

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

# Convert to a collection of packed arrays from a collection of NumPy arrays.
def to_packed_arrays(arrays, dest_byte_order, usage_flags):
    if isinstance(arrays, list):
        return [ to_packed_array(array=a, dest_byte_order=dest_byte_order, usage_flags=usage_flags) for a in arrays ]
    elif isinstance(arrays, dict):
        return { k: to_packed_array(array=v, dest_byte_order=dest_byte_order, usage_flags=usage_flags) for k, v in arrays.items() }
    else:
        assert False

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
    elif isinstance(obj, np.generic):
        return to_json_string(obj=to_scalar(obj=obj), stringify=stringify)
    elif isinstance(obj, np.ndarray):
        return to_json_string(obj=obj.tolist(), stringify=stringify)
    elif isinstance(obj, spear.SpHandle):
        return to_long_string_from_handle(handle=obj)
    elif isinstance(obj, spear.UnrealStruct):
        return to_json_string(obj=obj.ustruct)
    elif isinstance(obj, spear.UnrealClass):
        return to_json_string(obj=obj.uclass)
    elif isinstance(obj, spear.UnrealObject):
        # Only a top-level UnrealObject without a property name can be converted, because the conversion only
        # preserves the UObject handle.
        assert obj.is_top_level()
        assert obj.property_name == ""
        return to_json_string(obj=obj.uobject)
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

# Convert to a Python scalar from a NumPy scalar. All other objects are returned unmodified.
def to_scalar(obj):
    if isinstance(obj, np.generic):
        return obj.item()
    else:
        return obj

# Unreal collapses JSON object keys that differ only by case (e.g., "a" and "A"), so we assert that the JSON keys are
# still unique, ignoring case, after converting them to strings. This also detects distinct keys that convert to the
# same string (e.g., an SpHandle and an UnrealObject that refer to the same UObject). This assert is the set-side equivalent
# of the assert in UnrealUtils::getPropertyValueAsString(...) for TMap keys.
def _to_json_validated_dict(dict_):
    assert all(isinstance(k, str) for k in dict_.keys()) or not any(isinstance(k, str) for k in dict_.keys()) # keys must not mix strings and non-strings
    json_dict = { to_json_string(obj=k, stringify=True): to_json_string(obj=v, stringify=False) for k, v in dict_.items() }
    assert len(json_dict) == len(dict_)
    assert len({ k.lower() for k in json_dict.keys() }) == len(json_dict)
    return json_dict


#
# Converters for return values
#

# Convert to {a collection of arrays, a collection of unreal objects, an info string} from a data bundle.
def to_data_bundle_dict(data_bundle, src_byte_order, usage_flags, shared_memory_handles):
    return {
        "arrays": to_arrays(packed_arrays=data_bundle.packed_arrays, src_byte_order=src_byte_order, usage_flags=usage_flags, shared_memory_handles=shared_memory_handles),
        "unreal_objs": to_dicts(json_strings=data_bundle.unreal_obj_strings),
        "info": data_bundle.info}

# Convert to a collection of NumPy arrays from a collection of packed arrays.
def to_arrays(packed_arrays, src_byte_order, usage_flags, shared_memory_handles):
    if isinstance(packed_arrays, list):
        return [ to_array(packed_array=p, src_byte_order=src_byte_order, usage_flags=usage_flags, shared_memory_handles=shared_memory_handles) for p in packed_arrays ]
    elif isinstance(packed_arrays, dict):
        return { k: to_array(packed_array=v, src_byte_order=src_byte_order, usage_flags=usage_flags, shared_memory_handles=shared_memory_handles) for k, v in packed_arrays.items() }
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

# Convert to a collection of dicts from a collection of JSON strings returned by a service. If a string isn't valid JSON,
# return default_value in its place if default_value is not None, otherwise return the string unchanged.
def to_dicts(json_strings, default_value=None):
    if isinstance(json_strings, list):
        return [ to_dict(json_string=s, default_value=default_value) for s in json_strings ]
    elif isinstance(json_strings, dict):
        return { k: to_dict(json_string=v, default_value=default_value) for k, v in json_strings.items() }
    else:
        assert False

# Convert to a dict from a JSON string returned by a service. If the string isn't valid JSON, return default_value if
# default_value is not None, otherwise return the string unchanged.
def to_dict(json_string, default_value=None):
    try:
        return json.loads(json_string)
    except json.JSONDecodeError:
        if default_value is None:
            return json_string
        else:
            return default_value


#
# Converters for interoperability between SPEAR Python and Editor Python
#

# For passing values from SPEAR Python to Editor Python. Encode a SPEAR Python object into a script expression string
# of the form "spear.editor.from_script_expr(script_expr_dict={type_string, value})". When this string is evaluated in
# Editor Python, spear.editor.from_script_expr(...) decodes it into an equivalent Editor Python object. Used by
# PythonService.set(...) and for embedding SPEAR Python values in expressions and statements that are passed to
# PythonService.

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
        return to_long_string_from_handle(handle=to_handle(obj=obj))
    elif isinstance(obj, spear.UnrealClass):
        return to_long_string_from_handle(handle=to_handle(obj=obj))
    elif isinstance(obj, spear.UnrealObject):
        return to_long_string_from_handle(handle=to_handle(obj=obj))
    else:
        return obj

# For passing values from SPEAR Python to Editor Python. Encode a dict into a script expression string that evaluates
# to an Unreal struct of type type_string (e.g., "unreal.Vector") in Editor Python. This is needed because
# to_script_expr(...) would otherwise encode the dict as a dict.

def to_script_struct_expr(value, type_string):
    assert isinstance(value, dict)
    return to_script_expr(obj=ScriptStruct(value=value, type_string=type_string))

# For passing values from Editor Python to SPEAR Python. Decode a {type_string, value} script result dict, produced in
# Editor Python by spear.editor.to_script_result(...), into a SPEAR Python object. Used by
# PythonService.evaluate_expression(...). Unreal structs are returned as dicts, and Unreal struct, class, and object
# pointers are converted according to as_handle and as_unreal_*.

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
        return to_unreal_struct(obj=value, unreal_service=unreal_service, as_handle=as_handle, as_unreal_struct=as_unreal_struct)
    elif type_string == "UnrealClass":
        assert as_unreal_struct is None
        assert as_unreal_object is None
        assert with_sp_funcs is None
        return to_unreal_class(obj=value, unreal_service=unreal_service, as_handle=as_handle, as_unreal_class=as_unreal_class)
    elif type_string == "UnrealObject":
        assert as_unreal_struct is None
        assert as_unreal_class is None
        return to_unreal_object(obj=value, unreal_service=unreal_service, sp_func_service=sp_func_service, config=config, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)
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
