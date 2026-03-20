#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import cv2
import functools
import glob
import msgpackrpc
import numpy as np
import os
import pathlib
import posixpath
import subprocess
import sys
import unreal
import spear



#
# script() decorator and ScriptRunner
#

def script(func):
    @functools.wraps(func)
    def wrapper(*args, **kwargs):
        runner = ScriptRunner(func(*args, **kwargs))
        runner.start()
        return runner
    return wrapper


class ScriptRunner:
    def __init__(self, generator):
        assert generator is not None
        self._generator = generator
        self._running = False
        self._pre_tick_handle = unreal.register_slate_pre_tick_callback(self._pre_tick)
        self._post_tick_handle = unreal.register_slate_post_tick_callback(self._post_tick)

    def start(self):
        spear.log("Executing editor script: ", self._generator)
        self._running = True

    def stop(self):
        self._running = False
        unreal.unregister_slate_pre_tick_callback(self._pre_tick_handle)
        unreal.unregister_slate_post_tick_callback(self._post_tick_handle)
        spear.log("Finished executing editor script.")

    def _pre_tick(self, delta_time):
        assert self._running
        self._advance()

    def _post_tick(self, delta_time):
        assert self._running
        self._advance()

    def _advance(self):
        try:
            next(self._generator)
        except StopIteration:
            self.stop()

#
# Client
#

_to_data_type = {
    np.dtype("uint8"): "u1",
    np.dtype("int8"): "i1",
    np.dtype("uint16"): "u2",
    np.dtype("int16"): "i2",
    np.dtype("uint32"): "u4",
    np.dtype("int32"): "i4",
    np.dtype("uint64"): "u8",
    np.dtype("int64"): "i8",
    np.dtype("float16"): "f2",
    np.dtype("float32"): "f4",
    np.dtype("float64"): "f8"}

_to_dtype = {v: k for k, v in _to_data_type.items()}


class Client:
    def __init__(self, address, port, timeout, reconnect_limit):
        assert spear.__can_import_msgpackrpc__
        self.verbose_rpc_calls = False
        self._client = msgpackrpc.Client(
            msgpackrpc.Address(address, port),
            timeout=timeout,
            reconnect_limit=reconnect_limit)

    def initialize(self, address, port):
        assert False

    def terminate(self):
        self._client.close()

    def get_timeout(self):
        assert False

    def set_timeout(self, value):
        assert False

    def clear_timeout(self):
        assert False

    def __getattr__(self, name):
        if name.startswith("call_sync_on_worker_thread_as_"):
            return_as = name.removeprefix("call_sync_on_worker_thread_as_")
            return lambda func_name, *args: self._call_impl(name, func_name, return_as, *args)
        elif name.startswith("call_sync_on_game_thread_as_"):
            return_as = name.removeprefix("call_sync_on_game_thread_as_")
            return lambda func_name, *args: self._call_impl(name, func_name, return_as, *args)
        elif name.startswith("get_future_result_from_game_thread_as_"):
            return_as = name.removeprefix("get_future_result_from_game_thread_as_")
            return lambda future: self._call_impl(name, f"engine_service.get_future_result_from_game_thread_as_{return_as}", return_as, future)
        elif name.startswith("call_async_fast_") or name.startswith("send_async_fast_") or name.startswith("get_future_result_fast_"):
            assert False
        else:
            assert False

    def call_async_on_game_thread(self, func_name, *args):
        return self._call_impl("call_async_on_game_thread", func_name, "future", *args)

    def send_async_on_game_thread(self, func_name, *args):
        return self._call_impl("send_async_on_game_thread", func_name, "void", *args)

    def call_async_fast_on_worker_thread(self, func_name, *args):
        assert False

    def send_async_fast_on_worker_thread(self, func_name, *args):
        assert False

    def get_future_result_fast_from_worker_thread_as_uint64(self, future):
        assert False

    def call_async_fast_on_game_thread(self, func_name, *args):
        assert False

    def send_async_fast_on_game_thread(self, func_name, *args):
        assert False

    def get_future_result_fast_from_game_thread_as_uint64(self, future):
        assert False

    @staticmethod
    def get_entry_point_signature_type_descs():
        assert False

    @staticmethod
    def get_entry_point_signature_descs():
        assert False

    def _call_impl(self, method_name, func_name, return_as, *args):
        if self.verbose_rpc_calls:
            spear.log(f"[spear.editor.Client] Calling {method_name} -> {func_name}")
        result = self._client.call(func_name, *[ self._to_args(arg) for arg in args ])
        return self._to_return_values(obj=result, return_as=return_as)

    def _to_args(self, obj):
        if isinstance(obj, str):
            return obj.encode("utf-8")
        elif isinstance(obj, spear.PackedArray):
            packed_array_arg = self._to_packed_array_arg(obj)
            return self._to_args(packed_array_arg)
        elif isinstance(obj, ClientStruct):
            return { attribute_name: self._to_args(attribute_value) for attribute_name, attribute_value in vars(obj).items() }
        elif isinstance(obj, list):
            return [ self._to_args(o) for o in obj ]
        elif isinstance(obj, dict):
            return { self._to_args(k): self._to_args(v) for k, v in obj.items() }
        else:
            return obj

    def _to_packed_array_arg(self, packed_array):
        assert isinstance(packed_array.data, np.ndarray)
        assert packed_array.data_type == ""
        data_type = _to_data_type[np.dtype(packed_array.data.dtype)]
        if packed_array.data_source == "Internal":
            data = packed_array.data.tobytes()
        elif packed_array.data_source == "Shared":
            data = b""
        else:
            assert False
        return {"data": data, "data_source": packed_array.data_source, "shape": packed_array.shape, "data_type": data_type, "shared_memory_name": packed_array.shared_memory_name}

    def _to_return_values(self, obj, return_as):
        if return_as in ["void", "bool", "float", "int32", "int64", "uint64", "vector_of_uint64"]:
            return obj
        elif return_as in ["string", "vector_of_string", "map_of_string_to_uint64", "map_of_string_to_string"]:
            return self._to_str(obj)
        elif return_as == "vector_of_func_signature_type_desc":
            obj = self._to_dict(obj)
            return [ FuncSignatureTypeDesc(type_names=self._to_str(v["type_names"]), const_strings=self._to_str(v["const_strings"]), ref_strings=self._to_str(v["ref_strings"])) for v in obj ]
        elif return_as == "vector_of_static_struct_desc":
            obj = self._to_dict(obj)
            return [ StaticStructDesc(static_struct=v["static_struct"], name=self._to_str(v["name"]), ufunctions=self._to_str(v["ufunctions"])) for v in obj ]
        elif return_as == "map_of_string_to_property_value":
            obj = self._to_dict(obj)
            return { k: spear.PropertyValue(value=self._to_str(v["value"]), type_id=self._to_str(v["type_id"])) for k, v in obj.items() }
        elif return_as == "map_of_string_to_shared_memory_view":
            obj = self._to_dict(obj)
            return { k: SharedMemoryView(id=self._to_str(v["id"]), num_bytes=v["num_bytes"], offset_bytes=v["offset_bytes"], name=self._to_str(v["name"]), usage_flags=self._to_str(v["usage_flags"])) for k, v in obj.items() }
        elif return_as == "map_of_string_to_packed_array":
            obj = self._to_dict(obj)
            return { k: self._to_packed_array_return_value(v) for k, v in obj.items() }
        elif return_as == "map_of_string_to_vector_of_func_signature_desc":
            obj = self._to_dict(obj)
            return { k: [ FuncSignatureDesc(name=self._to_str(v["name"]), func_signature=[ FuncSignatureTypeDesc(type_names=self._to_str(s["type_names"]), const_strings=self._to_str(s["const_strings"]), ref_strings=self._to_str(s["ref_strings"])) for s in v["func_signature"] ], func_signature_id=v["func_signature_id"]) for v in values ] for k, values in obj.items() }
        elif return_as == "property_desc":
            obj = self._to_dict(obj)
            return PropertyDesc(property=obj["property"], value_ptr=obj["value_ptr"], type_id=self._to_str(obj["type_id"]))
        elif return_as == "property_value":
            obj = self._to_dict(obj)
            return spear.PropertyValue(value=self._to_str(obj["value"]), type_id=self._to_str(obj["type_id"]))
        elif return_as == "shared_memory_view":
            obj = self._to_dict(obj)
            return SharedMemoryView(id=self._to_str(obj["id"]), num_bytes=obj["num_bytes"], offset_bytes=obj["offset_bytes"], name=self._to_str(obj["name"]), usage_flags=self._to_str(obj["usage_flags"]))
        elif return_as == "packed_array":
            obj = self._to_dict(obj)
            return self._to_packed_array_return_value(obj)
        elif return_as == "data_bundle":
            obj = self._to_dict(obj)
            return spear.DataBundle(packed_arrays={ k: self._to_packed_array_return_value(v) for k, v in obj["packed_arrays"].items() }, unreal_obj_strings=self._to_str(obj["unreal_obj_strings"]), info=self._to_str(obj["info"]))
        elif return_as == "future":
            obj = self._to_dict(obj)
            return Future(future_ptr=obj["future_ptr"], type_id=self._to_str(obj["type_id"]))
        elif return_as == "static_struct_desc":
            obj = self._to_dict(obj)
            return StaticStructDesc(static_struct=obj["static_struct"], name=self._to_str(obj["name"]), ufunctions=self._to_str(obj["ufunctions"]))
        else:
            assert False

    def _to_str(self, obj):
        if isinstance(obj, bytes):
            return obj.decode("utf-8")
        elif isinstance(obj, list):
            return [ self._to_str(v) for v in obj ]
        elif isinstance(obj, dict):
            return { self._to_str(k): self._to_str(v) for k, v in obj.items() }
        else:
            return obj

    def _to_dict(self, obj):
        if isinstance(obj, list):
            return [ self._to_dict(v) for v in obj ]
        elif isinstance(obj, dict):
            return { self._to_str(k): self._to_dict(v) for k, v in obj.items() }
        else:
            return obj

    def _to_packed_array_return_value(self, obj):
        obj = self._to_dict(obj)
        if self._to_str(obj["data_source"]) == "Internal":
            data = np.frombuffer(obj["data"], dtype=_to_dtype[self._to_str(obj["data_type"])], count=-1).reshape(obj["shape"])
        elif self._to_str(obj["data_source"]) == "Shared":
            data = np.array([], dtype=_to_dtype[self._to_str(obj["data_type"])])
        else:
            assert False
        return spear.PackedArray(data=data, data_source=self._to_str(obj["data_source"]), shape=obj["shape"], data_type=self._to_str(obj["data_type"]), shared_memory_name=self._to_str(obj["shared_memory_name"]))


#
# ClientStruct and derived structs
#

class ClientStruct:
    pass

class PropertyDesc(ClientStruct):
    def __init__(self, property=0, value_ptr=0, type_id=""):
        self.property = property
        self.value_ptr = value_ptr
        self.type_id = type_id

    def __repr__(self):
        return f"spear.PropertyDesc(property={self.property}, value_ptr={self.value_ptr})"

class PropertyValue(ClientStruct):
    def __init__(self, value="", type_id=""):
        self.value = value
        self.type_id = type_id

    def __repr__(self):
        return f"spear.PropertyValue(value={self.value}, type_id={self.type_id})"

class SharedMemoryView(ClientStruct):
    def __init__(self, id="", num_bytes=0, offset_bytes=0, name="smem:invalid", usage_flags=None):
        usage_flags = usage_flags if usage_flags is not None else ["DoNotUse"]
        self.id = id
        self.num_bytes = num_bytes
        self.offset_bytes = offset_bytes
        self.name = name
        self.usage_flags = usage_flags

    def __repr__(self):
        return f"spear.SharedMemoryView(id={self.id}, num_bytes={self.num_bytes}, offset_bytes={self.offset_bytes}, usage_flags={self.usage_flags})"

class PackedArray(ClientStruct):
    def __init__(self, data=None, data_source="Invalid", shape=None, data_type="", shared_memory_name=""):
        shape = shape if shape is not None else []
        self.data = data
        self.data_source = data_source
        self.shape = shape
        self.data_type = data_type
        self.shared_memory_name = shared_memory_name

    def __repr__(self):
        return f'spear.PackedArray(data={self.data}, data_source="{self.data_source}", shape={self.shape}, data_type="{self.data_type}", shared_memory_name="{self.shared_memory_name}")'

class DataBundle(ClientStruct):
    def __init__(self, packed_arrays=None, unreal_obj_strings=None, info=""):
        packed_arrays = packed_arrays if packed_arrays is not None else {}
        unreal_obj_strings = unreal_obj_strings if unreal_obj_strings is not None else {}
        self.packed_arrays = packed_arrays
        self.unreal_obj_strings = unreal_obj_strings
        self.info = info

    def __repr__(self):
        return f"spear.DataBundle(packed_arrays={len(self.packed_arrays)}, unreal_obj_strings={len(self.unreal_obj_strings)}, info={len(self.info)})"

class FuncSignatureTypeDesc(ClientStruct):
    def __init__(self, type_names=None, const_strings=None, ref_strings=None):
        type_names = type_names if type_names is not None else {}
        const_strings = const_strings if const_strings is not None else {}
        ref_strings = ref_strings if ref_strings is not None else {}
        self.type_names = type_names
        self.const_strings = const_strings
        self.ref_strings = ref_strings

    def __repr__(self):
        return f"spear.FuncSignatureTypeDesc(type_names={self.type_names}, const_strings={self.const_strings}, ref_strings={self.ref_strings})"

class FuncSignatureDesc(ClientStruct):
    def __init__(self, name="", func_signature=None, func_signature_id=None):
        func_signature = func_signature if func_signature is not None else []
        func_signature_id = func_signature_id if func_signature_id is not None else []
        self.name = name
        self.func_signature = func_signature
        self.func_signature_id = func_signature_id

    def __repr__(self):
        return f"spear.FuncSignatureDesc(name='{self.name}', func_signature_id={self.func_signature_id})"

class Future(ClientStruct):
    def __init__(self, future_ptr=0, type_id=""):
        self.future_ptr = future_ptr
        self.type_id = type_id

    def __repr__(self):
        return f'spear.Future(future_ptr={self.future_ptr}, type_id="{self.type_id}")'

class StaticStructDesc(ClientStruct):
    def __init__(self, static_struct=0, name="", ufunctions=None):
        ufunctions = ufunctions if ufunctions is not None else {}
        self.static_struct = static_struct
        self.name = name
        self.ufunctions = ufunctions

    def __repr__(self):
        return f"spear.StaticStructDesc(name='{self.name}', static_struct={self.static_struct}, ufunctions={len(self.ufunctions)})"


#
# Module-level globals
#

asset_tools = unreal.AssetToolsHelpers.get_asset_tools()
editor_actor_subsystem = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
subobject_data_subsystem = unreal.get_engine_subsystem(unreal.SubobjectDataSubsystem)


#
# Get actors and components
#

def find_actors(actor_class=unreal.Actor):
    actors = editor_actor_subsystem.get_all_level_actors()
    actors = sorted(actors, key=lambda actor: get_stable_name_for_actor(actor=actor))
    if actor_class is not None:
        actors = [ a for a in actors if isinstance(a, actor_class) ]
    return actors

def find_actor(stable_name):
    actors = [ actor for actor in find_actors() if get_stable_name_for_actor(actor=actor) == stable_name ]
    if len(actors) == 1:
        return actors[0]
    else:
        return None

def get_components(actor, component_class=unreal.ActorComponent):
    components = []
    component_names = []

    # add main component hierarchy
    if actor.root_component is not None:
        candidate_components = [actor.root_component] + list(actor.root_component.get_children_components(include_all_descendants=True))
        candidate_components = [ c for c in candidate_components if get_stable_name_for_component(component=c) not in component_names ]
        candidate_components = [ c for c in candidate_components if isinstance(c, component_class) ]
        components = components + candidate_components
        component_names = component_names + [ get_stable_name_for_component(component=c) for c in candidate_components ]

    # add any components that are not in the main hierarchy, use component_names to make sure we're not
    # adding any components redundantly
    candidate_components = actor.get_components_by_class(component_class=component_class)
    candidate_components = [ c for c in candidate_components if get_stable_name_for_component(component=c) not in component_names ]
    candidate_components = [ c for c in candidate_components if isinstance(c, component_class) ]
    components = components + candidate_components
    component_names = component_names + [ get_stable_name_for_component(component=c) for c in candidate_components ]

    return components

def get_component(stable_name, actor=None):
    if actor is None:
        stable_actor_name, stable_component_name = stable_name.split(":")
        actor = find_actor(stable_name=stable_actor_name)
    else:
        stable_component_name = stable_name
    components = [ c for c in get_components(actor=actor) if get_stable_name_for_component(component=c) == stable_component_name ]
    if len(components) == 1:
        return components[0]
    else:
        return None

def get_stable_name_for_actor(actor):
    folder_path = actor.get_folder_path()
    if folder_path.is_none():
        return actor.get_actor_label()
    else:
        return posixpath.join(str(folder_path), actor.get_actor_label())

def get_stable_name_for_component(component, include_stable_actor_name=False):
    if include_stable_actor_name:
        actor_name_str = f"{get_stable_name_for_actor(actor=component.get_owner())}:"
    else:
        actor_name_str = ""

    if "get_parent_components" in dir(component):
        component_name_str = ".".join([ c.get_name() for c in list(component.get_parent_components())[::-1] ] + [component.get_name()])
    else:
        component_name_str = component.get_name()
    
    return actor_name_str + component_name_str


#
# Create blueprint asset
#

def create_blueprint_asset(asset_name, package_dir, parent_class):

    blueprint_factory = unreal.BlueprintFactory()
    blueprint_factory.set_editor_property("parent_class", parent_class)

    # asset_class should be set to None when creating a new blueprint asset
    blueprint_asset = asset_tools.create_asset(asset_name=asset_name, package_path=package_dir, asset_class=None, factory=blueprint_factory)
    assert isinstance(blueprint_asset, unreal.Blueprint)

    return blueprint_asset


#
# Add new subobject
#

def add_new_subobject_to_instance(parent_data_handle, subobject_name, subobject_class):
    add_new_subobject_params = unreal.AddNewSubobjectParams(parent_handle=parent_data_handle, new_class=subobject_class, blueprint_context=None)
    return add_new_subobject_using_params(add_new_subobject_params=add_new_subobject_params, subobject_name=subobject_name, subobject_class=subobject_class)

def add_new_subobject_to_blueprint_asset(blueprint_asset, parent_data_handle, subobject_name, subobject_class):
    assert isinstance(blueprint_asset, unreal.Blueprint)
    add_new_subobject_params = unreal.AddNewSubobjectParams(parent_handle=parent_data_handle, new_class=subobject_class, blueprint_context=blueprint_asset)
    return add_new_subobject_using_params(add_new_subobject_params=add_new_subobject_params, subobject_name=subobject_name, subobject_class=subobject_class)

def add_new_subobject_using_params(add_new_subobject_params, subobject_name, subobject_class):
    subobject_data_handle, fail_reason = subobject_data_subsystem.add_new_subobject(params=add_new_subobject_params)
    assert fail_reason.is_empty()
    subobject_data = unreal.SubobjectDataBlueprintFunctionLibrary.get_data(data_handle=subobject_data_handle)
    assert unreal.SubobjectDataBlueprintFunctionLibrary.is_valid(data=subobject_data)
    subobject_object = unreal.SubobjectDataBlueprintFunctionLibrary.get_object(data=subobject_data)
    assert isinstance(subobject_object, subobject_class)
    success = subobject_data_subsystem.rename_subobject(handle=subobject_data_handle, new_name=unreal.Text(subobject_name))
    assert success
    return {"data_handle": subobject_data_handle, "data": subobject_data, "object": subobject_object}


#
# Get subobject descs
#

def get_subobject_descs_for_instance(instance):
    assert isinstance(instance, unreal.Object)
    subobject_data_handles = subobject_data_subsystem.k2_gather_subobject_data_for_instance(instance)
    return get_subobject_descs_for_data_handles(subobject_data_handles)

def get_subobject_descs_for_blueprint_asset(blueprint_asset):
    assert isinstance(blueprint_asset, unreal.Blueprint)
    subobject_data_handles = subobject_data_subsystem.k2_gather_subobject_data_for_blueprint(blueprint_asset)
    return get_subobject_descs_for_data_handles(subobject_data_handles)

def get_subobject_descs_for_data_handles(subobject_data_handles):
    assert isinstance(subobject_data_handles, unreal.Array)
    return [ get_subobject_desc_for_data_handle(h) for h in subobject_data_handles ]

def get_subobject_desc_for_data_handle(subobject_data_handle):
    assert unreal.SubobjectDataBlueprintFunctionLibrary.is_handle_valid(data_handle=subobject_data_handle)
    subobject_data = unreal.SubobjectDataBlueprintFunctionLibrary.get_data(data_handle=subobject_data_handle)
    assert unreal.SubobjectDataBlueprintFunctionLibrary.is_valid(data=subobject_data)
    subobject_object = unreal.SubobjectDataBlueprintFunctionLibrary.get_object(data=subobject_data)
    return {"data_handle": subobject_data_handle, "data": subobject_data, "object": subobject_object}


#
# Get filesystem path from content path
#

def get_filesystem_path_from_content_path(content_path):

    content_path_tokens = pathlib.PurePosixPath(content_path).parts
    assert len(content_path_tokens) >= 2
    content_root = content_path_tokens[1]

    if content_root == "Game":
        filesystem_base_dir = unreal.Paths.project_content_dir()
    elif content_root == "Engine":
        filesystem_base_dir = unreal.Paths.engine_content_dir()
    else:
        filesystem_base_dir = unreal.PluginBlueprintLibrary.get_plugin_content_dir(plugin_name=content_root)
        assert filesystem_base_dir is not None

    if len(content_path_tokens) == 2:
        return filesystem_base_dir
    else:
        content_sub_path = os.path.join(*content_path_tokens[2:])
        filesystem_path = os.path.join(filesystem_base_dir, content_sub_path)
        if os.path.exists(filesystem_path) and os.path.isdir(filesystem_path):
            return filesystem_path
        else:
            content_file_tokens = content_path_tokens[-1].split(".")
            if len(content_file_tokens) == 1:
                filesystem_paths = glob.glob(os.path.join(filesystem_base_dir, *content_path_tokens[2:-1], f"{content_file_tokens[0]}.*"))
                if len(filesystem_paths) == 1:
                    return filesystem_paths[0]
                else:
                    return os.path.join(filesystem_base_dir, *content_path_tokens[2:])
            elif len(content_file_tokens) == 2:
                assert content_file_tokens[0] == content_file_tokens[1]
                filesystem_paths = glob.glob(os.path.join(filesystem_base_dir, *content_path_tokens[2:-1], f"{content_file_tokens[0]}.*"))
                if len(filesystem_paths) == 1:
                    return filesystem_paths[0]
                else:
                    return os.path.join(filesystem_base_dir, *content_path_tokens[2:])
            else:
                assert False


#
# Conversion functions
#

# Convert to a NumPy array from an unreal vector.
def to_numpy_array_from_vector(vector, as_matrix=None):
    if as_matrix is None:
        return np.array([vector.get_editor_property("x"), vector.get_editor_property("y"), vector.get_editor_property("z")])
    else:
        assert as_matrix
        return np.matrix([vector.get_editor_property("x"), vector.get_editor_property("y"), vector.get_editor_property("z")]).T

# Convert to an Unreal vector from a NumPy array or matrix.
def to_vector_from_numpy_array(array):
    if isinstance(array, np.matrix):
        assert array.shape == (3, 1)
        array = array.A1
    elif isinstance(array, np.ndarray):
        assert array.shape == (3,)
    else:
        assert False
    return unreal.Vector(x=array[0], y=array[1], z=array[2])


#
# imshow
#

def imshow(name, image, temp_dir=None, temp_file=None):
    if temp_dir is None:
        temp_dir = os.path.join(unreal.Paths.project_saved_dir(), "SPEAR")
    if temp_file is None:
        temp_file = f"{name}.png"
    os.makedirs(temp_dir, exist_ok=True)

    path = os.path.abspath(os.path.join(temp_dir, temp_file))
    spear.log("Saving image: ", path)
    cv2.imwrite(path, image)

    unreal_engine_dir = os.path.abspath(os.path.join(os.path.dirname(sys.executable), "..", "..", ".."))
    if sys.platform == "win32":
        python_bin = os.path.realpath(os.path.join(unreal_engine_dir, "Engine", "Binaries", "ThirdParty", "Python3", "Win64", "python.exe"))
    elif sys.platform == "darwin":
        python_bin = os.path.realpath(os.path.join(unreal_engine_dir, "Engine", "Binaries", "ThirdParty", "Python3", "Mac", "bin", "python3"))
    elif sys.platform == "linux":
        python_bin = os.path.realpath(os.path.join(unreal_engine_dir, "Engine", "Binaries", "ThirdParty", "Python3", "Linux", "bin", "python3"))
    else:
        assert False

    run_imshow = os.path.abspath(os.path.join(os.path.dirname(spear.__file__), "tools", "run_imshow.py"))
    cmd = [python_bin, run_imshow, "--path", path, "--name", name]
    spear.log("Executing: ", " ".join(cmd))
    subprocess.run(cmd, check=True)
