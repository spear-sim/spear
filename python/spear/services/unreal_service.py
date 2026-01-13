#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import numbers
import spear


class UnrealService(spear.Service):
    def __init__(self, entry_point_caller, sp_func_service, config, parent_service=None, create_children_services=True):
        assert sp_func_service.is_top_level_service()

        # do this after initializing local state
        super().__init__(
            entry_point_caller=entry_point_caller,
            sp_func_service=sp_func_service,
            unreal_service=None,
            config=config,
            parent_service=parent_service,
            create_children_services=create_children_services)

        # we need to account for the fact that we passed unreal_service=None above
        self._private_unreal_service = self.get_top_level_service()

    def create_child_service(self, entry_point_caller, sp_func_service=None, unreal_service=None, config=None):
        assert self.is_top_level_service() # this function should only be called from the top-level service
        return UnrealService(
            entry_point_caller=entry_point_caller,
            sp_func_service=sp_func_service,
            config=config,
            parent_service=self,
            create_children_services=False)

    def initialize(self):
        assert self.is_top_level_service() # this function should only be called on the top-level service
        self._static_struct_descs = self.get_static_struct_descs()
        self._static_class_descs = self.get_static_class_descs()

        self.static_struct_descs_by_name = { desc.name: desc for desc in self._static_struct_descs }
        self.static_class_descs_by_name = { desc.name: desc for desc in self._static_class_descs }

    #
    # Get static struct descs
    #

    def get_static_struct_descs(self):
        return self.entry_point_caller.call_on_game_thread("get_static_struct_descs", None)

    def get_static_struct_desc(self, uclass):
        return self.entry_point_caller.call_on_game_thread("get_static_struct_desc", None, uclass)

    def get_static_class_descs(self):
        return self.entry_point_caller.call_on_game_thread("get_static_class_descs", None)

    def get_static_class_desc(self, uclass):
        return self.entry_point_caller.call_on_game_thread("get_static_class_desc", None, uclass)

    #
    # Get engine subsystem
    #

    def get_engine_subsystem(self, uclass, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("get_engine_subsystem_by_class", None, uclass)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    #
    # Get editor subsystem
    #

    def get_editor_subsystem(self, uclass, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("get_editor_subsystem_by_class", None, uclass)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    #
    # Get subsystem
    #

    def get_subsystem(self, subsystem_provider_class_name, subsystem_uclass, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        subsystem_uclass = self.to_uclass(uclass=subsystem_uclass)
        result = self.entry_point_caller.call_on_game_thread("get_subsystem_by_class", None, subsystem_provider_class_name, subsystem_uclass)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    #
    # Functions for static structs and classes and interfaces
    #

    def get_static_struct(self, ustruct):
        spear.log(ustruct)
        ustruct = self.to_ustruct(ustruct=ustruct)
        return self.entry_point_caller.get(obj=ustruct)

    def find_static_structs(self):
        return self.entry_point_caller.call_on_game_thread("find_static_structs", None)

    def find_static_structs_as_dict(self):
        return self.entry_point_caller.call_on_game_thread("find_static_structs_as_map", None)

    def find_static_classes(self):
        return self.entry_point_caller.call_on_game_thread("find_static_classes", None)

    def find_static_classes_as_dict(self):
        return self.entry_point_caller.call_on_game_thread("find_static_classes_as_map", None)

    def get_derived_classes(self, uclass, recursive=True):
        uclass = self.to_uclass(uclass=uclass)
        return self.entry_point_caller.call_on_game_thread("get_derived_classes", None, uclass, recursive)

    def get_derived_classes_as_dict(self, uclass, recursive=True):
        uclass = self.to_uclass(uclass=uclass)
        return self.entry_point_caller.call_on_game_thread("get_derived_classes_as_map", None, uclass, recursive)

    def get_static_class(self, uclass):
        if isinstance(uclass, bool):
            assert False
        if isinstance(uclass, numbers.Integral):
            return self.entry_point_caller.get(obj=uclass)
        elif isinstance(uclass, str):
            if uclass in self.get_top_level_service().static_class_descs_by_name:
                uclass = self.to_uclass(uclass=uclass)
                return self.entry_point_caller.get(obj=uclass)
            else:
                return self.entry_point_caller.call_on_game_thread("get_static_class", None, uclass)
        else:
            assert False

    def get_super_class(self, uclass):
        uclass = self.to_uclass(uclass=uclass)
        return self.entry_point_caller.call_on_game_thread("get_super_class", None, uclass)

    def get_class_flags(self, uclass):
        uclass = self.to_uclass(uclass=uclass)
        return self.entry_point_caller.call_on_game_thread("get_class_flags", None, uclass)

    def get_default_object(self, uclass, create_if_needed=True, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("get_default_object", None, uclass, create_if_needed)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def get_class(self, uobject):
        uobject = spear.to_handle(obj=uobject)
        return self.entry_point_caller.call_on_game_thread("get_class", None, uobject)

    #
    # Functions for getting C++ types as strings
    #

    def get_type_for_struct_as_string(self, ustruct):
        ustruct = self.to_ustruct(ustruct=ustruct)
        return self.entry_point_caller.call_on_game_thread("get_type_for_struct_as_string", None, ustruct)

    def get_type_for_class_as_string(self, uclass):
        uclass = self.to_uclass(uclass=uclass)
        return self.entry_point_caller.call_on_game_thread("get_type_for_struct_as_string", None, uclass)

    def get_type_for_property_as_string(self, prop):
        return self.entry_point_caller.call_on_game_thread("get_type_for_property_as_string", None, prop)

    #
    # Get property metadata for structs
    #

    def find_properties_for_struct(self, ustruct, field_iteration_flags=None):
        field_iteration_flags = field_iteration_flags if field_iteration_flags is not None else ["Default"]
        ustruct = self.to_ustruct(ustruct=ustruct)
        return self.entry_point_caller.call_on_game_thread("find_properties_for_struct", None, ustruct, field_iteration_flags)

    def find_properties_for_struct_by_flags_any(self, ustruct, property_flags, field_iteration_flags=None):
        field_iteration_flags = field_iteration_flags if field_iteration_flags is not None else ["Default"]
        ustruct = self.to_ustruct(ustruct=ustruct)
        return self.entry_point_caller.call_on_game_thread("find_properties_for_struct_by_flags_any", None, ustruct, property_flags, field_iteration_flags)

    def find_properties_for_struct_by_flags_all(self, ustruct, property_flags, field_iteration_flags=None):
        field_iteration_flags = field_iteration_flags if field_iteration_flags is not None else ["Default"]
        ustruct = self.to_ustruct(ustruct=ustruct)
        return self.entry_point_caller.call_on_game_thread("find_properties_for_struct_by_flags_all", None, ustruct, property_flags, field_iteration_flags)

    def find_properties_for_struct_as_dict(self, ustruct, field_iteration_flags=None):
        field_iteration_flags = field_iteration_flags if field_iteration_flags is not None else ["Default"]
        ustruct = self.to_ustruct(ustruct=ustruct)
        return self.entry_point_caller.call_on_game_thread("find_properties_for_struct_as_map", None, ustruct, field_iteration_flags)

    def find_properties_for_struct_by_flags_any_as_dict(self, ustruct, property_flags, field_iteration_flags=None):
        field_iteration_flags = field_iteration_flags if field_iteration_flags is not None else ["Default"]
        ustruct = self.to_ustruct(ustruct=ustruct)
        return self.entry_point_caller.call_on_game_thread("find_properties_for_struct_by_flags_any_as_map", None, ustruct, property_flags, field_iteration_flags)

    def find_properties_for_struct_by_flags_all_as_dict(self, ustruct, property_flags, field_iteration_flags=None):
        field_iteration_flags = field_iteration_flags if field_iteration_flags is not None else ["Default"]
        ustruct = self.to_ustruct(ustruct=ustruct)
        return self.entry_point_caller.call_on_game_thread("find_properties_for_struct_by_flags_all_as_map", None, ustruct, property_flags, field_iteration_flags)

    #
    # Get property metadata for classes
    #

    def find_properties_for_class(self, uclass, field_iteration_flags=None):
        field_iteration_flags = field_iteration_flags if field_iteration_flags is not None else ["Default"]
        uclass = self.to_uclass(uclass=uclass)
        return self.entry_point_caller.call_on_game_thread("find_properties_for_struct", None, uclass, field_iteration_flags)

    def find_properties_for_class_by_flags_any(self, uclass, property_flags, field_iteration_flags=None):
        field_iteration_flags = field_iteration_flags if field_iteration_flags is not None else ["Default"]
        uclass = self.to_uclass(uclass=uclass)
        return self.entry_point_caller.call_on_game_thread("find_properties_for_struct_by_flags_any", None, uclass, property_flags, field_iteration_flags)

    def find_properties_for_class_by_flags_all(self, uclass, property_flags, field_iteration_flags=None):
        field_iteration_flags = field_iteration_flags if field_iteration_flags is not None else ["Default"]
        uclass = self.to_uclass(uclass=uclass)
        return self.entry_point_caller.call_on_game_thread("find_properties_for_struct_by_flags_all", None, uclass, property_flags, field_iteration_flags)

    def find_properties_for_class_as_dict(self, uclass, field_iteration_flags=None):
        field_iteration_flags = field_iteration_flags if field_iteration_flags is not None else ["Default"]
        uclass = self.to_uclass(uclass=uclass)
        return self.entry_point_caller.call_on_game_thread("find_properties_for_struct_as_map", None, uclass, field_iteration_flags)

    def find_properties_for_class_by_flags_any_as_dict(self, uclass, property_flags, field_iteration_flags=None):
        field_iteration_flags = field_iteration_flags if field_iteration_flags is not None else ["Default"]
        uclass = self.to_uclass(uclass=uclass)
        return self.entry_point_caller.call_on_game_thread("find_properties_for_struct_by_flags_any_as_map", None, uclass, property_flags, field_iteration_flags)

    def find_properties_for_class_by_flags_all_as_dict(self, uclass, property_flags, field_iteration_flags=None):
        field_iteration_flags = field_iteration_flags if field_iteration_flags is not None else ["Default"]
        uclass = self.to_uclass(uclass=uclass)
        return self.entry_point_caller.call_on_game_thread("find_properties_for_struct_by_flags_all_as_map", None, uclass, property_flags, field_iteration_flags)

    #
    # Get property metadata for functions
    #

    def find_properties_for_function(self, ufunction, field_iteration_flags=None):
        field_iteration_flags = field_iteration_flags if field_iteration_flags is not None else ["Default"]
        return self.entry_point_caller.call_on_game_thread("find_properties_for_function", None, ufunction, field_iteration_flags)

    def find_properties_for_function_by_flags_any(self, ufunction, property_flags, field_iteration_flags=None):
        field_iteration_flags = field_iteration_flags if field_iteration_flags is not None else ["Default"]
        return self.entry_point_caller.call_on_game_thread("find_properties_for_function_by_flags_any", None, ufunction, property_flags, field_iteration_flags)

    def find_properties_for_function_by_flags_all(self, ufunction, property_flags, field_iteration_flags=None):
        field_iteration_flags = field_iteration_flags if field_iteration_flags is not None else ["Default"]
        return self.entry_point_caller.call_on_game_thread("find_properties_for_function_by_flags_all", None, ufunction, property_flags, field_iteration_flags)

    def find_properties_for_function_as_dict(self, ufunction, field_iteration_flags=None):
        field_iteration_flags = field_iteration_flags if field_iteration_flags is not None else ["Default"]
        return self.entry_point_caller.call_on_game_thread("find_properties_for_function_as_map", None, ufunction, field_iteration_flags)

    def find_properties_for_function_by_flags_any_as_dict(self, ufunction, property_flags, field_iteration_flags=None):
        field_iteration_flags = field_iteration_flags if field_iteration_flags is not None else ["Default"]
        return self.entry_point_caller.call_on_game_thread("find_properties_for_function_by_flags_any_as_map", None, ufunction, property_flags, field_iteration_flags)

    def find_properties_for_function_by_flags_all_as_dict(self, ufunction, property_flags, field_iteration_flags=None):
        field_iteration_flags = field_iteration_flags if field_iteration_flags is not None else ["Default"]
        return self.entry_point_caller.call_on_game_thread("find_properties_for_function_by_flags_all_as_map", None, ufunction, property_flags, field_iteration_flags)

    #
    # Helper functions for property metadata
    #

    def get_property_flags(self, prop):
        return self.entry_point_caller.call_on_game_thread("get_property_flags", None, prop)

    #
    # Get and set object properties
    #

    def get_properties_for_object(self, uobject):
        uobject = spear.to_handle(obj=uobject)
        convert_func = lambda result: spear.try_to_dict(json_string=result, default_value={})
        return self.entry_point_caller.call_on_game_thread("get_properties_for_object_as_string", convert_func, uobject)

    def get_properties_for_struct(self, value_ptr, ustruct):
        ustruct = self.to_ustruct(ustruct=ustruct)
        convert_func = lambda result: spear.try_to_dict(json_string=result, default_value={})
        return self.entry_point_caller.call_on_game_thread("get_properties_for_struct_as_string", convert_func, value_ptr, ustruct)

    def get_properties_for_class(self, value_ptr, uclass):
        uclass = self.to_uclass(uclass=uclass)
        convert_func = lambda result: spear.try_to_dict(json_string=result, default_value={})
        return self.entry_point_caller.call_on_game_thread("get_properties_for_struct_as_string", convert_func, value_ptr, ustruct)

    def set_properties_for_object(self, uobject, properties):
        uobject = spear.to_handle(obj=uobject)
        properties = spear.to_json_string(obj=properties)
        return self.entry_point_caller.call_on_game_thread("set_properties_for_object_from_string", None, uobject, properties)

    def set_properties_for_struct(self, value_ptr, ustruct, properties):
        ustruct = self.to_ustruct(ustruct=ustruct)
        properties = spear.to_json_string(obj=properties)
        return self.entry_point_caller.call_on_game_thread("set_properties_for_struct_from_string", None, value_ptr, ustruct, properties)

    def set_properties_for_class(self, value_ptr, uclass, properties):
        uclass = self.to_uclass(uclass=uclass)
        properties = spear.to_json_string(obj=properties)
        return self.entry_point_caller.call_on_game_thread("set_properties_for_struct_from_string", None, value_ptr, uclass, properties)

    #
    # Get and set individual property values using property descs
    #

    def find_property_desc_for_object(self, uobject, property_name):
        uobject = spear.to_handle(obj=uobject)
        return self.entry_point_caller.call_on_game_thread("find_property_desc_for_object", None, uobject, property_name)

    def find_property_desc_for_struct(self, value_ptr, ustruct, property_name):
        ustruct = self.to_ustruct(ustruct=ustruct)
        return self.entry_point_caller.call_on_game_thread("find_property_desc_for_struct", None, value_ptr, ustruct, property_name)

    def find_property_desc_for_class(self, value_ptr, uclass, property_name):
        uclass = self.to_uclass(uclass=uclass)
        return self.entry_point_caller.call_on_game_thread("find_property_desc_for_struct", None, value_ptr, uclass, property_name)

    def get_property_value_for_desc(self, property_desc):
        convert_func = lambda result: spear.try_to_dict(json_string=result)
        return self.entry_point_caller.call_on_game_thread("get_property_value_for_desc_as_string", convert_func, property_desc)

    def set_property_value_for_desc(self, property_desc, property_value):
        property_value = spear.to_json_string(obj=property_value)
        return self.entry_point_caller.call_on_game_thread("set_property_value_for_desc_from_string", None, property_desc, property_value)

    #
    # Get and set individual property values
    #

    def get_property_value_for_object(self, uobject, property_name):
        uobject = spear.to_handle(obj=uobject)
        convert_func = lambda result: spear.PropertyValue(value=spear.try_to_dict(json_string=result.value), type_id=result.type_id)
        return self.entry_point_caller.call_on_game_thread("get_property_value_for_object_as_string", convert_func, uobject, property_name)

    def get_property_value_for_struct(self, value_ptr, ustruct, property_name):
        ustruct = self.to_ustruct(ustruct=ustruct)
        convert_func = lambda result: spear.PropertyValue(value=spear.try_to_dict(json_string=result.value), type_id=result.type_id)
        return self.entry_point_caller.call_on_game_thread( "get_property_value_for_struct_as_string", convert_func, value_ptr, ustruct, property_name)

    def get_property_value_for_class(self, value_ptr, uclass, property_name):
        uclass = self.to_uclass(uclass=uclass)
        convert_func = lambda result: spear.PropertyValue(value=spear.try_to_dict(json_string=result.value), type_id=result.type_id)
        return self.entry_point_caller.call_on_game_thread( "get_property_value_for_struct_as_string", convert_func, value_ptr, uclass, property_name)

    def set_property_value_for_object(self, uobject, property_name, property_value):
        uobject = spear.to_handle(obj=uobject)
        property_value = spear.to_json_string(obj=property_value)
        return self.entry_point_caller.call_on_game_thread("set_property_value_for_object_from_string", None, uobject, property_name, property_value)

    def set_property_value_for_struct(self, value_ptr, ustruct, property_name, property_value):
        property_value = spear.to_json_string(obj=property_value)
        return self.entry_point_caller.call_on_game_thread("set_property_value_for_struct_from_string", None, value_ptr, ustruct, property_name, property_value)

    def set_property_value_for_class(self, value_ptr, uclass, property_name, property_value):
        uclass = self.to_uclass(uclass=uclass)
        property_value = spear.to_json_string(obj=property_value)
        return self.entry_point_caller.call_on_game_thread("set_property_value_for_struct_from_string", None, value_ptr, uclass, property_name, property_value)

    #
    # Find and call functions
    #

    def find_functions(self, uclass, field_iteration_flags=None):
        field_iteration_flags = field_iteration_flags if field_iteration_flags is not None else ["Default"]
        uclass = self.to_uclass(uclass=uclass)
        return self.entry_point_caller.call_on_game_thread("find_functions", None, uclass, field_iteration_flags)

    def find_functions_by_flags_any(self, uclass, function_flags, field_iteration_flags=None):
        field_iteration_flags = field_iteration_flags if field_iteration_flags is not None else ["Default"]
        uclass = self.to_uclass(uclass=uclass)
        return self.entry_point_caller.call_on_game_thread("find_functions_by_flags_any", None, uclass, function_flags, field_iteration_flags)

    def find_functions_by_flags_all(self, uclass, function_flags, field_iteration_flags=None):
        field_iteration_flags = field_iteration_flags if field_iteration_flags is not None else ["Default"]
        uclass = self.to_uclass(uclass=uclass)
        return self.entry_point_caller.call_on_game_thread("find_functions_by_flags_all", None, uclass, function_flags, field_iteration_flags)

    def find_functions_as_dict(self, uclass, field_iteration_flags=None):
        field_iteration_flags = field_iteration_flags if field_iteration_flags is not None else ["Default"]
        uclass = self.to_uclass(uclass=uclass)
        return self.entry_point_caller.call_on_game_thread("find_functions_as_map", None, uclass, field_iteration_flags)

    def find_functions_by_flags_any_as_dict(self, uclass, function_flags, field_iteration_flags=None):
        field_iteration_flags = field_iteration_flags if field_iteration_flags is not None else ["Default"]
        uclass = self.to_uclass(uclass=uclass)
        return self.entry_point_caller.call_on_game_thread("find_functions_by_flags_any_as_map", None, uclass, function_flags, field_iteration_flags)

    def find_functions_by_flags_all_as_dict(self, uclass, function_flags, field_iteration_flags=None):
        field_iteration_flags = field_iteration_flags if field_iteration_flags is not None else ["Default"]
        uclass = self.to_uclass(uclass=uclass)
        return self.entry_point_caller.call_on_game_thread("find_functions_by_flags_all_as_map", None, uclass, function_flags, field_iteration_flags)

    def find_function_by_name(self, uclass, function_name, field_iteration_flags=None):
        field_iteration_flags = field_iteration_flags if field_iteration_flags is not None else ["Default"]
        uclass = self.to_uclass(uclass=uclass)
        return self.entry_point_caller.call_on_game_thread("find_function_by_name", None, uclass, function_name, field_iteration_flags)

    def get_function_flags(self, ufunction):
        return self.entry_point_caller.call_on_game_thread("get_function_flags", ufunction)

    #
    # Interface for calling functions. When using this interface, pointers must be handled specially. For
    # example, suppose you want to call a function that takes a pointer as input and returns a pointer as
    # output. Suppose that you already have a handle that you would like to pass as input to the function,
    # that you obtained from another UnrealService function, e.g.,
    #
    #     arg_handle = unreal_service.find_actor_by_name(..., as_handle=True)
    #
    # You would invoke your desired function as follows. After executing this code, return_value_handle will
    # be in the correct form to pass into other functions in unreal_service.
    # 
    #     args = {"Actor": spear.to_ptr(handle=arg_handle)}
    #     data_bundle = unreal_service.call_function(uobject=uobject, ufunction=ufunction, args=args)
    #     return_handle = spear.to_handle(string=data_bundle["ReturnValue"])
    #

    def call_function(self, uobject=None, uclass=None, ufunction=None, args=None, world_context_object="WorldContextObject"):
        assert uclass is not None
        assert ufunction is not None

        args = args if args is not None else {}
        uobject = uobject if uobject is not None else 0

        uobject = spear.to_handle(obj=uobject)
        uclass = self.to_uclass(uclass=uclass)
        args = spear.to_json_strings(objs=args)

        if uobject != 0:
            assert uclass == 0
        if uclass != 0:
            assert uobject == 0

        convert_func = lambda result: { k: spear.PropertyValue(value=spear.try_to_dict(v.value), type_id=v.type_id) for k, v in result.items() }
        return self.entry_point_caller.call_on_game_thread("call_function", convert_func, uobject, uclass, ufunction, args, world_context_object)

    #
    # Spawn actor
    #

    def spawn_actor(self, uclass, location=None, rotation=None, spawn_parameters=None, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        location = location if location is not None else {"X": 0.0, "Y": 0.0, "Z": 0.0}
        rotation = rotation if rotation is not None else {"Pitch": 0.0, "Yaw": 0.0, "Roll": 0.0}
        spawn_parameters = spawn_parameters if spawn_parameters is not None else {}

        uclass = self.to_uclass(uclass=uclass)
        location = spear.to_json_string(obj=location)
        rotation = spear.to_json_string(obj=rotation)
        spawn_parameters = spawn_parameters.copy() # shallow copy to prevent this function from mutating user input

        if "TransformScaleMethod" not in spawn_parameters:
            spawn_parameters["TransformScaleMethod"] = "MultiplyWithRoot" # see Engine/Source/Runtime/Engine/Classes/Engine/World.h

        if "ObjectFlags" in spawn_parameters:
            object_flags = spawn_parameters.pop("ObjectFlags")
        else:
            object_flags = ["RF_Transactional"] # see Engine/Source/Runtime/Engine/Private/World.cpp

        spawn_parameters = spear.to_json_string(obj=spawn_parameters)

        result = self.entry_point_caller.call_on_game_thread("spawn_actor", None, uclass, location, rotation, spawn_parameters, object_flags)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    #
    # Destroy actor
    #

    def destroy_actor(self, actor, net_force=False, should_modify_level=True):
        actor = spear.to_handle(obj=actor)
        return self.entry_point_caller.call_on_game_thread("destroy_actor", None, actor, net_force, should_modify_level)

    #
    # Create new object
    #

    def new_object(self, uclass, outer=0, name="", object_flags=None, template=0, copy_transients_from_class_defaults=False, in_instance_graph=0, external_package=0, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        object_flags = object_flags if object_flags is not None else ["RF_NoFlags"]
        uclass = self.to_uclass(uclass=uclass)
        outer = spear.to_handle(obj=outer)
        result = self.entry_point_caller.call_on_game_thread("new_object", None, outer, uclass, name, object_flags, template, copy_transients_from_class_defaults, in_instance_graph, external_package)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    #
    # Load object and class
    #

    def load_object(self, uclass, outer=0, name="", filename="", load_flags=None, sandbox=0, instancing_context=0, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        load_flags = load_flags if load_flags is not None else ["LOAD_None"]
        uclass = self.to_uclass(uclass=uclass)
        outer = spear.to_handle(obj=outer)

        # it is not supported to construct an UnrealObject using a UClass instance as the backing UObject
        if uclass == self.to_uclass(uclass="UClass"):
            assert as_handle

        return self.static_load_object(
            uclass=uclass,
            in_outer=outer,
            name=name,
            filename=filename,
            load_flags=load_flags,
            sandbox=sandbox,
            allow_object_reconciliation=True,
            instancing_context=instancing_context,
            as_handle=as_handle,
            as_unreal_object=as_unreal_object,
            with_sp_funcs=with_sp_funcs) # no need to convert result because we're delegating to another service function

    def load_class(self, uclass, outer=0, name="", filename="", load_flags=None, sandbox=0):
        load_flags = load_flags if load_flags is not None else ["LOAD_None"]
        uclass = self.to_uclass(uclass=uclass)
        outer = spear.to_handle(obj=outer)
        return self.static_load_class(
            uclass=uclass,
            in_outer=outer,
            name=name,
            filename=filename,
            load_flags=load_flags,
            sandbox=sandbox) # no need to convert result because we're delegating to another service function

    def static_load_object(self, uclass, in_outer, name="", filename="", load_flags=None, sandbox=0, allow_object_reconciliation=True, instancing_context=0, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        load_flags = load_flags if load_flags is not None else ["LOAD_None"]
        uclass = self.to_uclass(uclass=uclass)
        in_outer = spear.to_handle(obj=in_outer)

        # it is not supported to construct an UnrealObject using a UClass instance as the backing UObject
        if uclass == self.to_uclass(uclass="UClass"):
            assert as_handle

        result = self.entry_point_caller.call_on_game_thread("static_load_object", None, uclass, in_outer, name, filename, load_flags, sandbox, allow_object_reconciliation, instancing_context)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def static_load_class(self, uclass, in_outer, name="", filename="", load_flags=None, sandbox=0):
        load_flags = load_flags if load_flags is not None else ["LOAD_None"]
        uclass = self.to_uclass(uclass=uclass)
        in_outer = spear.to_handle(obj=in_outer)
        return self.entry_point_caller.call_on_game_thread("static_load_class", None, uclass, in_outer, name, filename, load_flags, sandbox)

    #
    # Enable and disable garbage collection for uobject
    #

    def add_object_to_root(self, uobject):
        uobject = spear.to_handle(obj=uobject)
        return self.entry_point_caller.call_on_game_thread("add_object_to_root", None, uobject)

    def remove_object_from_root(self, uobject):
        uobject = spear.to_handle(obj=uobject)
        return self.entry_point_caller.call_on_game_thread("remove_object_from_root", None, uobject)

    #
    # Find, get, and set console variable
    #

    def find_console_variable_by_name(self, console_variable_name):
        return self.entry_point_caller.call_on_game_thread("find_console_variable_by_name", None, console_variable_name)

    def get_console_variable_value_as_bool(self, cvar):
        return self.entry_point_caller.call_on_game_thread("get_console_variable_value_as_bool", None, cvar)

    def get_console_variable_value_as_int(self, cvar):
        return self.entry_point_caller.call_on_game_thread("get_console_variable_value_as_int", None, cvar)

    def get_console_variable_value_as_float(self, cvar):
        return self.entry_point_caller.call_on_game_thread("get_console_variable_value_as_float", None, cvar)

    def get_console_variable_value_as_string(self, cvar):
        return self.entry_point_caller.call_on_game_thread("get_console_variable_value_as_string", None, cvar)

    def set_console_variable_value(self, cvar, value, set_by_flags=None):
        set_by_flags = set_by_flags if set_by_flags is not None else ["ECVF_SetByCode"]
        if isinstance(value, bool):
            return self.entry_point_caller.call_on_game_thread("set_console_variable_value_from_bool", None, cvar, value, set_by_flags)
        elif isinstance(value, numbers.Integral):
            return self.entry_point_caller.call_on_game_thread("set_console_variable_value_from_int", None, cvar, value, set_by_flags)
        elif isinstance(value, float):
            return self.entry_point_caller.call_on_game_thread("set_console_variable_value_from_float", None, cvar, value, set_by_flags)
        elif isinstance(value, str):
            return self.entry_point_caller.call_on_game_thread("set_console_variable_value_from_string", None, cvar, value, set_by_flags)
        else:
            assert False

    #
    # Execute console command
    #

    def execute_console_command(self, command):
        return self.entry_point_caller.call_on_game_thread("execute_console_command", None, command)

    #
    # Stable name helper functions
    #

    def has_stable_name(self, actor):
        actor = spear.to_handle(obj=actor)
        return self.entry_point_caller.call_on_game_thread("has_stable_name", None, actor)

    def get_stable_name_for_actor(self, actor):
        actor = spear.to_handle(obj=actor)
        return self.entry_point_caller.call_on_game_thread("get_stable_name_for_actor", None, actor)

    def try_get_stable_name_for_actor(self, actor):
        actor = spear.to_handle(obj=actor)
        return self.entry_point_caller.call_on_game_thread("try_get_stable_name_for_actor", None, actor)

    def get_stable_name_for_component(self, component, include_actor_name=False):
        component = spear.to_handle(obj=component)
        return self.entry_point_caller.call_on_game_thread("get_stable_name_for_component", None, component, include_actor_name)

    #
    # Get actor and component tags
    #

    def get_actor_tags(self, actor):
        actor = spear.to_handle(obj=actor)
        return self.entry_point_caller.call_on_game_thread("get_actor_tags", None, actor)

    def get_component_tags(self, component):
        component = spear.to_handle(obj=component)
        return self.entry_point_caller.call_on_game_thread("get_component_tags", None, component)

    #
    # Find actors unconditionally and return a list or dict
    #

    def find_actors(self, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        result = self.entry_point_caller.call_on_game_thread("find_actors", None)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def find_actors_as_dict(self, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        result = self.entry_point_caller.call_on_game_thread("find_actors_as_map", None)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    #
    # Get components unconditionally and return a list or dict
    #

    def get_components(self, actor, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        actor = spear.to_handle(obj=actor)
        result = self.entry_point_caller.call_on_game_thread("get_components", None, actor)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def get_components_as_dict(self, actor, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        actor = spear.to_handle(obj=actor)
        result = self.entry_point_caller.call_on_game_thread("get_components_as_map", None, actor)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    #
    # Get children components unconditionally and return a list or dict
    #

    def get_children_components_for_actor(self, parent, include_all_descendants=True, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        parent = spear.to_handle(obj=parent)
        result = self.entry_point_caller.call_on_game_thread("get_children_components_for_actor", None, parent, include_all_descendants)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def get_children_components_for_actor_as_dict(self, parent, include_all_descendants=True, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        parent = spear.to_handle(obj=parent)
        result = self.entry_point_caller.call_on_game_thread("get_children_components_for_actor_as_map", None, parent, include_all_descendants)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def get_children_components_for_scene_component(self, parent, include_all_descendants=True, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        parent = spear.to_handle(obj=parent)
        result = self.entry_point_caller.call_on_game_thread("get_children_components_for_scene_component", None, parent, include_all_descendants)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def get_children_components_for_scene_component_as_dict(self, parent, include_all_descendants=True, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        parent = spear.to_handle(obj=parent)
        result = self.entry_point_caller.call_on_game_thread("get_children_components_for_scene_component_as_map", None, parent, include_all_descendants)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    #
    # Find actors conditionally and return a list
    #

    def find_actors_by_name(self, actor_name, uclass, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("find_actors_by_name", None, uclass, actor_name)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def find_actors_by_tag(self, tag, uclass, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("find_actors_by_tag", None, uclass, tag)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def find_actors_by_tags_any(self, tags, uclass, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("find_actors_by_tags_any", None, uclass, tags)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def find_actors_by_tags_all(self, tags, uclass, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("find_actors_by_tags_all", None, uclass, tags)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def find_actors_by_class(self, uclass, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("find_actors_by_class", None, uclass)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    #
    # Find actors conditionally and return a dict
    #

    def find_actors_by_name_as_dict(self, actor_name, uclass, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("find_actors_by_name_as_map", None, uclass, actor_name)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def find_actors_by_tag_as_dict(self, tag, uclass, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("find_actors_by_tag_as_map", None, uclass, tag)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def find_actors_by_tags_any_as_dict(self, tags, uclass, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("find_actors_by_tags_any_as_map", None, uclass, tags)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def find_actors_by_tags_all_as_dict(self, tags, uclass, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("find_actors_by_tags_all_as_map", None, uclass, tags)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def find_actors_by_class_as_dict(self, uclass, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("find_actors_by_class_as_map", None, uclass)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    #
    # Find actor conditionally
    #

    def find_actor_by_name(self, actor_name, uclass, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("find_actor_by_name", None, uclass, actor_name)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def find_actor_by_tag(self, tag, uclass, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("find_actor_by_tag", None, uclass, tag)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def find_actor_by_tags_any(self, tags, uclass, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("find_actor_by_tags_any", None, uclass, tags)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def find_actor_by_tags_all(self, tags, uclass, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("find_actor_by_tags_all", None, uclass, tags)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def find_actor_by_class(self, uclass, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("find_actor_by_class", None, uclass)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    #
    # Get components conditionally and return a list
    #

    def get_components_by_name(self, actor, component_name, uclass, include_from_child_actors=False, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        actor = spear.to_handle(obj=actor)
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("get_components_by_name", None, uclass, actor, component_name, include_from_child_actors)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def get_components_by_path(self, actor, component_path, uclass, include_from_child_actors=False, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        actor = spear.to_handle(obj=actor)
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("get_components_by_path", None, uclass, actor, component_path, include_from_child_actors)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def get_components_by_tag(self, actor, tag, uclass, include_from_child_actors=False, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        actor = spear.to_handle(obj=actor)
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("get_components_by_tag", None, uclass, actor, tag, include_from_child_actors)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def get_components_by_tags_any(self, actor, tags, uclass, include_from_child_actors=False, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        actor = spear.to_handle(obj=actor)
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("get_components_by_tags_any", None, uclass, actor, tags, include_from_child_actors)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def get_components_by_tags_all(self, actor, tags, uclass, include_from_child_actors=False, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        actor = spear.to_handle(obj=actor)
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("get_components_by_tags_all", None, uclass, actor, tags, include_from_child_actors)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def get_components_by_class(self, actor, uclass, include_from_child_actors=False, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        actor = spear.to_handle(obj=actor)
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("get_components_by_class", None, uclass, actor, include_from_child_actors)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    #
    # Get components conditionally and return a dict
    #

    def get_components_by_name_as_dict(self, actor, component_name, uclass, include_from_child_actors=False, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        actor = spear.to_handle(obj=actor)
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("get_components_by_name_as_map", None, uclass, actor, component_name, include_from_child_actors)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def get_components_by_path_as_dict(self, actor, component_path, uclass, include_from_child_actors=False, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        actor = spear.to_handle(obj=actor)
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("get_components_by_path_as_map", None, uclass, actor, component_path, include_from_child_actors)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def get_components_by_tag_as_dict(self, actor, tag, uclass, include_from_child_actors=False, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        actor = spear.to_handle(obj=actor)
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("get_components_by_tag_as_map", None, uclass, actor, tag, include_from_child_actors)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def get_components_by_tags_any_as_dict(self, actor, tags, uclass, include_from_child_actors=False, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        actor = spear.to_handle(obj=actor)
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("get_components_by_tags_any_as_map", None, uclass, actor, tags, include_from_child_actors)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def get_components_by_tags_all_as_dict(self, actor, tags, uclass, include_from_child_actors=False, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        actor = spear.to_handle(obj=actor)
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("get_components_by_tags_all_as_map", None, uclass, actor, tags, include_from_child_actors)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def get_components_by_class_as_dict(self, actor, uclass, include_from_child_actors=False, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        actor = spear.to_handle(obj=actor)
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("get_components_by_class_as_map", None, uclass, actor, include_from_child_actors)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    #
    # Get component conditionally
    #

    def get_component_by_name(self, actor, component_name, uclass, include_from_child_actors=False, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        actor = spear.to_handle(obj=actor)
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("get_component_by_name", None, uclass, actor, component_name, include_from_child_actors)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def get_component_by_path(self, actor, component_path, uclass, include_from_child_actors=False, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        actor = spear.to_handle(obj=actor)
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("get_component_by_path", None, uclass, actor, component_path, include_from_child_actors)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def get_component_by_tag(self, actor, tag, uclass, include_from_child_actors=False, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        actor = spear.to_handle(obj=actor)
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("get_component_by_tag", None, uclass, actor, tag, include_from_child_actors)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def get_component_by_tags_any(self, actor, tags, uclass, include_from_child_actors=False, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        actor = spear.to_handle(obj=actor)
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("get_component_by_tags_any", None, uclass, actor, tags, include_from_child_actors)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def get_component_by_tags_all(self, actor, tags, uclass, include_from_child_actors=False, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        actor = spear.to_handle(obj=actor)
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("get_component_by_tags_all", None, uclass, actor, tags, include_from_child_actors)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def get_component_by_class(self, actor, uclass, include_from_child_actors=False, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        actor = spear.to_handle(obj=actor)
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("get_component_by_class", None, uclass, actor, include_from_child_actors)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    #
    # Get children components conditionally from an actor and return a list
    #

    def get_children_components_for_actor_by_name(self, parent, children_component_name, uclass, include_all_descendants=True, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        parent = spear.to_handle(obj=parent)
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("get_children_components_for_actor_by_name", None, uclass, parent, children_component_name, include_all_descendants)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def get_children_components_for_actor_by_tag(self, parent, tag, uclass, include_all_descendants=True, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        parent = spear.to_handle(obj=parent)
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("get_children_components_for_actor_by_tag", None, uclass, parent, tag, include_all_descendants)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def get_children_components_for_actor_by_tags_any(self, parent, tags, uclass, include_all_descendants=True, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        parent = spear.to_handle(obj=parent)
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("get_children_components_for_actor_by_tags_any", None, uclass, parent, tags, include_all_descendants)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def get_children_components_for_actor_by_tags_all(self, parent, tags, uclass, include_all_descendants=True, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        parent = spear.to_handle(obj=parent)
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("get_children_components_for_actor_by_tags_all", None, uclass, parent, tags, include_all_descendants)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def get_children_components_for_actor_by_class(self, parent, uclass, include_all_descendants=True, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        parent = spear.to_handle(obj=parent)
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("get_children_components_for_actor_by_class", None, uclass, parent, include_all_descendants)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    #
    # Get children components conditionally from an actor and return a dict
    #

    def get_children_components_for_actor_by_name_as_dict(self, parent, children_component_name, uclass, include_all_descendants=True, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        parent = spear.to_handle(obj=parent)
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("get_children_components_for_actor_by_name_as_map", None, uclass, parent, children_component_name, include_all_descendants)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def get_children_components_for_actor_by_tag_as_dict(self, parent, tag, uclass, include_all_descendants=True, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        parent = spear.to_handle(obj=parent)
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("get_children_components_for_actor_by_tag_as_map", None, uclass, parent, tag, include_all_descendants)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def get_children_components_for_actor_by_tags_any_as_dict(self, parent, tags, uclass, include_all_descendants=True, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        parent = spear.to_handle(obj=parent)
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("get_children_components_for_actor_by_tags_any_as_map", None, uclass, parent, tags, include_all_descendants)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def get_children_components_for_actor_by_tags_all_as_dict(self, parent, tags, uclass, include_all_descendants=True, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        parent = spear.to_handle(obj=parent)
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("get_children_components_for_actor_by_tags_all_as_map", None, uclass, parent, tags, include_all_descendants)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def get_children_components_for_actor_by_class_as_dict(self, parent, uclass, include_all_descendants=True, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        parent = spear.to_handle(obj=parent)
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("get_children_components_for_actor_by_class_as_map", None, uclass, parent, include_all_descendants)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    #
    # Get child component conditionally from an actor
    #

    def get_child_component_for_actor_by_name(self, parent, child_component_name, uclass, include_all_descendants=True, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        parent = spear.to_handle(obj=parent)
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("get_child_component_for_actor_by_name", None, uclass, parent, child_component_name, include_all_descendants)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def get_child_component_for_actor_by_tag(self, parent, tag, uclass, include_all_descendants=True, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        parent = spear.to_handle(obj=parent)
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("get_child_component_for_actor_by_tag", None, uclass, parent, tag, include_all_descendants)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def get_child_component_for_actor_by_tags_any(self, parent, tags, uclass, include_all_descendants=True, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        parent = spear.to_handle(obj=parent)
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("get_child_component_for_actor_by_tags_any", None, uclass, parent, tags, include_all_descendants)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def get_child_component_for_actor_by_tags_all(self, parent, tags, uclass, include_all_descendants=True, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        parent = spear.to_handle(obj=parent)
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("get_child_component_for_actor_by_tags_all", None, uclass, parent, tags, include_all_descendants)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def get_child_component_for_actor_by_class(self, parent, uclass, include_all_descendants=True, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        parent = spear.to_handle(obj=parent)
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("get_child_component_for_actor_by_class", None, uclass, parent, include_all_descendants)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    #
    # Get children components conditionally from a scene component and return a list
    #

    def get_children_components_for_scene_component_by_name(self, parent, child_component_name, uclass, include_all_descendants=True, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        parent = spear.to_handle(obj=parent)
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("get_children_components_for_scene_component_by_name", None, uclass, parent, child_component_name, include_all_descendants)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def get_children_components_for_scene_component_by_tag_from_scene_component(self, parent, tag, uclass, include_all_descendants=True, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        parent = spear.to_handle(obj=parent)
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("get_children_components_for_scene_component_by_tag", None, uclass, parent, tag, include_all_descendants)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def get_children_components_for_scene_component_by_tags_any_from_scene_component(self, parent, tags, uclass, include_all_descendants=True, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        parent = spear.to_handle(obj=parent)
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("get_children_components_for_scene_component_by_tags_any", None, uclass, parent, tags, include_all_descendants)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def get_children_components_for_scene_component_by_tags_all_from_scene_component(self, parent, tags, uclass, include_all_descendants=True, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        parent = spear.to_handle(obj=parent)
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("get_children_components_for_scene_component_by_tags_all", None, uclass, parent, tags, include_all_descendants)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def get_children_components_for_scene_component_by_class_from_scene_component(self, parent, uclass, include_all_descendants=True, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        parent = spear.to_handle(obj=parent)
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("get_children_components_for_scene_component_by_class", None, uclass, parent, include_all_descendants)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    #
    # Get children components conditionally from a scene component and return a dict
    #

    def get_children_components_for_scene_component_by_name_as_dict(self, parent, child_component_name, uclass, include_all_descendants=True, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        parent = spear.to_handle(obj=parent)
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("get_children_components_for_scene_component_by_name_as_map", None, uclass, parent, child_component_name, include_all_descendants)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def get_children_components_for_scene_component_by_tag_as_dict(self, parent, tag, uclass, include_all_descendants=True, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        parent = spear.to_handle(obj=parent)
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("get_children_components_for_scene_component_by_tag_as_map", None, uclass, parent, tag, include_all_descendants)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def get_children_components_for_scene_component_by_tags_any_as_dict(self, parent, tags, uclass, include_all_descendants=True, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        parent = spear.to_handle(obj=parent)
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("get_children_components_for_scene_component_by_tags_any_as_map", None, uclass, parent, tags, include_all_descendants)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def get_children_components_for_scene_component_by_tags_all_as_dict(self, parent, tags, uclass, include_all_descendants=True, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        parent = spear.to_handle(obj=parent)
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("get_children_components_for_scene_component_by_tags_all_as_map", None, uclass, parent, tags, include_all_descendants)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def get_children_components_for_scene_component_by_class_as_dict(self, parent, uclass, include_all_descendants=True, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        parent = spear.to_handle(obj=parent)
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("get_children_components_for_scene_component_by_class_as_map", None, uclass, parent, include_all_descendants)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    #
    # Get child component conditionally from a scene component
    #

    def get_child_component_for_scene_component_by_name(self, parent, child_component_name, uclass, include_all_descendants=True, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        parent = spear.to_handle(obj=parent)
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("get_child_component_for_scene_component_by_name", None, uclass, parent, child_component_name, include_all_descendants)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def get_child_component_for_scene_component_by_tag(self, parent, tag, uclass, include_all_descendants=True, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        parent = spear.to_handle(obj=parent)
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("get_child_component_for_scene_component_by_tag", None, uclass, parent, tag, include_all_descendants)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def get_child_component_for_scene_component_by_tags_any(self, parent, tags, uclass, include_all_descendants=True, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        parent = spear.to_handle(obj=parent)
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("get_child_component_for_scene_component_by_tags_any", None, uclass, parent, tags, include_all_descendants)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def get_child_component_for_scene_component_by_tags_all(self, parent, tags, uclass, include_all_descendants=True, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        parent = spear.to_handle(obj=parent)
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("get_child_component_for_scene_component_by_tags_all", None, uclass, parent, tags, include_all_descendants)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def get_child_component_for_scene_component_by_class(self, parent, uclass, include_all_descendants=True, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        parent = spear.to_handle(obj=parent)
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("get_child_component_for_scene_component_by_class", None, uclass, parent, include_all_descendants)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    #
    # Create component
    #

    def create_component(self, owner, component_name, uclass, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        owner = spear.to_handle(obj=owner)
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("create_component_outside_owner_constructor_by_class", None, uclass, owner, component_name)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def create_scene_component_for_actor(self, owner, scene_component_name, uclass, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        owner = spear.to_handle(obj=owner)
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("create_scene_component_outside_owner_constructor_for_actor_by_class", None, uclass, owner, scene_component_name)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def create_scene_component_for_object(self, owner, parent, scene_component_name, uclass, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        owner = spear.to_handle(obj=owner)
        parent = spear.to_handle(obj=parent)
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("create_scene_component_outside_owner_constructor_for_object_by_class", None, uclass, owner, parent, scene_component_name)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def create_scene_component_for_scene_component(self, owner, scene_component_name, uclass, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        owner = spear.to_handle(obj=owner)
        uclass = self.to_uclass(uclass=uclass)
        result = self.entry_point_caller.call_on_game_thread("create_scene_component_outside_owner_constructor_for_scene_component_by_class", None, uclass, owner, scene_component_name)
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    #
    # Destroy component
    #

    def destroy_component(self, component, promote_children=False):
        component = spear.to_handle(obj=component)
        return self.entry_point_caller.call_on_game_thread("destroy_component_outside_owner_constructor", None, component, promote_children)

    #
    # Helper functions for getting ustruct and uclass handles
    #

    def to_ustruct(self, ustruct):
        if isinstance(ustruct, bool):
            assert False
        elif isinstance(ustruct, numbers.Integral):
            return ustruct
        elif isinstance(ustruct, str):
            return self.get_top_level_service().static_struct_descs_by_name[ustruct].static_struct
        else:
            assert False

    def to_uclass(self, uclass):
        if isinstance(uclass, bool):
            assert False
        elif isinstance(uclass, numbers.Integral):
            return uclass
        elif isinstance(uclass, str):
            return self.get_top_level_service().static_class_descs_by_name[uclass].static_struct
        else:
            assert False
