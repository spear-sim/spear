#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import spear


#
# The UnrealObject class provides a Pythonic interface to UObjects. UnrealObject is a thin layer on top of
# UnrealService and SpFuncService, and is only responsible for resolving Python attribute names to UFUNCTIONs,
# SpFuncs, and properties. UnrealObject doesn't perform any type conversions itself. Instead, it forwards args and
# as_* args unchanged to UnrealService (e.g., UnrealService.call_function(...) and
# UnrealService.get_property_value_for_object(...)) or SpFuncService, which perform all type conversions (see the
# comment above the UnrealService class in python/spear/services/unreal_service.py).
#

class UnrealObject:
    def __init__(self, unreal_service, sp_func_service, config, uobject=None, uclass=None, derived_info=None, property_name="", is_top_level=True, with_sp_funcs=False):

        assert unreal_service is not None
        assert sp_func_service is not None
        assert config is not None

        if is_top_level:
            assert unreal_service.is_top_level_service()
            assert sp_func_service.is_top_level_service()
        else:
            assert not unreal_service.is_top_level_service()
            assert not sp_func_service.is_top_level_service()

        self.property_name = property_name
        self._is_top_level = is_top_level
        self._unreal_service = unreal_service
        self._sp_func_service = sp_func_service
        self._config = config

        self._initialized_sp_funcs = False
        self._sp_func_names = None
        self._sp_func_shared_memory_handles = None

        if self._is_top_level:

            if uobject is None and uclass is None:
                assert False
            elif uobject is None and uclass is not None:
                self.uclass = self._unreal_service.to_uclass(uclass=uclass)
                assert self.uclass.ptr != 0
                self.uobject = self._unreal_service.get_default_object(uclass=self.uclass, create_if_needed=False, as_handle=True)
            elif uobject is not None and uclass is None:
                self.uobject = spear.to_handle(obj=uobject)
                assert self.uobject.ptr != 0
                self.uclass = self._unreal_service.get_class(uobject=self.uobject, as_handle=True)
            else:
                self.uobject = spear.to_handle(obj=uobject)
                self.uclass = self._unreal_service.to_uclass(uclass=uclass)

            assert self.uobject.ptr != 0
            assert self.uclass.ptr != 0

            self.derived_info = self._unreal_service.get_static_class_derived_info(uclass=self.uclass)

            if self.property_name == "":
                self.call_async = UnrealObject(
                    unreal_service=self._unreal_service.call_async,
                    sp_func_service=self._sp_func_service.call_async,
                    config=self._config,
                    uobject=self.uobject,
                    uclass=self.uclass,
                    derived_info=self.derived_info,
                    is_top_level=False)

                self.send_async = UnrealObject(
                    unreal_service=self._unreal_service.send_async,
                    sp_func_service=self._sp_func_service.send_async,
                    config=self._config,
                    uobject=self.uobject,
                    uclass=self.uclass,
                    derived_info=self.derived_info,
                    is_top_level=False)

        else:
            self.uobject = uobject
            self.uclass = uclass
            self.derived_info = derived_info

        if self._is_top_level:
            if with_sp_funcs:
                self.initialize_sp_funcs()
        else:
            assert not with_sp_funcs

    # SpFunc interface

    def initialize_sp_funcs(self):
        assert not self._initialized_sp_funcs

        has_functions = self._sp_func_service.has_functions(uobject=self.uobject)

        if has_functions:
            self._sp_func_names = self._sp_func_service.get_function_names(uobject=self.uobject)
            self._sp_func_shared_memory_handles = self._sp_func_service.create_shared_memory_handles_for_object(uobject=self.uobject)

        self.call_async._set_sp_funcs(sp_func_names=self._sp_func_names, sp_func_shared_memory_handles=self._sp_func_shared_memory_handles)
        self.send_async._set_sp_funcs(sp_func_names=self._sp_func_names, sp_func_shared_memory_handles=self._sp_func_shared_memory_handles)

        self._initialized_sp_funcs = True

    def terminate_sp_funcs(self):
        assert self._initialized_sp_funcs

        self._initialized_sp_funcs = False

        self.call_async._unset_sp_funcs()
        self.send_async._unset_sp_funcs()

        if self._sp_func_shared_memory_handles is not None:
            self._sp_func_service.destroy_shared_memory_handles_for_object(shared_memory_handles=self._sp_func_shared_memory_handles)
            self._sp_func_shared_memory_handles = None

        if self._sp_func_names is not None:
            self._sp_func_names = None

    def _set_sp_funcs(self, sp_func_names, sp_func_shared_memory_handles):
        self._sp_func_names = sp_func_names
        self._sp_func_shared_memory_handles = sp_func_shared_memory_handles
        self._initialized_sp_funcs = True

    def _unset_sp_funcs(self):
        self._sp_func_names = None
        self._sp_func_shared_memory_handles = None
        self._initialized_sp_funcs = False

    # interface for calling functions; this interface is not necessary most of the time because functions
    # with no spaces in their names can be accessed directly as Python functions, but this interface is
    # useful when the function name has spaces in its name

    def call(self, function_name, args=None, world_context_object="WorldContextObject", as_raw_dict=None, as_dict=None, as_value=None, as_handle=None, as_unreal_struct=None, as_unreal_class=None, as_unreal_object=None, with_sp_funcs=None, arrays=None, unreal_objs=None, info=""):
        args = args if args is not None else {}
        if function_name in self.derived_info["unqualified_function_descs"] or function_name in self.derived_info["qualified_function_descs"]:
            return self._call_ufunction(
                function_name=function_name,
                world_context_object=world_context_object,
                as_raw_dict=as_raw_dict,
                as_dict=as_dict,
                as_value=as_value,
                as_handle=as_handle,
                as_unreal_struct=as_unreal_struct,
                as_unreal_class=as_unreal_class,                
                as_unreal_object=as_unreal_object,
                with_sp_funcs=with_sp_funcs,
                **args)
        elif self._sp_func_names is not None and function_name in self._sp_func_names:
            return self._call_sp_func(
                function_name=function_name,
                arrays=arrays,
                unreal_objs=unreal_objs,
                info=info)
        else:
            assert False

    def _call_ufunction(self, function_name, world_context_object="WorldContextObject", as_raw_dict=None, as_dict=None, as_value=None, as_handle=None, as_unreal_struct=None, as_unreal_class=None, as_unreal_object=None, with_sp_funcs=None, **kwargs):
        if self._config.SPEAR.UNREAL_OBJECT.PRINT_CALL_DEBUG_INFO:
            spear.log(f"Calling uclass={self.derived_info['static_class_desc'].name}, function_name={function_name}, uobject={self.uobject}...")
            spear.log(f"    as_raw_dict:      {as_raw_dict}")
            spear.log(f"    as_dict:          {as_dict}")
            spear.log(f"    as_value:         {as_value}")
            spear.log(f"    as_handle:        {as_handle}")
            spear.log(f"    as_unreal_struct: {as_unreal_struct}")
            spear.log(f"    as_unreal_class:  {as_unreal_class}")
            spear.log(f"    as_unreal_object: {as_unreal_object}")
            spear.log(f"    with_sp_funcs:    {with_sp_funcs}")
            spear.log(f"    kwargs:           {kwargs}")
        if function_name in self.derived_info["unqualified_function_descs"]:
            ufunction = self.derived_info["unqualified_function_descs"][function_name].function
        elif function_name in self.derived_info["qualified_function_descs"]:
            ufunction = self.derived_info["qualified_function_descs"][function_name].function
        else:
            assert False
        return self._unreal_service.call_function(
            uobject=self.uobject,
            ufunction=ufunction,
            args=kwargs,
            world_context_object=world_context_object,
            as_raw_dict=as_raw_dict,
            as_dict=as_dict,
            as_value=as_value,
            as_handle=as_handle,
            as_unreal_struct=as_unreal_struct,
            as_unreal_class=as_unreal_class,
            as_unreal_object=as_unreal_object,
            with_sp_funcs=with_sp_funcs)

    def _call_sp_func(self, function_name, arrays=None, unreal_objs=None, info=""):
        if self._config.SPEAR.UNREAL_OBJECT.PRINT_CALL_DEBUG_INFO:
            spear.log(f"Calling uclass={self.derived_info['static_class_desc'].name}, sp_func={function_name}, uobject={self.uobject}...")
            spear.log(f"    arrays:      {arrays}")
            spear.log(f"    unreal_objs: {unreal_objs}")
            spear.log(f"    info:        {info}")
        if arrays is None:
            arrays = {}
        if unreal_objs is None:
            unreal_objs = {}
        if self._sp_func_shared_memory_handles is not None:
            uobject_shared_memory_handles = self._sp_func_shared_memory_handles
        else:
            uobject_shared_memory_handles = {}
        return self._sp_func_service.call_function(
            uobject=self.uobject,
            function_name=function_name,
            arrays=arrays,
            unreal_objs=unreal_objs,
            info=info,
            uobject_shared_memory_handles=uobject_shared_memory_handles)

    # interface for getting properties

    def get_properties(self, check_property_flags=None, skip_property_flags=None, force_no_skip=False):
        assert self.property_name == ""
        return self._unreal_service.get_properties_for_object(uobject=self.uobject, check_property_flags=check_property_flags, skip_property_flags=skip_property_flags, force_no_skip=force_no_skip)

    def set_properties(self, properties, notify_editor=False, check_property_flags=None, skip_property_flags=None, force_no_skip=False):
        assert self.property_name == ""
        return self._unreal_service.set_properties_for_object(uobject=self.uobject, properties=properties, notify_editor=notify_editor, check_property_flags=check_property_flags, skip_property_flags=skip_property_flags, force_no_skip=force_no_skip)

    def get(self, as_raw_value=None, as_value=None, as_handle=None, as_unreal_struct=None, as_unreal_class=None, as_unreal_object=None, with_sp_funcs=None):
        return self.get_property_value(property_name=self.property_name, as_raw_value=as_raw_value, as_value=as_value, as_handle=as_handle, as_unreal_struct=as_unreal_struct, as_unreal_class=as_unreal_class, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    # interface for getting and setting a specific property; this interface is not necessary most of the time
    # because properties with no spaces in their names can be accessed directly as Python attributes, but
    # this interface is useful when the property has spaces in its name

    def get_property_value(self, property_name, as_raw_value=None, as_value=None, as_handle=None, as_unreal_struct=None, as_unreal_class=None, as_unreal_object=None, with_sp_funcs=None):
        assert property_name != ""
        return self._unreal_service.get_property_value_for_object(
            uobject=self.uobject,
            property_name=property_name,
            as_raw_value=as_raw_value,
            as_value=as_value,
            as_handle=as_handle,
            as_unreal_struct=as_unreal_struct,
            as_unreal_class=as_unreal_class,
            as_unreal_object=as_unreal_object,
            with_sp_funcs=with_sp_funcs)

    def set_property_value(self, property_name, property_value, notify_editor=False):
        assert property_name != ""
        self._unreal_service.set_property_value_for_object(uobject=self.uobject, property_name=property_name, property_value=property_value, notify_editor=notify_editor)

    # property notification interface

    def pre_edit_change(self):
        if self.property_name == "":
            self._unreal_service.pre_edit_change(uobject=self.uobject)
        else:
            self._unreal_service.pre_edit_change_from_string(uobject=self.uobject, property_name=self.property_name)

    def post_edit_change(self):
        if self.property_name == "":
            self._unreal_service.post_edit_change(uobject=self.uobject)
        else:
            self._unreal_service.post_edit_change_from_string(uobject=self.uobject, property_name=self.property_name)

    # type checking interface

    def is_a(self, uclass):
        return self._unreal_service.to_uclass(uclass=uclass) in self.derived_info["static_class_desc"].derived_to_base_classes

    # Returns False for the call_async and send_async objects associated with an UnrealObject.
    def is_top_level(self):
        return self._is_top_level

    # interface for debug printing

    def print_debug_info(self, prefix=""):

        assert self._is_top_level

        kismet_system_library = UnrealObject(
            unreal_service=self._unreal_service,
            sp_func_service=self._sp_func_service,
            config=self._config,
            uclass="UKismetSystemLibrary")

        type_string = self._unreal_service.get_type_for_class_as_string(uclass=self.uclass)
        meta_uclass = self._unreal_service.get_class(uobject=self.uclass)
        meta_type_string = self._unreal_service.get_type_for_class_as_string(uclass=meta_uclass)

        is_actor = kismet_system_library.IsObjectOfSoftClass(Object=self, SoftClass="/Script/Engine.Actor")
        is_component = kismet_system_library.IsObjectOfSoftClass(Object=self, SoftClass="/Script/Engine.ActorComponent")

        spear.log(f"{prefix}Printing debug info: {self}")

        spear.log(f"{prefix}    Type: {type_string}")
        spear.log(f"{prefix}    Meta type: {meta_type_string}")

        if is_actor:
            spear.log(f"{prefix}    Actor: {self._unreal_service.get_stable_name_for_actor(actor=self)}")
            spear.log(f"{prefix}        Non-scene components: ")
            components = self._unreal_service.get_components_as_dict(actor=self)
            for component_name, component in components.items():
                is_scene_component = kismet_system_library.IsObjectOfSoftClass(Object=component, SoftClass="/Script/Engine.SceneComponent")
                if not is_scene_component:
                    component_class = self._unreal_service.get_class(uobject=component)
                    spear.log(f"{prefix}            {component_name} ({self._unreal_service.get_type_for_class_as_string(uclass=component_class)})")
            spear.log(f"{prefix}        Scene components: ")
            components = self._unreal_service.get_children_components_for_actor_as_dict(parent=self)
            for component_name, component in components.items():
                component_class = self._unreal_service.get_class(uobject=component)
                spear.log(f"{prefix}            {component_name} ({self._unreal_service.get_type_for_class_as_string(uclass=component_class)})")
        elif is_component:
            spear.log(f"{prefix}    Component: {self._unreal_service.get_stable_name_for_component(component=self)}")

        uclass = UnrealClass(
            unreal_service=self._unreal_service,
            uclass=self.uclass)

        uclass.print_debug_info(prefix=f"{prefix}    ")

    # built-in function overrides

    def __getattr__(self, attr_name):
        # Python looks up __dunder__ names to probe for protocol support, e.g., copy.deepcopy(...) looks up
        # __deepcopy__, and np.array(...) looks up __array_struct__. So we raise AttributeError for these names
        # instead of treating them as properties. Properties with these names can still be accessed via
        # get_property_value(...).
        if attr_name.startswith("__") and attr_name.endswith("__"):
            raise AttributeError(attr_name)
        else:
            assert self.uclass.ptr != 0
            assert self.uobject.ptr != 0
            assert attr_name not in ["call_async", "send_async"]

            # if self represents a UObject directly (i.e., it doesn't represent a possibly nested property), then we can call functions on it
            if self.property_name == "":
                # if attr_name is the name of a UFunction
                if attr_name in self.derived_info["unqualified_function_descs"]:
                    def call_ufunction(world_context_object="WorldContextObject", as_raw_dict=None, as_dict=None, as_value=None, as_handle=None, as_unreal_struct=None, as_unreal_class=None, as_unreal_object=None, with_sp_funcs=None, **kwargs):
                        return self._call_ufunction(
                            function_name=attr_name,
                            world_context_object=world_context_object,
                            as_raw_dict=as_raw_dict,
                            as_dict=as_dict,
                            as_value=as_value,
                            as_handle=as_handle,
                            as_unreal_struct=as_unreal_struct,
                            as_unreal_class=as_unreal_class,
                            as_unreal_object=as_unreal_object,
                            with_sp_funcs=with_sp_funcs,
                            **kwargs)
                    return call_ufunction

                # else if attr_name is the name of an SpFunc
                elif self._sp_func_names is not None and attr_name in self._sp_func_names:
                    def call_sp_func(arrays=None, unreal_objs=None, info=""):
                        return self._call_sp_func(
                            function_name=attr_name,
                            arrays=arrays,
                            unreal_objs=unreal_objs,
                            info=info)
                    return call_sp_func

            # if attr_name isn't a UFunction or an SpFunc, then assume it is a property, and return a new UnrealObject

            if self.property_name == "":
                property_name = attr_name
            else:
                property_name = self.property_name + "." + attr_name

            return UnrealObject(
                unreal_service=self._unreal_service,
                sp_func_service=self._sp_func_service,
                config=self._config,
                uobject=self.uobject,
                uclass=self.uclass,
                derived_info=self.derived_info,
                property_name=property_name,
                is_top_level=self._is_top_level)

    def __getitem__(self, item_index):
        assert self.property_name != ""

        if isinstance(item_index, str):
            property_name = f'{self.property_name}["{item_index}"]'
        else:
            property_name = f'{self.property_name}[{item_index}]'

        return UnrealObject(
            unreal_service=self._unreal_service,
            sp_func_service=self._sp_func_service,
            config=self._config,
            uobject=self.uobject,
            uclass=self.uclass,
            derived_info=self.derived_info,
            property_name=property_name,
            is_top_level=self._is_top_level)

    def __setattr__(self, attr_name, attr_value):
        if attr_name.startswith("_") or attr_name in ["uobject", "uclass", "property_name", "call_async", "send_async", "derived_info"]:
            super().__setattr__(attr_name, attr_value)
        else:
            assert self.uobject.ptr != 0
            assert self._is_top_level
            assert self._unreal_service.is_top_level_service()

            if self.property_name == "":
                property_name = attr_name
            else:
                property_name = self.property_name + "." + attr_name

            self._unreal_service.set_property_value_for_object(uobject=self.uobject, property_name=property_name, property_value=attr_value)

    # Two UnrealObjects are equal if they refer to the same UObject (and the same property path, and are both top-level or
    # both not top-level), regardless of whether they are distinct Python objects, so expressions like "actor in actors"
    # behave as expected.
    def __eq__(self, other):
        if isinstance(other, UnrealObject):
            return self.uobject == other.uobject and self.property_name == other.property_name and self._is_top_level == other._is_top_level
        else:
            return NotImplemented

    def __hash__(self):
        return hash((self.uobject, self.property_name, self._is_top_level))

    def __repr__(self):
        class_name = self.derived_info["static_class_desc"].name
        return f'UnrealObject(uobject={self.uobject}, uclass={self.uclass}, property_name="{self.property_name}", _is_top_level={self._is_top_level}, _class_name={class_name}, _initialized_sp_funcs={self._initialized_sp_funcs}, _sp_func_names={self._sp_func_names})'


#
# UnrealClass and UnrealStruct provide wrappers around UStruct and UClass objects. Most users will not need to instantiate these classes directly.
#

class UnrealStruct:
    def __init__(self, unreal_service, ustruct):
        assert unreal_service is not None
        self._unreal_service = unreal_service
        self.ustruct = spear.to_handle(obj=ustruct)
        assert self.ustruct.ptr != 0

    def print_debug_info(self, prefix=""):
        type_string = self._unreal_service.get_type_for_struct_as_string(ustruct=self.ustruct)
        meta_uclass = self._unreal_service.get_class(uobject=self.ustruct)
        meta_class_string = self._unreal_service.get_type_for_class_as_string(uclass=meta_uclass)

        spear.log(f"{prefix}Printing debug info: {self}")
        spear.log(f"{prefix}    Type: {self._unreal_service.get_type_for_struct_as_string(ustruct=self.ustruct)}")
        spear.log(f"{prefix}    Meta type: {meta_class_string}")

        spear.log(f"{prefix}    Properties for type: {type_string}")
        props = self._unreal_service.find_properties_as_dict(ustruct=self.ustruct)
        for name, prop in props.items():
            spear.log(f"{prefix}        Property: {name} ({self._unreal_service.get_type_for_property_as_string(prop=prop)})")

    # Two UnrealStructs are equal if they refer to the same UStruct, regardless of whether they are distinct Python objects.
    def __eq__(self, other):
        if isinstance(other, UnrealStruct):
            return self.ustruct == other.ustruct
        else:
            return NotImplemented

    def __hash__(self):
        return hash(self.ustruct)

    def __repr__(self):
        return f"UnrealStruct(ustruct={self.ustruct})"


class UnrealClass:
    def __init__(self, unreal_service, uclass):
        assert unreal_service is not None
        self._unreal_service = unreal_service
        self.uclass = spear.to_handle(obj=uclass)
        assert self.uclass.ptr != 0

    def print_debug_info(self, prefix=""):
        type_string = self._unreal_service.get_type_for_class_as_string(uclass=self.uclass)
        meta_uclass = self._unreal_service.get_class(uobject=self.uclass)
        meta_class_string = self._unreal_service.get_type_for_class_as_string(uclass=meta_uclass)

        spear.log(f"{prefix}Printing debug info: {self}")
        spear.log(f"{prefix}    Class: {type_string}")
        spear.log(f"{prefix}    Meta class: {meta_class_string}")

        spear.log(f"{prefix}    Class hierarchy:")
        current_uclass = self.uclass
        while current_uclass.ptr != 0:
            spear.log(f"{prefix}        {self._unreal_service.get_type_for_class_as_string(uclass=current_uclass)}")
            current_uclass = self._unreal_service.get_super_class(uclass=current_uclass, as_handle=True)

        spear.log(f"{prefix}    Functions:")
        current_uclass = self.uclass
        while current_uclass.ptr != 0:
            spear.log(f"{prefix}        Functions for class: {self._unreal_service.get_type_for_class_as_string(uclass=current_uclass)}")
            ufunctions = self._unreal_service.find_functions_as_dict(uclass=current_uclass, field_iteration_flags=["IncludeDeprecated"]) # exclude base classes
            for ufunction_name, ufunction in ufunctions.items():
                spear.log(f"{prefix}            Function: {ufunction_name}")
                props = self._unreal_service.find_properties_as_dict(ustruct=ufunction)
                for prop_name, prop in props.items():
                    property_flags = self._unreal_service.get_property_flags(prop=prop)
                    if "CPF_Parm" in property_flags and "CPF_ReturnParm" not in property_flags:
                        spear.log(f"{prefix}                Argument: {prop_name} ({self._unreal_service.get_type_for_property_as_string(prop=prop)})")
                for prop_name, prop in props.items():
                    property_flags = self._unreal_service.get_property_flags(prop=prop)
                    if "CPF_Parm" in property_flags and "CPF_ReturnParm" in property_flags:
                        spear.log(f"{prefix}                Return value: {prop_name} ({self._unreal_service.get_type_for_property_as_string(prop=prop)})")
                for prop_name, prop in props.items():
                    property_flags = self._unreal_service.get_property_flags(prop=prop)
                    if "CPF_Parm" not in property_flags and "CPF_ReturnParm" not in property_flags:
                        spear.log(f"{prefix}                Other property: {prop_name} ({self._unreal_service.get_type_for_property_as_string(prop=prop)})")
                for prop_name, prop in props.items():
                    property_flags = self._unreal_service.get_property_flags(prop=prop)
                    if "CPF_Parm" not in property_flags and "CPF_ReturnParm" in property_flags:
                        assert False
            current_uclass = self._unreal_service.get_super_class(uclass=current_uclass, as_handle=True)

        spear.log(f"{prefix}    Properties:")
        current_uclass = self.uclass
        while current_uclass.ptr != 0:
            spear.log(f"{prefix}        Properties for type: {self._unreal_service.get_type_for_class_as_string(uclass=current_uclass)}")
            props = self._unreal_service.find_properties_as_dict(ustruct=current_uclass, field_iteration_flags=["IncludeDeprecated"]) # exclude base classes
            for name, prop in props.items():
                spear.log(f"{prefix}            Property: {name} ({self._unreal_service.get_type_for_property_as_string(prop=prop)})")
            current_uclass = self._unreal_service.get_super_class(uclass=current_uclass, as_handle=True)

    # Two UnrealClasses are equal if they refer to the same UClass, regardless of whether they are distinct Python objects.
    def __eq__(self, other):
        if isinstance(other, UnrealClass):
            return self.uclass == other.uclass
        else:
            return NotImplemented

    def __hash__(self):
        return hash(self.uclass)

    def __repr__(self):
        return f"UnrealClass(uclass={self.uclass})"
