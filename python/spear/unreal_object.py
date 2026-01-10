#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import spear

#
# The UnrealObject class provides a Pythonic interface to Unreal objects.
#

class UnrealObject:
    def __init__(self, unreal_service, sp_func_service, config, uobject=None, uclass=None, static_class_desc=None, property_name="", is_top_level=True, with_sp_funcs=False):

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
                self.uobject = self._unreal_service.get_default_object(uclass=self.uclass, create_if_needed=False, as_handle=True)
            elif uobject is not None and uclass is None:
                self.uobject = spear.to_handle(obj=uobject)
                self.uclass = self._unreal_service.get_class(uobject=self.uobject)
            else:
                self.uobject = spear.to_handle(obj=uobject)
                self.uclass = self._unreal_service.to_uclass(uclass=uclass)

            assert self.uobject != 0
            assert self.uclass != 0

            self._static_class_desc = self._unreal_service.get_static_class_desc(uclass=self.uclass)

            self.call_async = UnrealObject(
                unreal_service=self._unreal_service.call_async,
                sp_func_service=self._sp_func_service.call_async,
                config=self._config,
                uobject=self.uobject,
                uclass=self.uclass,
                static_class_desc=self._static_class_desc,
                is_top_level=False)

            self.send_async = UnrealObject(
                unreal_service=self._unreal_service.send_async,
                sp_func_service=self._sp_func_service.send_async,
                config=self._config,
                uobject=self.uobject,
                uclass=self.uclass,
                static_class_desc=self._static_class_desc,
                is_top_level=False)

            self.call_async_fast = UnrealObject(
                unreal_service=self._unreal_service.call_async_fast,
                sp_func_service=self._sp_func_service.call_async_fast,
                config=self._config,
                uobject=self.uobject,
                uclass=self.uclass,
                static_class_desc=self._static_class_desc,
                is_top_level=False)

            self.send_async_fast = UnrealObject(
                unreal_service=self._unreal_service.send_async_fast,
                sp_func_service=self._sp_func_service.send_async_fast,
                config=self._config,
                uobject=self.uobject,
                uclass=self.uclass,
                static_class_desc=self._static_class_desc,
                is_top_level=False)

        else:
            self.uobject = uobject
            self.uclass = uclass
            self._static_class_desc = static_class_desc

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

        if has_functions:
            self._sp_func_shared_memory_handles = self._sp_func_service.create_shared_memory_handles_for_object(uobject=self.uobject)

        self.call_async._set_sp_funcs(sp_func_names=self._sp_func_names, sp_func_shared_memory_handles=self._sp_func_shared_memory_handles)
        self.send_async._set_sp_funcs(sp_func_names=self._sp_func_names, sp_func_shared_memory_handles=self._sp_func_shared_memory_handles)
        self.call_async_fast._set_sp_funcs(sp_func_names=self._sp_func_names, sp_func_shared_memory_handles=self._sp_func_shared_memory_handles)
        self.send_async_fast._set_sp_funcs(sp_func_names=self._sp_func_names, sp_func_shared_memory_handles=self._sp_func_shared_memory_handles)

        self._initialized_sp_funcs = True

    def terminate_sp_funcs(self):
        assert self._initialized_sp_funcs

        self._initialized_sp_funcs = False

        self.call_async._unset_sp_funcs()
        self.send_async._unset_sp_funcs()
        self.call_async_fast._unset_sp_funcs()
        self.send_async_fast._unset_sp_funcs()

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


    # interface for getting properties

    def get_properties(self):
        assert self.property_name == ""
        return self._unreal_service.get_properties_for_object(uobject=self.uobject)

    def set_properties(self, properties):
        assert self.property_name == ""
        return self._unreal_service.set_properties_for_object(uobject=self.uobject, properties=properties)

    def get(self, as_raw_value=None, as_value=None, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        assert self.property_name != ""
        result = self._unreal_service.get_property_value_for_object(uobject=self.uobject, property_name=self.property_name)
        if as_raw_value is not None:
            assert as_value is None
            assert as_handle is None
            assert as_unreal_object is None
            assert with_sp_funcs is None
            return result
        else:
            return self._try_to_handle_or_unreal_object(obj=result, as_value=as_value, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)


    # interface for debug printing

    def print_debug_info(self):

        assert self._is_top_level

        kismet_system_library = UnrealObject(
            unreal_service=self._unreal_service,
            sp_func_service=self._sp_func_service,
            config=self._config,
            uclass="UKismetSystemLibrary")

        is_actor = kismet_system_library.IsObjectOfSoftClass(Object=self, SoftClass="/Script/Engine.Actor")
        is_component = kismet_system_library.IsObjectOfSoftClass(Object=self, SoftClass="/Script/Engine.ActorComponent")

        if is_actor:
            spear.log("Printing debug info for actor: ", self._unreal_service.try_get_stable_name_for_actor(actor=self))
            spear.log("    Non-scene components: ")
            components = self._unreal_service.get_components_as_dict(actor=self)
            for component_name, component in components.items():
                is_scene_component = kismet_system_library.IsObjectOfSoftClass(Object=component, SoftClass="/Script/Engine.SceneComponent")
                if not is_scene_component:
                    component_class = self._unreal_service.get_class(uobject=component)
                    spear.log("        ", component_name, " (", self._unreal_service.get_type_for_class_as_string(uclass=component_class), ")")
            spear.log("    Scene components: ")
            components = self._unreal_service.get_children_components_for_actor_as_dict(parent=self)
            for component_name, component in components.items():
                component_class = self._unreal_service.get_class(uobject=component)
                spear.log("        ", component_name, " (", self._unreal_service.get_type_for_class_as_string(uclass=component_class), ")")

        elif is_component:
            spear.log("Printing debug info for component: ", self._unreal_service.get_stable_name_for_component(component=self))

        else:
            spear.log("Printing debug info for object: ", self)

        meta_uclass = self._unreal_service.get_class(uobject=self.uclass)

        spear.log("    Type: ", self._unreal_service.get_type_for_class_as_string(uclass=self.uclass))
        spear.log("    Meta type: ", self._unreal_service.get_type_for_class_as_string(uclass=meta_uclass))

        spear.log("    Type hierarchy: ")
        current_uclass = self.uclass
        while current_uclass != 0:
            spear.log("        ", self._unreal_service.get_type_for_class_as_string(uclass=current_uclass))
            current_uclass = self._unreal_service.get_super_class(uclass=current_uclass)

        spear.log("    Functions: ")
        current_uclass = self.uclass
        while current_uclass != 0:
            spear.log("        Functions for type: ", self._unreal_service.get_type_for_class_as_string(uclass=current_uclass))
            ufunctions = self._unreal_service.find_functions_as_dict(uclass=current_uclass, field_iteration_flags=["IncludeDeprecated"]) # exclude base classes
            for ufunction_name, ufunction in ufunctions.items():
                spear.log("            Function: ", ufunction_name)
                props = self._unreal_service.find_properties_for_function_as_dict(ufunction=ufunction)
                for prop_name, prop in props.items():
                    property_flags = self._unreal_service.get_property_flags(prop=prop)
                    assert "CPF_Parm" in property_flags
                    if "CPF_ReturnParm" not in property_flags:
                        spear.log("                Argument: ", prop_name, " (", self._unreal_service.get_type_for_property_as_string(prop=prop), ")")
                for prop_name, prop in props.items():
                    property_flags = self._unreal_service.get_property_flags(prop=prop)
                    assert "CPF_Parm" in property_flags
                    if "CPF_ReturnParm" in property_flags:
                        spear.log("                Return value: ", prop_name, " (", self._unreal_service.get_type_for_property_as_string(prop=prop), ")")
            current_uclass = self._unreal_service.get_super_class(uclass=current_uclass)

        spear.log("    Properties: ")
        current_uclass = self.uclass
        while current_uclass != 0:
            spear.log("        Properties for type: ", self._unreal_service.get_type_for_class_as_string(uclass=current_uclass))
            props = self._unreal_service.find_properties_for_class_as_dict(uclass=current_uclass, field_iteration_flags=["IncludeDeprecated"]) # exclude base classes
            for name, prop in props.items():
                spear.log("            Property: ", name, " (", self._unreal_service.get_type_for_property_as_string(prop=prop), ")")
            current_uclass = self._unreal_service.get_super_class(uclass=current_uclass)


    # built-in function overrides

    def __getattr__(self, attr_name):
        assert self.uclass != 0
        assert self.uobject != 0

        # if self represents a UObject directly (i.e., it doesn't represent a possibly nested property), then we can call functions on it
        if self.property_name == "":
            if attr_name in self._static_class_desc.ufunctions:
                def call_ufunc(*args, world_context_object="WorldContextObject", as_raw_dict=None, as_dict=None, as_value=None, as_handle=None, as_unreal_object=None, with_sp_funcs=None, **kwargs):
                    if self._config.SPEAR.UNREAL_OBJECT.PRINT_CALL_DEBUG_INFO:
                        spear.log(f"Calling uclass={self._static_class_desc.name}, ufunction={attr_name}, uobject={self.uobject}...")
                        spear.log(f"    as_raw_dict:      {as_raw_dict}")
                        spear.log(f"    as_dict:          {as_dict}")
                        spear.log(f"    as_value:         {as_value}")
                        spear.log(f"    as_handle:        {as_handle}")
                        spear.log(f"    as_unreal_object: {as_unreal_object}")
                        spear.log(f"    with_sp_funcs:    {with_sp_funcs}")
                        spear.log(f"    kwargs:           {kwargs}")
                    assert len(args) == 0
                    ufunction = self._static_class_desc.ufunctions[attr_name]
                    result = self._unreal_service.call_function(uobject=self.uobject, uclass=0, ufunction=ufunction, args=kwargs, world_context_object=world_context_object)
                    if as_raw_dict is not None:
                        assert as_dict is None
                        assert as_value is None
                        assert as_handle is None
                        assert as_unreal_object is None
                        assert with_sp_funcs is None
                        return result
                    elif as_dict is not None:
                        assert as_value is None
                        assert as_handle is None
                        assert as_unreal_object is None
                        assert with_sp_funcs is None
                        return self._unpack_property_values(obj=result)
                    else:
                        return_value = self._try_to_return_value(obj=result) # don't unpack yet because we still need PropertyValue metadata
                        return self._try_to_handle_or_unreal_object(obj=return_value, as_value=as_value, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)
                return call_ufunc

            elif self._sp_func_names is not None and attr_name in self._sp_func_names:
                def call_sp_func(arrays=None, unreal_objs=None, info=""):
                    if self._config.SPEAR.UNREAL_OBJECT.PRINT_CALL_DEBUG_INFO:
                        spear.log(f"Calling uclass={self._static_class_desc.name}, sp_func={attr_name}, uobject={self.uobject}...")
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
                        function_name=attr_name,
                        arrays=arrays,
                        unreal_objs=unreal_objs,
                        info=info,
                        uobject_shared_memory_handles=uobject_shared_memory_handles)
                return call_sp_func

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
            static_class_desc=self._static_class_desc,
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
            static_class_desc=self._static_class_desc,
            property_name=property_name,
            is_top_level=self._is_top_level)

    def __setattr__(self, attr_name, attr_value):
        if attr_name.startswith("_") or attr_name in ["uobject", "uclass", "property_name", "call_async", "send_async", "call_async_fast", "send_async_fast"]:
            super().__setattr__(attr_name, attr_value)
        else:
            assert self.uobject != 0
            assert self._is_top_level
            assert self._unreal_service.is_top_level_service()

            if self.property_name == "":
                property_name = attr_name
            else:
                property_name = self.property_name + "." + attr_name

            self._unreal_service.set_property_value_for_object(uobject=self.uobject, property_name=property_name, property_value=attr_value)

    def __repr__(self):
        return \
            f'UnrealObject(uobject={self.uobject}, uclass={self.uclass}, property_name="{self.property_name}", ' + \
            f'_is_top_level={self._is_top_level}, _static_class_desc={self._static_class_desc}, _initialized_sp_funcs={self._initialized_sp_funcs}, _sp_func_names={self._sp_func_names})'


    # private helper functions

    def _unpack_property_values(self, obj):
        if isinstance(obj, spear.PropertyValue):
            return obj.value
        elif isinstance(obj, list):
            return [ self._unpack_property_values(obj=o) for o in obj ]
        elif isinstance(obj, dict):
            return { k: self._unpack_property_values(obj=v) for k, v in obj.items() }
        elif isinstance(obj, spear.Future):
            def convert_func(o, s=self, inner_convert_func=obj.convert_func):
                if inner_convert_func is not None:
                    o = inner_convert_func(o)
                return s._unpack_property_values(obj=o)
            obj.convert_func = convert_func
            return obj
        else:
            assert False

    def _try_to_return_value(self, obj):
        if isinstance(obj, dict):
            if "ReturnValue" in obj:
                return obj["ReturnValue"]
            else:
                return None
        elif isinstance(obj, spear.Future):
            def convert_func(o, s=self, inner_convert_func=obj.convert_func):
                if inner_convert_func is not None:
                    o = inner_convert_func(o)
                return s._try_to_return_value(obj=o)
            obj.convert_func = convert_func
            return obj
        else:
            assert False

    def _try_to_handle_or_unreal_object(self, obj, as_value=None, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        if obj is None:
            assert as_value is None
            assert as_handle is None
            assert as_unreal_object is None
            assert with_sp_funcs is None
            return None
        elif isinstance(obj, spear.PropertyValue):
            if as_value is not None:
                assert as_handle is None
                assert as_unreal_object is None
                assert with_sp_funcs is None
                return obj.value
            elif as_handle is not None:
                assert as_unreal_object is None
                assert with_sp_funcs is None
                return spear.to_handle(obj=obj.value)
            elif as_unreal_object is not None:
                handle = spear.to_handle(obj=obj.value)
                return spear.to_unreal_object(
                    obj=handle,
                    unreal_service=self._unreal_service.get_top_level_service(),
                    sp_func_service=self._sp_func_service.get_top_level_service(),
                    config=self._config,
                    as_unreal_object=as_unreal_object,
                    with_sp_funcs=with_sp_funcs)
            elif self._is_uclass_pointer_or_container(type_id=obj.type_id):
                return spear.to_handle(obj=obj.value)             
            elif self._is_uobject_pointer_or_container(type_id=obj.type_id):
                handle = spear.to_handle(obj=obj.value)
                return spear.to_unreal_object(
                    obj=handle,
                    unreal_service=self._unreal_service.get_top_level_service(),
                    sp_func_service=self._sp_func_service.get_top_level_service(),
                    config=self._config,
                    as_unreal_object=as_unreal_object,
                    with_sp_funcs=with_sp_funcs)
            else:
                return obj.value
        elif isinstance(obj, spear.Future):
            def convert_func(o, s=self, inner_convert_func=obj.convert_func, as_value=as_value, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=as_handle):
                if inner_convert_func is not None:
                    o = inner_convert_func(o)
                return s._try_to_handle_or_unreal_object(obj=o, as_value=as_value, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)
            obj.convert_func = convert_func
            return obj
        else:
            assert False

    def _is_uclass_pointer_or_container(self, type_id):
        type_id = type_id.replace(" ", "")
        if type_id == "UClass*":
            return True
        elif type_id.startswith("TSubclassOf<") and type_id.endswith(">"):
            return True
        elif type_id.startswith("TSoftClassPtr<") and type_id.endswith(">"):
            return True
        elif type_id.startswith("TArray<") and type_id.endswith(">"):
            value_type_id = type_id.removeprefix("TArray<").removesuffix(">")
            return self._is_uclass_pointer_or_container(type_id=value_type_id)
        elif type_id.startswith("TMap<") and type_id.endswith(">"):
            value_type_id = type_id.removeprefix("TMap<").removesuffix(">").split(",", 1)[1] # split on the first comma only
            return self._is_uclass_pointer_or_container(type_id=value_type_id)
        elif type_id.startswith("TSet<") and type_id.endswith(">"):
            value_type_id = type_id.removeprefix("TSet<").removesuffix(">")
            return self._is_uclass_pointer_or_container(type_id=value_type_id)
        else:
            return False

    def _is_uobject_pointer_or_container(self, type_id):
        type_id = type_id.replace(" ", "")
        if type_id != "UClass*" and type_id.endswith("*"):
            return True
        elif type_id == "TScriptInterface":
            return True
        elif type_id.startswith("TObjectPtr<") and type_id.endswith(">"):
            return True
        elif type_id.startswith("TLazyObjectPtr<") and type_id.endswith(">"):
            return True
        elif type_id.startswith("TSoftObjectPtr<") and type_id.endswith(">"):
            return True
        elif type_id.startswith("TWeakObjectPtr<") and type_id.endswith(">"):
            return True
        elif type_id.startswith("TScriptInterface<") and type_id.endswith(">"):
            return True
        elif type_id.startswith("TArray<") and type_id.endswith(">"):
            value_type_id = type_id.removeprefix("TArray<").removesuffix(">")
            return self._is_uobject_pointer_or_container(type_id=value_type_id)
        elif type_id.startswith("TMap<") and type_id.endswith(">"):
            value_type_id = type_id.removeprefix("TMap<").removesuffix(">").split(",", 1)[1] # split on the first comma only
            return self._is_uobject_pointer_or_container(type_id=value_type_id)
        elif type_id.startswith("TSet<") and type_id.endswith(">"):
            value_type_id = type_id.removeprefix("TSet<").removesuffix(">")
            return self._is_uobject_pointer_or_container(type_id=value_type_id)
        else:
            return False
