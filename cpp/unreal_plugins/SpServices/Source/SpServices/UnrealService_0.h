//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <map>
#include <memory>  // std::unique_ptr
#include <string>
#include <utility> // std::move
#include <vector>

#include <Components/ActorComponent.h>
#include <Components/SceneComponent.h>
#include <Engine/World.h>                    // FActorSpawnParameters
#include <HAL/Platform.h>                    // uint64
#include <Math/Rotator.h>
#include <Math/Vector.h>
#include <StructUtils/UserDefinedStruct.h>
#include <UObject/Class.h>                   // FObjectInstancingGraph, UClass, UFunction, UScriptStruct, UStruct
#include <UObject/CoreNet.h>                 // UPackageMap
#include <UObject/LinkerInstancingContext.h> // FLinkerInstancingContext
#include <UObject/NameTypes.h>               // FName
#include <UObject/Object.h>                  // UObject
#include <UObject/ObjectMacros.h>            // ELoadFlags, EObjectFlags, EPropertyFlags
#include <UObject/Package.h>
#include <UObject/Script.h>                  // EFunctionFlags
#include <UObject/UnrealType.h>              // EFieldIterationFlags, FProperty
#include <UObject/UObjectGlobals.h>          // GetTransientPackage, NewObject, StaticLoadClass, StaticLoadObject

#include "SpCore/Assert.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"
#include "SpCore/UnrealClassRegistry.h"
#include "SpCore/UnrealUtils.h"

#include "SpServices/EntryPointBinder.h"
#include "SpServices/Service.h"
#include "SpServices/SpTypes.h"
#include "SpServices/UnrealServiceTypes.h"

class UnrealService_0 : public Service
{
public:
    UnrealService_0() = delete;
    UnrealService_0(CUnrealEntryPointBinder auto* unreal_entry_point_binder)
    {
        SP_ASSERT(unreal_entry_point_binder);

        //
        // Get static struct and static class descs
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_static_struct_descs",
            [this]() -> std::vector<SpStaticStructDesc> {

                std::vector<SpStaticStructDesc> static_struct_descs;
                std::vector<UScriptStruct*> script_structs = UnrealUtils::findStaticStructsByType<UScriptStruct>();

                for (auto script_struct : script_structs) {
                    SP_ASSERT(script_struct);

                    SpStaticStructDesc static_struct_desc;
                    static_struct_desc.static_struct_ = UnrealUtils::toHandle(script_struct);
                    if (script_struct->IsA(UUserDefinedStruct::StaticClass())) {
                        static_struct_desc.name_ = Unreal::getBlueprintTypeAsString(script_struct);
                    } else {
                        static_struct_desc.name_ = Unreal::getCppTypeAsString(script_struct);
                    }
                    static_struct_descs.push_back(std::move(static_struct_desc));
                }

                return static_struct_descs;
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_static_class_descs",
            [this]() -> std::vector<SpStaticClassDesc> {

                std::vector<SpStaticClassDesc> static_class_descs;
                std::vector<UClass*> static_classes = UnrealUtils::findStaticStructsByType<UClass>();

                for (auto static_class : static_classes) {
                    SP_ASSERT(static_class);
                    SpStaticClassDesc static_class_desc;
                    static_class_desc.static_class_ = UnrealUtils::toHandle(static_class);
                    static_class_desc.name_ = Unreal::getTypeAsString(static_class);

                    // populate derived-to-base class chain (includes the class itself)
                    UClass* current_static_class = static_class;
                    while (current_static_class) {
                        static_class_desc.derived_to_base_classes_.push_back(UnrealUtils::toHandle(current_static_class));
                        static_class_desc.derived_to_base_class_names_.push_back(Unreal::getTypeAsString(current_static_class));
                        current_static_class = current_static_class->GetSuperClass();
                    }

                    // leave function_descs_ blank as a performance optimization

                    static_class_descs.push_back(std::move(static_class_desc));
                }

                return static_class_descs;
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_static_struct_desc",
            [this](SpHandle& script_struct) -> SpStaticStructDesc {
                SP_ASSERT(script_struct.ptr_);
                UScriptStruct* script_struct_ptr = UnrealUtils::toPtr<UScriptStruct>(script_struct);
                SpStaticStructDesc static_struct_desc;
                static_struct_desc.static_struct_ = UnrealUtils::toHandle(script_struct_ptr);
                if (script_struct_ptr->IsA(UUserDefinedStruct::StaticClass())) {
                    static_struct_desc.name_ = Unreal::getBlueprintTypeAsString(script_struct_ptr);
                } else {
                    static_struct_desc.name_ = Unreal::getCppTypeAsString(script_struct_ptr);
                }
                return static_struct_desc;
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_static_class_desc",
            [this](SpHandle& uclass) -> SpStaticClassDesc {
                SP_ASSERT(uclass.ptr_);
                UClass* uclass_ptr = UnrealUtils::toPtr<UClass>(uclass);

                SpStaticClassDesc static_class_desc;
                static_class_desc.static_class_ = UnrealUtils::toHandle(uclass_ptr);
                static_class_desc.name_ = Unreal::getTypeAsString(uclass_ptr);

                // populate derived-to-base class chain (includes the class itself)
                UClass* current_static_class = uclass_ptr;
                while (current_static_class) {
                    static_class_desc.derived_to_base_classes_.push_back(UnrealUtils::toHandle(current_static_class));
                    static_class_desc.derived_to_base_class_names_.push_back(Unreal::getTypeAsString(current_static_class));
                    current_static_class = current_static_class->GetSuperClass();
                }

                // populate function descs with fully qualified names
                std::vector<UFunction*> ufunctions = UnrealUtils::findFunctions(uclass_ptr);
                for (auto ufunction : ufunctions) {
                    SP_ASSERT(ufunction);
                    SpFunctionDesc function_desc;
                    function_desc.function_ = UnrealUtils::toHandle(ufunction);
                    function_desc.function_name_ = Unreal::getUnrealName(ufunction);
                    UStruct* outer = Cast<UStruct>(ufunction->GetOuter());
                    SP_ASSERT(outer);
                    function_desc.static_class_ = UnrealUtils::toHandle(Cast<UClass>(outer));
                    function_desc.static_class_name_ = Unreal::getTypeAsString(outer);
                    std::string qualified_name = function_desc.static_class_name_ + "::" + function_desc.function_name_;
                    Std::insert(static_class_desc.function_descs_, std::move(qualified_name), std::move(function_desc));
                }

                return static_class_desc;
            });

        //
        // Get engine subsystem
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_engine_subsystem_by_class",
            [this](SpHandle& uclass) -> SpHandle {
                return UnrealUtils::toHandle(Unreal::getEngineSubsystemByClass(UnrealUtils::toPtr<UClass>(uclass)));
            });

        //
        // Get editor subsystem, WITH_EDITOR implementations in SpServicesEditor/UnrealServiceEditor.h
        //

        #if !WITH_EDITOR // defined in an auto-generated header
            unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_editor_subsystem_by_class",
                [this](SpHandle& uclass) -> SpHandle {
                    return SpHandle();
                });
        #endif

        //
        // Get subsystem
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_subsystem_by_class",
            [this](std::string& subsystem_provider_class_name, SpHandle& world, SpHandle& subsystem_uclass) -> SpHandle {
                return UnrealUtils::toHandle(UnrealClassRegistry::getSubsystemByClass(subsystem_provider_class_name, UnrealUtils::toPtr<UWorld>(world), UnrealUtils::toPtr<UClass>(subsystem_uclass)));
            });

        //
        // Functions for static structs and classes
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_static_structs",
            [this]() -> std::vector<SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::findStaticStructs());
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_static_structs_as_map",
            [this](bool& include_path) -> std::map<std::string, SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::findStaticStructsAsMap(include_path));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_struct_flags",
            [this](SpHandle& script_struct) -> std::vector<std::string> {
                SP_ASSERT(script_struct.ptr_);
                return Unreal::getStringsFromCombinedEnumFlagValueAs<ESpStructFlags>(UnrealUtils::toPtr<UScriptStruct>(script_struct)->StructFlags);
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_static_classes",
            [this]() -> std::vector<SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::findStaticClasses());
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_static_classes_as_map",
            [this](bool& include_path) -> std::map<std::string, SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::findStaticClassesAsMap(include_path));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_derived_classes",
            [this](SpHandle& uclass, bool& recursive) -> std::vector<SpHandle> {
                SP_ASSERT(uclass.ptr_);
                return UnrealUtils::toHandle(UnrealUtils::getDerivedClasses(UnrealUtils::toPtr<UClass>(uclass), recursive));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_derived_classes_as_map",
            [this](SpHandle& uclass, bool& recursive, bool& include_path) -> std::map<std::string, SpHandle> {
                SP_ASSERT(uclass.ptr_);
                return UnrealUtils::toHandle(UnrealUtils::getDerivedClassesAsMap(UnrealUtils::toPtr<UClass>(uclass), recursive, include_path));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_static_class",
            [this](std::string& class_name) -> SpHandle {
                return UnrealUtils::toHandle(UnrealClassRegistry::getStaticClass(class_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_super_class",
            [this](SpHandle& uclass) -> SpHandle {
                SP_ASSERT(uclass.ptr_);
                return UnrealUtils::toHandle(UnrealUtils::toPtr<UClass>(uclass)->GetSuperClass());
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_class_flags",
            [this](SpHandle& uclass) -> std::vector<std::string> {
                SP_ASSERT(uclass.ptr_);
                return Unreal::getStringsFromCombinedEnumFlagValueAs<ESpClassFlags>(UnrealUtils::toPtr<UClass>(uclass)->GetClassFlags());
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_default_object",
            [this](SpHandle& uclass, bool& create_if_needed) -> SpHandle {
                SP_ASSERT(uclass.ptr_);
                return UnrealUtils::toHandle(UnrealUtils::toPtr<UClass>(uclass)->GetDefaultObject(create_if_needed));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_class",
            [this](SpHandle& uobject) -> SpHandle {
                SP_ASSERT(uobject.ptr_);
                return UnrealUtils::toHandle(UnrealUtils::toPtr<UObject>(uobject)->GetClass());
            });

        //
        // Functions for getting C++ types as strings
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_type_for_struct_as_string",
            [this](SpHandle& ustruct) -> std::string {
                return Unreal::getTypeAsString(UnrealUtils::toPtr<UStruct>(ustruct));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_type_for_property_as_string",
            [this](SpHandle& property) -> std::string {
                return Unreal::getCppTypeAsString(UnrealUtils::toPtr<FProperty>(property));
            });

        //
        // Find functions and get function flags
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_functions",
            [this](SpHandle& uclass, std::vector<std::string>& field_iteration_strings) -> std::vector<SpHandle> {
                return UnrealUtils::toHandle(
                    UnrealUtils::findFunctions(
                        UnrealUtils::toPtr<UClass>(uclass),
                        Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_functions_as_map",
            [this](SpHandle& uclass, std::vector<std::string>& field_iteration_strings) -> std::map<std::string, SpHandle> {
                return UnrealUtils::toHandle(
                    UnrealUtils::findFunctionsAsMap(
                        UnrealUtils::toPtr<UClass>(uclass),
                        Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_functions_by_name",
            [this](SpHandle& uclass, std::string& function_name, std::vector<std::string>& field_iteration_strings) -> std::vector<SpHandle> {
                return UnrealUtils::toHandle(
                    UnrealUtils::findFunctionsByName(
                        UnrealUtils::toPtr<UClass>(uclass),
                        function_name,
                        Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_functions_by_flags_any",
            [this](SpHandle& uclass, std::vector<std::string>& function_flags, std::vector<std::string>& field_iteration_strings) -> std::vector<SpHandle> {
                return UnrealUtils::toHandle(
                    UnrealUtils::findFunctionsByFlagsAny(
                        UnrealUtils::toPtr<UClass>(uclass),
                        Unreal::getCombinedEnumFlagValueFromStringsAs<EFunctionFlags, ESpFunctionFlags>(function_flags),
                        Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_functions_by_flags_all",
            [this](SpHandle& uclass, std::vector<std::string>& function_flags, std::vector<std::string>& field_iteration_strings) -> std::vector<SpHandle> {
                return UnrealUtils::toHandle(
                    UnrealUtils::findFunctionsByFlagsAll(
                        UnrealUtils::toPtr<UClass>(uclass),
                        Unreal::getCombinedEnumFlagValueFromStringsAs<EFunctionFlags, ESpFunctionFlags>(function_flags),
                        Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_functions_by_name_as_map",
            [this](SpHandle& uclass, std::string& function_name, std::vector<std::string>& field_iteration_strings) -> std::map<std::string, SpHandle> {
                return UnrealUtils::toHandle(
                    UnrealUtils::findFunctionsByNameAsMap(
                        UnrealUtils::toPtr<UClass>(uclass),
                        function_name,
                        Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_functions_by_flags_any_as_map",
            [this](SpHandle& uclass, std::vector<std::string>& function_flags, std::vector<std::string>& field_iteration_strings) -> std::map<std::string, SpHandle> {
                return UnrealUtils::toHandle(
                    UnrealUtils::findFunctionsByFlagsAnyAsMap(
                        UnrealUtils::toPtr<UClass>(uclass),
                        Unreal::getCombinedEnumFlagValueFromStringsAs<EFunctionFlags, ESpFunctionFlags>(function_flags),
                        Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_functions_by_flags_all_as_map",
            [this](SpHandle& uclass, std::vector<std::string>& function_flags, std::vector<std::string>& field_iteration_strings) -> std::map<std::string, SpHandle> {
                return UnrealUtils::toHandle(
                    UnrealUtils::findFunctionsByFlagsAllAsMap(
                        UnrealUtils::toPtr<UClass>(uclass),
                        Unreal::getCombinedEnumFlagValueFromStringsAs<EFunctionFlags, ESpFunctionFlags>(function_flags),
                        Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_function_by_name",
            [this](SpHandle& uclass, std::string& function_name, std::vector<std::string>& field_iteration_strings) -> SpHandle {
                return UnrealUtils::toHandle(
                    UnrealUtils::findFunctionByName(
                        UnrealUtils::toPtr<UClass>(uclass),
                        function_name,
                        Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_function_by_flags_any",
            [this](SpHandle& uclass, std::vector<std::string>& function_flags, std::vector<std::string>& field_iteration_strings) -> SpHandle {
                return UnrealUtils::toHandle(
                    UnrealUtils::findFunctionByFlagsAny(
                        UnrealUtils::toPtr<UClass>(uclass),
                        Unreal::getCombinedEnumFlagValueFromStringsAs<EFunctionFlags, ESpFunctionFlags>(function_flags),
                        Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_function_by_flags_all",
            [this](SpHandle& uclass, std::vector<std::string>& function_flags, std::vector<std::string>& field_iteration_strings) -> SpHandle {
                return UnrealUtils::toHandle(
                    UnrealUtils::findFunctionByFlagsAll(
                        UnrealUtils::toPtr<UClass>(uclass),
                        Unreal::getCombinedEnumFlagValueFromStringsAs<EFunctionFlags, ESpFunctionFlags>(function_flags),
                        Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_function_flags",
            [this](SpHandle& ufunction) -> std::vector<std::string> {
                SP_ASSERT(ufunction.ptr_);
                return Unreal::getStringsFromCombinedEnumFlagValueAs<ESpFunctionFlags>(UnrealUtils::toPtr<UFunction>(ufunction)->FunctionFlags);
            });

        //
        // Call function
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "call_function",
            [this](
                SpHandle& world,
                SpHandle& uobject,
                SpHandle& uclass,
                SpHandle& ufunction,
                std::map<std::string, std::string>& args,
                std::string& world_context) -> std::map<std::string, SpPropertyValue> {

                UObject* uobject_ptr = UnrealUtils::toPtr<UObject>(uobject);
                UClass* uclass_ptr = UnrealUtils::toPtr<UClass>(uclass);
                UFunction* ufunction_ptr = UnrealUtils::toPtr<UFunction>(ufunction);

                if (uobject_ptr) {
                    SP_ASSERT(!uclass_ptr);
                } else {
                    SP_ASSERT(uclass_ptr);
                    bool create_if_needed = false;
                    uobject_ptr = uclass_ptr->GetDefaultObject(create_if_needed);
                    SP_ASSERT(uobject_ptr);
                }

                return UnrealUtils::callFunction(UnrealUtils::toPtr<UWorld>(world), uobject_ptr, ufunction_ptr, args, world_context);
            });

        //
        // Find properties and get property flags
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_properties",
            [this](SpHandle& ustruct, std::vector<std::string>& field_iteration_flag_strings) -> std::vector<SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::findProperties(
                    UnrealUtils::toPtr<UStruct>(ustruct),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_flag_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_properties_as_map",
            [this](SpHandle& ustruct, std::vector<std::string>& field_iteration_flag_strings) -> std::map<std::string, SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::findPropertiesAsMap(
                    UnrealUtils::toPtr<UStruct>(ustruct),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_flag_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_properties_by_name",
            [this](SpHandle& ustruct, std::string& property_name, std::vector<std::string>& field_iteration_flag_strings) -> std::vector<SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::findPropertiesByName(
                    UnrealUtils::toPtr<UStruct>(ustruct),
                    property_name,
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_flag_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_properties_by_flags_any",
            [this](SpHandle& ustruct, std::vector<std::string>& property_flag_strings, std::vector<std::string>& field_iteration_flag_strings) -> std::vector<SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::findPropertiesByFlagsAny(
                    UnrealUtils::toPtr<UStruct>(ustruct),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EPropertyFlags, ESpPropertyFlags>(property_flag_strings),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_flag_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_properties_by_flags_all",
            [this](SpHandle& ustruct, std::vector<std::string>& property_flag_strings, std::vector<std::string>& field_iteration_flag_strings) -> std::vector<SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::findPropertiesByFlagsAll(
                    UnrealUtils::toPtr<UStruct>(ustruct),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EPropertyFlags, ESpPropertyFlags>(property_flag_strings),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_flag_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_properties_by_name_as_map",
            [this](SpHandle& ustruct, std::string& property_name, std::vector<std::string>& field_iteration_flag_strings) -> std::map<std::string, SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::findPropertiesByNameAsMap(
                    UnrealUtils::toPtr<UStruct>(ustruct),
                    property_name,
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_flag_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_properties_by_flags_any_as_map",
            [this](SpHandle& ustruct, std::vector<std::string>& property_flag_strings, std::vector<std::string>& field_iteration_flag_strings) -> std::map<std::string, SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::findPropertiesByFlagsAnyAsMap(
                    UnrealUtils::toPtr<UStruct>(ustruct),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EPropertyFlags, ESpPropertyFlags>(property_flag_strings),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_flag_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_properties_by_flags_all_as_map",
            [this](SpHandle& ustruct, std::vector<std::string>& property_flag_strings, std::vector<std::string>& field_iteration_flag_strings) -> std::map<std::string, SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::findPropertiesByFlagsAllAsMap(
                    UnrealUtils::toPtr<UStruct>(ustruct),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EPropertyFlags, ESpPropertyFlags>(property_flag_strings),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_flag_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_property_by_name",
            [this](SpHandle& ustruct, std::string& property_name, std::vector<std::string>& field_iteration_flag_strings) -> SpHandle {
                return UnrealUtils::toHandle(UnrealUtils::findPropertyByName(
                    UnrealUtils::toPtr<UStruct>(ustruct),
                    property_name,
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_flag_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_property_by_flags_any",
            [this](SpHandle& ustruct, std::vector<std::string>& property_flag_strings, std::vector<std::string>& field_iteration_flag_strings) -> SpHandle {
                return UnrealUtils::toHandle(UnrealUtils::findPropertyByFlagsAny(
                    UnrealUtils::toPtr<UStruct>(ustruct),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EPropertyFlags, ESpPropertyFlags>(property_flag_strings),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_flag_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_property_by_flags_all",
            [this](SpHandle& ustruct, std::vector<std::string>& property_flag_strings, std::vector<std::string>& field_iteration_flag_strings) -> SpHandle {
                return UnrealUtils::toHandle(UnrealUtils::findPropertyByFlagsAll(
                    UnrealUtils::toPtr<UStruct>(ustruct),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EPropertyFlags, ESpPropertyFlags>(property_flag_strings),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_flag_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_property_flags",
            [this](SpHandle& property) -> std::vector<std::string> {
                SP_ASSERT(property.ptr_);
                return Unreal::getStringsFromCombinedEnumFlagValueAs<ESpPropertyFlags>(UnrealUtils::toPtr<FProperty>(property)->GetPropertyFlags());
            });

        //
        // Get and set multiple properties
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_properties_for_object_as_string",
            [this](SpHandle& uobject, std::vector<std::string>& check_property_flag_strings, std::vector<std::string>& skip_property_flag_strings, bool& force_no_skip) -> std::string {
                return UnrealUtils::getObjectPropertiesAsString(
                    UnrealUtils::toPtr<UObject>(uobject),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EPropertyFlags, ESpPropertyFlags>(check_property_flag_strings),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EPropertyFlags, ESpPropertyFlags>(skip_property_flag_strings),
                    force_no_skip);
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_properties_for_struct_as_string",
            [this](SpHandle& value_ptr, SpHandle& ustruct, std::vector<std::string>& check_property_flag_strings, std::vector<std::string>& skip_property_flag_strings, bool& force_no_skip) -> std::string {
                return UnrealUtils::getObjectPropertiesAsString(
                    UnrealUtils::toPtr<void>(value_ptr),
                    UnrealUtils::toPtr<UStruct>(ustruct),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EPropertyFlags, ESpPropertyFlags>(check_property_flag_strings),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EPropertyFlags, ESpPropertyFlags>(skip_property_flag_strings),
                    force_no_skip);
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "set_properties_for_object_from_string",
            [this](SpHandle& uobject, std::string& properties_string, bool& notify_editor, std::vector<std::string>& check_property_flag_strings, std::vector<std::string>& skip_property_flag_strings, bool& force_no_skip) -> void {
                UnrealUtils::setObjectPropertiesFromString(
                    UnrealUtils::toPtr<UObject>(uobject),
                    properties_string,
                    notify_editor,
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EPropertyFlags, ESpPropertyFlags>(check_property_flag_strings),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EPropertyFlags, ESpPropertyFlags>(skip_property_flag_strings),
                    force_no_skip);
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "set_properties_for_struct_from_string",
            [this](SpHandle& value_ptr, SpHandle& ustruct, std::string& properties_string, SpHandle& notify_object, std::vector<std::string>& check_property_flag_strings, std::vector<std::string>& skip_property_flag_strings, bool& force_no_skip) -> void {
                UnrealUtils::setObjectPropertiesFromString(
                    UnrealUtils::toPtr<void>(value_ptr),
                    UnrealUtils::toPtr<UStruct>(ustruct),
                    properties_string,
                    UnrealUtils::toPtr<UObject>(notify_object),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EPropertyFlags, ESpPropertyFlags>(check_property_flag_strings),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EPropertyFlags, ESpPropertyFlags>(skip_property_flag_strings),
                    force_no_skip);
            });

        //
        // Get and set individual properties
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "resolve_property_for_object",
            [this](SpHandle& uobject, SpHandle& property) -> SpPropertyDesc {
                return UnrealUtils::resolveProperty(UnrealUtils::toPtr<UObject>(uobject), UnrealUtils::toPtr<FProperty>(property));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "resolve_property_for_object_from_string",
            [this](SpHandle& uobject, std::string& property_name) -> SpPropertyDesc {
                return UnrealUtils::resolveProperty(UnrealUtils::toPtr<UObject>(uobject), property_name);
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "resolve_property_for_struct",
            [this](SpHandle& value_ptr, SpHandle& ustruct, SpHandle& property, SpHandle& notify_object, SpPropertyDesc& parent) -> SpPropertyDesc {
                return UnrealUtils::resolveProperty(UnrealUtils::toPtr<void>(value_ptr), UnrealUtils::toPtr<UStruct>(ustruct), UnrealUtils::toPtr<FProperty>(property), UnrealUtils::toPtr<UObject>(notify_object), parent);
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "resolve_property_for_struct_from_string",
            [this](SpHandle& value_ptr, SpHandle& ustruct, std::string& property_name, SpHandle& notify_object, SpPropertyDesc& parent) -> SpPropertyDesc {
                return UnrealUtils::resolveProperty(UnrealUtils::toPtr<void>(value_ptr), UnrealUtils::toPtr<UStruct>(ustruct), property_name, UnrealUtils::toPtr<UObject>(notify_object), parent);
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "resolve_property_for_property_desc",
            [this](SpPropertyDesc& parent, SpHandle& property) -> SpPropertyDesc {
                return UnrealUtils::resolveProperty(parent, UnrealUtils::toPtr<FProperty>(property));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "resolve_property_for_property_desc_from_string",
            [this](SpPropertyDesc& parent, std::string& property_name) -> SpPropertyDesc {
                return UnrealUtils::resolveProperty(parent, property_name);
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_property_value_as_string",
            [this](SpPropertyDesc& property_desc) -> SpPropertyValue {
                return UnrealUtils::getPropertyValueAsString(property_desc);
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_property_value_for_object_as_string",
            [this](SpHandle& uobject, std::string& property_name) -> SpPropertyValue {
                SpPropertyDesc property_desc = UnrealUtils::resolveProperty(UnrealUtils::toPtr<UObject>(uobject), property_name);
                return UnrealUtils::getPropertyValueAsString(property_desc);
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_property_value_for_struct_as_string",
            [this](SpHandle& value_ptr, SpHandle& ustruct, std::string& property_name) -> SpPropertyValue {
                SpPropertyDesc property_desc = UnrealUtils::resolveProperty(UnrealUtils::toPtr<void>(value_ptr), UnrealUtils::toPtr<UStruct>(ustruct), property_name);
                return UnrealUtils::getPropertyValueAsString(property_desc);
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "set_property_value_from_string",
            [this](SpPropertyDesc& property_desc, std::string& property_value_string, bool& notify_editor) -> void {
                UnrealUtils::setPropertyValueFromString(property_desc, property_value_string, notify_editor);
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "set_property_value_for_object_from_string",
            [this](SpHandle& uobject, std::string& property_name, std::string& property_value_string, bool& notify_editor) -> void {
                SpPropertyDesc property_desc = UnrealUtils::resolveProperty(UnrealUtils::toPtr<UObject>(uobject), property_name);
                UnrealUtils::setPropertyValueFromString(property_desc, property_value_string, notify_editor);
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "set_property_value_for_struct_from_string",
            [this](SpHandle& value_ptr, SpHandle& ustruct, std::string& property_name, std::string& property_value_string, SpHandle& notify_object, SpPropertyDesc& parent) -> void {
                bool notify_editor = notify_object != SpHandle() || parent != SpPropertyDesc(); // notify iff a notify target was supplied via notify_object or parent
                SpPropertyDesc property_desc = UnrealUtils::resolveProperty(UnrealUtils::toPtr<void>(value_ptr), UnrealUtils::toPtr<UStruct>(ustruct), property_name, UnrealUtils::toPtr<UObject>(notify_object), parent);
                UnrealUtils::setPropertyValueFromString(property_desc, property_value_string, notify_editor);
            });

        //
        // Property notification interface
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "supports_edit_change_notify",
            [this](SpPropertyDesc& property_desc) -> bool {
                return UnrealUtils::supportsEditChangeNotify(property_desc);
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "pre_edit_change",
            [this](SpHandle& uobject, SpHandle& property) -> void {
                #if WITH_EDITOR // defined in an auto-generated header
                    UnrealUtils::toPtr<UObject>(uobject)->PreEditChange(UnrealUtils::toPtr<FProperty>(property));
                #endif
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "pre_edit_change_from_string",
            [this](SpHandle& uobject, std::string& property_name) -> void {
                #if WITH_EDITOR // defined in an auto-generated header
                    SpPropertyDesc property_desc = UnrealUtils::resolveProperty(UnrealUtils::toPtr<UObject>(uobject), property_name);
                    UnrealUtils::preEditChange(property_desc);
                #endif
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "pre_edit_change_from_property_desc",
            [this](SpPropertyDesc& property_desc) -> void {
                #if WITH_EDITOR // defined in an auto-generated header
                    UnrealUtils::preEditChange(property_desc);
                #endif
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "post_edit_change",
            [this](SpHandle& uobject, SpHandle& property) -> void {
                #if WITH_EDITOR // defined in an auto-generated header
                    FPropertyChangedEvent property_changed_event(UnrealUtils::toPtr<FProperty>(property));
                    UnrealUtils::toPtr<UObject>(uobject)->PostEditChangeProperty(property_changed_event);
                #endif
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "post_edit_change_from_string",
            [this](SpHandle& uobject, std::string& property_name) -> void {
                #if WITH_EDITOR // defined in an auto-generated header
                    SpPropertyDesc property_desc = UnrealUtils::resolveProperty(UnrealUtils::toPtr<UObject>(uobject), property_name);
                    UnrealUtils::postEditChange(property_desc);
                #endif
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "post_edit_change_from_property_desc",
            [this](SpPropertyDesc& property_desc) -> void {
                #if WITH_EDITOR // defined in an auto-generated header
                    UnrealUtils::postEditChange(property_desc);
                #endif
            });

        //
        // Spawn actor
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "spawn_actor",
            [this](SpHandle& world, SpHandle& uclass, std::string& location_string, std::string& rotation_string, std::string& spawn_parameters_string, std::vector<std::string>& object_flag_strings) -> SpHandle {

                FVector location;
                FRotator rotation;
                FSpActorSpawnParameters sp_actor_spawn_parameters;

                UnrealUtils::setObjectPropertiesFromString(&location, Unreal::getStaticStruct<FVector>(), location_string);
                UnrealUtils::setObjectPropertiesFromString(&rotation, Unreal::getStaticStruct<FRotator>(), rotation_string);
                UnrealUtils::setObjectPropertiesFromString(&sp_actor_spawn_parameters, FSpActorSpawnParameters::StaticStruct(), spawn_parameters_string);

                FActorSpawnParameters actor_spawn_parameters;
                actor_spawn_parameters.Name = sp_actor_spawn_parameters.Name;
                actor_spawn_parameters.Template = sp_actor_spawn_parameters.Template;
                actor_spawn_parameters.Owner = sp_actor_spawn_parameters.Owner;
                actor_spawn_parameters.Instigator = sp_actor_spawn_parameters.Instigator;
                actor_spawn_parameters.OverrideLevel = sp_actor_spawn_parameters.OverrideLevel;
                actor_spawn_parameters.OverrideParentComponent = sp_actor_spawn_parameters.OverrideParentComponent;
                actor_spawn_parameters.SpawnCollisionHandlingOverride = sp_actor_spawn_parameters.SpawnCollisionHandlingOverride;
                actor_spawn_parameters.TransformScaleMethod = sp_actor_spawn_parameters.TransformScaleMethod;
                actor_spawn_parameters.bNoFail = sp_actor_spawn_parameters.bNoFail;
                actor_spawn_parameters.bDeferConstruction = sp_actor_spawn_parameters.bDeferConstruction;
                actor_spawn_parameters.bAllowDuringConstructionScript = sp_actor_spawn_parameters.bAllowDuringConstructionScript;
                actor_spawn_parameters.NameMode = Unreal::getEnumValueAs<FActorSpawnParameters::ESpawnActorNameMode>(sp_actor_spawn_parameters.NameMode);
                actor_spawn_parameters.ObjectFlags = Unreal::getCombinedEnumFlagValueFromStringsAs<EObjectFlags, ESpObjectFlags>(object_flag_strings);

                SP_ASSERT(world.ptr_);
                return UnrealUtils::toHandle(UnrealUtils::toPtr<UWorld>(world)->SpawnActor(UnrealUtils::toPtr<UClass>(uclass), &location, &rotation, actor_spawn_parameters));
            });

        //
        // Destroy actor
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "destroy_actor",
            [this](SpHandle& actor, bool& net_force, bool& should_modify_level) -> bool {
                SP_ASSERT(actor.ptr_);
                return UnrealUtils::toPtr<AActor>(actor)->Destroy(net_force, should_modify_level);
            });

        //
        // Create new object
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "new_object",
            [this](
                SpHandle& outer,
                SpHandle& uclass,
                std::string& name,
                std::vector<std::string>& object_flag_strings,
                SpHandle& uobject_template,
                bool& copy_transients_from_class_defaults,
                SpHandle& in_instance_graph,
                SpHandle& external_package) -> SpHandle {

                UObject* outer_ptr = UnrealUtils::toPtr<UObject>(outer);
                if (!outer_ptr) {
                    outer_ptr = GetTransientPackage();
                }

                return UnrealUtils::toHandle(
                    NewObject<UObject>(
                        outer_ptr,
                        UnrealUtils::toPtr<UClass>(uclass),
                        Unreal::createFName(name),
                        Unreal::getCombinedEnumFlagValueFromStringsAs<EObjectFlags, ESpObjectFlags>(object_flag_strings),
                        UnrealUtils::toPtr<UObject>(uobject_template),
                        copy_transients_from_class_defaults,
                        UnrealUtils::toPtr<FObjectInstancingGraph>(in_instance_graph),
                        UnrealUtils::toPtr<UPackage>(external_package)));
            });

        //
        // Load object and class
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "static_load_object",
            [this](
                SpHandle& uclass,
                SpHandle& in_outer,
                std::string& name,
                std::string& filename,
                std::vector<std::string>& load_flag_strings,
                SpHandle& sandbox,
                bool& allow_object_reconciliation,
                SpHandle& instancing_context) -> SpHandle {

                return UnrealUtils::toHandle(
                    StaticLoadObject(
                        UnrealUtils::toPtr<UClass>(uclass),
                        UnrealUtils::toPtr<UObject>(in_outer),
                        Unreal::toTCharPtr(name),
                        Unreal::toTCharPtr(filename),
                        Unreal::getCombinedEnumFlagValueFromStringsAs<ELoadFlags, ESpLoadFlags>(load_flag_strings),
                        UnrealUtils::toPtr<UPackageMap>(sandbox),
                        allow_object_reconciliation,
                        UnrealUtils::toPtr<FLinkerInstancingContext>(instancing_context)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "static_load_class",
            [this](
                SpHandle& uclass,
                SpHandle& in_outer,
                std::string& name,
                std::string& filename,
                std::vector<std::string>& load_flag_strings,
                SpHandle& sandbox) -> SpHandle {

                return UnrealUtils::toHandle(
                    StaticLoadClass(
                        UnrealUtils::toPtr<UClass>(uclass),
                        UnrealUtils::toPtr<UObject>(in_outer),
                        Unreal::toTCharPtr(name),
                        Unreal::toTCharPtr(filename),
                        Unreal::getCombinedEnumFlagValueFromStringsAs<ELoadFlags, ESpLoadFlags>(load_flag_strings),
                        UnrealUtils::toPtr<UPackageMap>(sandbox)));
            });

        //
        // Enable and disable garbage collection for uobject
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "add_object_to_root",
            [this](SpHandle& uobject) -> void {
                UObject* uobject_ptr = UnrealUtils::toPtr<UObject>(uobject);
                SP_ASSERT(uobject_ptr);
                uobject_ptr->AddToRoot();
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "remove_object_from_root",
            [this](SpHandle& uobject) -> void {
                UObject* uobject_ptr = UnrealUtils::toPtr<UObject>(uobject);
                SP_ASSERT(uobject_ptr);
                uobject_ptr->RemoveFromRoot();
            });
    }

    template <CUnrealEntryPointBinder TEntryPointBinder>
    static std::unique_ptr<UnrealService_0> create(TEntryPointBinder* unreal_entry_point_binder);
};
