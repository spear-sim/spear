//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // uint64_t

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
#include <UObject/UObjectGlobals.h>          // GetTransientPackage, NewObject, StaticLoadClass, StaticLoadObject
#include <UObject/UnrealType.h>              // EFieldIterationFlags, FProperty

#include "SpCore/Assert.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"
#include "SpCore/UnrealUtils.h"
#include "SpCore/UnrealClassRegistry.h"

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
                    static_struct_desc.static_struct_ = script_struct;
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
                    static_class_desc.static_class_ = static_class;
                    static_class_desc.name_ = Unreal::getTypeAsString(static_class);

                    // populate derived-to-base class chain (includes the class itself)
                    UClass* current_static_class = static_class;
                    while (current_static_class) {
                        static_class_desc.derived_classes_.push_back(current_static_class);
                        static_class_desc.derived_class_names_.push_back(Unreal::getTypeAsString(current_static_class));
                        current_static_class = current_static_class->GetSuperClass();
                    }

                    // leave function_descs_ blank as a performance optimization

                    static_class_descs.push_back(std::move(static_class_desc));
                }

                return static_class_descs;
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_static_struct_desc",
            [this](uint64_t& script_struct) -> SpStaticStructDesc {
                SP_ASSERT(script_struct);
                UScriptStruct* script_struct_ptr = toPtr<UScriptStruct>(script_struct);
                SpStaticStructDesc static_struct_desc;
                static_struct_desc.static_struct_ = script_struct_ptr;
                if (script_struct_ptr->IsA(UUserDefinedStruct::StaticClass())) {
                    static_struct_desc.name_ = Unreal::getBlueprintTypeAsString(script_struct_ptr);
                } else {
                    static_struct_desc.name_ = Unreal::getCppTypeAsString(script_struct_ptr);
                }
                return static_struct_desc;
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_static_class_desc",
            [this](uint64_t& uclass) -> SpStaticClassDesc {
                SP_ASSERT(uclass);
                UClass* uclass_ptr = toPtr<UClass>(uclass);

                SpStaticClassDesc static_class_desc;
                static_class_desc.static_class_ = uclass_ptr;
                static_class_desc.name_ = Unreal::getTypeAsString(uclass_ptr);

                // populate derived-to-base class chain (includes the class itself)
                UClass* current_static_class = uclass_ptr;
                while (current_static_class) {
                    static_class_desc.derived_classes_.push_back(current_static_class);
                    static_class_desc.derived_class_names_.push_back(Unreal::getTypeAsString(current_static_class));
                    current_static_class = current_static_class->GetSuperClass();
                }

                // populate function descs with fully qualified names
                std::vector<UFunction*> ufunctions = UnrealUtils::findFunctions(uclass_ptr);
                for (auto ufunction : ufunctions) {
                    SP_ASSERT(ufunction);
                    SpFunctionDesc function_desc;
                    function_desc.function_ = ufunction;
                    function_desc.function_name_ = Unreal::toStdString(ufunction->GetName());
                    UStruct* outer = Cast<UStruct>(ufunction->GetOuter());
                    SP_ASSERT(outer);
                    function_desc.static_class_ = Cast<UClass>(outer);
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
            [this](uint64_t& uclass) -> uint64_t {
                return toUInt64(Unreal::getEngineSubsystemByClass(toPtr<UClass>(uclass)));
            });

        //
        // Get editor subsystem, WITH_EDITOR implementations in SpServicesEditor/UnrealServiceEditor.h
        //

        #if !WITH_EDITOR // defined in an auto-generated header
            unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_editor_subsystem_by_class",
                [this](uint64_t& uclass) -> uint64_t {
                    SP_ASSERT(false);
                    return 0;
                });
        #endif

        //
        // Get subsystem
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_subsystem_by_class",
            [this](std::string& subsystem_provider_class_name, uint64_t& world, uint64_t& subsystem_uclass) -> uint64_t {
                return toUInt64(UnrealClassRegistry::getSubsystemByClass(subsystem_provider_class_name, toPtr<UWorld>(world), toPtr<UClass>(subsystem_uclass)));
            });

        //
        // Functions for static structs and classes
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_static_structs",
            [this]() -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::findStaticStructs());
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_static_structs_as_map",
            [this]() -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::findStaticStructsAsMap());
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_struct_flags",
            [this](uint64_t& script_struct) -> std::vector<std::string> {
                SP_ASSERT(script_struct);
                return Unreal::getStringsFromCombinedEnumFlagValueAs<ESpStructFlags>(toPtr<UScriptStruct>(script_struct)->StructFlags);
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_static_classes",
            [this]() -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::findStaticClasses());
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_static_classes_as_map",
            [this]() -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::findStaticClassesAsMap());
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_derived_classes",
            [this](uint64_t& uclass, bool& recursive) -> std::vector<uint64_t> {
                SP_ASSERT(uclass);
                return toUInt64(UnrealUtils::getDerivedClasses(toPtr<UClass>(uclass), recursive));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_derived_classes_as_map",
            [this](uint64_t& uclass, bool& recursive) -> std::map<std::string, uint64_t> {
                SP_ASSERT(uclass);
                return toUInt64(UnrealUtils::getDerivedClassesAsMap(toPtr<UClass>(uclass), recursive));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_static_class",
            [this](std::string& class_name) -> uint64_t {
                return toUInt64(UnrealClassRegistry::getStaticClass(class_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_super_class",
            [this](uint64_t& uclass) -> uint64_t {
                SP_ASSERT(uclass);
                return toUInt64(toPtr<UClass>(uclass)->GetSuperClass());
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_class_flags",
            [this](uint64_t& uclass) -> std::vector<std::string> {
                SP_ASSERT(uclass);
                return Unreal::getStringsFromCombinedEnumFlagValueAs<ESpClassFlags>(toPtr<UClass>(uclass)->GetClassFlags());
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_default_object",
            [this](uint64_t& uclass, bool& create_if_needed) -> uint64_t {
                SP_ASSERT(uclass);
                return toUInt64(toPtr<UClass>(uclass)->GetDefaultObject(create_if_needed));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_class",
            [this](uint64_t& uobject) -> uint64_t {
                SP_ASSERT(uobject);
                return toUInt64(toPtr<UObject>(uobject)->GetClass());
            });

        //
        // Functions for getting C++ types as strings
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_type_for_struct_as_string",
            [this](uint64_t& ustruct) -> std::string {
                return Unreal::getTypeAsString(toPtr<UStruct>(ustruct));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_type_for_property_as_string",
            [this](uint64_t& property) -> std::string {
                return Unreal::getCppTypeAsString(toPtr<FProperty>(property));
            });

        //
        // Find functions and get function flags
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_functions",
            [this](uint64_t& uclass, std::vector<std::string>& field_iteration_strings) -> std::vector<uint64_t> {
                return toUInt64(
                    UnrealUtils::findFunctions(
                        toPtr<UClass>(uclass),
                        Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_functions_as_map",
            [this](uint64_t& uclass, std::vector<std::string>& field_iteration_strings) -> std::map<std::string, uint64_t> {
                return toUInt64(
                    UnrealUtils::findFunctionsAsMap(
                        toPtr<UClass>(uclass),
                        Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_functions_by_name",
            [this](uint64_t& uclass, std::string& function_name, std::vector<std::string>& field_iteration_strings) -> std::vector<uint64_t> {
                return toUInt64(
                    UnrealUtils::findFunctionsByName(
                        toPtr<UClass>(uclass),
                        function_name,
                        Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_functions_by_flags_any",
            [this](uint64_t& uclass, std::vector<std::string>& function_flags, std::vector<std::string>& field_iteration_strings) -> std::vector<uint64_t> {
                return toUInt64(
                    UnrealUtils::findFunctionsByFlagsAny(
                        toPtr<UClass>(uclass),
                        Unreal::getCombinedEnumFlagValueFromStringsAs<EFunctionFlags, ESpFunctionFlags>(function_flags),
                        Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_functions_by_flags_all",
            [this](uint64_t& uclass, std::vector<std::string>& function_flags, std::vector<std::string>& field_iteration_strings) -> std::vector<uint64_t> {
                return toUInt64(
                    UnrealUtils::findFunctionsByFlagsAll(
                        toPtr<UClass>(uclass),
                        Unreal::getCombinedEnumFlagValueFromStringsAs<EFunctionFlags, ESpFunctionFlags>(function_flags),
                        Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_functions_by_name_as_map",
            [this](uint64_t& uclass, std::string& function_name, std::vector<std::string>& field_iteration_strings) -> std::map<std::string, uint64_t> {
                return toUInt64(
                    UnrealUtils::findFunctionsByNameAsMap(
                        toPtr<UClass>(uclass),
                        function_name,
                        Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_functions_by_flags_any_as_map",
            [this](uint64_t& uclass, std::vector<std::string>& function_flags, std::vector<std::string>& field_iteration_strings) -> std::map<std::string, uint64_t> {
                return toUInt64(
                    UnrealUtils::findFunctionsByFlagsAnyAsMap(
                        toPtr<UClass>(uclass),
                        Unreal::getCombinedEnumFlagValueFromStringsAs<EFunctionFlags, ESpFunctionFlags>(function_flags),
                        Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_functions_by_flags_all_as_map",
            [this](uint64_t& uclass, std::vector<std::string>& function_flags, std::vector<std::string>& field_iteration_strings) -> std::map<std::string, uint64_t> {
                return toUInt64(
                    UnrealUtils::findFunctionsByFlagsAllAsMap(
                        toPtr<UClass>(uclass),
                        Unreal::getCombinedEnumFlagValueFromStringsAs<EFunctionFlags, ESpFunctionFlags>(function_flags),
                        Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_function_by_name",
            [this](uint64_t& uclass, std::string& function_name, std::vector<std::string>& field_iteration_strings) -> uint64_t {
                return toUInt64(
                    UnrealUtils::findFunctionByName(
                        toPtr<UClass>(uclass),
                        function_name,
                        Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_function_by_flags_any",
            [this](uint64_t& uclass, std::vector<std::string>& function_flags, std::vector<std::string>& field_iteration_strings) -> uint64_t {
                return toUInt64(
                    UnrealUtils::findFunctionByFlagsAny(
                        toPtr<UClass>(uclass),
                        Unreal::getCombinedEnumFlagValueFromStringsAs<EFunctionFlags, ESpFunctionFlags>(function_flags),
                        Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_function_by_flags_all",
            [this](uint64_t& uclass, std::vector<std::string>& function_flags, std::vector<std::string>& field_iteration_strings) -> uint64_t {
                return toUInt64(
                    UnrealUtils::findFunctionByFlagsAll(
                        toPtr<UClass>(uclass),
                        Unreal::getCombinedEnumFlagValueFromStringsAs<EFunctionFlags, ESpFunctionFlags>(function_flags),
                        Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_function_flags",
            [this](uint64_t& ufunction) -> std::vector<std::string> {
                SP_ASSERT(ufunction);
                return Unreal::getStringsFromCombinedEnumFlagValueAs<ESpFunctionFlags>(toPtr<UFunction>(ufunction)->FunctionFlags);
            });

        //
        // Call function
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "call_function",
            [this](
                uint64_t& world,
                uint64_t& uobject,
                uint64_t& uclass,
                uint64_t& ufunction,
                std::map<std::string, std::string>& args,
                std::string& world_context) -> std::map<std::string, SpPropertyValue> {

                UObject* uobject_ptr = toPtr<UObject>(uobject);
                UClass* uclass_ptr = toPtr<UClass>(uclass);
                UFunction* ufunction_ptr = toPtr<UFunction>(ufunction);

                if (uobject_ptr) {
                    SP_ASSERT(!uclass_ptr);
                } else {
                    SP_ASSERT(uclass_ptr);
                    bool create_if_needed = false;
                    uobject_ptr = uclass_ptr->GetDefaultObject(create_if_needed);
                    SP_ASSERT(uobject_ptr);
                }

                return UnrealUtils::callFunction(toPtr<UWorld>(world), uobject_ptr, ufunction_ptr, args, world_context);
            });

        //
        // Find properties and get property flags
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_properties",
            [this](uint64_t& ustruct, std::vector<std::string>& field_iteration_flag_strings) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::findProperties(
                    toPtr<UStruct>(ustruct),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_flag_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_properties_as_map",
            [this](uint64_t& ustruct, std::vector<std::string>& field_iteration_flag_strings) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::findPropertiesAsMap(
                    toPtr<UStruct>(ustruct),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_flag_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_properties_by_name",
            [this](uint64_t& ustruct, std::string& property_name, std::vector<std::string>& field_iteration_flag_strings) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::findPropertiesByName(
                    toPtr<UStruct>(ustruct),
                    property_name,
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_flag_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_properties_by_flags_any",
            [this](uint64_t& ustruct, std::vector<std::string>& property_flag_strings, std::vector<std::string>& field_iteration_flag_strings) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::findPropertiesByFlagsAny(
                    toPtr<UStruct>(ustruct),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EPropertyFlags, ESpPropertyFlags>(property_flag_strings),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_flag_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_properties_by_flags_all",
            [this](uint64_t& ustruct, std::vector<std::string>& property_flag_strings, std::vector<std::string>& field_iteration_flag_strings) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::findPropertiesByFlagsAll(
                    toPtr<UStruct>(ustruct),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EPropertyFlags, ESpPropertyFlags>(property_flag_strings),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_flag_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_properties_by_name_as_map",
            [this](uint64_t& ustruct, std::string& property_name, std::vector<std::string>& field_iteration_flag_strings) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::findPropertiesByNameAsMap(
                    toPtr<UStruct>(ustruct),
                    property_name,
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_flag_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_properties_by_flags_any_as_map",
            [this](uint64_t& ustruct, std::vector<std::string>& property_flag_strings, std::vector<std::string>& field_iteration_flag_strings) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::findPropertiesByFlagsAnyAsMap(
                    toPtr<UStruct>(ustruct),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EPropertyFlags, ESpPropertyFlags>(property_flag_strings),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_flag_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_properties_by_flags_all_as_map",
            [this](uint64_t& ustruct, std::vector<std::string>& property_flag_strings, std::vector<std::string>& field_iteration_flag_strings) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::findPropertiesByFlagsAllAsMap(
                    toPtr<UStruct>(ustruct),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EPropertyFlags, ESpPropertyFlags>(property_flag_strings),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_flag_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_property_by_name",
            [this](uint64_t& ustruct, std::string& property_name, std::vector<std::string>& field_iteration_flag_strings) -> uint64_t {
                return toUInt64(UnrealUtils::findPropertyByName(
                    toPtr<UStruct>(ustruct),
                    property_name,
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_flag_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_property_by_flags_any",
            [this](uint64_t& ustruct, std::vector<std::string>& property_flag_strings, std::vector<std::string>& field_iteration_flag_strings) -> uint64_t {
                return toUInt64(UnrealUtils::findPropertyByFlagsAny(
                    toPtr<UStruct>(ustruct),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EPropertyFlags, ESpPropertyFlags>(property_flag_strings),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_flag_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_property_by_flags_all",
            [this](uint64_t& ustruct, std::vector<std::string>& property_flag_strings, std::vector<std::string>& field_iteration_flag_strings) -> uint64_t {
                return toUInt64(UnrealUtils::findPropertyByFlagsAll(
                    toPtr<UStruct>(ustruct),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EPropertyFlags, ESpPropertyFlags>(property_flag_strings),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_flag_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_property_flags",
            [this](uint64_t& property) -> std::vector<std::string> {
                SP_ASSERT(property);
                return Unreal::getStringsFromCombinedEnumFlagValueAs<ESpPropertyFlags>(toPtr<FProperty>(property)->GetPropertyFlags());
            });

        //
        // Get and set multiple properties
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_properties_for_object_as_string",
            [this](uint64_t& uobject) -> std::string {
                return UnrealUtils::getObjectPropertiesAsString(toPtr<UObject>(uobject));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_properties_for_struct_as_string",
            [this](uint64_t& value_ptr, uint64_t& ustruct) -> std::string {
                return UnrealUtils::getObjectPropertiesAsString(toPtr<void>(value_ptr), toPtr<UStruct>(ustruct));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "set_properties_for_object_from_string",
            [this](uint64_t& uobject, std::string& properties_string, bool& notify_editor) -> void {
                UnrealUtils::setObjectPropertiesFromString(toPtr<UObject>(uobject), properties_string, notify_editor);
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "set_properties_for_struct_from_string",
            [this](uint64_t& value_ptr, uint64_t& ustruct, std::string& properties_string, uint64_t& notify) -> void {
                UnrealUtils::setObjectPropertiesFromString(toPtr<void>(value_ptr), toPtr<UStruct>(ustruct), properties_string, toPtr<UObject>(notify));
            });

        //
        // Get and set individual properties
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "resolve_property_for_object",
            [this](uint64_t& uobject, uint64_t& property) -> SpPropertyDesc {
                return UnrealUtils::resolveProperty(toPtr<UObject>(uobject), toPtr<FProperty>(property));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "resolve_property_for_object_from_string",
            [this](uint64_t& uobject, std::string& property_name) -> SpPropertyDesc {
                return UnrealUtils::resolveProperty(toPtr<UObject>(uobject), property_name);
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "resolve_property_for_struct",
            [this](uint64_t& value_ptr, uint64_t& ustruct, uint64_t& property, uint64_t& notify) -> SpPropertyDesc {
                return UnrealUtils::resolveProperty(toPtr<void>(value_ptr), toPtr<UStruct>(ustruct), toPtr<FProperty>(property), toPtr<UObject>(notify));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "resolve_property_for_struct_from_string",
            [this](uint64_t& value_ptr, uint64_t& ustruct, std::string& property_name, uint64_t& notify) -> SpPropertyDesc {
                return UnrealUtils::resolveProperty(toPtr<void>(value_ptr), toPtr<UStruct>(ustruct), property_name, toPtr<UObject>(notify));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_property_value_as_string",
            [this](SpPropertyDesc& property_desc) -> SpPropertyValue {
                return UnrealUtils::getPropertyValueAsString(property_desc);
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_property_value_for_object_as_string",
            [this](uint64_t& uobject, std::string& property_name) -> SpPropertyValue {
                SpPropertyDesc property_desc = UnrealUtils::resolveProperty(toPtr<UObject>(uobject), property_name);
                return UnrealUtils::getPropertyValueAsString(property_desc);
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_property_value_for_struct_as_string",
            [this](uint64_t& value_ptr, uint64_t& ustruct, std::string& property_name) -> SpPropertyValue {
                SpPropertyDesc property_desc = UnrealUtils::resolveProperty(toPtr<void>(value_ptr), toPtr<UStruct>(ustruct), property_name);
                return UnrealUtils::getPropertyValueAsString(property_desc);
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "set_property_value_from_string",
            [this](SpPropertyDesc& property_desc, std::string& property_value_string, bool& notify_editor) -> void {
                UnrealUtils::setPropertyValueFromString(property_desc, property_value_string, notify_editor);
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "set_property_value_for_object_from_string",
            [this](uint64_t& uobject, std::string& property_name, std::string& property_value_string, bool& notify_editor) -> void {
                SpPropertyDesc property_desc = UnrealUtils::resolveProperty(toPtr<UObject>(uobject), property_name);
                UnrealUtils::setPropertyValueFromString(property_desc, property_value_string, notify_editor);
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "set_property_value_for_struct_from_string",
            [this](uint64_t& value_ptr, uint64_t& ustruct, std::string& property_name, std::string& property_value_string, uint64_t& notify) -> void {
                bool notify_editor = notify != 0;
                SpPropertyDesc property_desc = UnrealUtils::resolveProperty(toPtr<void>(value_ptr), toPtr<UStruct>(ustruct), property_name, toPtr<UObject>(notify));
                UnrealUtils::setPropertyValueFromString(property_desc, property_value_string, notify_editor);
            });

        //
        // Property notification interface
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "pre_edit_change",
            [this](uint64_t& uobject, uint64_t& property) -> void {
                #if WITH_EDITOR
                    toPtr<UObject>(uobject)->PreEditChange(toPtr<FProperty>(property));
                #endif
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "pre_edit_change_from_string",
            [this](uint64_t& uobject, std::string& property_name) -> void {
                #if WITH_EDITOR
                    SpPropertyDesc property_desc = UnrealUtils::resolveProperty(toPtr<UObject>(uobject), property_name);
                    toPtr<UObject>(uobject)->PreEditChange(property_desc.property_);
                #endif
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "post_edit_change",
            [this](uint64_t& uobject, uint64_t& property) -> void {
                #if WITH_EDITOR
                    FPropertyChangedEvent property_changed_event(toPtr<FProperty>(property));
                    toPtr<UObject>(uobject)->PostEditChangeProperty(property_changed_event);
                #endif
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "post_edit_change_from_string",
            [this](uint64_t& uobject, std::string& property_name) -> void {
                #if WITH_EDITOR
                    SpPropertyDesc property_desc = UnrealUtils::resolveProperty(toPtr<UObject>(uobject), property_name);
                    FPropertyChangedEvent property_changed_event(property_desc.property_);
                    toPtr<UObject>(uobject)->PostEditChangeProperty(property_changed_event);
                #endif
            });

        //
        // Spawn actor
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "spawn_actor",
            [this](uint64_t& world, uint64_t& uclass, std::string& location_string, std::string& rotation_string, std::string& spawn_parameters_string, std::vector<std::string>& object_flag_strings) -> uint64_t {

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

                SP_ASSERT(world);
                return toUInt64(toPtr<UWorld>(world)->SpawnActor(toPtr<UClass>(uclass), &location, &rotation, actor_spawn_parameters));
            });

        //
        // Destroy actor
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "destroy_actor",
            [this](uint64_t& actor, bool& net_force, bool& should_modify_level) -> bool {
                SP_ASSERT(actor);
                return toPtr<AActor>(actor)->Destroy(net_force, should_modify_level);
            });

        //
        // Create new object
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "new_object",
            [this](
                uint64_t& outer,
                uint64_t& uclass,
                std::string& name,
                std::vector<std::string>& object_flag_strings,
                uint64_t& uobject_template,
                bool& copy_transients_from_class_defaults,
                uint64_t& in_instance_graph,
                uint64_t& external_package) -> uint64_t {

                UObject* outer_ptr = toPtr<UObject>(outer);
                if (!outer_ptr) {
                    outer_ptr = GetTransientPackage();
                }

                FName fname = NAME_None;
                if (name != "") {
                    fname = Unreal::toFName(name);
                }

                return toUInt64(
                    NewObject<UObject>(
                        outer_ptr,
                        toPtr<UClass>(uclass),
                        fname,
                        Unreal::getCombinedEnumFlagValueFromStringsAs<EObjectFlags, ESpObjectFlags>(object_flag_strings),
                        toPtr<UObject>(uobject_template),
                        copy_transients_from_class_defaults,
                        toPtr<FObjectInstancingGraph>(in_instance_graph),
                        toPtr<UPackage>(external_package)));
            });

        //
        // Load object and class
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "static_load_object",
            [this](
                uint64_t& uclass,
                uint64_t& in_outer,
                std::string& name,
                std::string& filename,
                std::vector<std::string>& load_flag_strings,
                uint64_t& sandbox,
                bool& allow_object_reconciliation,
                uint64_t& instancing_context) -> uint64_t {

                return toUInt64(
                    StaticLoadObject(
                        toPtr<UClass>(uclass),
                        toPtr<UObject>(in_outer),
                        Unreal::toFStringView(name),
                        Unreal::toFStringView(filename),
                        Unreal::getCombinedEnumFlagValueFromStringsAs<ELoadFlags, ESpLoadFlags>(load_flag_strings),
                        toPtr<UPackageMap>(sandbox),
                        allow_object_reconciliation,
                        toPtr<FLinkerInstancingContext>(instancing_context)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "static_load_class",
            [this](
                uint64_t& uclass,
                uint64_t& in_outer,
                std::string& name,
                std::string& filename,
                std::vector<std::string>& load_flag_strings,
                uint64_t& sandbox) -> uint64_t {

                return toUInt64(
                    StaticLoadClass(
                        toPtr<UClass>(uclass),
                        toPtr<UObject>(in_outer),
                        Unreal::toFStringView(name),
                        Unreal::toFStringView(filename),
                        Unreal::getCombinedEnumFlagValueFromStringsAs<ELoadFlags, ESpLoadFlags>(load_flag_strings),
                        toPtr<UPackageMap>(sandbox)));
            });

        //
        // Enable and disable garbage collection for uobject
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "add_object_to_root",
            [this](uint64_t& uobject) -> void {
                UObject* uboject_ptr = toPtr<UObject>(uobject);
                SP_ASSERT(uboject_ptr);
                uboject_ptr->AddToRoot();
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "remove_object_from_root",
            [this](uint64_t& uobject) -> void {
                UObject* uboject_ptr = toPtr<UObject>(uobject);
                SP_ASSERT(uboject_ptr);
                uboject_ptr->RemoveFromRoot();
            });
    }

    template <CUnrealEntryPointBinder TEntryPointBinder>
    static std::unique_ptr<UnrealService_0> create(TEntryPointBinder* unreal_entry_point_binder);
};
