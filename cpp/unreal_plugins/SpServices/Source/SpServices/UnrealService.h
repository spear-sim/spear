//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

//
// UnrealService.h uses a "#define public private" hack to access several private variables, and therefore it
// must only be included in cpp files, and must be included after all other headers.
//

#include <stdint.h> // uint64_t

#include <map>
#include <ranges>  // std::views::transform
#include <string>
#include <utility> // std::make_pair, std::move
#include <vector>

#include <Components/ActorComponent.h>
#include <Components/SceneComponent.h>
#include <Engine/Engine.h>                   // GEngine
#include <Engine/LocalPlayer.h>              // ULocalPlayer
#include <Engine/World.h>                    // FActorSpawnParameters
#include <HAL/IConsoleManager.h>             // EConsoleVariableFlags, IConsoleVariable
#include <HAL/Platform.h>                    // uint64
#include <Math/Rotator.h>
#include <Math/Vector.h>
#include <Modules/ModuleManager.h>           // FModuleManager
#include <StructUtils/UserDefinedStruct.h>
#include <UObject/Class.h>                   // FObjectInstancingGraph, UClass, UFunction, UScriptStruct, UStruct
#include <UObject/CoreNet.h>                 // UPackageMap
#include <UObject/LinkerInstancingContext.h> // FLinkerInstancingContext
#include <UObject/NameTypes.h>               // FName
#include <UObject/Object.h>                  // UObject
#include <UObject/ObjectMacros.h>            // ELoadFlags, EObjectFlags, EPropertyFlags
#include <UObject/Package.h>
#include <UObject/Script.h>                  // EFunctionFlags
#include <UObject/UObjectGlobals.h>          // GetTransientPackage, StaticLoadClass, StaticLoadObject
#include <UObject/UnrealType.h>              // EFieldIterationFlags

// MessageLog is a Developer-category module and is therefore not available in Shipping builds.
#if WITH_UNREAL_DEVELOPER_TOOLS

    #include <IMessageLogListing.h>       // IMessageLogListing, IMessageLogListingPtr
    #include <Logging/TokenizedMessage.h> // FTokenizedMessage

    // private headers need custom include paths in SpModuleRules.Build.cs
    #include <Model/MessageLogListingModel.h> // FMessageLogListingModel, MessageContainer

#endif

#include "SpCore/Assert.h"
#include "SpCore/AssertModuleLoaded.h"
#include "SpCore/Log.h"
#include "SpCore/OutputLog.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"
#include "SpCore/UnrealUtils.h"
#include "SpCore/UnrealClassRegistry.h"

#include "SpServices/EntryPointBinder.h"
#include "SpServices/Service.h"
#include "SpServices/SpTypes.h"
#include "SpServices/UnrealServiceTypes.h"

// MessageLog is a Developer-category module and is therefore not available in Shipping builds.
#if WITH_UNREAL_DEVELOPER_TOOLS

    //
    // HACK: FMessageLogModule::MessageLogViewModel and FMessageLogListingViewModel::MessageLogListingModel are
    // both private members, and there is no public API for enumerating all registered message log listings, or
    // for reading all (as opposed to filtered) messages across all pages of a listing. We override private
    // visibility in the headers below, but only ever reach into these two members directly. Reading a private
    // member needs no exported symbol, since it's just a memory offset, not a function call, so this links
    // correctly. 
    //
    // Every method we call from the headers below is either inline (FMessageLogViewModel::GetLogListingViewModels())
    // or a member of a class that is properly exported from the MessageLog module (FMessageLogListingViewModel and
    // FMessageLogListingModel are both MESSAGELOG_API), so this links correctly. FMessageLogViewModel and
    // FMessageLogModule themselves are not exported, so we deliberately avoid calling any of their non-inline
    // methods (e.g., FindLogListingViewModel(...)). We must include Presentation/MessageLogListingViewModel.h
    // under this #define too (not just MessageLogModule.h), because Presentation/MessageLogViewModel.h
    // transitively includes it. If that transitive include happens first, outside this block, its header guard
    // would make our own include below a no-op, leaving MessageLogListingModel private after all.
    //

    #pragma push_macro("private")
    #undef private
    #define private public
        #include <Presentation/MessageLogListingViewModel.h> // FMessageLogListingViewModel
        #include <Presentation/MessageLogViewModel.h>        // FMessageLogViewModel
        #include <MessageLogModule.h>                        // FMessageLogModule
    #pragma pop_macro("private")

#endif

//
// UnrealService is our service for interacting with the Unreal property system.
//

class UnrealService : public Service
{
public:
    UnrealService() = delete;
    UnrealService(CUnrealEntryPointBinder auto* unreal_entry_point_binder)
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
                        Unreal::toTCharPtr(name),
                        Unreal::toTCharPtr(filename),
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
                        Unreal::toTCharPtr(name),
                        Unreal::toTCharPtr(filename),
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

        //
        // Find, get, and set console variable
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_console_variable_by_name",
            [this](std::string& cvar_name) -> uint64_t {
                return toUInt64(IConsoleManager::Get().FindConsoleVariable(Unreal::toTCharPtr(cvar_name)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_console_variable_value_as_bool",
            [this](uint64_t& cvar) -> bool {
                SP_ASSERT(cvar);
                return toPtr<IConsoleVariable>(cvar)->GetBool();
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_console_variable_value_as_int",
            [this](uint64_t& cvar) -> int64_t {
                SP_ASSERT(cvar);
                return toPtr<IConsoleVariable>(cvar)->GetInt();
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_console_variable_value_as_float",
            [this](uint64_t& cvar) -> float {
                SP_ASSERT(cvar);
                return toPtr<IConsoleVariable>(cvar)->GetFloat();
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_console_variable_value_as_string",
            [this](uint64_t& cvar) -> std::string {
                SP_ASSERT(cvar);
                return Unreal::toStdString(toPtr<IConsoleVariable>(cvar)->GetString());
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "set_console_variable_value_from_bool",
            [this](uint64_t& cvar, bool& val, std::vector<std::string>& set_by_strings) -> void {
                SP_ASSERT(cvar);
                toPtr<IConsoleVariable>(cvar)->Set(val, Unreal::getCombinedEnumFlagValueFromStringsAs<EConsoleVariableFlags, ESpConsoleVariableFlags>(set_by_strings));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "set_console_variable_value_from_int",
            [this](uint64_t& cvar, int32_t& val, std::vector<std::string>& set_by_strings) -> void {
                SP_ASSERT(cvar);
                toPtr<IConsoleVariable>(cvar)->Set(val, Unreal::getCombinedEnumFlagValueFromStringsAs<EConsoleVariableFlags, ESpConsoleVariableFlags>(set_by_strings));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "set_console_variable_value_from_float",
            [this](uint64_t& cvar, float& val, std::vector<std::string>& set_by_strings) -> void {
                SP_ASSERT(cvar);
                toPtr<IConsoleVariable>(cvar)->Set(val, Unreal::getCombinedEnumFlagValueFromStringsAs<EConsoleVariableFlags, ESpConsoleVariableFlags>(set_by_strings));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "set_console_variable_value_from_string",
            [this](uint64_t& cvar, std::string& val, std::vector<std::string>& set_by_strings) -> void {
                SP_ASSERT(cvar);
                toPtr<IConsoleVariable>(cvar)->Set(Unreal::toTCharPtr(val), Unreal::getCombinedEnumFlagValueFromStringsAs<EConsoleVariableFlags, ESpConsoleVariableFlags>(set_by_strings));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "set_console_variable_value_with_current_priority_from_bool",
            [this](uint64_t& cvar, bool& val) -> void {
                SP_ASSERT(cvar);
                toPtr<IConsoleVariable>(cvar)->SetWithCurrentPriority(val);
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "set_console_variable_value_with_current_priority_from_int",
            [this](uint64_t& cvar, int32_t& val) -> void {
                SP_ASSERT(cvar);
                toPtr<IConsoleVariable>(cvar)->SetWithCurrentPriority(val);
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "set_console_variable_value_with_current_priority_from_float",
            [this](uint64_t& cvar, float& val) -> void {
                SP_ASSERT(cvar);
                toPtr<IConsoleVariable>(cvar)->SetWithCurrentPriority(val);
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "set_console_variable_value_with_current_priority_from_string",
            [this](uint64_t& cvar, std::string& val) -> void {
                SP_ASSERT(cvar);
                toPtr<IConsoleVariable>(cvar)->SetWithCurrentPriority(Unreal::toTCharPtr(val));
            });

        //
        // Execute console command
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "execute_console_command",
            [this](uint64_t& world, std::string& command) -> void {
                SP_ASSERT(GEngine);
                UWorld* world_ptr = toPtr<UWorld>(world);
                ULocalPlayer* local_player = GEngine->GetFirstGamePlayer(world_ptr);

                bool handled = false;
                if (!handled && local_player) {
                    handled = local_player->Exec(world_ptr, Unreal::toTCharPtr(command), *GLog);
                } if (!handled) {
                    handled = GEngine->Exec(world_ptr, Unreal::toTCharPtr(command));
                } if (!handled) {
                    SP_LOG("WARNING: \"", command, "\" command not handled.");
                }
            });

        //
        // Flush output log messages
        //

        unreal_entry_point_binder->bindFuncToExecuteOnWorkerThread("unreal_service", "flush_output_log_messages",
            [this]() -> SpOutputLogMessages { return OutputLog::flush(); });

        //
        // Get message log names and messages
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_message_log_names",
            [this]() -> std::vector<std::string> {
                // Not available in Shipping builds, since MessageLog is a Developer-category module -- see
                // the WITH_UNREAL_DEVELOPER_TOOLS include guards above.
                #if WITH_UNREAL_DEVELOPER_TOOLS
                    SP_ASSERT_MODULE_LOADED("MessageLog");
                    FMessageLogModule* message_log_module = FModuleManager::Get().GetModulePtr<FMessageLogModule>("MessageLog");
                    SP_ASSERT(message_log_module);
                    SP_ASSERT(message_log_module->MessageLogViewModel);

                    return Std::toVector<std::string>(
                        Unreal::toStdVector(message_log_module->MessageLogViewModel->GetLogListingViewModels()) |
                        std::views::transform([](const IMessageLogListingPtr& listing) { SP_ASSERT(listing); return Unreal::toStdString(listing->GetName()); }));
                #else
                    SP_ASSERT(false);
                    return std::vector<std::string>();
                #endif
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_message_log_messages",
            [this](std::string& name) -> std::vector<std::string> {
                // Not available in Shipping builds, since MessageLog is a Developer-category module -- see
                // the WITH_UNREAL_DEVELOPER_TOOLS include guards above.
                #if WITH_UNREAL_DEVELOPER_TOOLS
                    SP_ASSERT_MODULE_LOADED("MessageLog");
                    FMessageLogModule* message_log_module = FModuleManager::Get().GetModulePtr<FMessageLogModule>("MessageLog");
                    SP_ASSERT(message_log_module);
                    SP_ASSERT(message_log_module->MessageLogViewModel);

                    // FMessageLogViewModel::FindLogListingViewModel(...) and FMessageLogModel::GetLogListingModel(...)
                    // are not exported from the MessageLog module (FMessageLogViewModel and FMessageLogModule have
                    // no MESSAGELOG_API), so we can't call them directly across the module boundary. We instead
                    // linearly search the (inline, and therefore always linkable) GetLogListingViewModels() array
                    // for the matching name.
                    FName log_name = Unreal::toFName(name);
                    TSharedPtr<FMessageLogListingViewModel> listing;
                    for (const IMessageLogListingPtr& candidate : message_log_module->MessageLogViewModel->GetLogListingViewModels()) {
                        SP_ASSERT(candidate);
                        if (candidate->GetName() == log_name) {
                            listing = StaticCastSharedPtr<FMessageLogListingViewModel>(candidate);
                            break;
                        }
                    }
                    SP_ASSERT(listing);
                    SP_ASSERT(listing->MessageLogListingModel);

                    // Deliberately read from FMessageLogListingModel rather than
                    // FMessageLogListingViewModel::GetFilteredMessages(...), so we return every message across
                    // every page, regardless of any severity/search filtering that might be active in the UI.
                    TSharedPtr<FMessageLogListingModel> listing_model = listing->MessageLogListingModel;

                    std::vector<std::string> messages;
                    for (uint32 page_index = 0; page_index < listing_model->NumPages(); page_index++) {
                        for (MessageContainer::TConstIterator it = listing_model->GetMessageIterator(page_index); it; ++it) {
                            messages.push_back(Unreal::toStdString((*it)->ToText().ToString()));
                        }
                    }

                    return messages;
                #else
                    SP_ASSERT(false);
                    return std::vector<std::string>();
                #endif
            });

        //
        // Stable name helper functions
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_stable_name_for_actor",
            [this](uint64_t& actor, bool& include_unreal_name) -> std::string { return UnrealUtils::getStableName(toPtr<AActor>(actor), include_unreal_name); });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "set_stable_name_for_actor",
            [this](uint64_t& actor, std::string& stable_name) -> void { UnrealUtils::setStableName(toPtr<AActor>(actor), stable_name); });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_stable_name_for_component",
            [this](uint64_t& actor_component, bool& include_actor_stable_name, bool& include_actor_unreal_name) -> std::string {
                return UnrealUtils::getStableName(toPtr<UActorComponent>(actor_component), include_actor_stable_name, include_actor_unreal_name);
            });

        //
        // Get actor and component tags
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_actor_tags",
            [this](uint64_t& actor) -> std::vector<std::string> { return Unreal::getTags(toPtr<AActor>(actor)); });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_component_tags",
            [this](uint64_t& component) -> std::vector<std::string> { return Unreal::getTags(toPtr<UActorComponent>(component)); });

        //
        // Find actors unconditionally and return an std::vector or std::map
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actors",
            [this](uint64_t& world) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::findActors(toPtr<UWorld>(world)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actors_as_map",
            [this](uint64_t& world, bool& include_unreal_name) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::findActorsAsMap(toPtr<UWorld>(world), include_unreal_name));
            });

        //
        // Get components unconditionally and return an std::vector or std::map
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_components",
            [this](uint64_t& actor) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::getComponents(toPtr<AActor>(actor)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_components_as_map",
            [this](uint64_t& actor, bool& include_actor_stable_name, bool& include_actor_unreal_name) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::getComponentsAsMap(toPtr<AActor>(actor), include_actor_stable_name, include_actor_unreal_name));
            });

        //
        // Get children components unconditionally and return an std::vector or std::map
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_actor",
            [this](uint64_t& parent, bool& include_all_descendants) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponents(toPtr<AActor>(parent), include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_actor_as_map",
            [this](uint64_t& parent, bool& include_all_descendants, bool& include_actor_stable_name, bool& include_actor_unreal_name) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsAsMap(toPtr<AActor>(parent), include_all_descendants, include_actor_stable_name, include_actor_unreal_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_scene_component",
            [this](uint64_t& parent, bool& include_all_descendants) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponents(toPtr<USceneComponent>(parent), include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_scene_component_as_map",
            [this](uint64_t& parent, bool& include_all_descendants, bool& include_actor_stable_name, bool& include_actor_unreal_name) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsAsMap(toPtr<USceneComponent>(parent), include_all_descendants, include_actor_stable_name, include_actor_unreal_name));
            });

        //
        // Find actors conditionally and return an std::vector
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actors_by_name",
            [this](uint64_t& uclass, uint64_t& world, std::string& actor_name) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::findActorsByName(toPtr<UClass>(uclass), toPtr<UWorld>(world), actor_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actors_by_tag",
            [this](uint64_t& uclass, uint64_t& world, std::string& tag) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::findActorsByTag(toPtr<UClass>(uclass), toPtr<UWorld>(world), tag));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actors_by_tags_any",
            [this](uint64_t& uclass, uint64_t& world, std::vector<std::string>& tags) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::findActorsByTagsAny(toPtr<UClass>(uclass), toPtr<UWorld>(world), tags));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actors_by_tags_all",
            [this](uint64_t& uclass, uint64_t& world, std::vector<std::string>& tags) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::findActorsByTagsAll(toPtr<UClass>(uclass), toPtr<UWorld>(world), tags));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actors_by_class",
            [this](uint64_t& uclass, uint64_t& world) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::findActorsByClass(toPtr<UClass>(uclass), toPtr<UWorld>(world)));
            });

        //
        // Find actors conditionally and return an std::map
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actors_by_name_as_map",
            [this](uint64_t& uclass, uint64_t& world, std::string& actor_name, bool& include_unreal_name) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::findActorsByNameAsMap(toPtr<UClass>(uclass), toPtr<UWorld>(world), actor_name, include_unreal_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actors_by_tag_as_map",
            [this](uint64_t& uclass, uint64_t& world, std::string& tag, bool& include_unreal_name) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::findActorsByTagAsMap(toPtr<UClass>(uclass), toPtr<UWorld>(world), tag, include_unreal_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actors_by_tags_any_as_map",
            [this](uint64_t& uclass, uint64_t& world, std::vector<std::string>& tags, bool& include_unreal_name) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::findActorsByTagsAnyAsMap(toPtr<UClass>(uclass), toPtr<UWorld>(world), tags, include_unreal_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actors_by_tags_all_as_map",
            [this](uint64_t& uclass, uint64_t& world, std::vector<std::string>& tags, bool& include_unreal_name) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::findActorsByTagsAllAsMap(toPtr<UClass>(uclass), toPtr<UWorld>(world), tags, include_unreal_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actors_by_class_as_map",
            [this](uint64_t& uclass, uint64_t& world, bool& include_unreal_name) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::findActorsByClassAsMap(toPtr<UClass>(uclass), toPtr<UWorld>(world), include_unreal_name));
            });

        //
        // Find actor conditionally
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actor_by_name",
            [this](uint64_t& uclass, uint64_t& world, std::string& actor_name) -> uint64_t {
                return toUInt64(UnrealUtils::findActorByName(toPtr<UClass>(uclass), toPtr<UWorld>(world), actor_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actor_by_tag",
            [this](uint64_t& uclass, uint64_t& world, std::string& tag) -> uint64_t {
                return toUInt64(UnrealUtils::findActorByTag(toPtr<UClass>(uclass), toPtr<UWorld>(world), tag));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actor_by_tags_any",
            [this](uint64_t& uclass, uint64_t& world, std::vector<std::string>& tags) -> uint64_t {
                return toUInt64(UnrealUtils::findActorByTagsAny(toPtr<UClass>(uclass), toPtr<UWorld>(world), tags));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actor_by_tags_all",
            [this](uint64_t& uclass, uint64_t& world, std::vector<std::string>& tags) -> uint64_t {
                return toUInt64(UnrealUtils::findActorByTagsAll(toPtr<UClass>(uclass), toPtr<UWorld>(world), tags));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actor_by_class",
            [this](uint64_t& uclass, uint64_t& world) -> uint64_t {
                return toUInt64(UnrealUtils::findActorByClass(toPtr<UClass>(uclass), toPtr<UWorld>(world)));
            });

        //
        // Get components conditionally and return an std::vector
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_components_by_name",
            [this](uint64_t& uclass, uint64_t& actor, std::string& component_name, bool& include_from_child_actors) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::getComponentsByName(toPtr<UClass>(uclass), toPtr<AActor>(actor), component_name, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_components_by_path",
            [this](uint64_t& uclass, uint64_t& actor, std::string& component_path, bool& include_from_child_actors) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::getComponentsByPath(toPtr<UClass>(uclass), toPtr<AActor>(actor), component_path, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_components_by_tag",
            [this](uint64_t& uclass, uint64_t& actor, std::string& tag, bool& include_from_child_actors) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::getComponentsByTag(toPtr<UClass>(uclass), toPtr<AActor>(actor), tag, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_components_by_tags_any",
            [this](uint64_t& uclass, uint64_t& actor, std::vector<std::string>& tags, bool& include_from_child_actors) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::getComponentsByTagsAny(toPtr<UClass>(uclass), toPtr<AActor>(actor), tags, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_components_by_tags_all",
            [this](uint64_t& uclass, uint64_t& actor, std::vector<std::string>& tags, bool& include_from_child_actors) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::getComponentsByTagsAll(toPtr<UClass>(uclass), toPtr<AActor>(actor), tags, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_components_by_class",
            [this](uint64_t& uclass, uint64_t& actor, bool& include_from_child_actors) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::getComponentsByClass(toPtr<UClass>(uclass), toPtr<AActor>(actor), include_from_child_actors));
            });

        //
        // Get components conditionally and return an std::map
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_components_by_name_as_map",
            [this](uint64_t& uclass, uint64_t& actor, std::string& component_name, bool& include_from_child_actors, bool& include_actor_stable_name, bool& include_actor_unreal_name) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::getComponentsByNameAsMap(toPtr<UClass>(uclass), toPtr<AActor>(actor), component_name, include_from_child_actors, include_actor_stable_name, include_actor_unreal_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_components_by_path_as_map",
            [this](uint64_t& uclass, uint64_t& actor, std::string& component_path, bool& include_from_child_actors, bool& include_actor_stable_name, bool& include_actor_unreal_name) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::getComponentsByPathAsMap(toPtr<UClass>(uclass), toPtr<AActor>(actor), component_path, include_from_child_actors, include_actor_stable_name, include_actor_unreal_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_components_by_tag_as_map",
            [this](uint64_t& uclass, uint64_t& actor, std::string& tag, bool& include_from_child_actors, bool& include_actor_stable_name, bool& include_actor_unreal_name) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::getComponentsByTagAsMap(toPtr<UClass>(uclass), toPtr<AActor>(actor), tag, include_from_child_actors, include_actor_stable_name, include_actor_unreal_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_components_by_tags_any_as_map",
            [this](uint64_t& uclass, uint64_t& actor, std::vector<std::string>& tags, bool& include_from_child_actors, bool& include_actor_stable_name, bool& include_actor_unreal_name) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::getComponentsByTagsAnyAsMap(toPtr<UClass>(uclass), toPtr<AActor>(actor), tags, include_from_child_actors, include_actor_stable_name, include_actor_unreal_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_components_by_tags_all_as_map",
            [this](uint64_t& uclass, uint64_t& actor, std::vector<std::string>& tags, bool& include_from_child_actors, bool& include_actor_stable_name, bool& include_actor_unreal_name) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::getComponentsByTagsAllAsMap(toPtr<UClass>(uclass), toPtr<AActor>(actor), tags, include_from_child_actors, include_actor_stable_name, include_actor_unreal_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_components_by_class_as_map",
            [this](uint64_t& uclass, uint64_t& actor, bool& include_from_child_actors, bool& include_actor_stable_name, bool& include_actor_unreal_name) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::getComponentsByClassAsMap(toPtr<UClass>(uclass), toPtr<AActor>(actor), include_from_child_actors, include_actor_stable_name, include_actor_unreal_name));
            });

        //
        // Get component conditionally
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_component_by_name",
            [this](uint64_t& uclass, uint64_t& actor, std::string& component_name, bool& include_from_child_actors) -> uint64_t {
                return toUInt64(UnrealUtils::getComponentByName(toPtr<UClass>(uclass), toPtr<AActor>(actor), component_name, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_component_by_path",
            [this](uint64_t& uclass, uint64_t& actor, std::string& component_path, bool& include_from_child_actors) -> uint64_t {
                return toUInt64(UnrealUtils::getComponentByPath(toPtr<UClass>(uclass), toPtr<AActor>(actor), component_path, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_component_by_tag",
            [this](uint64_t& uclass, uint64_t& actor, std::string& tag, bool& include_from_child_actors) -> uint64_t {
                return toUInt64(UnrealUtils::getComponentByTag(toPtr<UClass>(uclass), toPtr<AActor>(actor), tag, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_component_by_tags_any",
            [this](uint64_t& uclass, uint64_t& actor, std::vector<std::string>& tags, bool& include_from_child_actors) -> uint64_t {
                return toUInt64(UnrealUtils::getComponentByTagsAny(toPtr<UClass>(uclass), toPtr<AActor>(actor), tags, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_component_by_tags_all",
            [this](uint64_t& uclass, uint64_t& actor, std::vector<std::string>& tags, bool& include_from_child_actors) -> uint64_t {
                return toUInt64(UnrealUtils::getComponentByTagsAll(toPtr<UClass>(uclass), toPtr<AActor>(actor), tags, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_component_by_class",
            [this](uint64_t& uclass, uint64_t& actor, bool& include_from_child_actors) -> uint64_t {
                return toUInt64(UnrealUtils::getComponentByClass(toPtr<UClass>(uclass), toPtr<AActor>(actor), include_from_child_actors));
            });

        //
        // Get children components conditionally from an actor and return an std::vector
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_actor_by_name",
            [this](uint64_t& uclass, uint64_t& parent, std::string& child_component_name, bool& include_all_descendants) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsByName(toPtr<UClass>(uclass), toPtr<AActor>(parent), child_component_name, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_actor_by_tag",
            [this](uint64_t& uclass, uint64_t& parent, std::string& tag, bool& include_all_descendants) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsByTag(toPtr<UClass>(uclass), toPtr<AActor>(parent), tag, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_actor_by_tags_any",
            [this](uint64_t& uclass, uint64_t& parent, std::vector<std::string>& tags, bool& include_all_descendants) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsByTagsAny(toPtr<UClass>(uclass), toPtr<AActor>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_actor_by_tags_all",
            [this](uint64_t& uclass, uint64_t& parent, std::vector<std::string>& tags, bool& include_all_descendants) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsByTagsAll(toPtr<UClass>(uclass), toPtr<AActor>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_actor_by_class",
            [this](uint64_t& uclass, uint64_t& parent, bool& include_all_descendants) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsByClass(toPtr<UClass>(uclass), toPtr<AActor>(parent), include_all_descendants));
            });

        //
        // Get children components conditionally from an actor and return an std::map
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_actor_by_name_as_map",
            [this](uint64_t& uclass, uint64_t& parent, std::string& child_component_name, bool& include_all_descendants, bool& include_actor_stable_name, bool& include_actor_unreal_name) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsByNameAsMap(toPtr<UClass>(uclass), toPtr<AActor>(parent), child_component_name, include_all_descendants, include_actor_stable_name, include_actor_unreal_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_actor_by_tag_as_map",
            [this](uint64_t& uclass, uint64_t& parent, std::string& tag, bool& include_all_descendants, bool& include_actor_stable_name, bool& include_actor_unreal_name) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsByTagAsMap(toPtr<UClass>(uclass), toPtr<AActor>(parent), tag, include_all_descendants, include_actor_stable_name, include_actor_unreal_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_actor_by_tags_any_as_map",
            [this](uint64_t& uclass, uint64_t& parent, std::vector<std::string>& tags, bool& include_all_descendants, bool& include_actor_stable_name, bool& include_actor_unreal_name) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsByTagsAnyAsMap(toPtr<UClass>(uclass), toPtr<AActor>(parent), tags, include_all_descendants, include_actor_stable_name, include_actor_unreal_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_actor_by_tags_all_as_map",
            [this](uint64_t& uclass, uint64_t& parent, std::vector<std::string>& tags, bool& include_all_descendants, bool& include_actor_stable_name, bool& include_actor_unreal_name) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsByTagsAllAsMap(toPtr<UClass>(uclass), toPtr<AActor>(parent), tags, include_all_descendants, include_actor_stable_name, include_actor_unreal_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_actor_by_class_as_map",
            [this](uint64_t& uclass, uint64_t& parent, bool& include_all_descendants, bool& include_actor_stable_name, bool& include_actor_unreal_name) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsByClassAsMap(toPtr<UClass>(uclass), toPtr<AActor>(parent), include_all_descendants, include_actor_stable_name, include_actor_unreal_name));
            });

        //
        // Get child component conditionally from an actor
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_child_component_for_actor_by_name",
            [this](uint64_t& uclass, uint64_t& parent, std::string& child_component_name, bool& include_all_descendants) -> uint64_t {
                return toUInt64(UnrealUtils::getChildComponentByName(toPtr<UClass>(uclass), toPtr<AActor>(parent), child_component_name, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_child_component_for_actor_by_tag",
            [this](uint64_t& uclass, uint64_t& parent, std::string& tag, bool& include_all_descendants) -> uint64_t {
                return toUInt64(UnrealUtils::getChildComponentByTag(toPtr<UClass>(uclass), toPtr<AActor>(parent), tag, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_child_component_for_actor_by_tags_any",
            [this](uint64_t& uclass, uint64_t& parent, std::vector<std::string>& tags, bool& include_all_descendants) -> uint64_t {
                return toUInt64(UnrealUtils::getChildComponentByTagsAny(toPtr<UClass>(uclass), toPtr<AActor>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_child_component_for_actor_by_tags_all",
            [this](uint64_t& uclass, uint64_t& parent, std::vector<std::string>& tags, bool& include_all_descendants) -> uint64_t {
                return toUInt64(UnrealUtils::getChildComponentByTagsAll(toPtr<UClass>(uclass), toPtr<AActor>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_child_component_for_actor_by_class",
            [this](uint64_t& uclass, uint64_t& parent, bool& include_all_descendants) -> uint64_t {
                return toUInt64(UnrealUtils::getChildComponentByClass(toPtr<UClass>(uclass), toPtr<AActor>(parent), include_all_descendants));
            });

        //
        // Get children components conditionally from a scene component and return an std::vector
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_scene_component_by_name",
            [this](uint64_t& uclass, uint64_t& parent, std::string& child_component_name, bool& include_all_descendants) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsByName(toPtr<UClass>(uclass), toPtr<USceneComponent>(parent), child_component_name, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_scene_component_by_tag",
            [this](uint64_t& uclass, uint64_t& parent, std::string& tag, bool& include_all_descendants) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsByTag(toPtr<UClass>(uclass), toPtr<USceneComponent>(parent), tag, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_scene_component_by_tags_any",
            [this](uint64_t& uclass, uint64_t& parent, std::vector<std::string>& tags, bool& include_all_descendants) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsByTagsAny(toPtr<UClass>(uclass), toPtr<USceneComponent>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_scene_component_by_tags_all",
            [this](uint64_t& uclass, uint64_t& parent, std::vector<std::string>& tags, bool& include_all_descendants) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsByTagsAll(toPtr<UClass>(uclass), toPtr<USceneComponent>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_scene_component_by_class",
            [this](uint64_t& uclass, uint64_t& parent, bool& include_all_descendants) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsByClass(toPtr<UClass>(uclass), toPtr<USceneComponent>(parent), include_all_descendants));
            });

        //
        // Get children components conditionally from a scene component and return an std::map
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_scene_component_by_name_as_map",
            [this](uint64_t& uclass, uint64_t& parent, std::string& child_component_name, bool& include_all_descendants, bool& include_actor_stable_name, bool& include_actor_unreal_name) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsByNameAsMap(toPtr<UClass>(uclass), toPtr<USceneComponent>(parent), child_component_name, include_all_descendants, include_actor_stable_name, include_actor_unreal_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_scene_component_by_tag_as_map",
            [this](uint64_t& uclass, uint64_t& parent, std::string& tag, bool& include_all_descendants, bool& include_actor_stable_name, bool& include_actor_unreal_name) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsByTagAsMap(toPtr<UClass>(uclass), toPtr<USceneComponent>(parent), tag, include_all_descendants, include_actor_stable_name, include_actor_unreal_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_scene_component_by_tags_any_as_map",
            [this](uint64_t& uclass, uint64_t& parent, std::vector<std::string>& tags, bool& include_all_descendants, bool& include_actor_stable_name, bool& include_actor_unreal_name) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsByTagsAnyAsMap(toPtr<UClass>(uclass), toPtr<USceneComponent>(parent), tags, include_all_descendants, include_actor_stable_name, include_actor_unreal_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_scene_component_by_tags_all_as_map",
            [this](uint64_t& uclass, uint64_t& parent, std::vector<std::string>& tags, bool& include_all_descendants, bool& include_actor_stable_name, bool& include_actor_unreal_name) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsByTagsAllAsMap(toPtr<UClass>(uclass), toPtr<USceneComponent>(parent), tags, include_all_descendants, include_actor_stable_name, include_actor_unreal_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_scene_component_by_class_as_map",
            [this](uint64_t& uclass, uint64_t& parent, bool& include_all_descendants, bool& include_actor_stable_name, bool& include_actor_unreal_name) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsByClassAsMap(toPtr<UClass>(uclass), toPtr<USceneComponent>(parent), include_all_descendants, include_actor_stable_name, include_actor_unreal_name));
            });

        //
        // Get child component conditionally from a scene component
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_child_component_for_scene_component_by_name",
            [this](uint64_t& uclass, uint64_t& parent, std::string& child_component_name, bool& include_all_descendants) -> uint64_t {
                return toUInt64(UnrealUtils::getChildComponentByName(toPtr<UClass>(uclass), toPtr<USceneComponent>(parent), child_component_name, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_child_component_for_scene_component_by_tag",
            [this](uint64_t& uclass, uint64_t& parent, std::string& tag, bool& include_all_descendants) -> uint64_t {
                return toUInt64(UnrealUtils::getChildComponentByTag(toPtr<UClass>(uclass), toPtr<USceneComponent>(parent), tag, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_child_component_for_scene_component_by_tags_any",
            [this](uint64_t& uclass, uint64_t& parent, std::vector<std::string>& tags, bool& include_all_descendants) -> uint64_t {
                return toUInt64(UnrealUtils::getChildComponentByTagsAny(toPtr<UClass>(uclass), toPtr<USceneComponent>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_child_component_for_scene_component_by_tags_all",
            [this](uint64_t& uclass, uint64_t& parent, std::vector<std::string>& tags, bool& include_all_descendants) -> uint64_t {
                return toUInt64(UnrealUtils::getChildComponentByTagsAll(toPtr<UClass>(uclass), toPtr<USceneComponent>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_child_component_for_scene_component_by_class",
            [this](uint64_t& uclass, uint64_t& parent, bool& include_all_descendants) -> uint64_t {
                return toUInt64(UnrealUtils::getChildComponentByClass(toPtr<UClass>(uclass), toPtr<USceneComponent>(parent), include_all_descendants));
            });

        //
        // Create component
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "create_component_outside_owner_constructor_by_class",
            [this](uint64_t& uclass, uint64_t& owner, std::string& component_name) -> uint64_t {
                return toUInt64(UnrealUtils::createComponentOutsideOwnerConstructorByClass(toPtr<UClass>(uclass), toPtr<AActor>(owner), component_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "create_scene_component_outside_owner_constructor_for_actor_by_class",
            [this](uint64_t& uclass, uint64_t& owner, std::string& scene_component_name) -> uint64_t {
                return toUInt64(UnrealUtils::createSceneComponentOutsideOwnerConstructorByClass(toPtr<UClass>(uclass), toPtr<AActor>(owner), scene_component_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "create_scene_component_outside_owner_constructor_for_object_by_class",
            [this](uint64_t& uclass, uint64_t& owner, uint64_t& parent, std::string& scene_component_name) -> uint64_t {
                return toUInt64(UnrealUtils::createSceneComponentOutsideOwnerConstructorByClass(toPtr<UClass>(uclass), toPtr<UObject>(owner), toPtr<USceneComponent>(parent), scene_component_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "create_scene_component_outside_owner_constructor_for_scene_component_by_class",
            [this](uint64_t& uclass, uint64_t& owner, std::string& scene_component_name) -> uint64_t {
                return toUInt64(UnrealUtils::createSceneComponentOutsideOwnerConstructorByClass(toPtr<UClass>(uclass), toPtr<USceneComponent>(owner), scene_component_name));
            });

        //
        // Destroy component
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "destroy_component_outside_owner_constructor",
            [this](uint64_t& component, bool& promote_children) -> void {
                SP_ASSERT(component);
                UnrealUtils::destroyComponentOutsideOwnerConstructor(toPtr<UActorComponent>(component), promote_children);
            });
    }
};
