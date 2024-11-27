//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // uint64_t

#include <map>
#include <string>
#include <utility> // std::make_pair, std::move
#include <vector>

#include <Components/ChildActorComponent.h>
#include <Delegates/IDelegateInstance.h> // FDelegateHandle
#include <Engine/Engine.h>               // GEngine
#include <Engine/Level.h>                // ULevel
#include <Engine/World.h>                // FWorldDelegates, FActorSpawnParameters
#include <HAL/IConsoleManager.h>
#include <Misc/EnumClassFlags.h>         // ENUM_CLASS_FLAGS
#include <UObject/Class.h>               // EIncludeSuperFlag::Type
#include <UObject/ObjectMacros.h>        // EObjectFlags, ELoadFlags
#include <UObject/Package.h>

#include "SpCore/Assert.h"
#include "SpCore/Unreal.h"
#include "SpCore/UnrealClassRegistrar.h"

#include "SpServices/EntryPointBinder.h"
#include "SpServices/Msgpack.h"
#include "SpServices/Rpclib.h"
#include "SpServices/Service.h"

#include "UnrealService.generated.h"

// Each USTRUCT below is intended to be a wrapper for a particular UENUM type. Wrapping enums in structs like
// this helps us take advantage of the Unreal property system to pass enums to and from Python as human-
// readable strings. Additionally, we make use of these enum structs to combine strings enum strings as
// though they were bit flags. Unfortunately, we can't abbreviate these declarations using higher-level
// macros, any more than they already are, because then they wouldn't interact correctly with the Unreal
// build system.

//
// This enum corresponds to EIncludeSuperFlag::Type declared in Engine/Source/Runtime/CoreUObject/Public/UObject/Class.h
//

UENUM()
enum class ESpIncludeSuperFlag
{
    ExcludeSuper = Unreal::getConstEnumValue(EIncludeSuperFlag::Type::ExcludeSuper),
    IncludeSuper = Unreal::getConstEnumValue(EIncludeSuperFlag::Type::IncludeSuper)
};

//
// This enum corresponds to EObjectFlags declared in Engine/Source/Runtime/CoreUObject/Public/UObject/ObjectMacros.h
//

UENUM(Flags)
enum class ESpObjectFlags
{
    RF_NoFlags                      = Unreal::getConstEnumValue(EObjectFlags::RF_NoFlags),
    RF_Public                       = Unreal::getConstEnumValue(EObjectFlags::RF_Public),
    RF_Standalone                   = Unreal::getConstEnumValue(EObjectFlags::RF_Standalone),
    RF_MarkAsNative                 = Unreal::getConstEnumValue(EObjectFlags::RF_MarkAsNative),
    RF_Transactional                = Unreal::getConstEnumValue(EObjectFlags::RF_Transactional),
    RF_ClassDefaultObject           = Unreal::getConstEnumValue(EObjectFlags::RF_ClassDefaultObject),
    RF_ArchetypeObject              = Unreal::getConstEnumValue(EObjectFlags::RF_ArchetypeObject),
    RF_Transient                    = Unreal::getConstEnumValue(EObjectFlags::RF_Transient),
    RF_MarkAsRootSet                = Unreal::getConstEnumValue(EObjectFlags::RF_MarkAsRootSet),
    RF_TagGarbageTemp               = Unreal::getConstEnumValue(EObjectFlags::RF_TagGarbageTemp),
    RF_NeedInitialization           = Unreal::getConstEnumValue(EObjectFlags::RF_NeedInitialization),
    RF_NeedLoad                     = Unreal::getConstEnumValue(EObjectFlags::RF_NeedLoad),
    RF_KeepForCooker                = Unreal::getConstEnumValue(EObjectFlags::RF_KeepForCooker),
    RF_NeedPostLoad                 = Unreal::getConstEnumValue(EObjectFlags::RF_NeedPostLoad),
    RF_NeedPostLoadSubobjects       = Unreal::getConstEnumValue(EObjectFlags::RF_NeedPostLoadSubobjects),
    RF_NewerVersionExists           = Unreal::getConstEnumValue(EObjectFlags::RF_NewerVersionExists),
    RF_BeginDestroyed               = Unreal::getConstEnumValue(EObjectFlags::RF_BeginDestroyed),
    RF_FinishDestroyed              = Unreal::getConstEnumValue(EObjectFlags::RF_FinishDestroyed),
    RF_BeingRegenerated             = Unreal::getConstEnumValue(EObjectFlags::RF_BeingRegenerated),
    RF_DefaultSubObject             = Unreal::getConstEnumValue(EObjectFlags::RF_DefaultSubObject),
    RF_WasLoaded                    = Unreal::getConstEnumValue(EObjectFlags::RF_WasLoaded),
    RF_TextExportTransient          = Unreal::getConstEnumValue(EObjectFlags::RF_TextExportTransient),
    RF_LoadCompleted                = Unreal::getConstEnumValue(EObjectFlags::RF_LoadCompleted),
    RF_InheritableComponentTemplate = Unreal::getConstEnumValue(EObjectFlags::RF_InheritableComponentTemplate),
    RF_DuplicateTransient           = Unreal::getConstEnumValue(EObjectFlags::RF_DuplicateTransient),
    RF_StrongRefOnFrame             = Unreal::getConstEnumValue(EObjectFlags::RF_StrongRefOnFrame),
    RF_NonPIEDuplicateTransient     = Unreal::getConstEnumValue(EObjectFlags::RF_NonPIEDuplicateTransient),
    RF_WillBeLoaded                 = Unreal::getConstEnumValue(EObjectFlags::RF_WillBeLoaded),
    RF_HasExternalPackage           = Unreal::getConstEnumValue(EObjectFlags::RF_HasExternalPackage),
    RF_AllocatedInSharedPage        = Unreal::getConstEnumValue(EObjectFlags::RF_AllocatedInSharedPage)
};
ENUM_CLASS_FLAGS(ESpObjectFlags); // required if combining values using bitwise operations

//
// This enum corresponds to ELoadFlags declared in Engine/Source/Runtime/CoreUObject/Public/UObject/ObjectMacros.h
//

UENUM(Flags)
enum class ESpLoadFlags
{
    LOAD_None                        = Unreal::getConstEnumValue(ELoadFlags::LOAD_None),
    LOAD_Async                       = Unreal::getConstEnumValue(ELoadFlags::LOAD_Async),
    LOAD_NoWarn                      = Unreal::getConstEnumValue(ELoadFlags::LOAD_NoWarn),
    LOAD_EditorOnly                  = Unreal::getConstEnumValue(ELoadFlags::LOAD_EditorOnly),
    LOAD_ResolvingDeferredExports    = Unreal::getConstEnumValue(ELoadFlags::LOAD_ResolvingDeferredExports),
    LOAD_Verify                      = Unreal::getConstEnumValue(ELoadFlags::LOAD_Verify),
    LOAD_NoVerify                    = Unreal::getConstEnumValue(ELoadFlags::LOAD_NoVerify),
    LOAD_IsVerifying                 = Unreal::getConstEnumValue(ELoadFlags::LOAD_IsVerifying),
    LOAD_SkipLoadImportedPackages    = Unreal::getConstEnumValue(ELoadFlags::LOAD_SkipLoadImportedPackages),
    LOAD_RegenerateBulkDataGuids     = Unreal::getConstEnumValue(ELoadFlags::LOAD_RegenerateBulkDataGuids),
    LOAD_DisableDependencyPreloading = Unreal::getConstEnumValue(ELoadFlags::LOAD_DisableDependencyPreloading),
    LOAD_Quiet                       = Unreal::getConstEnumValue(ELoadFlags::LOAD_Quiet),
    LOAD_FindIfFail                  = Unreal::getConstEnumValue(ELoadFlags::LOAD_FindIfFail),
    LOAD_MemoryReader                = Unreal::getConstEnumValue(ELoadFlags::LOAD_MemoryReader),
    LOAD_NoRedirects                 = Unreal::getConstEnumValue(ELoadFlags::LOAD_NoRedirects),
    LOAD_ForDiff                     = Unreal::getConstEnumValue(ELoadFlags::LOAD_ForDiff),
    LOAD_PackageForPIE               = Unreal::getConstEnumValue(ELoadFlags::LOAD_PackageForPIE),
    LOAD_DeferDependencyLoads        = Unreal::getConstEnumValue(ELoadFlags::LOAD_DeferDependencyLoads),
    LOAD_ForFileDiff                 = Unreal::getConstEnumValue(ELoadFlags::LOAD_ForFileDiff),
    LOAD_DisableCompileOnLoad        = Unreal::getConstEnumValue(ELoadFlags::LOAD_DisableCompileOnLoad),
    LOAD_DisableEngineVersionChecks  = Unreal::getConstEnumValue(ELoadFlags::LOAD_DisableEngineVersionChecks)
};
ENUM_CLASS_FLAGS(ESpLoadFlags); // required if combining values using bitwise operations

//
// This enum corresponds to EConsoleVariableFlags declared in Engine/Source/Runtime/Core/Public/HAL/IConsoleManager.h
//

UENUM(Flags)
enum class ESpConsoleVariableFlags
{
    ECVF_FlagMask                 = Unreal::getConstEnumValue(EConsoleVariableFlags::ECVF_FlagMask),
    ECVF_Default                  = Unreal::getConstEnumValue(EConsoleVariableFlags::ECVF_Default),
    ECVF_Cheat                    = Unreal::getConstEnumValue(EConsoleVariableFlags::ECVF_Cheat),
    ECVF_ReadOnly                 = Unreal::getConstEnumValue(EConsoleVariableFlags::ECVF_ReadOnly),
    ECVF_Unregistered             = Unreal::getConstEnumValue(EConsoleVariableFlags::ECVF_Unregistered),
    ECVF_CreatedFromIni           = Unreal::getConstEnumValue(EConsoleVariableFlags::ECVF_CreatedFromIni),
    ECVF_RenderThreadSafe         = Unreal::getConstEnumValue(EConsoleVariableFlags::ECVF_RenderThreadSafe),
    ECVF_Scalability              = Unreal::getConstEnumValue(EConsoleVariableFlags::ECVF_Scalability),
    ECVF_ScalabilityGroup         = Unreal::getConstEnumValue(EConsoleVariableFlags::ECVF_ScalabilityGroup),
    ECVF_Preview                  = Unreal::getConstEnumValue(EConsoleVariableFlags::ECVF_Preview),
    ECVF_GeneralShaderChange      = Unreal::getConstEnumValue(EConsoleVariableFlags::ECVF_GeneralShaderChange),
    ECVF_MobileShaderChange       = Unreal::getConstEnumValue(EConsoleVariableFlags::ECVF_MobileShaderChange),
    ECVF_DesktopShaderChange      = Unreal::getConstEnumValue(EConsoleVariableFlags::ECVF_DesktopShaderChange),
    ECVF_ExcludeFromPreview       = Unreal::getConstEnumValue(EConsoleVariableFlags::ECVF_ExcludeFromPreview),
    ECVF_SetFlagMask              = Unreal::getConstEnumValue(EConsoleVariableFlags::ECVF_SetFlagMask),
    ECVF_Set_NoSinkCall_Unsafe    = Unreal::getConstEnumValue(EConsoleVariableFlags::ECVF_Set_NoSinkCall_Unsafe),
    ECVF_SetByMask                = Unreal::getConstEnumValue(EConsoleVariableFlags::ECVF_SetByMask),
    ECVF_SetByConstructor         = Unreal::getConstEnumValue(EConsoleVariableFlags::ECVF_SetByConstructor),
    ECVF_SetByScalability         = Unreal::getConstEnumValue(EConsoleVariableFlags::ECVF_SetByScalability),
    ECVF_SetByGameSetting         = Unreal::getConstEnumValue(EConsoleVariableFlags::ECVF_SetByGameSetting),
    ECVF_SetByProjectSetting      = Unreal::getConstEnumValue(EConsoleVariableFlags::ECVF_SetByProjectSetting),
    ECVF_SetBySystemSettingsIni   = Unreal::getConstEnumValue(EConsoleVariableFlags::ECVF_SetBySystemSettingsIni),
    ECVF_SetByDeviceProfile       = Unreal::getConstEnumValue(EConsoleVariableFlags::ECVF_SetByDeviceProfile),
    ECVF_SetByGameOverride        = Unreal::getConstEnumValue(EConsoleVariableFlags::ECVF_SetByGameOverride),
    ECVF_SetByConsoleVariablesIni = Unreal::getConstEnumValue(EConsoleVariableFlags::ECVF_SetByConsoleVariablesIni),
    ECVF_SetByCommandline         = Unreal::getConstEnumValue(EConsoleVariableFlags::ECVF_SetByCommandline),
    ECVF_SetByCode                = Unreal::getConstEnumValue(EConsoleVariableFlags::ECVF_SetByCode),
    ECVF_SetByConsole             = Unreal::getConstEnumValue(EConsoleVariableFlags::ECVF_SetByConsole)
};
ENUM_CLASS_FLAGS(ESpConsoleVariableFlags); // required if combining values using bitwise operations

//
// This enum corresponds to ESpawnActorNameMode declared in Engine/Source/Runtime/Engine/Classes/Engine/World.h
//

UENUM()
enum class ESpSpawnActorNameMode
{
    Required_Fatal              = Unreal::getConstEnumValue(FActorSpawnParameters::ESpawnActorNameMode::Required_Fatal),
    Required_ErrorAndReturnNull = Unreal::getConstEnumValue(FActorSpawnParameters::ESpawnActorNameMode::Required_ErrorAndReturnNull),
    Required_ReturnNull         = Unreal::getConstEnumValue(FActorSpawnParameters::ESpawnActorNameMode::Required_ReturnNull),
    Requested                   = Unreal::getConstEnumValue(FActorSpawnParameters::ESpawnActorNameMode::Requested)
};

//
// This struct is intended to be identical to Unreal's FActorSpawnParameters struct, see Engine/Source/Runtime/Engine/Classes/Engine/World.h
//

USTRUCT()
struct FSpActorSpawnParameters
{
    GENERATED_BODY()

    UPROPERTY()
    FName Name = NAME_None;

    UPROPERTY()
    AActor* Template = nullptr;

    UPROPERTY()
    AActor* Owner = nullptr;

    UPROPERTY()
    APawn* Instigator = nullptr;

    UPROPERTY()
    ULevel* OverrideLevel = nullptr;

    UPROPERTY()
    UChildActorComponent* OverrideParentComponent = nullptr;

    UPROPERTY()
    ESpawnActorCollisionHandlingMethod SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::Undefined;

    UPROPERTY()
    ESpawnActorScaleMethod TransformScaleMethod = ESpawnActorScaleMethod::MultiplyWithRoot;

    UPROPERTY()
    bool bNoFail = false;

    UPROPERTY()
    bool bDeferConstruction = false;

    UPROPERTY()
    bool bAllowDuringConstructionScript = false;

    UPROPERTY()
    ESpSpawnActorNameMode NameMode = ESpSpawnActorNameMode::Required_Fatal;
};

//
// UnrealService is our service for interacting with the Unreal property system.
//

class UnrealService : public Service {
public:
    UnrealService() = delete;
    UnrealService(CUnrealEntryPointBinder auto* unreal_entry_point_binder)
    {
        SP_ASSERT(unreal_entry_point_binder);

        //
        // Get subsystems
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_subsystem_by_type",
            [this](std::string& class_name) -> uint64_t {
                return toUInt64(UnrealClassRegistrar::getSubsystemByType(class_name, getWorld()));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_subsystem_by_class",
            [this](std::string& class_name, uint64_t& uclass) -> uint64_t {
                return toUInt64(UnrealClassRegistrar::getSubsystemByClass(class_name, getWorld(), toPtr<UClass>(uclass)));
            });

        //
        // Get UClass from class name, get default object from UClass, get UClass from object
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_static_class",
            [this](std::string& class_name) -> uint64_t {
                return toUInt64(UnrealClassRegistrar::getStaticClass(class_name));
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
        // Get static struct
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_static_struct",
            [this](std::string& struct_name) -> uint64_t {
                return toUInt64(UnrealClassRegistrar::getStaticStruct(struct_name));
            });

        //
        // Get and set object properties
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_object_properties_as_string_from_uobject",
            [this](uint64_t& uobject) -> std::string {
                return Unreal::getObjectPropertiesAsString(toPtr<UObject>(uobject));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_object_properties_as_string_from_ustruct",
            [this](uint64_t& value_ptr, uint64_t& ustruct) -> std::string {
                return Unreal::getObjectPropertiesAsString(toPtr<void>(value_ptr), toPtr<UStruct>(ustruct));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "set_object_properties_from_string_for_uobject",
            [this](uint64_t& uobject, std::string& string) -> void {
                Unreal::setObjectPropertiesFromString(toPtr<UObject>(uobject), string);
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "set_object_properties_from_string_for_ustruct",
            [this](uint64_t& value_ptr, uint64_t& ustruct, std::string& string) -> void {
                Unreal::setObjectPropertiesFromString(toPtr<void>(value_ptr), toPtr<UStruct>(ustruct), string);
            });

        //
        // Find properties
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_property_by_name_on_uobject",
            [this](uint64_t& uobject, std::string& property_name) -> Unreal::PropertyDesc {
                return Unreal::findPropertyByName(toPtr<UObject>(uobject), property_name);
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_property_by_name_on_ustruct",
            [this](uint64_t& value_ptr, uint64_t& ustruct, std::string& property_name) -> Unreal::PropertyDesc {
                return Unreal::findPropertyByName(toPtr<void>(value_ptr), toPtr<UStruct>(ustruct), property_name);
            });

        //
        // Get property values
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_property_value_as_string",
            [this](Unreal::PropertyDesc& property_desc) -> std::string {
                return Unreal::getPropertyValueAsString(property_desc);
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "set_property_value_from_string",
            [this](Unreal::PropertyDesc& property_desc, std::string& string) -> void {
                Unreal::setPropertyValueFromString(property_desc, string);
            });

        //
        // Find and call functions
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_function_by_name",
            [this](uint64_t& uclass, std::string& function_name, std::string& include_super_flag_string) -> uint64_t {
                return toUInt64(Unreal::findFunctionByName(
                    toPtr<UClass>(uclass), function_name, Unreal::getEnumValueFromStringAs<EIncludeSuperFlag::Type, ESpIncludeSuperFlag>(include_super_flag_string)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "call_function",
            [this](uint64_t& uobject, uint64_t& ufunction, std::map<std::string, std::string>& args, std::string& world_context) -> std::map<std::string, std::string> {
                return Unreal::callFunction(getWorld(), toPtr<UObject>(uobject), toPtr<UFunction>(ufunction), args, world_context);
            });

        //
        // Find actors unconditionally and return an std::vector or std::map
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actors",
            [this]() -> std::vector<uint64_t> {
                return toUInt64(Unreal::findActors(getWorld()));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actors_as_map",
            [this]() -> std::map<std::string, uint64_t> {
                return toUInt64(Unreal::findActorsAsMap(getWorld()));
            });

        //
        // Get components unconditionally and return an std::vector or std::map
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_components",
            [this](uint64_t& actor) -> std::vector<uint64_t> {
                return toUInt64(Unreal::getComponents(toPtr<AActor>(actor)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_components_as_map",
            [this](uint64_t& actor) -> std::map<std::string, uint64_t> {
                return toUInt64(Unreal::getComponentsAsMap(toPtr<AActor>(actor)));
            });

        //
        // Get children components unconditionally and return an std::vector or std::map
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components",
            [this](uint64_t& parent, bool& include_all_descendants) -> std::vector<uint64_t> {
                return toUInt64(Unreal::getChildrenComponents(toPtr<USceneComponent>(parent), include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_as_map",
            [this](uint64_t& parent, bool& include_all_descendants) -> std::map<std::string, uint64_t> {
                return toUInt64(Unreal::getChildrenComponentsAsMap(toPtr<USceneComponent>(parent), include_all_descendants));
            });

        //
        // Find actors conditionally and return an std::vector
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actors_by_name",
            [this](std::string& class_name, std::vector<std::string>& actor_names, bool& return_null_if_not_found) -> std::vector<uint64_t> {
                return toUInt64(UnrealClassRegistrar::findActorsByName(class_name, getWorld(), actor_names, return_null_if_not_found));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actors_by_tag",
            [this](std::string& class_name, std::string& tag) -> std::vector<uint64_t> {
                return toUInt64(UnrealClassRegistrar::findActorsByTag(class_name, getWorld(), tag));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actors_by_tag_any",
            [this](std::string& class_name, std::vector<std::string>& tags) -> std::vector<uint64_t> {
                return toUInt64(UnrealClassRegistrar::findActorsByTagAny(class_name, getWorld(), tags));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actors_by_tag_all",
            [this](std::string& class_name, std::vector<std::string>& tags) -> std::vector<uint64_t> {
                return toUInt64(UnrealClassRegistrar::findActorsByTagAll(class_name, getWorld(), tags));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actors_by_type",
            [this](std::string& class_name) -> std::vector<uint64_t> {
                return toUInt64(UnrealClassRegistrar::findActorsByType(class_name, getWorld()));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actors_by_class",
            [this](uint64_t& uclass) -> std::vector<uint64_t> {
                return toUInt64(Unreal::findActorsByClass(getWorld(), toPtr<UClass>(uclass)));
            });

        //
        // Find actors conditionally and return an std::map
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actors_by_name_as_map",
            [this](std::string& class_name, std::vector<std::string>& actor_names, bool& return_null_if_not_found) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealClassRegistrar::findActorsByNameAsMap(class_name, getWorld(), actor_names, return_null_if_not_found));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actors_by_tag_as_map",
            [this](std::string& class_name, std::string& tag) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealClassRegistrar::findActorsByTagAsMap(class_name, getWorld(), tag));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actors_by_tag_any_as_map",
            [this](std::string& class_name, std::vector<std::string>& tags) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealClassRegistrar::findActorsByTagAnyAsMap(class_name, getWorld(), tags));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actors_by_tag_all_as_map",
            [this](std::string& class_name, std::vector<std::string>& tags) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealClassRegistrar::findActorsByTagAllAsMap(class_name, getWorld(), tags));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actors_by_type_as_map",
            [this](std::string& class_name) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealClassRegistrar::findActorsByTypeAsMap(class_name, getWorld()));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actors_by_class_as_map",
            [this](uint64_t& uclass) -> std::map<std::string, uint64_t> {
                return toUInt64(Unreal::findActorsByClassAsMap(getWorld(), toPtr<UClass>(uclass)));
            });

        //
        // Find actor conditionally
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actor_by_name",
            [this](std::string& class_name, std::string& actor_name) -> uint64_t {
                return toUInt64(UnrealClassRegistrar::findActorByName(class_name, getWorld(), actor_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actor_by_tag",
            [this](std::string& class_name, std::string& tag) -> uint64_t {
                return toUInt64(UnrealClassRegistrar::findActorByTag(class_name, getWorld(), tag));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actor_by_tag_any",
            [this](std::string& class_name, std::vector<std::string>& tags) -> uint64_t {
                return toUInt64(UnrealClassRegistrar::findActorByTagAny(class_name, getWorld(), tags));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actor_by_tag_all",
            [this](std::string& class_name, std::vector<std::string>& tags) -> uint64_t {
                return toUInt64(UnrealClassRegistrar::findActorByTagAll(class_name, getWorld(), tags));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actor_by_type",
            [this](std::string& class_name) -> uint64_t {
                return toUInt64(UnrealClassRegistrar::findActorByType(class_name, getWorld()));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actor_by_class",
            [this](uint64_t& uclass) -> uint64_t {
                return toUInt64(Unreal::findActorByClass(getWorld(), toPtr<UClass>(uclass)));
            });

        //
        // Get components conditionally and return an std::vector
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_components_by_name",
            [this](std::string& class_name, uint64_t& actor, std::vector<std::string>& component_names, bool& include_from_child_actors, bool& return_null_if_not_found) -> std::vector<uint64_t> {
                return toUInt64(UnrealClassRegistrar::getComponentsByName(class_name, toPtr<AActor>(actor), component_names, include_from_child_actors, return_null_if_not_found));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_components_by_tag",
            [this](std::string& class_name, uint64_t& actor, std::string& tag, bool& include_from_child_actors) -> std::vector<uint64_t> {
                return toUInt64(UnrealClassRegistrar::getComponentsByTag(class_name, toPtr<AActor>(actor), tag, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_components_by_tag_any",
            [this](std::string& class_name, uint64_t& actor, std::vector<std::string>& tags, bool& include_from_child_actors) -> std::vector<uint64_t> {
                return toUInt64(UnrealClassRegistrar::getComponentsByTagAny(class_name, toPtr<AActor>(actor), tags, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_components_by_tag_all",
            [this](std::string& class_name, uint64_t& actor, std::vector<std::string>& tags, bool& include_from_child_actors) -> std::vector<uint64_t> {
                return toUInt64(UnrealClassRegistrar::getComponentsByTagAll(class_name, toPtr<AActor>(actor), tags, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_components_by_type",
            [this](std::string& class_name, uint64_t& actor, bool& include_from_child_actors) -> std::vector<uint64_t> {
                return toUInt64(UnrealClassRegistrar::getComponentsByType(class_name, toPtr<AActor>(actor), include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_components_by_class",
            [this](uint64_t& actor, uint64_t& uclass, bool& include_from_child_actors) -> std::vector<uint64_t> {
                return toUInt64(Unreal::getComponentsByClass(toPtr<AActor>(actor), toPtr<UClass>(uclass), include_from_child_actors));
            });

        //
        // Get components conditionally and return an std::map
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_components_by_name_as_map",
            [this](std::string& class_name, uint64_t& actor, std::vector<std::string>& component_names, bool& include_from_child_actors, bool& return_null_if_not_found) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealClassRegistrar::getComponentsByNameAsMap(class_name, toPtr<AActor>(actor), component_names, include_from_child_actors, return_null_if_not_found));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_components_by_tag_as_map",
            [this](std::string& class_name, uint64_t& actor, std::string& tag, bool& include_from_child_actors) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealClassRegistrar::getComponentsByTagAsMap(class_name, toPtr<AActor>(actor), tag, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_components_by_tag_any_as_map",
            [this](std::string& class_name, uint64_t& actor, std::vector<std::string>& tags, bool& include_from_child_actors) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealClassRegistrar::getComponentsByTagAnyAsMap(class_name, toPtr<AActor>(actor), tags, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_components_by_tag_all_as_map",
            [this](std::string& class_name, uint64_t& actor, std::vector<std::string>& tags, bool& include_from_child_actors) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealClassRegistrar::getComponentsByTagAllAsMap(class_name, toPtr<AActor>(actor), tags, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_components_by_type_as_map",
            [this](std::string& class_name, uint64_t& actor, bool& include_from_child_actors) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealClassRegistrar::getComponentsByTypeAsMap(class_name, toPtr<AActor>(actor), include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_components_by_class_as_map",
            [this](uint64_t& actor, uint64_t& uclass, bool& include_from_child_actors) -> std::map<std::string, uint64_t> {
                return toUInt64(Unreal::getComponentsByClassAsMap(toPtr<AActor>(actor), toPtr<UClass>(uclass), include_from_child_actors));
            });

        //
        // Get component conditionally
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_component_by_name",
            [this](std::string& class_name, uint64_t& actor, std::string& component_name, bool& include_from_child_actors) -> uint64_t {
                return toUInt64(UnrealClassRegistrar::getComponentByName(class_name, toPtr<AActor>(actor), component_name, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_component_by_tag",
            [this](std::string& class_name, uint64_t& actor, std::string& tag, bool& include_from_child_actors) -> uint64_t {
                return toUInt64(UnrealClassRegistrar::getComponentByTag(class_name, toPtr<AActor>(actor), tag, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_component_by_tag_any",
            [this](std::string& class_name, uint64_t& actor, std::vector<std::string>& tags, bool& include_from_child_actors) -> uint64_t {
                return toUInt64(UnrealClassRegistrar::getComponentByTagAny(class_name, toPtr<AActor>(actor), tags, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_component_by_tag_all",
            [this](std::string& class_name, uint64_t& actor, std::vector<std::string>& tags, bool& include_from_child_actors) -> uint64_t {
                return toUInt64(UnrealClassRegistrar::getComponentByTagAll(class_name, toPtr<AActor>(actor), tags, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_component_by_type",
            [this](std::string& class_name, uint64_t& actor, bool& include_from_child_actors) -> uint64_t {
                return toUInt64(UnrealClassRegistrar::getComponentByType(class_name, toPtr<AActor>(actor), include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_component_by_class",
            [this](uint64_t& actor, uint64_t& uclass, bool& include_from_child_actors) -> uint64_t {
                return toUInt64(Unreal::getComponentByClass(toPtr<AActor>(actor), toPtr<UClass>(uclass), include_from_child_actors));
            });

        //
        // Get children components conditionally from an actor and return an std::vector
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_by_name_from_actor",
            [this](std::string& class_name, uint64_t& parent, std::vector<std::string>& children_component_names, bool& include_all_descendants, bool& return_null_if_not_found) -> std::vector<uint64_t> {
                return toUInt64(UnrealClassRegistrar::getChildrenComponentsByName(class_name, toPtr<AActor>(parent), children_component_names, include_all_descendants, return_null_if_not_found));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_by_tag_from_actor",
            [this](std::string& class_name, uint64_t& parent, std::string& tag, bool& include_all_descendants) -> std::vector<uint64_t> {
                return toUInt64(UnrealClassRegistrar::getChildrenComponentsByTag(class_name, toPtr<AActor>(parent), tag, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_by_tag_any_from_actor",
            [this](std::string& class_name, uint64_t& parent, std::vector<std::string>& tags, bool& include_all_descendants) -> std::vector<uint64_t> {
                return toUInt64(UnrealClassRegistrar::getChildrenComponentsByTagAny(class_name, toPtr<AActor>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_by_tag_all_from_actor",
            [this](std::string& class_name, uint64_t& parent, std::vector<std::string>& tags, bool& include_all_descendants) -> std::vector<uint64_t> {
                return toUInt64(UnrealClassRegistrar::getChildrenComponentsByTagAll(class_name, toPtr<AActor>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_by_type_from_actor",
            [this](std::string& class_name, uint64_t& parent, bool& include_all_descendants) -> std::vector<uint64_t> {
                return toUInt64(UnrealClassRegistrar::getChildrenComponentsByType(class_name, toPtr<AActor>(parent), include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_by_class_from_actor",
            [this](uint64_t& parent, uint64_t& uclass, bool& include_all_descendants) -> std::vector<uint64_t> {
                return toUInt64(Unreal::getChildrenComponentsByClass(toPtr<AActor>(parent), toPtr<UClass>(uclass), include_all_descendants));
            });

        //
        // Get children components conditionally from an actor and return an std::map
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_by_name_as_map_from_actor",
            [this](std::string& class_name, uint64_t& parent, std::vector<std::string>& children_component_names, bool& include_all_descendants, bool& return_null_if_not_found) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealClassRegistrar::getChildrenComponentsByNameAsMap(class_name, toPtr<AActor>(parent), children_component_names, include_all_descendants, return_null_if_not_found));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_by_tag_as_map_from_actor",
            [this](std::string& class_name, uint64_t& parent, std::string& tag, bool& include_all_descendants) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealClassRegistrar::getChildrenComponentsByTagAsMap(class_name, toPtr<AActor>(parent), tag, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_by_tag_any_as_map_from_actor",
            [this](std::string& class_name, uint64_t& parent, std::vector<std::string>& tags, bool& include_all_descendants) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealClassRegistrar::getChildrenComponentsByTagAnyAsMap(class_name, toPtr<AActor>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_by_tag_all_as_map_from_actor",
            [this](std::string& class_name, uint64_t& parent, std::vector<std::string>& tags, bool& include_all_descendants) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealClassRegistrar::getChildrenComponentsByTagAllAsMap(class_name, toPtr<AActor>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_by_type_as_map_from_actor",
            [this](std::string& class_name, uint64_t& parent, bool& include_all_descendants) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealClassRegistrar::getChildrenComponentsByTypeAsMap(class_name, toPtr<AActor>(parent), include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_by_class_as_map_from_actor",
            [this](uint64_t& parent, uint64_t& uclass, bool& include_all_descendants) -> std::map<std::string, uint64_t> {
                return toUInt64(Unreal::getChildrenComponentsByClassAsMap(toPtr<AActor>(parent), toPtr<UClass>(uclass), include_all_descendants));
            });

        //
        // Get child component conditionally from an actor
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_child_component_by_name_from_actor",
            [this](std::string& class_name, uint64_t& parent, std::string& child_component_name, bool& include_all_descendants) -> uint64_t {
                return toUInt64(UnrealClassRegistrar::getChildComponentByName(class_name, toPtr<AActor>(parent), child_component_name, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_child_component_by_tag_from_actor",
            [this](std::string& class_name, uint64_t& parent, std::string& tag, bool& include_all_descendants) -> uint64_t {
                return toUInt64(UnrealClassRegistrar::getChildComponentByTag(class_name, toPtr<AActor>(parent), tag, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_child_component_by_tag_any_from_actor",
            [this](std::string& class_name, uint64_t& parent, std::vector<std::string>& tags, bool& include_all_descendants) -> uint64_t {
                return toUInt64(UnrealClassRegistrar::getChildComponentByTagAny(class_name, toPtr<AActor>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_child_component_by_tag_all_from_actor",
            [this](std::string& class_name, uint64_t& parent, std::vector<std::string>& tags, bool& include_all_descendants) -> uint64_t {
                return toUInt64(UnrealClassRegistrar::getChildComponentByTagAll(class_name, toPtr<AActor>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_child_component_by_type_from_actor",
            [this](std::string& class_name, uint64_t& parent, bool& include_all_descendants) -> uint64_t {
                return toUInt64(UnrealClassRegistrar::getChildComponentByType(class_name, toPtr<AActor>(parent), include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_child_component_by_class_from_actor",
            [this](uint64_t& parent, uint64_t& uclass, bool& include_all_descendants) -> uint64_t {
                return toUInt64(Unreal::getChildComponentByClass(toPtr<AActor>(parent), toPtr<UClass>(uclass), include_all_descendants));
            });

        //
        // Get children components conditionally from a scene component and return an std::vector
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_by_name_from_scene_component",
            [this](std::string& class_name, uint64_t& parent, std::vector<std::string>& children_component_names, bool& include_all_descendants, bool& return_null_if_not_found) -> std::vector<uint64_t> {
                return toUInt64(UnrealClassRegistrar::getChildrenComponentsByName(class_name, toPtr<USceneComponent>(parent), children_component_names, include_all_descendants, return_null_if_not_found));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_by_tag_from_scene_component",
            [this](std::string& class_name, uint64_t& parent, std::string& tag, bool& include_all_descendants) -> std::vector<uint64_t> {
                return toUInt64(UnrealClassRegistrar::getChildrenComponentsByTag(class_name, toPtr<USceneComponent>(parent), tag, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_by_tag_any_from_scene_component",
            [this](std::string& class_name, uint64_t& parent, std::vector<std::string>& tags, bool& include_all_descendants) -> std::vector<uint64_t> {
                return toUInt64(UnrealClassRegistrar::getChildrenComponentsByTagAny(class_name, toPtr<USceneComponent>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_by_tag_all_from_scene_component",
            [this](std::string& class_name, uint64_t& parent, std::vector<std::string>& tags, bool& include_all_descendants) -> std::vector<uint64_t> {
                return toUInt64(UnrealClassRegistrar::getChildrenComponentsByTagAll(class_name, toPtr<USceneComponent>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_by_type_from_scene_component",
            [this](std::string& class_name, uint64_t& parent, bool& include_all_descendants) -> std::vector<uint64_t> {
                return toUInt64(UnrealClassRegistrar::getChildrenComponentsByType(class_name, toPtr<USceneComponent>(parent), include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_by_class_from_scene_component",
            [this](uint64_t& parent, uint64_t& uclass, bool& include_all_descendants) -> std::vector<uint64_t> {
                return toUInt64(Unreal::getChildrenComponentsByClass(toPtr<USceneComponent>(parent), toPtr<UClass>(uclass), include_all_descendants));
            });

        //
        // Get children components conditionally from a scene component and return an std::map
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_by_name_as_map_from_scene_component",
            [this](std::string& class_name, uint64_t& parent, std::vector<std::string>& children_component_names, bool& include_all_descendants, bool& return_null_if_not_found) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealClassRegistrar::getChildrenComponentsByNameAsMap(class_name, toPtr<USceneComponent>(parent), children_component_names, include_all_descendants, return_null_if_not_found));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_by_tag_as_map_from_scene_component",
            [this](std::string& class_name, uint64_t& parent, std::string& tag, bool& include_all_descendants) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealClassRegistrar::getChildrenComponentsByTagAsMap(class_name, toPtr<USceneComponent>(parent), tag, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_by_tag_any_as_map_from_scene_component",
            [this](std::string& class_name, uint64_t& parent, std::vector<std::string>& tags, bool& include_all_descendants) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealClassRegistrar::getChildrenComponentsByTagAnyAsMap(class_name, toPtr<USceneComponent>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_by_tag_all_as_map_from_scene_component",
            [this](std::string& class_name, uint64_t& parent, std::vector<std::string>& tags, bool& include_all_descendants) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealClassRegistrar::getChildrenComponentsByTagAllAsMap(class_name, toPtr<USceneComponent>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_by_type_as_map_from_scene_component",
            [this](std::string& class_name, uint64_t& parent, bool& include_all_descendants) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealClassRegistrar::getChildrenComponentsByTypeAsMap(class_name, toPtr<USceneComponent>(parent), include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_by_class_as_map_from_scene_component",
            [this](uint64_t& parent, uint64_t& uclass, bool& include_all_descendants) -> std::map<std::string, uint64_t> {
                return toUInt64(Unreal::getChildrenComponentsByClassAsMap(toPtr<USceneComponent>(parent), toPtr<UClass>(uclass), include_all_descendants));
            });

        //
        // Get child component conditionally from a scene component
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_child_component_by_name_from_scene_component",
            [this](std::string& class_name, uint64_t& parent, std::string& child_component_name, bool& include_all_descendants) -> uint64_t {
                return toUInt64(UnrealClassRegistrar::getChildComponentByName(class_name, toPtr<USceneComponent>(parent), child_component_name, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_child_component_by_tag_from_scene_component",
            [this](std::string& class_name, uint64_t& parent, std::string& tag, bool& include_all_descendants) -> uint64_t {
                return toUInt64(UnrealClassRegistrar::getChildComponentByTag(class_name, toPtr<USceneComponent>(parent), tag, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_child_component_by_tag_any_from_scene_component",
            [this](std::string& class_name, uint64_t& parent, std::vector<std::string>& tags, bool& include_all_descendants) -> uint64_t {
                return toUInt64(UnrealClassRegistrar::getChildComponentByTagAny(class_name, toPtr<USceneComponent>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_child_component_by_tag_all_from_scene_component",
            [this](std::string& class_name, uint64_t& parent, std::vector<std::string>& tags, bool& include_all_descendants) -> uint64_t {
                return toUInt64(UnrealClassRegistrar::getChildComponentByTagAll(class_name, toPtr<USceneComponent>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_child_component_by_type_from_scene_component",
            [this](std::string& class_name, uint64_t& parent, bool& include_all_descendants) -> uint64_t {
                return toUInt64(UnrealClassRegistrar::getChildComponentByType(class_name, toPtr<USceneComponent>(parent), include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_child_component_by_class_from_scene_component",
            [this](uint64_t& parent, uint64_t& uclass, bool& include_all_descendants) -> uint64_t {
                return toUInt64(Unreal::getChildComponentByClass(toPtr<USceneComponent>(parent), toPtr<UClass>(uclass), include_all_descendants));
            });

        //
        // Spawn actor
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "spawn_actor",
            [this](std::string& class_name, std::string& location_string, std::string& rotation_string, std::string& spawn_parameters_string, std::vector<std::string>& object_flag_strings) -> uint64_t {

                FVector location;
                FRotator rotation;
                FSpActorSpawnParameters sp_actor_spawn_parameters;

                Unreal::setObjectPropertiesFromString(&location, UnrealClassRegistrar::getStaticStruct<FVector>(), location_string);
                Unreal::setObjectPropertiesFromString(&rotation, UnrealClassRegistrar::getStaticStruct<FRotator>(), rotation_string);
                Unreal::setObjectPropertiesFromString(&sp_actor_spawn_parameters, FSpActorSpawnParameters::StaticStruct(), spawn_parameters_string);

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

                return toUInt64(UnrealClassRegistrar::spawnActor(class_name, getWorld(), location, rotation, actor_spawn_parameters));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "spawn_actor_from_uclass",
            [this](uint64_t& uclass, std::string& location_string, std::string& rotation_string, std::string& spawn_parameters_string, std::vector<std::string>& object_flag_strings) -> uint64_t {

                FVector location;
                FRotator rotation;
                FSpActorSpawnParameters sp_actor_spawn_parameters;

                Unreal::setObjectPropertiesFromString(&location, UnrealClassRegistrar::getStaticStruct<FVector>(), location_string);
                Unreal::setObjectPropertiesFromString(&rotation, UnrealClassRegistrar::getStaticStruct<FRotator>(), rotation_string);
                Unreal::setObjectPropertiesFromString(&sp_actor_spawn_parameters, FSpActorSpawnParameters::StaticStruct(), spawn_parameters_string);

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

                SP_ASSERT(getWorld());
                return toUInt64(getWorld()->SpawnActor(toPtr<UClass>(uclass), &location, &rotation, actor_spawn_parameters));
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
        // Create component
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "create_component_outside_owner_constructor", 
            [this](std::string& class_name, uint64_t& owner, std::string& component_name) -> uint64_t {
                return toUInt64(UnrealClassRegistrar::createComponentOutsideOwnerConstructor(class_name, toPtr<AActor>(owner), component_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "create_component_outside_owner_constructor_by_class",
            [this](uint64_t& owner, uint64_t& component_class, std::string& component_name) -> uint64_t {
                AActor* actor          = toPtr<AActor>(owner);
                USceneComponent* scene_component = NewObject<USceneComponent>(actor, toPtr<UClass>(component_class), Unreal::toFName(component_name));
                scene_component->RegisterComponent();
                return toUInt64(scene_component);
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "create_scene_component_outside_owner_constructor_from_actor",
            [this](std::string& class_name, uint64_t& actor, std::string& scene_component_name) -> uint64_t {
                return toUInt64(UnrealClassRegistrar::createSceneComponentOutsideOwnerConstructor(class_name, toPtr<AActor>(actor), scene_component_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "create_scene_component_outside_owner_constructor_from_object",
            [this](std::string& class_name, uint64_t& owner, uint64_t& parent, std::string& scene_component_name) -> uint64_t {
                return toUInt64(UnrealClassRegistrar::createSceneComponentOutsideOwnerConstructor(class_name, toPtr<UObject>(owner), toPtr<USceneComponent>(parent), scene_component_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "create_scene_component_outside_owner_constructor_from_component",
            [this](std::string& class_name, uint64_t& owner, std::string& scene_component_name) -> uint64_t {
                return toUInt64(UnrealClassRegistrar::createSceneComponentOutsideOwnerConstructor(class_name, toPtr<USceneComponent>(owner), scene_component_name));
            });

        //
        // Destroy component
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "destroy_component",
            [this](uint64_t& component, bool& promote_children) -> void {
                SP_ASSERT(component);
                toPtr<UActorComponent>(component)->DestroyComponent(promote_children);
            });

        //
        // Create new object
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "new_object",
            [this](
                std::string& class_name,
                uint64_t& outer,
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
                    UnrealClassRegistrar::newObject(
                        class_name,
                        outer_ptr,
                        fname,
                        Unreal::getCombinedEnumFlagValueFromStringsAs<EObjectFlags, ESpObjectFlags>(object_flag_strings),
                        toPtr<UObject>(uobject_template),
                        copy_transients_from_class_defaults,
                        toPtr<FObjectInstancingGraph>(in_instance_graph),
                        toPtr<UPackage>(external_package)));
            });

        //
        // Load objects and classes
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "load_object",
            [this](
                std::string& class_name,
                uint64_t& outer,
                std::string& name,
                std::string& filename,
                std::vector<std::string>& load_flag_strings,
                uint64_t& sandbox,
                uint64_t& instancing_context) -> uint64_t {

                return toUInt64(
                    UnrealClassRegistrar::loadObject(
                        class_name,
                        toPtr<UObject>(outer),
                        *Unreal::toFString(name),
                        *Unreal::toFString(filename),
                        Unreal::getCombinedEnumFlagValueFromStringsAs<ELoadFlags, ESpLoadFlags>(load_flag_strings),
                        toPtr<UPackageMap>(sandbox),
                        toPtr<FLinkerInstancingContext>(instancing_context)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "load_class",
            [this](
                std::string& class_name,
                uint64_t& outer,
                std::string& name,
                std::string& filename,
                std::vector<std::string>& load_flag_strings,
                uint64_t& sandbox) -> uint64_t {

                return toUInt64(
                    UnrealClassRegistrar::loadClass(
                        class_name,
                        toPtr<UObject>(outer),
                        *Unreal::toFString(name),
                        *Unreal::toFString(filename),
                        Unreal::getCombinedEnumFlagValueFromStringsAs<ELoadFlags, ESpLoadFlags>(load_flag_strings),
                        toPtr<UPackageMap>(sandbox)));
            });

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
                        toPtr<UClass>(in_outer),
                        *Unreal::toFString(name),
                        *Unreal::toFString(filename),
                        Unreal::getCombinedEnumFlagValueFromStringsAs<ELoadFlags, ESpLoadFlags>(load_flag_strings),
                        toPtr<UPackageMap>(sandbox),
                        allow_object_reconciliation,
                        toPtr<FLinkerInstancingContext>(instancing_context)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "static_load_class",
            [this](
                uint64_t& base_uclass,
                uint64_t& in_outer,
                std::string& name,
                std::string& filename,
                std::vector<std::string>& load_flag_strings,
                uint64_t& sandbox) -> uint64_t {

                return toUInt64(
                    StaticLoadClass(
                        toPtr<UClass>(base_uclass),
                        toPtr<UClass>(in_outer),
                        *Unreal::toFString(name),
                        *Unreal::toFString(filename),
                        Unreal::getCombinedEnumFlagValueFromStringsAs<ELoadFlags, ESpLoadFlags>(load_flag_strings),
                        toPtr<UPackageMap>(sandbox)));
            });

        //
        // Load objects and classes
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "add_uobject_to_root",
            [this](uint64_t& uobject) -> void {
                UObject* uboject_ptr = toPtr<UObject>(uobject);
                SP_ASSERT(uboject_ptr);
                uboject_ptr->AddToRoot();
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "remove_uobject_from_root",
            [this](uint64_t& uobject) -> void {
                UObject* uboject_ptr = toPtr<UObject>(uobject);
                SP_ASSERT(uboject_ptr);
                uboject_ptr->RemoveFromRoot();
            });

        //
        // Find, get, and set console variables
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_console_variable_by_name",
            [this](std::string& cvar_name) -> uint64_t {
                return toUInt64(IConsoleManager::Get().FindConsoleVariable(*Unreal::toFString(cvar_name)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_console_variable_value_as_bool",
            [this](uint64_t& cvar) -> bool {
                SP_ASSERT(cvar);
                return toPtr<IConsoleVariable>(cvar)->GetBool();
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_console_variable_value_as_int",
            [this](uint64_t& cvar) -> int32_t {
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
            [this](uint64_t& cvar, int& val, std::vector<std::string>& set_by_strings) -> void {
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
                toPtr<IConsoleVariable>(cvar)->Set(*Unreal::toFString(val), Unreal::getCombinedEnumFlagValueFromStringsAs<EConsoleVariableFlags, ESpConsoleVariableFlags>(set_by_strings));
            });

        //
        // Execute console commands
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "execute_console_command",
            [this](std::string& command) -> void {
                SP_ASSERT(GEngine);
                GEngine->Exec(getWorld(), *Unreal::toFString(command));
            });

        //
        // Stable name helper functions
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "has_stable_name",
            [this](uint64_t& actor) -> bool { return Unreal::hasStableName(toPtr<AActor>(actor)); });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_stable_name_for_actor",
            [this](uint64_t& actor) -> std::string { return Unreal::getStableName(toPtr<AActor>(actor)); });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_stable_name_for_component",
            [this](uint64_t& actor_component, bool& include_actor_name) -> std::string { return Unreal::getStableName(toPtr<UActorComponent>(actor_component), include_actor_name); });

        //
        // Get actor and component tags
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_actor_tags",
            [this](uint64_t& actor) -> std::vector<std::string> { return Unreal::getTags(toPtr<AActor>(actor)); });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_component_tags",
            [this](uint64_t& component) -> std::vector<std::string> { return Unreal::getTags(toPtr<UActorComponent>(component)); });
    }

    ~UnrealService() override = default;
};

//
// Unreal::PropertyDesc
//

template <> // needed to receive a custom type as an arg
struct clmdep_msgpack::adaptor::convert<Unreal::PropertyDesc> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& object, Unreal::PropertyDesc& property_desc) const {
        std::map<std::string, clmdep_msgpack::object> map = Msgpack::toMap(object);
        SP_ASSERT(map.size() == 2);
        property_desc.property_ = Msgpack::toPtr<FProperty>(map.at("property"));
        property_desc.value_ptr_ = Msgpack::toPtr<void>(map.at("value_ptr"));
        return object;
    }
};

template <> // needed to send a custom type as a return value
struct clmdep_msgpack::adaptor::object_with_zone<Unreal::PropertyDesc> {
    void operator()(clmdep_msgpack::object::with_zone& object, Unreal::PropertyDesc const& property_desc) const {
        Msgpack::toObject(object, {
            {"property", Msgpack::toObject(property_desc.property_, object.zone)},
            {"value_ptr", Msgpack::toObject(property_desc.value_ptr_, object.zone)}});
    }
};
