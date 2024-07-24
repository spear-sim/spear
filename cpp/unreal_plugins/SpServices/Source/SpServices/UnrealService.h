//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <map>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <Components/ChildActorComponent.h>
#include <Delegates/IDelegateInstance.h> // FDelegateHandle
#include <Engine/Level.h>                // ULevel
#include <Engine/World.h>                // FWorldDelegates, FActorSpawnParameters
#include <Kismet/GameplayStatics.h>
#include "UObject/Class.h"               // EIncludeSuperFlag::Type
#include "UObject/ObjectMacros.h"        // EObjectFlags, ELoadFlags
#include <UObject/Package.h>

#include "SpCore/Assert.h"
#include "SpCore/Log.h"
#include "SpCore/Rpclib.h"
#include "SpCore/Unreal.h"
#include "SpCore/UnrealClassRegistrar.h"
#include "SpCore/UnrealObj.h"

#include "SpServices/EntryPointBinder.h"

#include "UnrealService.generated.h"

// This corresponds to EIncludeSuperFlag::Type declared in Engine/Source/Runtime/CoreUObject/Public/UObject/Class.h
UENUM()
enum class ESpIncludeSuperFlag
{
    ExcludeSuper = static_cast<std::underlying_type_t<EIncludeSuperFlag::Type>>(EIncludeSuperFlag::Type::ExcludeSuper),
    IncludeSuper = static_cast<std::underlying_type_t<EIncludeSuperFlag::Type>>(EIncludeSuperFlag::Type::IncludeSuper)
};

// This corresponds to ESpawnActorNameMode declared in Engine/Source/Runtime/Engine/Classes/Engine/World.h
UENUM()
enum class ESpSpawnActorNameMode : uint8
{
    Required_Fatal              = static_cast<std::underlying_type_t<FActorSpawnParameters::ESpawnActorNameMode>>(FActorSpawnParameters::ESpawnActorNameMode::Required_Fatal),
    Required_ErrorAndReturnNull = static_cast<std::underlying_type_t<FActorSpawnParameters::ESpawnActorNameMode>>(FActorSpawnParameters::ESpawnActorNameMode::Required_ErrorAndReturnNull),
    Required_ReturnNull         = static_cast<std::underlying_type_t<FActorSpawnParameters::ESpawnActorNameMode>>(FActorSpawnParameters::ESpawnActorNameMode::Required_ReturnNull),
    Requested                   = static_cast<std::underlying_type_t<FActorSpawnParameters::ESpawnActorNameMode>>(FActorSpawnParameters::ESpawnActorNameMode::Requested)
};

// This corresponds to EObjectFlags declared in Engine/Source/Runtime/CoreUObject/Public/UObject/ObjectMacros.h
UENUM()
enum class ESpObjectFlags
{
    RF_NoFlags                      = static_cast<std::underlying_type_t<EObjectFlags>>(EObjectFlags::RF_NoFlags),
    RF_Public                       = static_cast<std::underlying_type_t<EObjectFlags>>(EObjectFlags::RF_Public),
    RF_Standalone                   = static_cast<std::underlying_type_t<EObjectFlags>>(EObjectFlags::RF_Standalone),
    RF_MarkAsNative                 = static_cast<std::underlying_type_t<EObjectFlags>>(EObjectFlags::RF_MarkAsNative),
    RF_Transactional                = static_cast<std::underlying_type_t<EObjectFlags>>(EObjectFlags::RF_Transactional),
    RF_ClassDefaultObject           = static_cast<std::underlying_type_t<EObjectFlags>>(EObjectFlags::RF_ClassDefaultObject),
    RF_ArchetypeObject              = static_cast<std::underlying_type_t<EObjectFlags>>(EObjectFlags::RF_ArchetypeObject),
    RF_Transient                    = static_cast<std::underlying_type_t<EObjectFlags>>(EObjectFlags::RF_Transient),
    RF_MarkAsRootSet                = static_cast<std::underlying_type_t<EObjectFlags>>(EObjectFlags::RF_MarkAsRootSet),
    RF_TagGarbageTemp               = static_cast<std::underlying_type_t<EObjectFlags>>(EObjectFlags::RF_TagGarbageTemp),
    RF_NeedInitialization           = static_cast<std::underlying_type_t<EObjectFlags>>(EObjectFlags::RF_NeedInitialization),
    RF_NeedLoad                     = static_cast<std::underlying_type_t<EObjectFlags>>(EObjectFlags::RF_NeedLoad),
    RF_KeepForCooker                = static_cast<std::underlying_type_t<EObjectFlags>>(EObjectFlags::RF_KeepForCooker),
    RF_NeedPostLoad                 = static_cast<std::underlying_type_t<EObjectFlags>>(EObjectFlags::RF_NeedPostLoad),
    RF_NeedPostLoadSubobjects       = static_cast<std::underlying_type_t<EObjectFlags>>(EObjectFlags::RF_NeedPostLoadSubobjects),
    RF_NewerVersionExists           = static_cast<std::underlying_type_t<EObjectFlags>>(EObjectFlags::RF_NewerVersionExists),
    RF_BeginDestroyed               = static_cast<std::underlying_type_t<EObjectFlags>>(EObjectFlags::RF_BeginDestroyed),
    RF_FinishDestroyed              = static_cast<std::underlying_type_t<EObjectFlags>>(EObjectFlags::RF_FinishDestroyed),
    RF_BeingRegenerated             = static_cast<std::underlying_type_t<EObjectFlags>>(EObjectFlags::RF_BeingRegenerated),
    RF_DefaultSubObject             = static_cast<std::underlying_type_t<EObjectFlags>>(EObjectFlags::RF_DefaultSubObject),
    RF_WasLoaded                    = static_cast<std::underlying_type_t<EObjectFlags>>(EObjectFlags::RF_WasLoaded),
    RF_TextExportTransient          = static_cast<std::underlying_type_t<EObjectFlags>>(EObjectFlags::RF_TextExportTransient),
    RF_LoadCompleted                = static_cast<std::underlying_type_t<EObjectFlags>>(EObjectFlags::RF_LoadCompleted),
    RF_InheritableComponentTemplate = static_cast<std::underlying_type_t<EObjectFlags>>(EObjectFlags::RF_InheritableComponentTemplate),
    RF_DuplicateTransient           = static_cast<std::underlying_type_t<EObjectFlags>>(EObjectFlags::RF_DuplicateTransient),
    RF_StrongRefOnFrame             = static_cast<std::underlying_type_t<EObjectFlags>>(EObjectFlags::RF_StrongRefOnFrame),
    RF_NonPIEDuplicateTransient     = static_cast<std::underlying_type_t<EObjectFlags>>(EObjectFlags::RF_NonPIEDuplicateTransient),
    RF_WillBeLoaded                 = static_cast<std::underlying_type_t<EObjectFlags>>(EObjectFlags::RF_WillBeLoaded),
    RF_HasExternalPackage           = static_cast<std::underlying_type_t<EObjectFlags>>(EObjectFlags::RF_HasExternalPackage),
    RF_AllocatedInSharedPage        = static_cast<std::underlying_type_t<EObjectFlags>>(EObjectFlags::RF_AllocatedInSharedPage),
};

// This corresponds to ELoadFlags declared in Engine/Source/Runtime/CoreUObject/Public/UObject/ObjectMacros.h
UENUM()
enum class ESpLoadFlags
{
    LOAD_None                        = static_cast<std::underlying_type_t<ELoadFlags>>(ELoadFlags::LOAD_None),
    LOAD_Async                       = static_cast<std::underlying_type_t<ELoadFlags>>(ELoadFlags::LOAD_Async),
    LOAD_NoWarn                      = static_cast<std::underlying_type_t<ELoadFlags>>(ELoadFlags::LOAD_NoWarn),
    LOAD_EditorOnly                  = static_cast<std::underlying_type_t<ELoadFlags>>(ELoadFlags::LOAD_EditorOnly),
    LOAD_ResolvingDeferredExports    = static_cast<std::underlying_type_t<ELoadFlags>>(ELoadFlags::LOAD_ResolvingDeferredExports),
    LOAD_Verify                      = static_cast<std::underlying_type_t<ELoadFlags>>(ELoadFlags::LOAD_Verify),
    LOAD_NoVerify                    = static_cast<std::underlying_type_t<ELoadFlags>>(ELoadFlags::LOAD_NoVerify),
    LOAD_IsVerifying                 = static_cast<std::underlying_type_t<ELoadFlags>>(ELoadFlags::LOAD_IsVerifying),
    LOAD_SkipLoadImportedPackages    = static_cast<std::underlying_type_t<ELoadFlags>>(ELoadFlags::LOAD_SkipLoadImportedPackages),
    LOAD_RegenerateBulkDataGuids     = static_cast<std::underlying_type_t<ELoadFlags>>(ELoadFlags::LOAD_RegenerateBulkDataGuids),
    LOAD_DisableDependencyPreloading = static_cast<std::underlying_type_t<ELoadFlags>>(ELoadFlags::LOAD_DisableDependencyPreloading),
    LOAD_Quiet                       = static_cast<std::underlying_type_t<ELoadFlags>>(ELoadFlags::LOAD_Quiet),
    LOAD_FindIfFail                  = static_cast<std::underlying_type_t<ELoadFlags>>(ELoadFlags::LOAD_FindIfFail),
    LOAD_MemoryReader                = static_cast<std::underlying_type_t<ELoadFlags>>(ELoadFlags::LOAD_MemoryReader),
    LOAD_NoRedirects                 = static_cast<std::underlying_type_t<ELoadFlags>>(ELoadFlags::LOAD_NoRedirects),
    LOAD_ForDiff                     = static_cast<std::underlying_type_t<ELoadFlags>>(ELoadFlags::LOAD_ForDiff),
    LOAD_PackageForPIE               = static_cast<std::underlying_type_t<ELoadFlags>>(ELoadFlags::LOAD_PackageForPIE),
    LOAD_DeferDependencyLoads        = static_cast<std::underlying_type_t<ELoadFlags>>(ELoadFlags::LOAD_DeferDependencyLoads),
    LOAD_ForFileDiff                 = static_cast<std::underlying_type_t<ELoadFlags>>(ELoadFlags::LOAD_ForFileDiff),
    LOAD_DisableCompileOnLoad        = static_cast<std::underlying_type_t<ELoadFlags>>(ELoadFlags::LOAD_DisableCompileOnLoad),
    LOAD_DisableEngineVersionChecks  = static_cast<std::underlying_type_t<ELoadFlags>>(ELoadFlags::LOAD_DisableEngineVersionChecks)
};

// These USTRUCT types are intended to be wrappers for the UENUM types declared above. Wrapping enums in
// structs like this helps us take advantage of UnrealObj and UnrealObjUtils to pass enums to and from
// Python as a JSON string.
USTRUCT()
struct FSpIncludeSuperFlag
{
    GENERATED_BODY()

    UPROPERTY()
    ESpIncludeSuperFlag Enum;

    using TEnumType = ESpIncludeSuperFlag;
    static std::string getEnumName() { return "Enum"; }
    TEnumType getEnumValue() { return Enum; }
};

USTRUCT()
struct FSpObjectFlags
{
    GENERATED_BODY()

    UPROPERTY()
    ESpObjectFlags Enum;

    using TEnumType = ESpObjectFlags;
    static std::string getEnumName() { return "Enum"; }
    TEnumType getEnumValue() { return Enum; }
};

USTRUCT()
struct FSpLoadFlags
{
    GENERATED_BODY()

    UPROPERTY()
    ESpLoadFlags Enum;

    using TEnumType = ESpLoadFlags;
    static std::string getEnumName() { return "Enum"; }
    TEnumType getEnumValue() { return Enum; }
};

// This struct is intended to be identical to Unreal's FActorSpawnParameters struct, see Engine/Source/Runtime/Engine/Classes/Engine/World.h
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

    UPROPERTY()
    ESpObjectFlags ObjectFlags = ESpObjectFlags::RF_Transactional;
};

struct UnrealServicePropertyDesc
{
    uint64_t property_;
    uint64_t value_ptr_;

    MSGPACK_DEFINE_MAP(property_, value_ptr_);
};


class UnrealService {
public:
    UnrealService() = delete;
    UnrealService(CUnrealEntryPointBinder auto* unreal_entry_point_binder)
    {
        SP_ASSERT(unreal_entry_point_binder);

        post_world_initialization_handle_ = FWorldDelegates::OnPostWorldInitialization.AddRaw(this, &UnrealService::postWorldInitializationHandler);
        world_cleanup_handle_ = FWorldDelegates::OnWorldCleanup.AddRaw(this, &UnrealService::worldCleanupHandler);

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_world_name",
            [this]() -> std::string {
                SP_ASSERT(world_);
                return Unreal::toStdString(world_->GetName());
            });

        //
        // Get UClass from class name, get default object from UClass, get UClass from object
        //

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_static_class",
            [this](const std::string& class_name) -> uint64_t {
                return reinterpret_cast<uint64_t>(UnrealClassRegistrar::getStaticClass(class_name));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_default_object",
            [this](const uint64_t& uclass, const bool& create_if_needed) -> uint64_t {
                return reinterpret_cast<uint64_t>(reinterpret_cast<UClass*>(uclass)->GetDefaultObject(create_if_needed));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_class",
            [this](const uint64_t& uobject) -> uint64_t {
                return reinterpret_cast<uint64_t>(reinterpret_cast<UObject*>(uobject)->GetClass());
            });

        //
        // Get static struct
        //

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_static_struct",
            [this](const std::string& struct_name) -> uint64_t {
                return reinterpret_cast<uint64_t>(UnrealClassRegistrar::getStaticStruct(struct_name));
            });

        //
        // Get and set object properties
        //

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_object_properties_as_string_from_uobject",
            [this](const uint64_t& uobject) -> std::string {
                return Unreal::getObjectPropertiesAsString(reinterpret_cast<UObject*>(uobject));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_object_properties_as_string_from_ustruct",
            [this](const uint64_t& value_ptr, const uint64_t& ustruct) -> std::string {
                return Unreal::getObjectPropertiesAsString(reinterpret_cast<void*>(value_ptr), reinterpret_cast<UStruct*>(ustruct));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "set_object_properties_from_string_for_uobject",
            [this](const uint64_t& uobject, const std::string& string) -> void {
                Unreal::setObjectPropertiesFromString(reinterpret_cast<UObject*>(uobject), string);
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "set_object_properties_from_string_for_ustruct",
            [this](const uint64_t& value_ptr, const uint64_t& ustruct, const std::string& string) -> void {
                Unreal::setObjectPropertiesFromString(reinterpret_cast<void*>(value_ptr), reinterpret_cast<UStruct*>(ustruct), string);
            });

        //
        // Find properties
        //

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "find_property_by_name_on_uobject",
            [this](const uint64_t& uobject, const std::string& name) -> UnrealServicePropertyDesc {
                return toServicePropertyDesc(Unreal::findPropertyByName(reinterpret_cast<UObject*>(uobject), name));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "find_property_by_name_on_ustruct",
            [this](const uint64_t& value_ptr, const uint64_t& ustruct, const std::string& name) -> UnrealServicePropertyDesc {
                return toServicePropertyDesc(Unreal::findPropertyByName(reinterpret_cast<void*>(value_ptr), reinterpret_cast<UStruct*>(ustruct), name));
            });

        //
        // Get property values
        //

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_property_value_as_string",
            [this](const UnrealServicePropertyDesc& property_desc) -> std::string {
                return Unreal::getPropertyValueAsString(toPropertyDesc(property_desc));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "set_property_value_from_string",
            [this](const UnrealServicePropertyDesc& property_desc, const std::string& string) -> void {
                Unreal::setPropertyValueFromString(toPropertyDesc(property_desc), string);
            });

        //
        // Find and call functions
        //

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "find_function_by_name",
            [this](const uint64_t& uclass, const std::string& name, const std::map<std::string, std::string>& unreal_obj_strings) -> uint64_t {

                UnrealObj<FSpIncludeSuperFlag> sp_include_super_flag_obj("IncludeSuperFlag");
                UnrealObjUtils::setObjectPropertiesFromStrings({&sp_include_super_flag_obj}, unreal_obj_strings);

                FSpIncludeSuperFlag sp_include_super_flag = sp_include_super_flag_obj.getObj();
                EIncludeSuperFlag::Type include_super_flag = static_cast<EIncludeSuperFlag::Type>(sp_include_super_flag.Enum);

                return reinterpret_cast<uint64_t>(Unreal::findFunctionByName(reinterpret_cast<UClass*>(uclass), name, include_super_flag));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "call_function",
            [this](
                const uint64_t& uobject,
                const uint64_t& ufunction,
                const std::map<std::string, std::string>& args,
                const std::string& world_context) -> std::map<std::string, std::string> {
                return Unreal::callFunction(world_, reinterpret_cast<UObject*>(uobject), reinterpret_cast<UFunction*>(ufunction), args, world_context);
            });

        //
        // Find actors unconditionally and return an std::vector or std::map
        //

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "find_actors",
            [this]() -> std::vector<uint64_t> {
                return toUint64(Unreal::findActors(world_));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "find_actors_as_map",
            [this]() -> std::map<std::string, uint64_t> {
                return toUint64(Unreal::findActorsAsMap(world_));
            });

        //
        // Get components unconditionally and return an std::vector or std::map
        //

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_components",
            [this](const uint64_t& actor) -> std::vector<uint64_t> {
                return toUint64(Unreal::getComponents(reinterpret_cast<AActor*>(actor)));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_components_as_map",
            [this](const uint64_t& actor) -> std::map<std::string, uint64_t> {
                return toUint64(Unreal::getComponentsAsMap(reinterpret_cast<AActor*>(actor)));
            });

        //
        // Get children components unconditionally and return an std::vector or std::map
        //

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_children_components",
            [this](const uint64_t& parent, const bool& include_all_descendants) -> std::vector<uint64_t> {
                return toUint64(Unreal::getChildrenComponents(reinterpret_cast<USceneComponent*>(parent), include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_children_components_as_map",
            [this](const uint64_t& parent, const bool& include_all_descendants) -> std::map<std::string, uint64_t> {
                return toUint64(Unreal::getChildrenComponentsAsMap(reinterpret_cast<USceneComponent*>(parent), include_all_descendants));
            });

        //
        // Find actors conditionally and return an std::vector
        //

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "find_actors_by_name",
            [this](const std::string& class_name, const std::vector<std::string>& names,const bool& return_null_if_not_found) -> std::vector<uint64_t> {
                return Std::reinterpretAsVectorOf<uint64_t>(UnrealClassRegistrar::findActorsByName(class_name, world_, names, return_null_if_not_found));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "find_actors_by_tag",
            [this](const std::string& class_name, const std::string& tag) -> std::vector<uint64_t> {
                return Std::reinterpretAsVectorOf<uint64_t>(UnrealClassRegistrar::findActorsByTag(class_name, world_, tag));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "find_actors_by_tag_any",
            [this](const std::string& class_name, const std::vector<std::string>& tags) -> std::vector<uint64_t> {
                return Std::reinterpretAsVectorOf<uint64_t>(UnrealClassRegistrar::findActorsByTagAny(class_name, world_, tags));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "find_actors_by_tag_all",
            [this](const std::string& class_name, const std::vector<std::string>& tags) -> std::vector<uint64_t> {
                return Std::reinterpretAsVectorOf<uint64_t>(UnrealClassRegistrar::findActorsByTagAll(class_name, world_, tags));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "find_actors_by_type",
            [this](const std::string& class_name) -> std::vector<uint64_t> {
                return Std::reinterpretAsVectorOf<uint64_t>(UnrealClassRegistrar::findActorsByType(class_name, world_));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "find_actors_by_class",
            [this](const uint64_t& uclass) -> std::vector<uint64_t> {
                return Std::reinterpretAsVectorOf<uint64_t>(Unreal::findActorsByClass(world_, reinterpret_cast<UClass*>(uclass)));
            });

        //
        // Find actors conditionally and return an std::map
        //

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "find_actors_by_name_as_map",
            [this](const std::string& class_name, const std::vector<std::string>& names, const bool& return_null_if_not_found) -> std::map<std::string, uint64_t> {
                return toUint64(UnrealClassRegistrar::findActorsByNameAsMap(class_name, world_, names, return_null_if_not_found));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "find_actors_by_tag_as_map",
            [this](const std::string& class_name, const std::string& tag) -> std::map<std::string, uint64_t> {
                return toUint64(UnrealClassRegistrar::findActorsByTagAsMap(class_name, world_, tag));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "find_actors_by_tag_any_as_map",
            [this](const std::string& class_name, const std::vector<std::string>& tags) -> std::map<std::string, uint64_t> {
                return toUint64(UnrealClassRegistrar::findActorsByTagAnyAsMap(class_name, world_, tags));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "find_actors_by_tag_all_as_map",
            [this](const std::string& class_name, const std::vector<std::string>& tags) -> std::map<std::string, uint64_t> {
                return toUint64(UnrealClassRegistrar::findActorsByTagAllAsMap(class_name, world_, tags));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "find_actors_by_type_as_map",
            [this](const std::string& class_name) -> std::map<std::string, uint64_t> {
                return toUint64(UnrealClassRegistrar::findActorsByTypeAsMap(class_name, world_));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "find_actors_by_class_as_map",
            [this](const uint64_t& uclass) -> std::map<std::string, uint64_t> {
                return toUint64(Unreal::findActorsByClassAsMap(world_, reinterpret_cast<UClass*>(uclass)));
            });

        //
        // Find actor conditionally
        //

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "find_actor_by_name",
            [this](const std::string& class_name, const std::string& name, const bool& assert_if_not_found) -> uint64_t {
                return reinterpret_cast<uint64_t>(UnrealClassRegistrar::findActorByName(class_name, world_, name,assert_if_not_found));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "find_actor_by_tag",
            [this](const std::string& class_name, const std::string& tag, const bool& assert_if_not_found, const bool& assert_if_multiple_found) -> uint64_t {
                return reinterpret_cast<uint64_t>(UnrealClassRegistrar::findActorByTag(class_name, world_, tag, assert_if_not_found, assert_if_multiple_found));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "find_actor_by_tag_any",
            [this](const std::string& class_name, const std::vector<std::string>& tags, const bool& assert_if_not_found, const bool& assert_if_multiple_found) -> uint64_t {
                return reinterpret_cast<uint64_t>(UnrealClassRegistrar::findActorByTagAny(class_name, world_, tags, assert_if_not_found, assert_if_multiple_found));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "find_actor_by_tag_all",
            [this](const std::string& class_name, const std::vector<std::string>& tags, const bool& assert_if_not_found, const bool& assert_if_multiple_found) -> uint64_t {
                return reinterpret_cast<uint64_t>(UnrealClassRegistrar::findActorByTagAll(class_name, world_, tags, assert_if_not_found, assert_if_multiple_found));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "find_actor_by_type",
            [this](const std::string& class_name, const bool& assert_if_not_found, const bool& assert_if_multiple_found) -> uint64_t {
                return reinterpret_cast<uint64_t>(UnrealClassRegistrar::findActorByType(class_name, world_, assert_if_not_found, assert_if_multiple_found));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "find_actor_by_class",
            [this](const uint64_t& uclass, const bool& assert_if_not_found, const bool& assert_if_multiple_found) -> uint64_t {
                return reinterpret_cast<uint64_t>(Unreal::findActorByClass(world_, reinterpret_cast<UClass*>(uclass), assert_if_not_found, assert_if_multiple_found));
            });

        //
        // Get components conditionally and return an std::vector
        //

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_components_by_name",
            [this](
                const std::string& class_name,
                const uint64_t& actor,
                const std::vector<std::string>& names,
                const bool& include_from_child_actors,
                const bool& return_null_if_not_found) -> std::vector<uint64_t> {
                return Std::reinterpretAsVectorOf<uint64_t>(UnrealClassRegistrar::getComponentsByName(class_name, reinterpret_cast<AActor*>(actor), names, include_from_child_actors, return_null_if_not_found));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_components_by_tag",
            [this](const std::string& class_name, const uint64_t& actor, const std::string& tag, const bool& include_from_child_actors) -> std::vector<uint64_t> {
                return Std::reinterpretAsVectorOf<uint64_t>(UnrealClassRegistrar::getComponentsByTag(class_name, reinterpret_cast<AActor*>(actor), tag, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_components_by_tag_any",
            [this](const std::string& class_name, const uint64_t& actor, const std::vector<std::string>& tags, const bool& include_from_child_actors) -> std::vector<uint64_t> {
                return Std::reinterpretAsVectorOf<uint64_t>(UnrealClassRegistrar::getComponentsByTagAny(class_name, reinterpret_cast<AActor*>(actor), tags, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_components_by_tag_all",
            [this](const std::string& class_name, const uint64_t& actor, const std::vector<std::string>& tags, const bool& include_from_child_actors) -> std::vector<uint64_t> {
                return Std::reinterpretAsVectorOf<uint64_t>(UnrealClassRegistrar::getComponentsByTagAll(class_name, reinterpret_cast<AActor*>(actor), tags, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_components_by_type",
            [this](const std::string& class_name, const uint64_t& actor, const bool& include_from_child_actors) -> std::vector<uint64_t> {
                return Std::reinterpretAsVectorOf<uint64_t>(UnrealClassRegistrar::getComponentsByType(class_name, reinterpret_cast<AActor*>(actor), include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_components_by_class",
            [this](const uint64_t& actor, const uint64_t& uclass, const bool& include_from_child_actors) -> std::vector<uint64_t> {
                return Std::reinterpretAsVectorOf<uint64_t>(Unreal::getComponentsByClass(reinterpret_cast<AActor*>(actor), reinterpret_cast<UClass*>(uclass), include_from_child_actors));
            });

        //
        // Get components conditionally and return an std::map
        //

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_components_by_name_as_map",
            [this](
                const std::string& class_name,
                const uint64_t& actor,
                const std::vector<std::string>& names,
                const bool& include_from_child_actors,
                const bool& return_null_if_not_found) -> std::map<std::string, uint64_t> {
                return toUint64(UnrealClassRegistrar::getComponentsByNameAsMap(class_name, reinterpret_cast<AActor*>(actor), names, include_from_child_actors, return_null_if_not_found));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_components_by_tag_as_map",
            [this](const std::string& class_name, const uint64_t& actor, const std::string& tag, const bool& include_from_child_actors) -> std::map<std::string, uint64_t> {
                return toUint64(UnrealClassRegistrar::getComponentsByTagAsMap(class_name, reinterpret_cast<AActor*>(actor), tag, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_components_by_tag_any_as_map",
            [this](const std::string& class_name, const uint64_t& actor, const std::vector<std::string>& tags, const bool& include_from_child_actors) -> std::map<std::string, uint64_t> {
                return toUint64(UnrealClassRegistrar::getComponentsByTagAnyAsMap(class_name, reinterpret_cast<AActor*>(actor), tags, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_components_by_tag_all_as_map",
            [this](const std::string& class_name, const uint64_t& actor, const std::vector<std::string>& tags, const bool& include_from_child_actors) -> std::map<std::string, uint64_t> {
                return toUint64(UnrealClassRegistrar::getComponentsByTagAllAsMap(class_name, reinterpret_cast<AActor*>(actor), tags, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_components_by_type_as_map",
            [this](const std::string& class_name, const uint64_t& actor, const bool& include_from_child_actors) -> std::map<std::string, uint64_t> {
                return toUint64(UnrealClassRegistrar::getComponentsByTypeAsMap(class_name, reinterpret_cast<AActor*>(actor), include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_components_by_class_as_map",
            [this](const uint64_t& actor, const uint64_t& uclass, const bool& include_from_child_actors) -> std::map<std::string, uint64_t> {
                return toUint64(Unreal::getComponentsByClassAsMap(reinterpret_cast<AActor*>(actor), reinterpret_cast<UClass*>(uclass), include_from_child_actors));
            });

        //
        // Get component conditionally
        //

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_component_by_name",
            [this](const std::string& class_name, const uint64_t& actor, const std::string& name, const bool& include_from_child_actors, const bool& assert_if_not_found) -> uint64_t {
                return reinterpret_cast<uint64_t>(UnrealClassRegistrar::getComponentByName(class_name, reinterpret_cast<AActor*>(actor), name, include_from_child_actors, assert_if_not_found));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_component_by_tag",
            [this](
                const std::string& class_name,
                const uint64_t& actor,
                const std::string& tag,
                const bool& include_from_child_actors,
                const bool& assert_if_not_found,
                const bool& assert_if_multiple_found) -> uint64_t {
                return reinterpret_cast<uint64_t>(
                    UnrealClassRegistrar::getComponentByTag(class_name, reinterpret_cast<AActor*>(actor), tag, include_from_child_actors, assert_if_not_found, assert_if_multiple_found));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_component_by_tag_any",
            [this](
                const std::string& class_name,
                const uint64_t& actor,
                const std::vector<std::string>& tags,
                const bool& include_from_child_actors,
                const bool& assert_if_not_found,
                const bool& assert_if_multiple_found) -> uint64_t {
                return reinterpret_cast<uint64_t>(
                    UnrealClassRegistrar::getComponentByTagAny(class_name, reinterpret_cast<AActor*>(actor), tags, include_from_child_actors, assert_if_not_found, assert_if_multiple_found));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_component_by_tag_all",
            [this](
                const std::string& class_name,
                const uint64_t& actor,
                const std::vector<std::string>& tags,
                const bool& include_from_child_actors,
                const bool& assert_if_not_found,
                const bool& assert_if_multiple_found) -> uint64_t {
                return reinterpret_cast<uint64_t>(
                    UnrealClassRegistrar::getComponentByTagAll(class_name, reinterpret_cast<AActor*>(actor), tags, include_from_child_actors, assert_if_not_found, assert_if_multiple_found));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_component_by_type",
            [this](
                const std::string& class_name,
                const uint64_t& actor,
                const bool& include_from_child_actors,
                const bool& assert_if_not_found,
                const bool& assert_if_multiple_found) -> uint64_t {
                return reinterpret_cast<uint64_t>(
                    UnrealClassRegistrar::getComponentByType(class_name, reinterpret_cast<AActor*>(actor), include_from_child_actors, assert_if_not_found, assert_if_multiple_found));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_component_by_class",
            [this](const uint64_t& actor, const uint64_t& uclass, const bool& assert_if_not_found, const bool& include_from_child_actors, const bool& assert_if_multiple_found) -> uint64_t {
                return reinterpret_cast<uint64_t>(
                    Unreal::getComponentByClass(reinterpret_cast<AActor*>(actor), reinterpret_cast<UClass*>(uclass), include_from_child_actors, assert_if_not_found, assert_if_multiple_found));
            });

        //
        // Get children components conditionally from an actor and return an std::vector
        //

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_children_components_by_name_from_actor",
            [this](
                const std::string& class_name,
                const uint64_t& parent,
                const std::vector<std::string>& names,
                const bool& include_all_descendants,
                const bool& return_null_if_not_found) -> std::vector<uint64_t> {
                return Std::reinterpretAsVectorOf<uint64_t>(
                    UnrealClassRegistrar::getChildrenComponentsByName(class_name, reinterpret_cast<AActor*>(parent), names, include_all_descendants, return_null_if_not_found));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_children_components_by_tag_from_actor",
            [this](const std::string& class_name, const uint64_t& parent, const std::string& tag, const bool& include_all_descendants) -> std::vector<uint64_t> {
                return Std::reinterpretAsVectorOf<uint64_t>(UnrealClassRegistrar::getChildrenComponentsByTag(class_name, reinterpret_cast<AActor*>(parent), tag, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_children_components_by_tag_any_from_actor",
            [this](const std::string& class_name, const uint64_t& parent, const std::vector<std::string>& tags, const bool& include_all_descendants) -> std::vector<uint64_t> {
                return Std::reinterpretAsVectorOf<uint64_t>(UnrealClassRegistrar::getChildrenComponentsByTagAny(class_name, reinterpret_cast<AActor*>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_children_components_by_tag_all_from_actor",
            [this](const std::string& class_name, const uint64_t& parent, const std::vector<std::string>& tags, const bool& include_all_descendants) -> std::vector<uint64_t> {
                return Std::reinterpretAsVectorOf<uint64_t>(UnrealClassRegistrar::getChildrenComponentsByTagAll(class_name, reinterpret_cast<AActor*>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_children_components_by_type_from_actor",
            [this](const std::string& class_name, const uint64_t& parent, const bool& include_all_descendants) -> std::vector<uint64_t> {
                return Std::reinterpretAsVectorOf<uint64_t>(UnrealClassRegistrar::getChildrenComponentsByType(class_name, reinterpret_cast<AActor*>(parent), include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_children_components_by_class_from_actor",
            [this](const uint64_t& parent, const uint64_t& uclass, const bool& include_all_descendants) -> std::vector<uint64_t> {
                return Std::reinterpretAsVectorOf<uint64_t>(Unreal::getChildrenComponentsByClass(reinterpret_cast<AActor*>(parent), reinterpret_cast<UClass*>(uclass), include_all_descendants));
            });

        //
        // Get children components conditionally from an actor and return an std::map
        //

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_children_components_by_name_as_map_from_actor",
            [this](
                const std::string& class_name,
                const uint64_t& parent,
                const std::vector<std::string>& names,
                const bool& include_all_descendants,
                const bool& return_null_if_not_found) -> std::map<std::string, uint64_t> {
                return toUint64(
                    UnrealClassRegistrar::getChildrenComponentsByNameAsMap(class_name, reinterpret_cast<AActor*>(parent), names, include_all_descendants, return_null_if_not_found));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_children_components_by_tag_as_map_from_actor",
            [this](const std::string& class_name, const uint64_t& parent, const std::string& tag, const bool& include_all_descendants) -> std::map<std::string, uint64_t> {
                return toUint64(UnrealClassRegistrar::getChildrenComponentsByTagAsMap(class_name, reinterpret_cast<AActor*>(parent), tag, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_children_components_by_tag_any_as_map_from_actor",
            [this](const std::string& class_name, const uint64_t& parent, const std::vector<std::string>& tags, const bool& include_all_descendants) -> std::map<std::string, uint64_t> {
                return toUint64(UnrealClassRegistrar::getChildrenComponentsByTagAnyAsMap(class_name, reinterpret_cast<AActor*>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_children_components_by_tag_all_as_map_from_actor",
            [this](const std::string& class_name, const uint64_t& parent, const std::vector<std::string>& tags, const bool& include_all_descendants) -> std::map<std::string, uint64_t> {
                return toUint64(UnrealClassRegistrar::getChildrenComponentsByTagAllAsMap(class_name, reinterpret_cast<AActor*>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_children_components_by_type_as_map_from_actor",
            [this](const std::string& class_name, const uint64_t& parent, const bool& include_all_descendants) -> std::map<std::string, uint64_t> {
                return toUint64(UnrealClassRegistrar::getChildrenComponentsByTypeAsMap(class_name, reinterpret_cast<AActor*>(parent), include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_children_components_by_class_as_map_from_actor",
            [this](const uint64_t& parent, const uint64_t& uclass, const bool& include_all_descendants) -> std::map<std::string, uint64_t> {
                return toUint64(Unreal::getChildrenComponentsByClassAsMap(reinterpret_cast<AActor*>(parent), reinterpret_cast<UClass*>(uclass), include_all_descendants));
            });

        //
        // Get child component conditionally from an actor
        //

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_child_component_by_name_from_actor",
            [this](const std::string& class_name, const uint64_t& parent, const std::string& name, const bool& include_all_descendants, const bool& assert_if_not_found) -> uint64_t {
                return reinterpret_cast<uint64_t>(
                    UnrealClassRegistrar::getChildComponentByName(class_name, reinterpret_cast<AActor*>(parent), name, include_all_descendants, assert_if_not_found));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_child_component_by_tag_from_actor",
            [this](
                const std::string& class_name,
                const uint64_t& parent,
                const std::string& tag,
                const bool& include_all_descendants,
                const bool& assert_if_not_found,
                const bool& assert_if_multiple_found) -> uint64_t {
                return reinterpret_cast<uint64_t>(
                    UnrealClassRegistrar::getChildComponentByTag(
                        class_name,
                        reinterpret_cast<AActor*>(parent),
                        tag,
                        include_all_descendants,
                        assert_if_not_found,
                        assert_if_multiple_found));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_child_component_by_tag_any_from_actor",
            [this](
                const std::string& class_name,
                const uint64_t& parent,
                const std::vector<std::string>& tags,
                const bool& include_all_descendants,
                const bool& assert_if_not_found,
                const bool& assert_if_multiple_found) -> uint64_t {
                return reinterpret_cast<uint64_t>(
                    UnrealClassRegistrar::getChildComponentByTagAny(
                        class_name,
                        reinterpret_cast<AActor*>(parent),
                        tags,
                        include_all_descendants,
                        assert_if_not_found,
                        assert_if_multiple_found));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_child_component_by_tag_all_from_actor",
            [this](
                const std::string& class_name,
                const uint64_t& parent,
                const std::vector<std::string>& tags,
                const bool& include_all_descendants,
                const bool& assert_if_not_found,
                const bool& assert_if_multiple_found) -> uint64_t {
                return reinterpret_cast<uint64_t>(
                    UnrealClassRegistrar::getChildComponentByTagAll(
                        class_name,
                        reinterpret_cast<AActor*>(parent),
                        tags,
                        include_all_descendants,
                        assert_if_not_found,
                        assert_if_multiple_found));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_child_component_by_type_from_actor",
            [this](
                const std::string& class_name,
                const uint64_t& parent,
                const bool& include_all_descendants,
                const bool& assert_if_not_found,
                const bool& assert_if_multiple_found) -> uint64_t {
                return reinterpret_cast<uint64_t>(
                    UnrealClassRegistrar::getChildComponentByType(class_name, reinterpret_cast<AActor*>(parent), include_all_descendants, assert_if_not_found, assert_if_multiple_found));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_child_component_by_class_from_actor",
            [this](
                const uint64_t& parent,
                const uint64_t& uclass,
                const bool& include_all_descendants,
                const bool& assert_if_not_found,
                const bool& assert_if_multiple_found) -> uint64_t {
                return reinterpret_cast<uint64_t>(
                    Unreal::getChildComponentByClass(reinterpret_cast<AActor*>(parent), reinterpret_cast<UClass*>(uclass), include_all_descendants, assert_if_not_found, assert_if_multiple_found));
            });

        //
        // Get children components conditionally from a scene component and return an std::vector
        //

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_children_components_by_name_from_scene_component",
            [this](
                const std::string& class_name,
                const uint64_t& parent,
                const std::vector<std::string>& names,
                const bool& include_all_descendants,
                const bool& return_null_if_not_found) -> std::vector<uint64_t> {
                return Std::reinterpretAsVectorOf<uint64_t>(
                    UnrealClassRegistrar::getChildrenComponentsByName(class_name, reinterpret_cast<USceneComponent*>(parent), names, include_all_descendants, return_null_if_not_found));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_children_components_by_tag_from_scene_component",
            [this](const std::string& class_name, const uint64_t& parent, const std::string& tag, const bool& include_all_descendants) -> std::vector<uint64_t> {
                return Std::reinterpretAsVectorOf<uint64_t>(UnrealClassRegistrar::getChildrenComponentsByTag(class_name, reinterpret_cast<USceneComponent*>(parent), tag, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_children_components_by_tag_any_from_scene_component",
            [this](const std::string& class_name, const uint64_t& parent, const std::vector<std::string>& tags, const bool& include_all_descendants) -> std::vector<uint64_t> {
                return Std::reinterpretAsVectorOf<uint64_t>(UnrealClassRegistrar::getChildrenComponentsByTagAny(class_name, reinterpret_cast<USceneComponent*>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_children_components_by_tag_all_from_scene_component",
            [this](const std::string& class_name, const uint64_t& parent, const std::vector<std::string>& tags, const bool& include_all_descendants) -> std::vector<uint64_t> {
                return Std::reinterpretAsVectorOf<uint64_t>(UnrealClassRegistrar::getChildrenComponentsByTagAll(class_name, reinterpret_cast<USceneComponent*>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_children_components_by_type_from_scene_component",
            [this](const std::string& class_name, const uint64_t& parent, const bool& include_all_descendants) -> std::vector<uint64_t> {
                return Std::reinterpretAsVectorOf<uint64_t>(UnrealClassRegistrar::getChildrenComponentsByType(class_name, reinterpret_cast<USceneComponent*>(parent), include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_children_components_by_class_from_scene_component",
            [this](const uint64_t& parent, const uint64_t& uclass, const bool& include_all_descendants) -> std::vector<uint64_t> {
                return Std::reinterpretAsVectorOf<uint64_t>(Unreal::getChildrenComponentsByClass(reinterpret_cast<USceneComponent*>(parent), reinterpret_cast<UClass*>(uclass), include_all_descendants));
            });

        //
        // Get children components conditionally from a scene component and return an std::map
        //

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_children_components_by_name_as_map_from_scene_component",
            [this](
                const std::string& class_name,
                const uint64_t& parent,
                const std::vector<std::string>& names,
                const bool& include_all_descendants,
                const bool& return_null_if_not_found) -> std::map<std::string, uint64_t> {
                return toUint64(
                    UnrealClassRegistrar::getChildrenComponentsByNameAsMap(class_name, reinterpret_cast<USceneComponent*>(parent), names, include_all_descendants, return_null_if_not_found));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_children_components_by_tag_as_map_from_scene_component",
            [this](const std::string& class_name, const uint64_t& parent, const std::string& tag, const bool& include_all_descendants) -> std::map<std::string, uint64_t> {
                return toUint64(UnrealClassRegistrar::getChildrenComponentsByTagAsMap(class_name, reinterpret_cast<USceneComponent*>(parent), tag, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_children_components_by_tag_any_as_map_from_scene_component",
            [this](const std::string& class_name, const uint64_t& parent, const std::vector<std::string>& tags, const bool& include_all_descendants) -> std::map<std::string, uint64_t> {
                return toUint64(UnrealClassRegistrar::getChildrenComponentsByTagAnyAsMap(class_name, reinterpret_cast<USceneComponent*>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_children_components_by_tag_all_as_map_from_scene_component",
            [this](const std::string& class_name, const uint64_t& parent, const std::vector<std::string>& tags, const bool& include_all_descendants) -> std::map<std::string, uint64_t> {
                return toUint64(UnrealClassRegistrar::getChildrenComponentsByTagAllAsMap(class_name, reinterpret_cast<USceneComponent*>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_children_components_by_type_as_map_from_scene_component",
            [this](const std::string& class_name, const uint64_t& parent, const bool& include_all_descendants) -> std::map<std::string, uint64_t> {
                return toUint64(UnrealClassRegistrar::getChildrenComponentsByTypeAsMap(class_name, reinterpret_cast<USceneComponent*>(parent), include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_children_components_by_class_as_map_from_scene_component",
            [this](const uint64_t& parent, const uint64_t& uclass, const bool& include_all_descendants) -> std::map<std::string, uint64_t> {
                return toUint64(Unreal::getChildrenComponentsByClassAsMap(reinterpret_cast<USceneComponent*>(parent), reinterpret_cast<UClass*>(uclass), include_all_descendants));
            });

        //
        // Get child component conditionally from a scene component
        //

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_child_component_by_name_from_scene_component",
            [this](const std::string& class_name, const uint64_t& parent, const std::string& name, const bool& include_all_descendants, const bool& assert_if_not_found) -> uint64_t {
                return reinterpret_cast<uint64_t>(
                    UnrealClassRegistrar::getChildComponentByName(class_name, reinterpret_cast<USceneComponent*>(parent), name, include_all_descendants, assert_if_not_found));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_child_component_by_tag_from_scene_component",
            [this](
                const std::string& class_name,
                const uint64_t& parent,
                const std::string& tag,
                const bool& include_all_descendants,
                const bool& assert_if_not_found,
                const bool& assert_if_multiple_found) -> uint64_t {
                return reinterpret_cast<uint64_t>(
                    UnrealClassRegistrar::getChildComponentByTag(
                        class_name,
                        reinterpret_cast<USceneComponent*>(parent),
                        tag,
                        include_all_descendants,
                        assert_if_not_found,
                        assert_if_multiple_found));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_child_component_by_tag_any_from_scene_component",
            [this](
                const std::string& class_name,
                const uint64_t& parent,
                const std::vector<std::string>& tags,
                const bool& include_all_descendants,
                const bool& assert_if_not_found,
                const bool& assert_if_multiple_found) -> uint64_t {
                return reinterpret_cast<uint64_t>(
                    UnrealClassRegistrar::getChildComponentByTagAny(
                        class_name,
                        reinterpret_cast<USceneComponent*>(parent),
                        tags,
                        include_all_descendants,
                        assert_if_not_found,
                        assert_if_multiple_found));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_child_component_by_tag_all_from_scene_component",
            [this](
                const std::string& class_name,
                const uint64_t& parent,
                const std::vector<std::string>& tags,
                const bool& include_all_descendants,
                const bool& assert_if_not_found,
                const bool& assert_if_multiple_found) -> uint64_t {
                return reinterpret_cast<uint64_t>(
                    UnrealClassRegistrar::getChildComponentByTagAll(
                        class_name,
                        reinterpret_cast<USceneComponent*>(parent),
                        tags,
                        include_all_descendants,
                        assert_if_not_found,
                        assert_if_multiple_found));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_child_component_by_type_from_scene_component",
            [this](const std::string& class_name, const uint64_t& parent, const bool& include_all_descendants, const bool& assert_if_not_found, const bool& assert_if_multiple_found) -> uint64_t {
                return reinterpret_cast<uint64_t>(
                    UnrealClassRegistrar::getChildComponentByType(class_name, reinterpret_cast<USceneComponent*>(parent), include_all_descendants, assert_if_not_found, assert_if_multiple_found));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_child_component_by_class_from_scene_component",
            [this](const uint64_t& parent, const uint64_t& uclass, const bool& include_all_descendants, const bool& assert_if_not_found, const bool& assert_if_multiple_found) -> uint64_t {
                return reinterpret_cast<uint64_t>(
                    Unreal::getChildComponentByClass(reinterpret_cast<USceneComponent*>(parent), reinterpret_cast<UClass*>(uclass), include_all_descendants, assert_if_not_found, assert_if_multiple_found));
            });

        //
        // Spawn actor
        //

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "spawn_actor",
            [this](const std::string& class_name, const std::map<std::string, std::string>& unreal_obj_strings) -> uint64_t {
                UnrealObj<FSpActorSpawnParameters> sp_actor_spawn_parameters_obj("SpawnParameters");
                UnrealObj<FVector> location_obj("Location");
                UnrealObj<FRotator> rotation_obj("Rotation");
                UnrealObjUtils::setObjectPropertiesFromStrings({&sp_actor_spawn_parameters_obj, &location_obj, &rotation_obj}, unreal_obj_strings);

                FVector location = location_obj.getObj();
                FRotator rotation = rotation_obj.getObj();

                FSpActorSpawnParameters sp_actor_spawn_parameters = sp_actor_spawn_parameters_obj.getObj();
                FActorSpawnParameters actor_spawn_parameters;
                actor_spawn_parameters.Name = sp_actor_spawn_parameters.Name;
                actor_spawn_parameters.Template = sp_actor_spawn_parameters.Template;
                actor_spawn_parameters.Owner = sp_actor_spawn_parameters.Owner;
                actor_spawn_parameters.Instigator = sp_actor_spawn_parameters.Instigator;
                actor_spawn_parameters.OverrideLevel = sp_actor_spawn_parameters.OverrideLevel;
                actor_spawn_parameters.OverrideParentComponent = sp_actor_spawn_parameters.OverrideParentComponent;
                actor_spawn_parameters.SpawnCollisionHandlingOverride = static_cast<ESpawnActorCollisionHandlingMethod>(sp_actor_spawn_parameters.SpawnCollisionHandlingOverride);
                actor_spawn_parameters.TransformScaleMethod = sp_actor_spawn_parameters.TransformScaleMethod;
                actor_spawn_parameters.bNoFail = sp_actor_spawn_parameters.bNoFail;
                actor_spawn_parameters.bDeferConstruction = sp_actor_spawn_parameters.bDeferConstruction;
                actor_spawn_parameters.bAllowDuringConstructionScript = sp_actor_spawn_parameters.bAllowDuringConstructionScript;
                actor_spawn_parameters.NameMode = static_cast<FActorSpawnParameters::ESpawnActorNameMode>(sp_actor_spawn_parameters.NameMode);
                actor_spawn_parameters.ObjectFlags = static_cast<EObjectFlags>(sp_actor_spawn_parameters.ObjectFlags);

                return reinterpret_cast<uint64_t>(UnrealClassRegistrar::spawnActor(class_name, world_, location, rotation, actor_spawn_parameters));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "spawn_actor_from_uclass",
            [this](const uint64_t& uclass, const std::map<std::string, std::string>& unreal_obj_strings) -> uint64_t {
                UnrealObj<FSpActorSpawnParameters> sp_actor_spawn_parameters_obj("SpawnParameters");
                UnrealObj<FVector> location_obj("Location");
                UnrealObj<FRotator> rotation_obj("Rotation");
                UnrealObjUtils::setObjectPropertiesFromStrings({&sp_actor_spawn_parameters_obj, &location_obj, &rotation_obj}, unreal_obj_strings);

                FVector location = location_obj.getObj();
                FRotator rotation = rotation_obj.getObj();

                FSpActorSpawnParameters sp_actor_spawn_parameters = sp_actor_spawn_parameters_obj.getObj();
                FActorSpawnParameters actor_spawn_parameters;
                actor_spawn_parameters.Name = sp_actor_spawn_parameters.Name;
                actor_spawn_parameters.Template = sp_actor_spawn_parameters.Template;
                actor_spawn_parameters.Owner = sp_actor_spawn_parameters.Owner;
                actor_spawn_parameters.Instigator = sp_actor_spawn_parameters.Instigator;
                actor_spawn_parameters.OverrideLevel = sp_actor_spawn_parameters.OverrideLevel;
                actor_spawn_parameters.OverrideParentComponent = sp_actor_spawn_parameters.OverrideParentComponent;
                actor_spawn_parameters.SpawnCollisionHandlingOverride = static_cast<ESpawnActorCollisionHandlingMethod>(sp_actor_spawn_parameters.SpawnCollisionHandlingOverride);
                actor_spawn_parameters.TransformScaleMethod = sp_actor_spawn_parameters.TransformScaleMethod;
                actor_spawn_parameters.bNoFail = sp_actor_spawn_parameters.bNoFail;
                actor_spawn_parameters.bDeferConstruction = sp_actor_spawn_parameters.bDeferConstruction;
                actor_spawn_parameters.bAllowDuringConstructionScript = sp_actor_spawn_parameters.bAllowDuringConstructionScript;
                actor_spawn_parameters.NameMode = static_cast<FActorSpawnParameters::ESpawnActorNameMode>(sp_actor_spawn_parameters.NameMode);
                actor_spawn_parameters.ObjectFlags = static_cast<EObjectFlags>(sp_actor_spawn_parameters.ObjectFlags);

                return reinterpret_cast<uint64_t>(world_->SpawnActor(reinterpret_cast<UClass*>(uclass), &location, &rotation, actor_spawn_parameters));
            });

        //
        // Destroy actor
        //

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "destroy_actor",
            [this](const uint64_t& actor, const bool& net_force, const bool& should_modify_level) -> bool {
                AActor* actor_ptr = reinterpret_cast<AActor*>(actor);
                SP_ASSERT(actor_ptr);
                return actor_ptr->Destroy(net_force, should_modify_level);
            });

        //
        // Create component
        //

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "create_component_outside_owner_constructor", 
            [this](const std::string& class_name, const uint64_t& owner, const std::string& name) -> uint64_t {
                return reinterpret_cast<uint64_t>(UnrealClassRegistrar::createComponentOutsideOwnerConstructor(class_name, reinterpret_cast<AActor*>(owner), name));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "create_scene_component_outside_owner_constructor_from_actor",
            [this](const std::string& class_name, const uint64_t& actor, const std::string& name) -> uint64_t {
                return reinterpret_cast<uint64_t>(UnrealClassRegistrar::createSceneComponentOutsideOwnerConstructor(class_name, reinterpret_cast<AActor*>(actor), name));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "create_scene_component_outside_owner_constructor_from_object",
            [this](const std::string& class_name, const uint64_t& owner, const uint64_t& parent, const std::string& name) -> uint64_t {
                return reinterpret_cast<uint64_t>(
                    UnrealClassRegistrar::createSceneComponentOutsideOwnerConstructor(class_name, reinterpret_cast<UObject*>(owner), reinterpret_cast<USceneComponent*>(parent), name));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "create_scene_component_outside_owner_constructor_from_component",
            [this](const std::string& class_name, const uint64_t& owner, const std::string& name) -> uint64_t {
                return reinterpret_cast<uint64_t>(UnrealClassRegistrar::createSceneComponentOutsideOwnerConstructor(class_name, reinterpret_cast<USceneComponent*>(owner), name));
            });

        //
        // Destroy component
        //

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "destroy_component",
            [this](const uint64_t& component, const bool& promote_children) -> void {
                UActorComponent* actor_component = reinterpret_cast<UActorComponent*>(component);
                SP_ASSERT(actor_component);
                actor_component->DestroyComponent(promote_children);
            });

        //
        // Create new object
        //

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "new_object",
            [this](
                const std::string& class_name,
                const uint64_t& outer,
                const std::string& name,
                const uint64_t& uobject_template,
                const bool& copy_transients_from_class_defaults,
                const uint64_t& in_instance_graph,
                const uint64_t& external_package,
                const std::map<std::string, std::vector<std::string>>& unreal_flag_strings) -> uint64_t {

                FName fname = NAME_None;
                if (name != "") {
                    fname = Unreal::toFName(name);
                }

                EObjectFlags object_flags = static_cast<EObjectFlags>(Unreal::combineEnumFlagStrings<FSpObjectFlags>(unreal_flag_strings.at("ObjectFlags")));

                return reinterpret_cast<uint64_t>(
                    UnrealClassRegistrar::newObject(
                        class_name,
                        reinterpret_cast<UObject*>(outer),
                        fname,
                        object_flags,
                        reinterpret_cast<UObject*>(uobject_template),
                        copy_transients_from_class_defaults,
                        reinterpret_cast<FObjectInstancingGraph*>(in_instance_graph),
                        reinterpret_cast<UPackage*>(external_package)));
            });

        //
        // Load objects and classes
        //

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "load_object",
            [this](
                const std::string& class_name,
                const uint64_t& outer,
                const std::string& name,
                const std::string& filename,
                const uint64_t& sandbox,
                const uint64_t& instancing_context,
                const std::map<std::string, std::vector<std::string>>& unreal_flag_strings) -> uint64_t {

                ELoadFlags load_flags = static_cast<ELoadFlags>(Unreal::combineEnumFlagStrings<FSpLoadFlags>(unreal_flag_strings.at("LoadFlags")));

                return reinterpret_cast<uint64_t>(
                    UnrealClassRegistrar::loadObject(
                        class_name,
                        reinterpret_cast<UObject*>(outer),
                        *Unreal::toFString(name),
                        *Unreal::toFString(filename),
                        load_flags,
                        reinterpret_cast<UPackageMap*>(sandbox),
                        reinterpret_cast<FLinkerInstancingContext*>(instancing_context)));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "load_class",
            [this](
                const std::string& class_name,
                const uint64_t& outer,
                const std::string& name,
                const std::string& filename,
                const uint64_t& sandbox,
                const std::map<std::string, std::vector<std::string>>& unreal_flag_strings) -> uint64_t {

                ELoadFlags load_flags = static_cast<ELoadFlags>(Unreal::combineEnumFlagStrings<FSpLoadFlags>(unreal_flag_strings.at("LoadFlags")));

                return reinterpret_cast<uint64_t>(
                    UnrealClassRegistrar::loadClass(
                        class_name,
                        reinterpret_cast<UObject*>(outer),
                        *Unreal::toFString(name),
                        *Unreal::toFString(filename),
                        load_flags,
                        reinterpret_cast<UPackageMap*>(sandbox)));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "static_load_object",
            [this](
                const uint64_t& uclass,
                const uint64_t& in_outer,
                const std::string& name,
                const std::string& filename,
                const uint64_t& sandbox,
                const bool& allow_object_reconciliation,
                const uint64_t& instancing_context,
                const std::map<std::string, std::vector<std::string>>& unreal_flag_strings) -> uint64_t {

                ELoadFlags load_flags = static_cast<ELoadFlags>(Unreal::combineEnumFlagStrings<FSpLoadFlags>(unreal_flag_strings.at("LoadFlags")));

                return reinterpret_cast<uint64_t>(
                    StaticLoadObject(
                        reinterpret_cast<UClass*>(uclass),
                        reinterpret_cast<UClass*>(in_outer),
                        *Unreal::toFString(name),
                        *Unreal::toFString(filename),
                        load_flags,
                        reinterpret_cast<UPackageMap*>(sandbox),
                        allow_object_reconciliation,
                        reinterpret_cast<FLinkerInstancingContext*>(instancing_context)));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "static_load_class",
            [this](
                const uint64_t& base_uclass,
                const uint64_t& in_outer,
                const std::string& name,
                const std::string& filename,
                const uint64_t& sandbox,
                const std::map<std::string, std::vector<std::string>>& unreal_flag_strings) -> uint64_t {

                ELoadFlags load_flags = static_cast<ELoadFlags>(Unreal::combineEnumFlagStrings<FSpLoadFlags>(unreal_flag_strings.at("LoadFlags")));

                return reinterpret_cast<uint64_t>(
                    StaticLoadClass(
                        reinterpret_cast<UClass*>(base_uclass),
                        reinterpret_cast<UClass*>(in_outer),
                        *Unreal::toFString(name),
                        *Unreal::toFString(filename),
                        load_flags,
                        reinterpret_cast<UPackageMap*>(sandbox)));
            });

        //
        // Stable name helper functions
        //

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "has_stable_name",
            [this](const uint64_t& actor) -> bool {
                return Unreal::hasStableName(reinterpret_cast<AActor*>(actor));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_stable_name_for_actor",
            [this](const uint64_t& actor) -> std::string {
                return Unreal::getStableName(reinterpret_cast<AActor*>(actor));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_stable_name_for_component",
            [this](const uint64_t& actor_component, const bool& include_actor_name) -> std::string {
                return Unreal::getStableName(reinterpret_cast<UActorComponent*>(actor_component), include_actor_name);
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_stable_name_for_scene_component",
            [this](const uint64_t& scene_component, const bool& include_actor_name) -> std::string {
                return Unreal::getStableName(reinterpret_cast<USceneComponent*>(scene_component), include_actor_name);
            });

        //
        // Get actor and component tags
        //

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_actor_tags",
            [this](const uint64_t& actor) -> std::vector<std::string> {
                return Unreal::getTags(reinterpret_cast<AActor*>(actor));
            });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_component_tags",
            [this](const uint64_t& component) -> std::vector<std::string> {
                return Unreal::getTags(reinterpret_cast<UActorComponent*>(component));
            });
    }

    ~UnrealService()
    {
        FWorldDelegates::OnWorldCleanup.Remove(world_cleanup_handle_);
        FWorldDelegates::OnPostWorldInitialization.Remove(post_world_initialization_handle_);

        world_cleanup_handle_.Reset();
        post_world_initialization_handle_.Reset();
    }

    void postWorldInitializationHandler(UWorld* world, const UWorld::InitializationValues initialization_values);
    void worldCleanupHandler(UWorld* world, bool session_ended, bool cleanup_resources);

private:

    template <typename TKey, typename TSrcValue>
    static std::map<TKey, uint64_t> toUint64(std::map<TKey, TSrcValue>&& input_map)
    {
        return Std::toMap<TKey, uint64_t>(
            input_map | 
            std::views::transform([](auto& pair) { auto& [key, value] = pair; return std::make_pair(key, reinterpret_cast<uint64_t>(value)); }));
    }

    template <typename TValueType>
    static std::vector<uint64_t> toUint64(const std::vector<TValueType>& src)
    {
        return Std::reinterpretAsVectorOf<uint64_t>(src);
    }

    static UnrealServicePropertyDesc toServicePropertyDesc(const Unreal::PropertyDesc& property_desc);
    static Unreal::PropertyDesc toPropertyDesc(const UnrealServicePropertyDesc& service_property_desc);

    FDelegateHandle post_world_initialization_handle_;
    FDelegateHandle world_cleanup_handle_;

    UWorld* world_ = nullptr;
};
