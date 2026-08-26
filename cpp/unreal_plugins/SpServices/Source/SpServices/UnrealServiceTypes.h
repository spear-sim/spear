//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Engine/EngineTypes.h>   // ESpawnActorCollisionHandlingMethod
#include <Engine/World.h>         // FActorSpawnParameters
#include <GameFramework/Actor.h>  // ESpawnActorScaleMethod
#include <HAL/IConsoleManager.h>  // EConsoleVariableFlags
#include <HAL/Platform.h>         // uint64
#include <Misc/EnumClassFlags.h>  // ENUM_CLASS_FLAGS
#include <UObject/Class.h>        // EIncludeSuperFlag, EStructFlags
#include <UObject/ObjectMacros.h> // ELoadFlags, EObjectFlags, EPropertyFlags, GENERATED_BODY, UENUM, UPROPERTY, USTRUCT
#include <UObject/Script.h>       // EFunctionFlags
#include <UObject/UnrealType.h>   // EFieldIterationFlags

#include "SpCore/Unreal.h"

#include "UnrealServiceTypes.generated.h"

class APawn;
class UChildActorComponent;
class ULevel;

//
// Each USTRUCT below is intended to be a wrapper for a particular UENUM type. Wrapping enums in structs like
// this helps us take advantage of the Unreal property system to pass enums to and from Python as human-
// readable strings. Additionally, we make use of these enum structs to combine strings enum strings as
// though they were bit flags. Unfortunately, we can't abbreviate these declarations using higher-level
// macros any more than they already are, because then they wouldn't interact correctly with the Unreal
// build system.
//

//
// This enum corresponds to EStructFlags declared in Engine/Source/Runtime/CoreUObject/Public/UObject/Class.h
//

// UENUM(Flags) decorator is required to obtain an "A | B | C" string representation from a value, not intended to be Blueprint-accessible
UENUM(Flags)
enum class ESpStructFlags
{
    STRUCT_NoFlags                    = Unreal::getConstEnumValue(EStructFlags::STRUCT_NoFlags),
    STRUCT_Native                     = Unreal::getConstEnumValue(EStructFlags::STRUCT_Native),
    STRUCT_IdenticalNative            = Unreal::getConstEnumValue(EStructFlags::STRUCT_IdenticalNative),
    STRUCT_HasInstancedReference      = Unreal::getConstEnumValue(EStructFlags::STRUCT_HasInstancedReference),
    STRUCT_NoExport                   = Unreal::getConstEnumValue(EStructFlags::STRUCT_NoExport),
    STRUCT_Atomic                     = Unreal::getConstEnumValue(EStructFlags::STRUCT_Atomic),
    STRUCT_Immutable                  = Unreal::getConstEnumValue(EStructFlags::STRUCT_Immutable),
    STRUCT_AddStructReferencedObjects = Unreal::getConstEnumValue(EStructFlags::STRUCT_AddStructReferencedObjects),
    STRUCT_RequiredAPI                = Unreal::getConstEnumValue(EStructFlags::STRUCT_RequiredAPI),
    STRUCT_NetSerializeNative         = Unreal::getConstEnumValue(EStructFlags::STRUCT_NetSerializeNative),
    STRUCT_SerializeNative            = Unreal::getConstEnumValue(EStructFlags::STRUCT_SerializeNative),
    STRUCT_CopyNative                 = Unreal::getConstEnumValue(EStructFlags::STRUCT_CopyNative),
    STRUCT_IsPlainOldData             = Unreal::getConstEnumValue(EStructFlags::STRUCT_IsPlainOldData),
    STRUCT_NoDestructor               = Unreal::getConstEnumValue(EStructFlags::STRUCT_NoDestructor),
    STRUCT_ZeroConstructor            = Unreal::getConstEnumValue(EStructFlags::STRUCT_ZeroConstructor),
    STRUCT_ExportTextItemNative       = Unreal::getConstEnumValue(EStructFlags::STRUCT_ExportTextItemNative),
    STRUCT_ImportTextItemNative       = Unreal::getConstEnumValue(EStructFlags::STRUCT_ImportTextItemNative),
    STRUCT_PostSerializeNative        = Unreal::getConstEnumValue(EStructFlags::STRUCT_PostSerializeNative),
    STRUCT_SerializeFromMismatchedTag = Unreal::getConstEnumValue(EStructFlags::STRUCT_SerializeFromMismatchedTag),
    STRUCT_NetDeltaSerializeNative    = Unreal::getConstEnumValue(EStructFlags::STRUCT_NetDeltaSerializeNative),
    STRUCT_PostScriptConstruct        = Unreal::getConstEnumValue(EStructFlags::STRUCT_PostScriptConstruct),
    STRUCT_NetSharedSerialization     = Unreal::getConstEnumValue(EStructFlags::STRUCT_NetSharedSerialization),
    STRUCT_Trashed                    = Unreal::getConstEnumValue(EStructFlags::STRUCT_Trashed),
    STRUCT_NewerVersionExists         = Unreal::getConstEnumValue(EStructFlags::STRUCT_NewerVersionExists),
    STRUCT_CanEditChange              = Unreal::getConstEnumValue(EStructFlags::STRUCT_CanEditChange),
    STRUCT_Visitor                    = Unreal::getConstEnumValue(EStructFlags::STRUCT_Visitor),

    // These enum values are actually defined by macros so we need to handle them differently
    STRUCT_Inherit_                   = STRUCT_Inherit,
    STRUCT_ComputedFlags_             = STRUCT_ComputedFlags
};
ENUM_CLASS_FLAGS(ESpStructFlags); // required if combining values using bitwise operations

//
// This enum corresponds to EClassFlags declared in Engine/Source/Runtime/CoreUObject/Public/UObject/ObjectMacros.h
//

// UENUM(Flags) decorator is required to obtain an "A | B | C" string representation from a value, not intended to be Blueprint-accessible
UENUM(Flags)
enum class ESpClassFlags : uint64
{
    CLASS_None                           = Unreal::getConstEnumValue(EClassFlags::CLASS_None),
    CLASS_Abstract                       = Unreal::getConstEnumValue(EClassFlags::CLASS_Abstract),
    CLASS_DefaultConfig                  = Unreal::getConstEnumValue(EClassFlags::CLASS_DefaultConfig),
    CLASS_Config                         = Unreal::getConstEnumValue(EClassFlags::CLASS_Config),
    CLASS_Transient                      = Unreal::getConstEnumValue(EClassFlags::CLASS_Transient),
    CLASS_Optional                       = Unreal::getConstEnumValue(EClassFlags::CLASS_Optional),
    CLASS_MatchedSerializers             = Unreal::getConstEnumValue(EClassFlags::CLASS_MatchedSerializers),
    CLASS_ProjectUserConfig              = Unreal::getConstEnumValue(EClassFlags::CLASS_ProjectUserConfig),
    CLASS_Native                         = Unreal::getConstEnumValue(EClassFlags::CLASS_Native),
    // CLASS_NoExport                       = Unreal::getConstEnumValue(EClassFlags::CLASS_NoExport), deprecated in UE 5.5
    CLASS_NotPlaceable                   = Unreal::getConstEnumValue(EClassFlags::CLASS_NotPlaceable),
    CLASS_PerObjectConfig                = Unreal::getConstEnumValue(EClassFlags::CLASS_PerObjectConfig),
    CLASS_ReplicationDataIsSetUp         = Unreal::getConstEnumValue(EClassFlags::CLASS_ReplicationDataIsSetUp),
    CLASS_EditInlineNew                  = Unreal::getConstEnumValue(EClassFlags::CLASS_EditInlineNew),
    CLASS_CollapseCategories             = Unreal::getConstEnumValue(EClassFlags::CLASS_CollapseCategories),
    CLASS_Interface                      = Unreal::getConstEnumValue(EClassFlags::CLASS_Interface),
    CLASS_PerPlatformConfig              = Unreal::getConstEnumValue(EClassFlags::CLASS_PerPlatformConfig),
    CLASS_Const                          = Unreal::getConstEnumValue(EClassFlags::CLASS_Const),
    CLASS_NeedsDeferredDependencyLoading = Unreal::getConstEnumValue(EClassFlags::CLASS_NeedsDeferredDependencyLoading),
    CLASS_CompiledFromBlueprint          = Unreal::getConstEnumValue(EClassFlags::CLASS_CompiledFromBlueprint),
    CLASS_MinimalAPI                     = Unreal::getConstEnumValue(EClassFlags::CLASS_MinimalAPI),
    CLASS_RequiredAPI                    = Unreal::getConstEnumValue(EClassFlags::CLASS_RequiredAPI),
    CLASS_DefaultToInstanced             = Unreal::getConstEnumValue(EClassFlags::CLASS_DefaultToInstanced),
    CLASS_TokenStreamAssembled           = Unreal::getConstEnumValue(EClassFlags::CLASS_TokenStreamAssembled),
    CLASS_HasInstancedReference          = Unreal::getConstEnumValue(EClassFlags::CLASS_HasInstancedReference),
    CLASS_Hidden                         = Unreal::getConstEnumValue(EClassFlags::CLASS_Hidden),
    CLASS_Deprecated                     = Unreal::getConstEnumValue(EClassFlags::CLASS_Deprecated),
    CLASS_HideDropDown                   = Unreal::getConstEnumValue(EClassFlags::CLASS_HideDropDown),
    CLASS_GlobalUserConfig               = Unreal::getConstEnumValue(EClassFlags::CLASS_GlobalUserConfig),
    CLASS_Intrinsic                      = Unreal::getConstEnumValue(EClassFlags::CLASS_Intrinsic),
    CLASS_Constructed                    = Unreal::getConstEnumValue(EClassFlags::CLASS_Constructed),
    CLASS_ConfigDoNotCheckDefaults       = Unreal::getConstEnumValue(EClassFlags::CLASS_ConfigDoNotCheckDefaults),
    CLASS_NewerVersionExists             = Unreal::getConstEnumValue(EClassFlags::CLASS_NewerVersionExists)
};
ENUM_CLASS_FLAGS(ESpClassFlags); // required if combining values using bitwise operations

//
// This enum corresponds to EPropertyFlags declared in Engine/Source/Runtime/CoreUObject/Public/UObject/ObjectMacros.h
//

// UENUM(Flags) decorator is required to obtain an "A | B | C" string representation from a value, not intended to be Blueprint-accessible
UENUM(Flags)
enum class ESpPropertyFlags : uint64
{
    CPF_None                           = Unreal::getConstEnumValue(EPropertyFlags::CPF_None),
    CPF_Edit                           = Unreal::getConstEnumValue(EPropertyFlags::CPF_Edit),
    CPF_ConstParm                      = Unreal::getConstEnumValue(EPropertyFlags::CPF_ConstParm),
    CPF_BlueprintVisible               = Unreal::getConstEnumValue(EPropertyFlags::CPF_BlueprintVisible),
    CPF_ExportObject                   = Unreal::getConstEnumValue(EPropertyFlags::CPF_ExportObject),
    CPF_BlueprintReadOnly              = Unreal::getConstEnumValue(EPropertyFlags::CPF_BlueprintReadOnly),
    CPF_Net                            = Unreal::getConstEnumValue(EPropertyFlags::CPF_Net),
    CPF_EditFixedSize                  = Unreal::getConstEnumValue(EPropertyFlags::CPF_EditFixedSize),
    CPF_Parm                           = Unreal::getConstEnumValue(EPropertyFlags::CPF_Parm),
    CPF_OutParm                        = Unreal::getConstEnumValue(EPropertyFlags::CPF_OutParm),
    CPF_ZeroConstructor                = Unreal::getConstEnumValue(EPropertyFlags::CPF_ZeroConstructor),
    CPF_ReturnParm                     = Unreal::getConstEnumValue(EPropertyFlags::CPF_ReturnParm),
    CPF_DisableEditOnTemplate          = Unreal::getConstEnumValue(EPropertyFlags::CPF_DisableEditOnTemplate),
    CPF_NonNullable                    = Unreal::getConstEnumValue(EPropertyFlags::CPF_NonNullable),
    CPF_Transient                      = Unreal::getConstEnumValue(EPropertyFlags::CPF_Transient),
    CPF_Config                         = Unreal::getConstEnumValue(EPropertyFlags::CPF_Config),
    CPF_RequiredParm                   = Unreal::getConstEnumValue(EPropertyFlags::CPF_RequiredParm),
    CPF_DisableEditOnInstance          = Unreal::getConstEnumValue(EPropertyFlags::CPF_DisableEditOnInstance),
    CPF_EditConst                      = Unreal::getConstEnumValue(EPropertyFlags::CPF_EditConst),
    CPF_GlobalConfig                   = Unreal::getConstEnumValue(EPropertyFlags::CPF_GlobalConfig),
    CPF_InstancedReference             = Unreal::getConstEnumValue(EPropertyFlags::CPF_InstancedReference),
    CPF_DuplicateTransient             = Unreal::getConstEnumValue(EPropertyFlags::CPF_DuplicateTransient),
    CPF_SaveGame                       = Unreal::getConstEnumValue(EPropertyFlags::CPF_SaveGame),
    CPF_NoClear                        = Unreal::getConstEnumValue(EPropertyFlags::CPF_NoClear),
    CPF_ReferenceParm                  = Unreal::getConstEnumValue(EPropertyFlags::CPF_ReferenceParm),
    CPF_BlueprintAssignable            = Unreal::getConstEnumValue(EPropertyFlags::CPF_BlueprintAssignable),
    CPF_Deprecated                     = Unreal::getConstEnumValue(EPropertyFlags::CPF_Deprecated),
    CPF_IsPlainOldData                 = Unreal::getConstEnumValue(EPropertyFlags::CPF_IsPlainOldData),
    CPF_RepSkip                        = Unreal::getConstEnumValue(EPropertyFlags::CPF_RepSkip),
    CPF_RepNotify                      = Unreal::getConstEnumValue(EPropertyFlags::CPF_RepNotify),
    CPF_Interp                         = Unreal::getConstEnumValue(EPropertyFlags::CPF_Interp),
    CPF_NonTransactional               = Unreal::getConstEnumValue(EPropertyFlags::CPF_NonTransactional),
    CPF_EditorOnly                     = Unreal::getConstEnumValue(EPropertyFlags::CPF_EditorOnly),
    CPF_NoDestructor                   = Unreal::getConstEnumValue(EPropertyFlags::CPF_NoDestructor),
    CPF_AutoWeak                       = Unreal::getConstEnumValue(EPropertyFlags::CPF_AutoWeak),
    CPF_ContainsInstancedReference     = Unreal::getConstEnumValue(EPropertyFlags::CPF_ContainsInstancedReference),
    CPF_AssetRegistrySearchable        = Unreal::getConstEnumValue(EPropertyFlags::CPF_AssetRegistrySearchable),
    CPF_SimpleDisplay                  = Unreal::getConstEnumValue(EPropertyFlags::CPF_SimpleDisplay),
    CPF_AdvancedDisplay                = Unreal::getConstEnumValue(EPropertyFlags::CPF_AdvancedDisplay),
    CPF_Protected                      = Unreal::getConstEnumValue(EPropertyFlags::CPF_Protected),
    CPF_BlueprintCallable              = Unreal::getConstEnumValue(EPropertyFlags::CPF_BlueprintCallable),
    CPF_BlueprintAuthorityOnly         = Unreal::getConstEnumValue(EPropertyFlags::CPF_BlueprintAuthorityOnly),
    CPF_TextExportTransient            = Unreal::getConstEnumValue(EPropertyFlags::CPF_TextExportTransient),
    CPF_NonPIEDuplicateTransient       = Unreal::getConstEnumValue(EPropertyFlags::CPF_NonPIEDuplicateTransient),
    CPF_ExposeOnSpawn                  = Unreal::getConstEnumValue(EPropertyFlags::CPF_ExposeOnSpawn),
    CPF_PersistentInstance             = Unreal::getConstEnumValue(EPropertyFlags::CPF_PersistentInstance),
    CPF_UObjectWrapper                 = Unreal::getConstEnumValue(EPropertyFlags::CPF_UObjectWrapper),
    CPF_HasGetValueTypeHash            = Unreal::getConstEnumValue(EPropertyFlags::CPF_HasGetValueTypeHash),
    CPF_NativeAccessSpecifierPublic    = Unreal::getConstEnumValue(EPropertyFlags::CPF_NativeAccessSpecifierPublic),
    CPF_NativeAccessSpecifierProtected = Unreal::getConstEnumValue(EPropertyFlags::CPF_NativeAccessSpecifierProtected),
    CPF_NativeAccessSpecifierPrivate   = Unreal::getConstEnumValue(EPropertyFlags::CPF_NativeAccessSpecifierPrivate),
    CPF_SkipSerialization              = Unreal::getConstEnumValue(EPropertyFlags::CPF_SkipSerialization),
    CPF_TObjectPtr                     = Unreal::getConstEnumValue(EPropertyFlags::CPF_TObjectPtr),
    CPF_ExperimentalOverridableLogic   = Unreal::getConstEnumValue(EPropertyFlags::CPF_ExperimentalOverridableLogic),
    CPF_ExperimentalAlwaysOverriden    = Unreal::getConstEnumValue(EPropertyFlags::CPF_ExperimentalAlwaysOverriden),
    CPF_ExperimentalNeverOverriden     = Unreal::getConstEnumValue(EPropertyFlags::CPF_ExperimentalNeverOverriden),
    CPF_AllowSelfReference             = Unreal::getConstEnumValue(EPropertyFlags::CPF_AllowSelfReference),

    // These enum values are actually defined by macros so we need to handle them differently
    CPF_NativeAccessSpecifiers_        = CPF_NativeAccessSpecifiers,
    CPF_ParmFlags_                     = CPF_ParmFlags,
    CPF_PropagateToArrayInner_         = CPF_PropagateToArrayInner,
    CPF_PropagateToOptionalInner_      = CPF_PropagateToOptionalInner,
    CPF_PropagateToMapValue_           = CPF_PropagateToMapValue,
    CPF_PropagateToMapKey_             = CPF_PropagateToMapKey,
    CPF_PropagateToSetElement_         = CPF_PropagateToSetElement,
    CPF_InterfaceClearMask_            = CPF_InterfaceClearMask,
    CPF_DevelopmentAssets_             = CPF_DevelopmentAssets,
    CPF_ComputedFlags_                 = CPF_ComputedFlags,
    CPF_TObjectPtrWrapper_             = CPF_TObjectPtrWrapper,
    CPF_AllFlags_                      = CPF_AllFlags
};
ENUM_CLASS_FLAGS(ESpPropertyFlags); // required if combining values using bitwise operations

//
// This enum corresponds to EFieldIterationFlags declared in Engine/Source/Runtime/CoreUObject/Public/UObject/UnrealType.h
//

// UENUM(Flags) decorator is required to obtain an "A | B | C" string representation from a value, not intended to be Blueprint-accessible
UENUM(Flags)
enum class ESpFieldIterationFlags
{
    None              = Unreal::getConstEnumValue(EFieldIterationFlags::None),
    IncludeSuper      = Unreal::getConstEnumValue(EFieldIterationFlags::IncludeSuper),
    IncludeDeprecated = Unreal::getConstEnumValue(EFieldIterationFlags::IncludeDeprecated),
    IncludeInterfaces = Unreal::getConstEnumValue(EFieldIterationFlags::IncludeInterfaces),
    IncludeAll        = Unreal::getConstEnumValue(EFieldIterationFlags::IncludeAll),
    Default           = Unreal::getConstEnumValue(EFieldIterationFlags::Default)
};
ENUM_CLASS_FLAGS(ESpFieldIterationFlags); // required if combining values using bitwise operations

//
// This enum corresponds to EFunctionFlags declared in Engine/Source/Runtime/CoreUObject/Public/UObject/Script.h
//

// UENUM(Flags) decorator is required to obtain an "A | B | C" string representation from a value, not intended to be Blueprint-accessible
UENUM(Flags)
enum class ESpFunctionFlags : uint32
{
    FUNC_None                   = Unreal::getConstEnumValue(EFunctionFlags::FUNC_None),
    FUNC_Final                  = Unreal::getConstEnumValue(EFunctionFlags::FUNC_Final),
    FUNC_RequiredAPI            = Unreal::getConstEnumValue(EFunctionFlags::FUNC_RequiredAPI),
    FUNC_BlueprintAuthorityOnly = Unreal::getConstEnumValue(EFunctionFlags::FUNC_BlueprintAuthorityOnly),
    FUNC_BlueprintCosmetic      = Unreal::getConstEnumValue(EFunctionFlags::FUNC_BlueprintCosmetic),
    FUNC_Net                    = Unreal::getConstEnumValue(EFunctionFlags::FUNC_Net),
    FUNC_NetReliable            = Unreal::getConstEnumValue(EFunctionFlags::FUNC_NetReliable),
    FUNC_NetRequest             = Unreal::getConstEnumValue(EFunctionFlags::FUNC_NetRequest),
    FUNC_Exec                   = Unreal::getConstEnumValue(EFunctionFlags::FUNC_Exec),
    FUNC_Native                 = Unreal::getConstEnumValue(EFunctionFlags::FUNC_Native),
    FUNC_Event                  = Unreal::getConstEnumValue(EFunctionFlags::FUNC_Event),
    FUNC_NetResponse            = Unreal::getConstEnumValue(EFunctionFlags::FUNC_NetResponse),
    FUNC_Static                 = Unreal::getConstEnumValue(EFunctionFlags::FUNC_Static),
    FUNC_NetMulticast           = Unreal::getConstEnumValue(EFunctionFlags::FUNC_NetMulticast),
    FUNC_UbergraphFunction      = Unreal::getConstEnumValue(EFunctionFlags::FUNC_UbergraphFunction),
    FUNC_MulticastDelegate      = Unreal::getConstEnumValue(EFunctionFlags::FUNC_MulticastDelegate),
    FUNC_Public                 = Unreal::getConstEnumValue(EFunctionFlags::FUNC_Public),
    FUNC_Private                = Unreal::getConstEnumValue(EFunctionFlags::FUNC_Private),
    FUNC_Protected              = Unreal::getConstEnumValue(EFunctionFlags::FUNC_Protected),
    FUNC_Delegate               = Unreal::getConstEnumValue(EFunctionFlags::FUNC_Delegate),
    FUNC_NetServer              = Unreal::getConstEnumValue(EFunctionFlags::FUNC_NetServer),
    FUNC_HasOutParms            = Unreal::getConstEnumValue(EFunctionFlags::FUNC_HasOutParms),
    FUNC_HasDefaults            = Unreal::getConstEnumValue(EFunctionFlags::FUNC_HasDefaults),
    FUNC_NetClient              = Unreal::getConstEnumValue(EFunctionFlags::FUNC_NetClient),
    FUNC_DLLImport              = Unreal::getConstEnumValue(EFunctionFlags::FUNC_DLLImport),
    FUNC_BlueprintCallable      = Unreal::getConstEnumValue(EFunctionFlags::FUNC_BlueprintCallable),
    FUNC_BlueprintEvent         = Unreal::getConstEnumValue(EFunctionFlags::FUNC_BlueprintEvent),
    FUNC_BlueprintPure          = Unreal::getConstEnumValue(EFunctionFlags::FUNC_BlueprintPure),
    FUNC_EditorOnly             = Unreal::getConstEnumValue(EFunctionFlags::FUNC_EditorOnly),
    FUNC_Const                  = Unreal::getConstEnumValue(EFunctionFlags::FUNC_Const),
    FUNC_NetValidate            = Unreal::getConstEnumValue(EFunctionFlags::FUNC_NetValidate),

    // These enum values are actually defined by macros so we need to handle them differently
    FUNC_AllFlags_              = FUNC_AllFlags
};
ENUM_CLASS_FLAGS(ESpFunctionFlags); // required if combining values using bitwise operations

//
// This enum corresponds to EIncludeSuperFlag::Type declared in Engine/Source/Runtime/CoreUObject/Public/UObject/Class.h
//

UENUM() // not intended to be Blueprint-accessible
enum class ESpIncludeSuperFlag
{
    ExcludeSuper = Unreal::getConstEnumValue(EIncludeSuperFlag::Type::ExcludeSuper),
    IncludeSuper = Unreal::getConstEnumValue(EIncludeSuperFlag::Type::IncludeSuper)
};

//
// This enum corresponds to EObjectFlags declared in Engine/Source/Runtime/CoreUObject/Public/UObject/ObjectMacros.h
//

// UENUM(Flags) decorator is required to obtain an "A | B | C" string representation from a value, not intended to be Blueprint-accessible
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
    // RF_KeepForCooker                = Unreal::getConstEnumValue(EObjectFlags::RF_KeepForCooker), deprecated in UE 5.6
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
    RF_HasExternalPackage           = Unreal::getConstEnumValue(EObjectFlags::RF_HasExternalPackage)
    // RF_AllocatedInSharedPage        = Unreal::getConstEnumValue(EObjectFlags::RF_AllocatedInSharedPage) deprecated in 5.8
};
ENUM_CLASS_FLAGS(ESpObjectFlags); // required if combining values using bitwise operations

//
// This enum corresponds to ELoadFlags declared in Engine/Source/Runtime/CoreUObject/Public/UObject/ObjectMacros.h
//

// UENUM(Flags) decorator is required to obtain an "A | B | C" string representation from a value, not intended to be Blueprint-accessible
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
    // LOAD_RegenerateBulkDataGuids     = Unreal::getConstEnumValue(ELoadFlags::LOAD_RegenerateBulkDataGuids), deprecated in UE 5.5
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

// UENUM(Flags) decorator is required to obtain an "A | B | C" string representation from a value, not intended to be Blueprint-accessible
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

UENUM() // not intended to be Blueprint-accessible
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

USTRUCT() // not intended to be Blueprint-accessible
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
