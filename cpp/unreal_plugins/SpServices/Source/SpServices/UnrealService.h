//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // uint64_t

#include <map>
#include <string>
#include <utility> // std::make_pair, std::move
#include <vector>

#include <Components/ActorComponent.h>
#include <Delegates/IDelegateInstance.h> // FDelegateHandle
#include <Engine/Engine.h>               // GEngine
#include <Engine/EngineTypes.h>          // ESpawnActorCollisionHandlingMethod
#include <Engine/World.h>                // FActorSpawnParameters
#include <GameFramework/Actor.h>         // ESpawnActorScaleMethod
#include <HAL/IConsoleManager.h>         // EConsoleVariableFlags, IConsoleVariable
#include <HAL/Platform.h>                // uint64
#include <Math/Rotator.h>
#include <Math/Vector.h>
#include <Misc/EnumClassFlags.h>         // ENUM_CLASS_FLAGS
#include <StructUtils/UserDefinedStruct.h>
#include <UObject/Class.h>               // EIncludeSuperFlag, UClass
#include <UObject/Object.h>              // UObject
#include <UObject/ObjectMacros.h>        // EObjectFlags, ELoadFlags
#include <UObject/NameTypes.h>           // FName
#include <UObject/ObjectMacros.h>        // EPropertyFlags, GENERATED_BODY, UCLASS, UENUM, UFUNCTION, UPROPERTY
#include <UObject/Package.h>
#include <UObject/Class.h>               // EStructFlags, UScriptStruct
#include <UObject/Script.h>              // EFunctionFlags
#include <UObject/UObjectGlobals.h>      // GetTransientPackage, StaticLoadClass, StaticLoadObject
#include <UObject/UnrealType.h>          // EFieldIterationFlags

#include "SpCore/Assert.h"
#include "SpCore/Unreal.h"
#include "SpCore/UnrealUtils.h"
#include "SpCore/UnrealClassRegistry.h"

#include "SpServices/EntryPointBinder.h"
#include "SpServices/Service.h"
#include "SpServices/SpTypes.h"

#include "UnrealService.generated.h"

class APawn;
class UChildActorComponent;
class UClass;
class UPackageMap;
class UFunction;
class ULevel;
class UObject;
class UStruct;
class USceneComponent;
struct FObjectInstancingGraph;

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

// UENUM(Flags) decorator is required to obtain an "A | B | C" string representation from a value
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
    STRUCT_Inherit                    = Unreal::getConstEnumValue(EStructFlags::STRUCT_Inherit),
    STRUCT_ComputedFlags              = Unreal::getConstEnumValue(EStructFlags::STRUCT_ComputedFlags)
};
ENUM_CLASS_FLAGS(ESpStructFlags); // required if combining values using bitwise operations

//
// This enum corresponds to EClassFlags declared in Engine/Source/Runtime/CoreUObject/Public/UObject/ObjectMacros.h
//

// UENUM(Flags) decorator is required to obtain an "A | B | C" string representation from a value
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

// UENUM(Flags) decorator is required to obtain an "A | B | C" string representation from a value
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

// UENUM(Flags) decorator is required to obtain an "A | B | C" string representation from a value
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

// UENUM(Flags) decorator is required to obtain an "A | B | C" string representation from a value
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
    FUNC_AllFlags               = Unreal::getConstEnumValue(EFunctionFlags::FUNC_AllFlags)
};
ENUM_CLASS_FLAGS(ESpFunctionFlags); // required if combining values using bitwise operations

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

// UENUM(Flags) decorator is required to obtain an "A | B | C" string representation from a value
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
    RF_ImmutableDefaultObject       = Unreal::getConstEnumValue(EObjectFlags::RF_ImmutableDefaultObject),
    RF_WillBeLoaded                 = Unreal::getConstEnumValue(EObjectFlags::RF_WillBeLoaded),
    RF_HasExternalPackage           = Unreal::getConstEnumValue(EObjectFlags::RF_HasExternalPackage),
    RF_MigratingAsset               = Unreal::getConstEnumValue(EObjectFlags::RF_MigratingAsset),
    RF_MirroredGarbage              = Unreal::getConstEnumValue(EObjectFlags::RF_MirroredGarbage),
    RF_AllocatedInSharedPage        = Unreal::getConstEnumValue(EObjectFlags::RF_AllocatedInSharedPage)
};
ENUM_CLASS_FLAGS(ESpObjectFlags); // required if combining values using bitwise operations

//
// This enum corresponds to ELoadFlags declared in Engine/Source/Runtime/CoreUObject/Public/UObject/ObjectMacros.h
//

// UENUM(Flags) decorator is required to obtain an "A | B | C" string representation from a value
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

// UENUM(Flags) decorator is required to obtain an "A | B | C" string representation from a value
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

class UnrealService : public Service
{
public:
    UnrealService() = delete;
    UnrealService(CUnrealEntryPointBinder auto* unreal_entry_point_binder, Service::WorldFilter* world_filter) : Service("UnrealService", world_filter)
    {
        SP_ASSERT(unreal_entry_point_binder);

        std::string service_name = getWorldTypeName() + ".unreal_service";

        //
        // Get static struct and static class descs
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_static_struct_descs",
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

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_static_class_descs",
            [this]() -> std::vector<SpStaticStructDesc> {

                std::vector<SpStaticStructDesc> static_struct_descs;
                std::vector<UClass*> static_classes = UnrealUtils::findStaticStructsByType<UClass>();

                for (auto static_class : static_classes) {
                    SP_ASSERT(static_class);
                    SpStaticStructDesc static_struct_desc;
                    static_struct_desc.static_struct_ = static_class;
                    static_struct_desc.name_ = Unreal::getTypeAsString(static_class);

                    // leave map of ufunctions blank when getting all static class descs as a performance optimization
                    // static_struct_desc.ufunctions_ = UnrealUtils::findFunctionsAsMap(static_class, EFieldIterationFlags::Default);

                    static_struct_descs.push_back(std::move(static_struct_desc));
                }

                return static_struct_descs;
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_static_struct_desc",
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

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_static_class_desc",
            [this](uint64_t& uclass) -> SpStaticStructDesc {
                SP_ASSERT(uclass);
                UClass* uclass_ptr = toPtr<UClass>(uclass);
                SpStaticStructDesc static_struct_desc;
                static_struct_desc.static_struct_ = uclass_ptr;
                static_struct_desc.name_ = Unreal::getTypeAsString(uclass_ptr);

                // don't leave map of ufunctions blank when getting a specific static class desc
                static_struct_desc.ufunctions_ = UnrealUtils::findFunctionsAsMap(uclass_ptr, EFieldIterationFlags::Default);

                return static_struct_desc;
            });

        //
        // Get engine subsystem
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_engine_subsystem_by_class",
            [this](uint64_t& uclass) -> uint64_t {
                return toUInt64(Unreal::getEngineSubsystemByClass(toPtr<UClass>(uclass)));
            });

        //
        // Get editor subsystem, WITH_EDITOR implementations in SpServicesEditor/UnrealServiceEditor.h
        //

        #if !WITH_EDITOR // defined in an auto-generated header
            unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_editor_subsystem_by_class",
                [this](uint64_t& uclass) -> uint64_t {
                    return 0;
                });
        #endif

        //
        // Get subsystem
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_subsystem_by_class",
            [this](std::string& subsystem_provider_class_name, uint64_t& subsystem_uclass) -> uint64_t {
                return toUInt64(UnrealClassRegistry::getSubsystemByClass(subsystem_provider_class_name, getWorld(), toPtr<UClass>(subsystem_uclass)));
            });

        //
        // Functions for static structs and classes
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_static_structs",
            [this]() -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::findStaticStructs());
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_static_structs_as_map",
            [this]() -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::findStaticStructsAsMap());
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_struct_flags",
            [this](uint64_t& script_struct) -> std::vector<std::string> {
                SP_ASSERT(script_struct);
                return Unreal::getStringsFromCombinedEnumFlagValueAs<ESpStructFlags>(toPtr<UScriptStruct>(script_struct)->StructFlags);
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_static_classes",
            [this]() -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::findStaticClasses());
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_static_classes_as_map",
            [this]() -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::findStaticClassesAsMap());
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_derived_classes",
            [this](uint64_t& uclass, bool& recursive) -> std::vector<uint64_t> {
                SP_ASSERT(uclass);
                return toUInt64(UnrealUtils::getDerivedClasses(toPtr<UClass>(uclass), recursive));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_derived_classes_as_map",
            [this](uint64_t& uclass, bool& recursive) -> std::map<std::string, uint64_t> {
                SP_ASSERT(uclass);
                return toUInt64(UnrealUtils::getDerivedClassesAsMap(toPtr<UClass>(uclass), recursive));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_static_class",
            [this](std::string& class_name) -> uint64_t {
                return toUInt64(UnrealClassRegistry::getStaticClass(class_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_super_class",
            [this](uint64_t& uclass) -> uint64_t {
                SP_ASSERT(uclass);
                return toUInt64(toPtr<UClass>(uclass)->GetSuperClass());
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_class_flags",
            [this](uint64_t& uclass) -> std::vector<std::string> {
                SP_ASSERT(uclass);
                return Unreal::getStringsFromCombinedEnumFlagValueAs<ESpClassFlags>(toPtr<UClass>(uclass)->GetClassFlags());
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_default_object",
            [this](uint64_t& uclass, bool& create_if_needed) -> uint64_t {
                SP_ASSERT(uclass);
                return toUInt64(toPtr<UClass>(uclass)->GetDefaultObject(create_if_needed));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_class",
            [this](uint64_t& uobject) -> uint64_t {
                SP_ASSERT(uobject);
                return toUInt64(toPtr<UObject>(uobject)->GetClass());
            });

        //
        // Functions for getting C++ types as strings
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_type_for_struct_as_string",
            [this](uint64_t& ustruct) -> std::string {
                return Unreal::getTypeAsString(toPtr<UStruct>(ustruct));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_type_for_property_as_string",
            [this](uint64_t& property) -> std::string {
                return Unreal::getCppTypeAsString(toPtr<FProperty>(property));
            });

        //
        // Get property metadata for classes
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_properties_for_struct",
            [this](uint64_t& ustruct, std::vector<std::string>& field_iteration_flag_strings) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::findProperties(
                    toPtr<UStruct>(ustruct),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_flag_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_properties_for_struct_by_flags_any",
            [this](uint64_t& ustruct, std::vector<std::string>& property_flag_strings, std::vector<std::string>& field_iteration_flag_strings) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::findPropertiesByFlagsAny(
                    toPtr<UStruct>(ustruct),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EPropertyFlags, ESpPropertyFlags>(property_flag_strings),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_flag_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_properties_for_struct_by_flags_all",
            [this](uint64_t& ustruct, std::vector<std::string>& property_flag_strings, std::vector<std::string>& field_iteration_flag_strings) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::findPropertiesByFlagsAll(
                    toPtr<UStruct>(ustruct),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EPropertyFlags, ESpPropertyFlags>(property_flag_strings),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_flag_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_properties_for_struct_as_map",
            [this](uint64_t& ustruct, std::vector<std::string>& field_iteration_flag_strings) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::findPropertiesAsMap(
                    toPtr<UStruct>(ustruct),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_flag_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_properties_for_struct_by_flags_any_as_map",
            [this](uint64_t& ustruct, std::vector<std::string>& property_flag_strings, std::vector<std::string>& field_iteration_flag_strings) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::findPropertiesByFlagsAnyAsMap(
                    toPtr<UStruct>(ustruct),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EPropertyFlags, ESpPropertyFlags>(property_flag_strings),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_flag_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_properties_for_struct_by_flags_all_as_map",
            [this](uint64_t& ustruct, std::vector<std::string>& property_flag_strings, std::vector<std::string>& field_iteration_flag_strings) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::findPropertiesByFlagsAllAsMap(
                    toPtr<UStruct>(ustruct),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EPropertyFlags, ESpPropertyFlags>(property_flag_strings),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_flag_strings)));
            });

        //
        // Get property metadata for functions
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_properties_for_function",
            [this](uint64_t& ufunction, std::vector<std::string>& field_iteration_flag_strings) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::findProperties(
                    toPtr<UFunction>(ufunction),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_flag_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_properties_for_function_by_flags_any",
            [this](uint64_t& ufunction, std::vector<std::string>& property_flag_strings, std::vector<std::string>& field_iteration_flag_strings) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::findPropertiesByFlagsAny(
                    toPtr<UFunction>(ufunction),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EPropertyFlags, ESpPropertyFlags>(property_flag_strings),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_flag_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_properties_for_function_by_flags_all",
            [this](uint64_t& ufunction, std::vector<std::string>& property_flag_strings, std::vector<std::string>& field_iteration_flag_strings) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::findPropertiesByFlagsAll(
                    toPtr<UFunction>(ufunction),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EPropertyFlags, ESpPropertyFlags>(property_flag_strings),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_flag_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_properties_for_function_as_map",
            [this](uint64_t& ufunction, std::vector<std::string>& field_iteration_flag_strings) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::findPropertiesAsMap(
                    toPtr<UFunction>(ufunction),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_flag_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_properties_for_function_by_flags_any_as_map",
            [this](uint64_t& ufunction, std::vector<std::string>& property_flag_strings, std::vector<std::string>& field_iteration_flag_strings) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::findPropertiesByFlagsAnyAsMap(
                    toPtr<UFunction>(ufunction),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EPropertyFlags, ESpPropertyFlags>(property_flag_strings),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_flag_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_properties_for_function_by_flags_all_as_map",
            [this](uint64_t& ufunction, std::vector<std::string>& property_flag_strings, std::vector<std::string>& field_iteration_flag_strings) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::findPropertiesByFlagsAllAsMap(
                    toPtr<UFunction>(ufunction),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EPropertyFlags, ESpPropertyFlags>(property_flag_strings),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_flag_strings)));
            });

        //
        // Helper functions for property metadata
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_property_flags",
            [this](uint64_t& property) -> std::vector<std::string> {
                SP_ASSERT(property);
                return Unreal::getStringsFromCombinedEnumFlagValueAs<ESpPropertyFlags>(toPtr<FProperty>(property)->GetPropertyFlags());
            });

        //
        // Get and set object properties
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_properties_for_object_as_string",
            [this](uint64_t& uobject) -> std::string {
                return UnrealUtils::getObjectPropertiesAsString(toPtr<UObject>(uobject));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_properties_for_struct_as_string",
            [this](uint64_t& value_ptr, uint64_t& ustruct) -> std::string {
                return UnrealUtils::getObjectPropertiesAsString(toPtr<void>(value_ptr), toPtr<UStruct>(ustruct));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "set_properties_for_object_from_string",
            [this](uint64_t& uobject, std::string& properties_string) -> void {
                UnrealUtils::setObjectPropertiesFromString(toPtr<UObject>(uobject), properties_string);
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "set_properties_for_struct_from_string",
            [this](uint64_t& value_ptr, uint64_t& ustruct, std::string& properties_string) -> void {
                UnrealUtils::setObjectPropertiesFromString(toPtr<void>(value_ptr), toPtr<UStruct>(ustruct), properties_string);
            });

        //
        // Get and set individual object property values using property descs
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_property_desc_for_object",
            [this](uint64_t& uobject, std::string& property_name) -> SpPropertyDesc {
                return UnrealUtils::findPropertyByName(toPtr<UObject>(uobject), property_name);
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_property_desc_for_struct",
            [this](uint64_t& value_ptr, uint64_t& ustruct, std::string& property_name) -> SpPropertyDesc {
                return UnrealUtils::findPropertyByName(toPtr<void>(value_ptr), toPtr<UStruct>(ustruct), property_name);
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_property_value_for_desc_as_string",
            [this](SpPropertyDesc& property_desc) -> SpPropertyValue {
                return UnrealUtils::getPropertyValueAsString(property_desc);
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "set_property_value_for_desc_from_string",
            [this](SpPropertyDesc& property_desc, std::string& property_value_string) -> void {
                UnrealUtils::setPropertyValueFromString(property_desc, property_value_string);
            });

        //
        // Get and set individual object property values
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_property_value_for_object_as_string",
            [this](uint64_t& uobject, std::string& property_name) -> SpPropertyValue {
                SpPropertyDesc property_desc = UnrealUtils::findPropertyByName(toPtr<UObject>(uobject), property_name);
                return UnrealUtils::getPropertyValueAsString(property_desc);
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_property_value_for_struct_as_string",
            [this](uint64_t& value_ptr, uint64_t& ustruct, std::string& property_name) -> SpPropertyValue {
                SpPropertyDesc property_desc = UnrealUtils::findPropertyByName(toPtr<void>(value_ptr), toPtr<UStruct>(ustruct), property_name);
                return UnrealUtils::getPropertyValueAsString(property_desc);
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "set_property_value_for_object_from_string",
            [this](uint64_t& uobject, std::string& property_name, std::string& property_value_string) -> void {
                SpPropertyDesc property_desc = UnrealUtils::findPropertyByName(toPtr<UObject>(uobject), property_name);
                UnrealUtils::setPropertyValueFromString(property_desc, property_value_string);
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "set_property_value_for_struct_from_string",
            [this](uint64_t& value_ptr, uint64_t& ustruct, std::string& property_name, std::string& property_value_string) -> void {
                SpPropertyDesc property_desc = UnrealUtils::findPropertyByName(toPtr<void>(value_ptr), toPtr<UStruct>(ustruct), property_name);
                UnrealUtils::setPropertyValueFromString(property_desc, property_value_string);
            });

        //
        // Find functions and get function flags
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_functions",
            [this](uint64_t& uclass, std::vector<std::string>& field_iteration_strings) -> std::vector<uint64_t> {
                return toUInt64(
                    UnrealUtils::findFunctions(
                        toPtr<UClass>(uclass),
                        Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_functions_by_flags_any",
            [this](uint64_t& uclass, std::vector<std::string>& function_flags, std::vector<std::string>& field_iteration_strings) -> std::vector<uint64_t> {
                return toUInt64(
                    UnrealUtils::findFunctionsByFlagsAny(
                        toPtr<UClass>(uclass),
                        Unreal::getCombinedEnumFlagValueFromStringsAs<EFunctionFlags, ESpFunctionFlags>(function_flags),
                        Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_functions_by_flags_all",
            [this](uint64_t& uclass, std::vector<std::string>& function_flags, std::vector<std::string>& field_iteration_strings) -> std::vector<uint64_t> {
                return toUInt64(
                    UnrealUtils::findFunctionsByFlagsAll(
                        toPtr<UClass>(uclass),
                        Unreal::getCombinedEnumFlagValueFromStringsAs<EFunctionFlags, ESpFunctionFlags>(function_flags),
                        Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_functions_as_map",
            [this](uint64_t& uclass, std::vector<std::string>& field_iteration_strings) -> std::map<std::string, uint64_t> {
                return toUInt64(
                    UnrealUtils::findFunctionsAsMap(
                        toPtr<UClass>(uclass),
                        Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_functions_by_flags_any_as_map",
            [this](uint64_t& uclass, std::vector<std::string>& function_flags, std::vector<std::string>& field_iteration_strings) -> std::map<std::string, uint64_t> {
                return toUInt64(
                    UnrealUtils::findFunctionsByFlagsAnyAsMap(
                        toPtr<UClass>(uclass),
                        Unreal::getCombinedEnumFlagValueFromStringsAs<EFunctionFlags, ESpFunctionFlags>(function_flags),
                        Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_functions_by_flags_all_as_map",
            [this](uint64_t& uclass, std::vector<std::string>& function_flags, std::vector<std::string>& field_iteration_strings) -> std::map<std::string, uint64_t> {
                return toUInt64(
                    UnrealUtils::findFunctionsByFlagsAllAsMap(
                        toPtr<UClass>(uclass),
                        Unreal::getCombinedEnumFlagValueFromStringsAs<EFunctionFlags, ESpFunctionFlags>(function_flags),
                        Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_function_by_name",
            [this](uint64_t& uclass, std::string& function_name, std::vector<std::string>& field_iteration_strings) -> uint64_t {
                return toUInt64(
                    UnrealUtils::findFunctionByName(
                        toPtr<UClass>(uclass),
                        function_name,
                        Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_function_flags",
            [this](uint64_t& ufunction) -> std::vector<std::string> {
                SP_ASSERT(ufunction);
                return Unreal::getStringsFromCombinedEnumFlagValueAs<ESpFunctionFlags>(toPtr<UFunction>(ufunction)->FunctionFlags);
            });

        //
        // Call function
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "call_function",
            [this](
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

                return UnrealUtils::callFunction(getWorld(), uobject_ptr, ufunction_ptr, args, world_context);
            });

        //
        // Spawn actor
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "spawn_actor",
            [this](uint64_t& uclass, std::string& location_string, std::string& rotation_string, std::string& spawn_parameters_string, std::vector<std::string>& object_flag_strings) -> uint64_t {

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

                SP_ASSERT(getWorld());
                return toUInt64(getWorld()->SpawnActor(toPtr<UClass>(uclass), &location, &rotation, actor_spawn_parameters));
            });

        //
        // Destroy actor
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "destroy_actor",
            [this](uint64_t& actor, bool& net_force, bool& should_modify_level) -> bool {
                SP_ASSERT(actor);
                return toPtr<AActor>(actor)->Destroy(net_force, should_modify_level);
            });

        //
        // Create new object
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "new_object",
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

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "static_load_object",
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
                        Unreal::toFString(name),
                        Unreal::toFString(filename),
                        Unreal::getCombinedEnumFlagValueFromStringsAs<ELoadFlags, ESpLoadFlags>(load_flag_strings),
                        toPtr<UPackageMap>(sandbox),
                        allow_object_reconciliation,
                        toPtr<FLinkerInstancingContext>(instancing_context)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "static_load_class",
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
                        Unreal::toFString(name),
                        Unreal::toFString(filename),
                        Unreal::getCombinedEnumFlagValueFromStringsAs<ELoadFlags, ESpLoadFlags>(load_flag_strings),
                        toPtr<UPackageMap>(sandbox)));
            });

        //
        // Enable and disable garbage collection for uobject
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "add_object_to_root",
            [this](uint64_t& uobject) -> void {
                UObject* uboject_ptr = toPtr<UObject>(uobject);
                SP_ASSERT(uboject_ptr);
                uboject_ptr->AddToRoot();
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "remove_object_from_root",
            [this](uint64_t& uobject) -> void {
                UObject* uboject_ptr = toPtr<UObject>(uobject);
                SP_ASSERT(uboject_ptr);
                uboject_ptr->RemoveFromRoot();
            });

        //
        // Find, get, and set console variable
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_console_variable_by_name",
            [this](std::string& cvar_name) -> uint64_t {
                return toUInt64(IConsoleManager::Get().FindConsoleVariable(Unreal::toTCharPtr(cvar_name)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_console_variable_value_as_bool",
            [this](uint64_t& cvar) -> bool {
                SP_ASSERT(cvar);
                return toPtr<IConsoleVariable>(cvar)->GetBool();
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_console_variable_value_as_int",
            [this](uint64_t& cvar) -> int64_t {
                SP_ASSERT(cvar);
                return toPtr<IConsoleVariable>(cvar)->GetInt();
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_console_variable_value_as_float",
            [this](uint64_t& cvar) -> float {
                SP_ASSERT(cvar);
                return toPtr<IConsoleVariable>(cvar)->GetFloat();
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_console_variable_value_as_string",
            [this](uint64_t& cvar) -> std::string {
                SP_ASSERT(cvar);
                return Unreal::toStdString(toPtr<IConsoleVariable>(cvar)->GetString());
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "set_console_variable_value_from_bool",
            [this](uint64_t& cvar, bool& val, std::vector<std::string>& set_by_strings) -> void {
                SP_ASSERT(cvar);
                toPtr<IConsoleVariable>(cvar)->Set(val, Unreal::getCombinedEnumFlagValueFromStringsAs<EConsoleVariableFlags, ESpConsoleVariableFlags>(set_by_strings));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "set_console_variable_value_from_int",
            [this](uint64_t& cvar, int32_t& val, std::vector<std::string>& set_by_strings) -> void {
                SP_ASSERT(cvar);
                toPtr<IConsoleVariable>(cvar)->Set(val, Unreal::getCombinedEnumFlagValueFromStringsAs<EConsoleVariableFlags, ESpConsoleVariableFlags>(set_by_strings));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "set_console_variable_value_from_float",
            [this](uint64_t& cvar, float& val, std::vector<std::string>& set_by_strings) -> void {
                SP_ASSERT(cvar);
                toPtr<IConsoleVariable>(cvar)->Set(val, Unreal::getCombinedEnumFlagValueFromStringsAs<EConsoleVariableFlags, ESpConsoleVariableFlags>(set_by_strings));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "set_console_variable_value_from_string",
            [this](uint64_t& cvar, std::string& val, std::vector<std::string>& set_by_strings) -> void {
                SP_ASSERT(cvar);
                toPtr<IConsoleVariable>(cvar)->Set(Unreal::toTCharPtr(val), Unreal::getCombinedEnumFlagValueFromStringsAs<EConsoleVariableFlags, ESpConsoleVariableFlags>(set_by_strings));
            });

        //
        // Execute console command
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "execute_console_command",
            [this](std::string& command) -> void {
                SP_ASSERT(GEngine);
                GEngine->Exec(getWorld(), Unreal::toTCharPtr(command));
            });

        //
        // Stable name helper functions
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "has_stable_name",
            [this](uint64_t& actor) -> bool { return UnrealUtils::hasStableName(toPtr<AActor>(actor)); });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_stable_name_for_actor",
            [this](uint64_t& actor) -> std::string { return UnrealUtils::getStableName(toPtr<AActor>(actor)); });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "try_get_stable_name_for_actor",
            [this](uint64_t& actor) -> std::string { return UnrealUtils::tryGetStableName(toPtr<AActor>(actor)); });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_stable_name_for_component",
            [this](uint64_t& actor_component, bool& include_actor_name) -> std::string { return UnrealUtils::getStableName(toPtr<UActorComponent>(actor_component), include_actor_name); });

        //
        // Get actor and component tags
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_actor_tags",
            [this](uint64_t& actor) -> std::vector<std::string> { return Unreal::getTags(toPtr<AActor>(actor)); });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_component_tags",
            [this](uint64_t& component) -> std::vector<std::string> { return Unreal::getTags(toPtr<UActorComponent>(component)); });

        //
        // Find actors unconditionally and return an std::vector or std::map
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_actors",
            [this]() -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::findActors(getWorld()));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_actors_as_map",
            [this]() -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::findActorsAsMap(getWorld()));
            });

        //
        // Get components unconditionally and return an std::vector or std::map
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_components",
            [this](uint64_t& actor) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::getComponents(toPtr<AActor>(actor)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_components_as_map",
            [this](uint64_t& actor) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::getComponentsAsMap(toPtr<AActor>(actor)));
            });

        //
        // Get children components unconditionally and return an std::vector or std::map
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_children_components_for_actor",
            [this](uint64_t& parent, bool& include_all_descendants) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponents(toPtr<AActor>(parent), include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_children_components_for_actor_as_map",
            [this](uint64_t& parent, bool& include_all_descendants) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsAsMap(toPtr<AActor>(parent), include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_children_components_for_scene_component",
            [this](uint64_t& parent, bool& include_all_descendants) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponents(toPtr<USceneComponent>(parent), include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_children_components_for_scene_component_as_map",
            [this](uint64_t& parent, bool& include_all_descendants) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsAsMap(toPtr<USceneComponent>(parent), include_all_descendants));
            });

        //
        // Find actors conditionally and return an std::vector
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_actors_by_name",
            [this](uint64_t& uclass, std::string& actor_name) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::findActorsByName(toPtr<UClass>(uclass), getWorld(), actor_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_actors_by_tag",
            [this](uint64_t& uclass, std::string& tag) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::findActorsByTag(toPtr<UClass>(uclass), getWorld(), tag));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_actors_by_tags_any",
            [this](uint64_t& uclass, std::vector<std::string>& tags) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::findActorsByTagsAny(toPtr<UClass>(uclass), getWorld(), tags));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_actors_by_tags_all",
            [this](uint64_t& uclass, std::vector<std::string>& tags) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::findActorsByTagsAll(toPtr<UClass>(uclass), getWorld(), tags));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_actors_by_class",
            [this](uint64_t& uclass) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::findActorsByClass(toPtr<UClass>(uclass), getWorld()));
            });

        //
        // Find actors conditionally and return an std::map
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_actors_by_name_as_map",
            [this](uint64_t& uclass, std::string& actor_name) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::findActorsByNameAsMap(toPtr<UClass>(uclass), getWorld(), actor_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_actors_by_tag_as_map",
            [this](uint64_t& uclass, std::string& tag) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::findActorsByTagAsMap(toPtr<UClass>(uclass), getWorld(), tag));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_actors_by_tags_any_as_map",
            [this](uint64_t& uclass, std::vector<std::string>& tags) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::findActorsByTagsAnyAsMap(toPtr<UClass>(uclass), getWorld(), tags));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_actors_by_tags_all_as_map",
            [this](uint64_t& uclass, std::vector<std::string>& tags) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::findActorsByTagsAllAsMap(toPtr<UClass>(uclass), getWorld(), tags));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_actors_by_class_as_map",
            [this](uint64_t& uclass) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::findActorsByClassAsMap(toPtr<UClass>(uclass), getWorld()));
            });

        //
        // Find actor conditionally
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_actor_by_name",
            [this](uint64_t& uclass, std::string& actor_name) -> uint64_t {
                return toUInt64(UnrealUtils::findActorByName(toPtr<UClass>(uclass), getWorld(), actor_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_actor_by_tag",
            [this](uint64_t& uclass, std::string& tag) -> uint64_t {
                return toUInt64(UnrealUtils::findActorByTag(toPtr<UClass>(uclass), getWorld(), tag));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_actor_by_tags_any",
            [this](uint64_t& uclass, std::vector<std::string>& tags) -> uint64_t {
                return toUInt64(UnrealUtils::findActorByTagsAny(toPtr<UClass>(uclass), getWorld(), tags));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_actor_by_tags_all",
            [this](uint64_t& uclass, std::vector<std::string>& tags) -> uint64_t {
                return toUInt64(UnrealUtils::findActorByTagsAll(toPtr<UClass>(uclass), getWorld(), tags));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_actor_by_class",
            [this](uint64_t& uclass) -> uint64_t {
                return toUInt64(UnrealUtils::findActorByClass(toPtr<UClass>(uclass), getWorld()));
            });

        //
        // Get components conditionally and return an std::vector
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_components_by_name",
            [this](uint64_t& uclass, uint64_t& actor, std::string& component_name, bool& include_from_child_actors) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::getComponentsByName(toPtr<UClass>(uclass), toPtr<AActor>(actor), component_name, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_components_by_path",
            [this](uint64_t& uclass, uint64_t& actor, std::string& component_path, bool& include_from_child_actors) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::getComponentsByPath(toPtr<UClass>(uclass), toPtr<AActor>(actor), component_path, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_components_by_tag",
            [this](uint64_t& uclass, uint64_t& actor, std::string& tag, bool& include_from_child_actors) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::getComponentsByTag(toPtr<UClass>(uclass), toPtr<AActor>(actor), tag, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_components_by_tags_any",
            [this](uint64_t& uclass, uint64_t& actor, std::vector<std::string>& tags, bool& include_from_child_actors) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::getComponentsByTagsAny(toPtr<UClass>(uclass), toPtr<AActor>(actor), tags, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_components_by_tags_all",
            [this](uint64_t& uclass, uint64_t& actor, std::vector<std::string>& tags, bool& include_from_child_actors) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::getComponentsByTagsAll(toPtr<UClass>(uclass), toPtr<AActor>(actor), tags, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_components_by_class",
            [this](uint64_t& uclass, uint64_t& actor, bool& include_from_child_actors) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::getComponentsByClass(toPtr<UClass>(uclass), toPtr<AActor>(actor), include_from_child_actors));
            });

        //
        // Get components conditionally and return an std::map
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_components_by_name_as_map",
            [this](uint64_t& uclass, uint64_t& actor, std::string& component_name, bool& include_from_child_actors) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::getComponentsByNameAsMap(toPtr<UClass>(uclass), toPtr<AActor>(actor), component_name, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_components_by_path_as_map",
            [this](uint64_t& uclass, uint64_t& actor, std::string& component_path, bool& include_from_child_actors) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::getComponentsByPathAsMap(toPtr<UClass>(uclass), toPtr<AActor>(actor), component_path, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_components_by_tag_as_map",
            [this](uint64_t& uclass, uint64_t& actor, std::string& tag, bool& include_from_child_actors) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::getComponentsByTagAsMap(toPtr<UClass>(uclass), toPtr<AActor>(actor), tag, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_components_by_tags_any_as_map",
            [this](uint64_t& uclass, uint64_t& actor, std::vector<std::string>& tags, bool& include_from_child_actors) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::getComponentsByTagsAnyAsMap(toPtr<UClass>(uclass), toPtr<AActor>(actor), tags, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_components_by_tags_all_as_map",
            [this](uint64_t& uclass, uint64_t& actor, std::vector<std::string>& tags, bool& include_from_child_actors) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::getComponentsByTagsAllAsMap(toPtr<UClass>(uclass), toPtr<AActor>(actor), tags, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_components_by_class_as_map",
            [this](uint64_t& uclass, uint64_t& actor, bool& include_from_child_actors) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::getComponentsByClassAsMap(toPtr<UClass>(uclass), toPtr<AActor>(actor), include_from_child_actors));
            });

        //
        // Get component conditionally
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_component_by_name",
            [this](uint64_t& uclass, uint64_t& actor, std::string& component_name, bool& include_from_child_actors) -> uint64_t {
                return toUInt64(UnrealUtils::getComponentByName(toPtr<UClass>(uclass), toPtr<AActor>(actor), component_name, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_component_by_path",
            [this](uint64_t& uclass, uint64_t& actor, std::string& component_path, bool& include_from_child_actors) -> uint64_t {
                return toUInt64(UnrealUtils::getComponentByPath(toPtr<UClass>(uclass), toPtr<AActor>(actor), component_path, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_component_by_tag",
            [this](uint64_t& uclass, uint64_t& actor, std::string& tag, bool& include_from_child_actors) -> uint64_t {
                return toUInt64(UnrealUtils::getComponentByTag(toPtr<UClass>(uclass), toPtr<AActor>(actor), tag, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_component_by_tags_any",
            [this](uint64_t& uclass, uint64_t& actor, std::vector<std::string>& tags, bool& include_from_child_actors) -> uint64_t {
                return toUInt64(UnrealUtils::getComponentByTagsAny(toPtr<UClass>(uclass), toPtr<AActor>(actor), tags, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_component_by_tags_all",
            [this](uint64_t& uclass, uint64_t& actor, std::vector<std::string>& tags, bool& include_from_child_actors) -> uint64_t {
                return toUInt64(UnrealUtils::getComponentByTagsAll(toPtr<UClass>(uclass), toPtr<AActor>(actor), tags, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_component_by_class",
            [this](uint64_t& uclass, uint64_t& actor, bool& include_from_child_actors) -> uint64_t {
                return toUInt64(UnrealUtils::getComponentByClass(toPtr<UClass>(uclass), toPtr<AActor>(actor), include_from_child_actors));
            });

        //
        // Get children components conditionally from an actor and return an std::vector
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_children_components_for_actor_by_name",
            [this](uint64_t& uclass, uint64_t& parent, std::string& child_component_name, bool& include_all_descendants) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsByName(toPtr<UClass>(uclass), toPtr<AActor>(parent), child_component_name, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_children_components_for_actor_by_tag",
            [this](uint64_t& uclass, uint64_t& parent, std::string& tag, bool& include_all_descendants) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsByTag(toPtr<UClass>(uclass), toPtr<AActor>(parent), tag, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_children_components_for_actor_by_tags_any",
            [this](uint64_t& uclass, uint64_t& parent, std::vector<std::string>& tags, bool& include_all_descendants) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsByTagsAny(toPtr<UClass>(uclass), toPtr<AActor>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_children_components_for_actor_by_tags_all",
            [this](uint64_t& uclass, uint64_t& parent, std::vector<std::string>& tags, bool& include_all_descendants) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsByTagsAll(toPtr<UClass>(uclass), toPtr<AActor>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_children_components_for_actor_by_class",
            [this](uint64_t& uclass, uint64_t& parent, bool& include_all_descendants) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsByClass(toPtr<UClass>(uclass), toPtr<AActor>(parent), include_all_descendants));
            });

        //
        // Get children components conditionally from an actor and return an std::map
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_children_components_for_actor_by_name_as_map",
            [this](uint64_t& uclass, uint64_t& parent, std::string& child_component_name, bool& include_all_descendants) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsByNameAsMap(toPtr<UClass>(uclass), toPtr<AActor>(parent), child_component_name, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_children_components_for_actor_by_tag_as_map",
            [this](uint64_t& uclass, uint64_t& parent, std::string& tag, bool& include_all_descendants) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsByTagAsMap(toPtr<UClass>(uclass), toPtr<AActor>(parent), tag, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_children_components_for_actor_by_tags_any_as_map",
            [this](uint64_t& uclass, uint64_t& parent, std::vector<std::string>& tags, bool& include_all_descendants) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsByTagsAnyAsMap(toPtr<UClass>(uclass), toPtr<AActor>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_children_components_for_actor_by_tags_all_as_map",
            [this](uint64_t& uclass, uint64_t& parent, std::vector<std::string>& tags, bool& include_all_descendants) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsByTagsAllAsMap(toPtr<UClass>(uclass), toPtr<AActor>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_children_components_for_actor_by_class_as_map",
            [this](uint64_t& uclass, uint64_t& parent, bool& include_all_descendants) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsByClassAsMap(toPtr<UClass>(uclass), toPtr<AActor>(parent), include_all_descendants));
            });

        //
        // Get child component conditionally from an actor
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_child_component_for_actor_by_name",
            [this](uint64_t& uclass, uint64_t& parent, std::string& child_component_name, bool& include_all_descendants) -> uint64_t {
                return toUInt64(UnrealUtils::getChildComponentByName(toPtr<UClass>(uclass), toPtr<AActor>(parent), child_component_name, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_child_component_for_actor_by_tag",
            [this](uint64_t& uclass, uint64_t& parent, std::string& tag, bool& include_all_descendants) -> uint64_t {
                return toUInt64(UnrealUtils::getChildComponentByTag(toPtr<UClass>(uclass), toPtr<AActor>(parent), tag, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_child_component_for_actor_by_tags_any",
            [this](uint64_t& uclass, uint64_t& parent, std::vector<std::string>& tags, bool& include_all_descendants) -> uint64_t {
                return toUInt64(UnrealUtils::getChildComponentByTagsAny(toPtr<UClass>(uclass), toPtr<AActor>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_child_component_for_actor_by_tags_all",
            [this](uint64_t& uclass, uint64_t& parent, std::vector<std::string>& tags, bool& include_all_descendants) -> uint64_t {
                return toUInt64(UnrealUtils::getChildComponentByTagsAll(toPtr<UClass>(uclass), toPtr<AActor>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_child_component_for_actor_by_class",
            [this](uint64_t& uclass, uint64_t& parent, bool& include_all_descendants) -> uint64_t {
                return toUInt64(UnrealUtils::getChildComponentByClass(toPtr<UClass>(uclass), toPtr<AActor>(parent), include_all_descendants));
            });

        //
        // Get children components conditionally from a scene component and return an std::vector
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_children_components_for_scene_component_by_name",
            [this](uint64_t& uclass, uint64_t& parent, std::string& child_component_name, bool& include_all_descendants) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsByName(toPtr<UClass>(uclass), toPtr<USceneComponent>(parent), child_component_name, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_children_components_for_scene_component_by_tag",
            [this](uint64_t& uclass, uint64_t& parent, std::string& tag, bool& include_all_descendants) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsByTag(toPtr<UClass>(uclass), toPtr<USceneComponent>(parent), tag, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_children_components_for_scene_component_by_tags_any",
            [this](uint64_t& uclass, uint64_t& parent, std::vector<std::string>& tags, bool& include_all_descendants) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsByTagsAny(toPtr<UClass>(uclass), toPtr<USceneComponent>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_children_components_for_scene_component_by_tags_all",
            [this](uint64_t& uclass, uint64_t& parent, std::vector<std::string>& tags, bool& include_all_descendants) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsByTagsAll(toPtr<UClass>(uclass), toPtr<USceneComponent>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_children_components_for_scene_component_by_class",
            [this](uint64_t& uclass, uint64_t& parent, bool& include_all_descendants) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsByClass(toPtr<UClass>(uclass), toPtr<USceneComponent>(parent), include_all_descendants));
            });

        //
        // Get children components conditionally from a scene component and return an std::map
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_children_components_for_scene_component_by_name_as_map",
            [this](uint64_t& uclass, uint64_t& parent, std::string& child_component_name, bool& include_all_descendants) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsByNameAsMap(toPtr<UClass>(uclass), toPtr<USceneComponent>(parent), child_component_name, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_children_components_for_scene_component_by_tag_as_map",
            [this](uint64_t& uclass, uint64_t& parent, std::string& tag, bool& include_all_descendants) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsByTagAsMap(toPtr<UClass>(uclass), toPtr<USceneComponent>(parent), tag, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_children_components_for_scene_component_by_tags_any_as_map",
            [this](uint64_t& uclass, uint64_t& parent, std::vector<std::string>& tags, bool& include_all_descendants) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsByTagsAnyAsMap(toPtr<UClass>(uclass), toPtr<USceneComponent>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_children_components_for_scene_component_by_tags_all_as_map",
            [this](uint64_t& uclass, uint64_t& parent, std::vector<std::string>& tags, bool& include_all_descendants) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsByTagsAllAsMap(toPtr<UClass>(uclass), toPtr<USceneComponent>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_children_components_for_scene_component_by_class_as_map",
            [this](uint64_t& uclass, uint64_t& parent, bool& include_all_descendants) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsByClassAsMap(toPtr<UClass>(uclass), toPtr<USceneComponent>(parent), include_all_descendants));
            });

        //
        // Get child component conditionally from a scene component
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_child_component_for_scene_component_by_name",
            [this](uint64_t& uclass, uint64_t& parent, std::string& child_component_name, bool& include_all_descendants) -> uint64_t {
                return toUInt64(UnrealUtils::getChildComponentByName(toPtr<UClass>(uclass), toPtr<USceneComponent>(parent), child_component_name, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_child_component_for_scene_component_by_tag",
            [this](uint64_t& uclass, uint64_t& parent, std::string& tag, bool& include_all_descendants) -> uint64_t {
                return toUInt64(UnrealUtils::getChildComponentByTag(toPtr<UClass>(uclass), toPtr<USceneComponent>(parent), tag, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_child_component_for_scene_component_by_tags_any",
            [this](uint64_t& uclass, uint64_t& parent, std::vector<std::string>& tags, bool& include_all_descendants) -> uint64_t {
                return toUInt64(UnrealUtils::getChildComponentByTagsAny(toPtr<UClass>(uclass), toPtr<USceneComponent>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_child_component_for_scene_component_by_tags_all",
            [this](uint64_t& uclass, uint64_t& parent, std::vector<std::string>& tags, bool& include_all_descendants) -> uint64_t {
                return toUInt64(UnrealUtils::getChildComponentByTagsAll(toPtr<UClass>(uclass), toPtr<USceneComponent>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_child_component_for_scene_component_by_class",
            [this](uint64_t& uclass, uint64_t& parent, bool& include_all_descendants) -> uint64_t {
                return toUInt64(UnrealUtils::getChildComponentByClass(toPtr<UClass>(uclass), toPtr<USceneComponent>(parent), include_all_descendants));
            });

        //
        // Create component
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "create_component_outside_owner_constructor_by_class",
            [this](uint64_t& uclass, uint64_t& owner, std::string& component_name) -> uint64_t {
                return toUInt64(UnrealUtils::createComponentOutsideOwnerConstructorByClass(toPtr<UClass>(uclass), toPtr<AActor>(owner), component_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "create_scene_component_outside_owner_constructor_for_actor_by_class",
            [this](uint64_t& uclass, uint64_t& owner, std::string& scene_component_name) -> uint64_t {
                return toUInt64(UnrealUtils::createSceneComponentOutsideOwnerConstructorByClass(toPtr<UClass>(uclass), toPtr<AActor>(owner), scene_component_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "create_scene_component_outside_owner_constructor_for_object_by_class",
            [this](uint64_t& uclass, uint64_t& owner, uint64_t& parent, std::string& scene_component_name) -> uint64_t {
                return toUInt64(UnrealUtils::createSceneComponentOutsideOwnerConstructorByClass(toPtr<UClass>(uclass), toPtr<UObject>(owner), toPtr<USceneComponent>(parent), scene_component_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "create_scene_component_outside_owner_constructor_for_scene_component_by_class",
            [this](uint64_t& uclass, uint64_t& owner, std::string& scene_component_name) -> uint64_t {
                return toUInt64(UnrealUtils::createSceneComponentOutsideOwnerConstructorByClass(toPtr<UClass>(uclass), toPtr<USceneComponent>(owner), scene_component_name));
            });

        //
        // Destroy component
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "destroy_component_outside_owner_constructor",
            [this](uint64_t& component, bool& promote_children) -> void {
                SP_ASSERT(component);
                UnrealUtils::destroyComponentOutsideOwnerConstructor(toPtr<UActorComponent>(component), promote_children);
            });
    }
};
