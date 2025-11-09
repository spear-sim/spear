//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
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
#include <UObject/Class.h>               // EIncludeSuperFlag, UClass
#include <UObject/Object.h>              // UObject
#include <UObject/ObjectMacros.h>        // EObjectFlags, ELoadFlags
#include <UObject/NameTypes.h>           // FName
#include <UObject/ObjectMacros.h>        // EPropertyFlags, GENERATED_BODY, UCLASS, UENUM, UFUNCTION, UPROPERTY
#include <UObject/Package.h>
#include <UObject/Script.h>              // EFunctionFlags
#include <UObject/UObjectGlobals.h>      // GetTransientPackage, StaticLoadClass, StaticLoadObject
#include <UObject/UnrealType.h>          // EFieldIterationFlags

#include "SpCore/Assert.h"
#include "SpCore/Unreal.h"
#include "SpCore/UnrealClassRegistry.h"

#include "SpServices/EntryPointBinder.h"
#include "SpServices/Service.h"

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
        // Get engine subsystem
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_engine_subsystem_by_type",
            [this](std::string& class_name) -> uint64_t {
                return toUInt64(UnrealClassRegistry::getEngineSubsystemByType(class_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_engine_subsystem_by_class",
            [this](uint64_t& uclass) -> uint64_t {
                return toUInt64(Unreal::getEngineSubsystemByClass(toPtr<UClass>(uclass))); // UnrealClassRegistry not needed because Unreal::getEngineSubsystemBase(...) has no template parameters
            });

        //
        // Get editor subsystem, WITH_EDITOR implementations in SpServicesEditor/UnrealServiceEditor.h
        //

        #if !WITH_EDITOR // defined in an auto-generated header
            unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_editor_subsystem_by_type",
                [this](std::string& class_name) -> uint64_t {
                    return 0;
                });

            unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_editor_subsystem_by_class",
                [this](uint64_t& uclass) -> uint64_t {
                    return 0;
                });
        #endif

        //
        // Get subsystem
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_subsystem_by_type",
            [this](std::string& class_name) -> uint64_t {
                return toUInt64(UnrealClassRegistry::getSubsystemByType(class_name, getWorld()));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_subsystem_by_class",
            [this](std::string& class_name, uint64_t& uclass) -> uint64_t {
                return toUInt64(UnrealClassRegistry::getSubsystemByClass(class_name, getWorld(), toPtr<UClass>(uclass)));
            });

        //
        // Helper function for static structs and classes
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_static_struct",
            [this](std::string& struct_name) -> uint64_t {
                return toUInt64(UnrealClassRegistry::getStaticStruct(struct_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_static_structs",
            [this]() -> std::vector<uint64_t> {
                return toUInt64(Unreal::findStaticStructs());
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_static_structs_as_map",
            [this](bool& use_cpp_type_as_key) -> std::map<std::string, uint64_t> {
                return toUInt64(Unreal::findStaticStructsAsMap(use_cpp_type_as_key));
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

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_derived_classes",
            [this](uint64_t& uclass, bool& recursive) -> std::vector<uint64_t> {
                SP_ASSERT(uclass);
                return toUInt64(Unreal::getDerivedClasses(toPtr<UClass>(uclass), recursive));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_derived_classes_as_map",
            [this](uint64_t& uclass, bool& recursive, bool& use_cpp_type_as_key) -> std::map<std::string, uint64_t> {
                SP_ASSERT(uclass);
                return toUInt64(Unreal::getDerivedClassesAsMap(toPtr<UClass>(uclass), recursive, use_cpp_type_as_key));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_class",
            [this](uint64_t& uobject) -> uint64_t {
                SP_ASSERT(uobject);
                return toUInt64(toPtr<UObject>(uobject)->GetClass());
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_default_object",
            [this](uint64_t& uclass, bool& create_if_needed) -> uint64_t {
                SP_ASSERT(uclass);
                return toUInt64(toPtr<UClass>(uclass)->GetDefaultObject(create_if_needed));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_cpp_type_for_struct_as_string",
            [this](uint64_t& ustruct) -> std::string {
                return Unreal::getCppTypeAsString(toPtr<UStruct>(ustruct));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_cpp_type_for_property_as_string",
            [this](uint64_t& property) -> std::string {
                return Unreal::getCppTypeAsString(toPtr<FProperty>(property));
            });

        //
        // Get property metadata for classes
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_properties_for_struct",
            [this](uint64_t& ustruct, std::vector<std::string>& field_iteration_flag_strings) -> std::vector<uint64_t> {
                return toUInt64(Unreal::findProperties(
                    toPtr<UStruct>(ustruct),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_flag_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_properties_for_struct_by_flags_any",
            [this](uint64_t& ustruct, std::vector<std::string>& property_flag_strings, std::vector<std::string>& field_iteration_flag_strings) -> std::vector<uint64_t> {
                return toUInt64(Unreal::findPropertiesByFlagsAny(
                    toPtr<UStruct>(ustruct),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EPropertyFlags, ESpPropertyFlags>(property_flag_strings),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_flag_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_properties_for_struct_by_flags_all",
            [this](uint64_t& ustruct, std::vector<std::string>& property_flag_strings, std::vector<std::string>& field_iteration_flag_strings) -> std::vector<uint64_t> {
                return toUInt64(Unreal::findPropertiesByFlagsAll(
                    toPtr<UStruct>(ustruct),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EPropertyFlags, ESpPropertyFlags>(property_flag_strings),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_flag_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_properties_for_struct_as_map",
            [this](uint64_t& ustruct, std::vector<std::string>& field_iteration_flag_strings) -> std::map<std::string, uint64_t> {
                return toUInt64(Unreal::findPropertiesAsMap(
                    toPtr<UStruct>(ustruct),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_flag_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_properties_for_struct_by_flags_any_as_map",
            [this](uint64_t& ustruct, std::vector<std::string>& property_flag_strings, std::vector<std::string>& field_iteration_flag_strings) -> std::map<std::string, uint64_t> {
                return toUInt64(Unreal::findPropertiesByFlagsAnyAsMap(
                    toPtr<UStruct>(ustruct),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EPropertyFlags, ESpPropertyFlags>(property_flag_strings),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_flag_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_properties_for_struct_by_flags_all_as_map",
            [this](uint64_t& ustruct, std::vector<std::string>& property_flag_strings, std::vector<std::string>& field_iteration_flag_strings) -> std::map<std::string, uint64_t> {
                return toUInt64(Unreal::findPropertiesByFlagsAllAsMap(
                    toPtr<UStruct>(ustruct),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EPropertyFlags, ESpPropertyFlags>(property_flag_strings),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_flag_strings)));
            });

        //
        // Get property metadata for functions
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_properties_for_function",
            [this](uint64_t& ufunction, std::vector<std::string>& field_iteration_flag_strings) -> std::vector<uint64_t> {
                return toUInt64(Unreal::findProperties(
                    toPtr<UFunction>(ufunction),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_flag_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_properties_for_function_by_flags_any",
            [this](uint64_t& ufunction, std::vector<std::string>& property_flag_strings, std::vector<std::string>& field_iteration_flag_strings) -> std::vector<uint64_t> {
                return toUInt64(Unreal::findPropertiesByFlagsAny(
                    toPtr<UFunction>(ufunction),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EPropertyFlags, ESpPropertyFlags>(property_flag_strings),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_flag_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_properties_for_function_by_flags_all",
            [this](uint64_t& ufunction, std::vector<std::string>& property_flag_strings, std::vector<std::string>& field_iteration_flag_strings) -> std::vector<uint64_t> {
                return toUInt64(Unreal::findPropertiesByFlagsAll(
                    toPtr<UFunction>(ufunction),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EPropertyFlags, ESpPropertyFlags>(property_flag_strings),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_flag_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_properties_for_function_as_map",
            [this](uint64_t& ufunction, std::vector<std::string>& field_iteration_flag_strings) -> std::map<std::string, uint64_t> {
                return toUInt64(Unreal::findPropertiesAsMap(
                    toPtr<UFunction>(ufunction),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_flag_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_properties_for_function_by_flags_any_as_map",
            [this](uint64_t& ufunction, std::vector<std::string>& property_flag_strings, std::vector<std::string>& field_iteration_flag_strings) -> std::map<std::string, uint64_t> {
                return toUInt64(Unreal::findPropertiesByFlagsAnyAsMap(
                    toPtr<UFunction>(ufunction),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EPropertyFlags, ESpPropertyFlags>(property_flag_strings),
                    Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_flag_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_properties_for_function_by_flags_all_as_map",
            [this](uint64_t& ufunction, std::vector<std::string>& property_flag_strings, std::vector<std::string>& field_iteration_flag_strings) -> std::map<std::string, uint64_t> {
                return toUInt64(Unreal::findPropertiesByFlagsAllAsMap(
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

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_properties_as_string_from_object",
            [this](uint64_t& uobject) -> std::string {
                return Unreal::getObjectPropertiesAsString(toPtr<UObject>(uobject));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_properties_as_string_from_struct",
            [this](uint64_t& value_ptr, uint64_t& ustruct) -> std::string {
                return Unreal::getObjectPropertiesAsString(toPtr<void>(value_ptr), toPtr<UStruct>(ustruct));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "set_properties_from_string_for_object",
            [this](uint64_t& uobject, std::string& string) -> void {
                Unreal::setObjectPropertiesFromString(toPtr<UObject>(uobject), string);
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "set_properties_from_string_for_struct",
            [this](uint64_t& value_ptr, uint64_t& ustruct, std::string& string) -> void {
                Unreal::setObjectPropertiesFromString(toPtr<void>(value_ptr), toPtr<UStruct>(ustruct), string);
            });

        //
        // Find property
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_property_by_name_on_object",
            [this](uint64_t& uobject, std::string& property_name) -> SpPropertyDesc {
                return Unreal::findPropertyByName(toPtr<UObject>(uobject), property_name);
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_property_by_name_on_struct",
            [this](uint64_t& value_ptr, uint64_t& ustruct, std::string& property_name) -> SpPropertyDesc {
                return Unreal::findPropertyByName(toPtr<void>(value_ptr), toPtr<UStruct>(ustruct), property_name);
            });

        //
        // Get property value
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_property_value_as_string",
            [this](SpPropertyDesc& property_desc) -> std::string {
                return Unreal::getPropertyValueAsString(property_desc);
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "set_property_value_from_string",
            [this](SpPropertyDesc& property_desc, std::string& string) -> void {
                Unreal::setPropertyValueFromString(property_desc, string);
            });

        //
        // Find and call functions
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_functions",
            [this](uint64_t& uclass, std::vector<std::string>& field_iteration_strings) -> std::vector<uint64_t> {
                return toUInt64(
                    Unreal::findFunctions(
                        toPtr<UClass>(uclass),
                        Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_functions_by_flags_any",
            [this](uint64_t& uclass, std::vector<std::string>& function_flags, std::vector<std::string>& field_iteration_strings) -> std::vector<uint64_t> {
                return toUInt64(
                    Unreal::findFunctionsByFlagsAny(
                        toPtr<UClass>(uclass),
                        Unreal::getCombinedEnumFlagValueFromStringsAs<EFunctionFlags, ESpFunctionFlags>(function_flags),
                        Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_functions_by_flags_all",
            [this](uint64_t& uclass, std::vector<std::string>& function_flags, std::vector<std::string>& field_iteration_strings) -> std::vector<uint64_t> {
                return toUInt64(
                    Unreal::findFunctionsByFlagsAll(
                        toPtr<UClass>(uclass),
                        Unreal::getCombinedEnumFlagValueFromStringsAs<EFunctionFlags, ESpFunctionFlags>(function_flags),
                        Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_functions_as_map",
            [this](uint64_t& uclass, std::vector<std::string>& field_iteration_strings) -> std::map<std::string, uint64_t> {
                return toUInt64(
                    Unreal::findFunctionsAsMap(
                        toPtr<UClass>(uclass),
                        Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_functions_by_flags_any_as_map",
            [this](uint64_t& uclass, std::vector<std::string>& function_flags, std::vector<std::string>& field_iteration_strings) -> std::map<std::string, uint64_t> {
                return toUInt64(
                    Unreal::findFunctionsByFlagsAnyAsMap(
                        toPtr<UClass>(uclass),
                        Unreal::getCombinedEnumFlagValueFromStringsAs<EFunctionFlags, ESpFunctionFlags>(function_flags),
                        Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_functions_by_flags_all_as_map",
            [this](uint64_t& uclass, std::vector<std::string>& function_flags, std::vector<std::string>& field_iteration_strings) -> std::map<std::string, uint64_t> {
                return toUInt64(
                    Unreal::findFunctionsByFlagsAllAsMap(
                        toPtr<UClass>(uclass),
                        Unreal::getCombinedEnumFlagValueFromStringsAs<EFunctionFlags, ESpFunctionFlags>(function_flags),
                        Unreal::getCombinedEnumFlagValueFromStringsAs<EFieldIterationFlags, ESpFieldIterationFlags>(field_iteration_strings)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_function_by_name",
            [this](uint64_t& uclass, std::string& function_name, std::string& include_super_flag_string) -> uint64_t {
                return toUInt64(
                    Unreal::findFunctionByName(
                        toPtr<UClass>(uclass),
                        function_name,
                        Unreal::getEnumValueFromStringAs<EIncludeSuperFlag::Type, ESpIncludeSuperFlag>(include_super_flag_string)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "call_function",
            [this](uint64_t& uobject, uint64_t& ufunction, std::map<std::string, std::string>& args, std::string& world_context) -> std::map<std::string, std::string> {
                return Unreal::callFunction(getWorld(), toPtr<UObject>(uobject), toPtr<UFunction>(ufunction), args, world_context);
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_function_flags",
            [this](uint64_t& ufunction) -> std::vector<std::string> {
                SP_ASSERT(ufunction);
                return Unreal::getStringsFromCombinedEnumFlagValueAs<ESpFunctionFlags>(toPtr<UFunction>(ufunction)->FunctionFlags);
            });

        //
        // Find actors unconditionally and return an std::vector or std::map
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_actors",
            [this]() -> std::vector<uint64_t> {
                return toUInt64(Unreal::findActors(getWorld()));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_actors_as_map",
            [this]() -> std::map<std::string, uint64_t> {
                return toUInt64(Unreal::findActorsAsMap(getWorld()));
            });

        //
        // Get components unconditionally and return an std::vector or std::map
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_components",
            [this](uint64_t& actor) -> std::vector<uint64_t> {
                return toUInt64(Unreal::getComponents(toPtr<AActor>(actor)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_components_as_map",
            [this](uint64_t& actor) -> std::map<std::string, uint64_t> {
                return toUInt64(Unreal::getComponentsAsMap(toPtr<AActor>(actor)));
            });

        //
        // Get children components unconditionally and return an std::vector or std::map
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_children_components_from_actor",
            [this](uint64_t& parent, bool& include_all_descendants) -> std::vector<uint64_t> {
                return toUInt64(Unreal::getChildrenComponents(toPtr<AActor>(parent), include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_children_components_from_actor_as_map",
            [this](uint64_t& parent, bool& include_all_descendants) -> std::map<std::string, uint64_t> {
                return toUInt64(Unreal::getChildrenComponentsAsMap(toPtr<AActor>(parent), include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_children_components_from_scene_component",
            [this](uint64_t& parent, bool& include_all_descendants) -> std::vector<uint64_t> {
                return toUInt64(Unreal::getChildrenComponents(toPtr<USceneComponent>(parent), include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_children_components_from_scene_component_as_map",
            [this](uint64_t& parent, bool& include_all_descendants) -> std::map<std::string, uint64_t> {
                return toUInt64(Unreal::getChildrenComponentsAsMap(toPtr<USceneComponent>(parent), include_all_descendants));
            });

        //
        // Find actors conditionally and return an std::vector
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_actors_by_name",
            [this](std::string& class_name, std::vector<std::string>& actor_names, bool& return_null_if_not_found) -> std::vector<uint64_t> {
                return toUInt64(UnrealClassRegistry::findActorsByName(class_name, getWorld(), actor_names, return_null_if_not_found));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_actors_by_tag",
            [this](std::string& class_name, std::string& tag) -> std::vector<uint64_t> {
                return toUInt64(UnrealClassRegistry::findActorsByTag(class_name, getWorld(), tag));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_actors_by_tag_any",
            [this](std::string& class_name, std::vector<std::string>& tags) -> std::vector<uint64_t> {
                return toUInt64(UnrealClassRegistry::findActorsByTagAny(class_name, getWorld(), tags));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_actors_by_tag_all",
            [this](std::string& class_name, std::vector<std::string>& tags) -> std::vector<uint64_t> {
                return toUInt64(UnrealClassRegistry::findActorsByTagAll(class_name, getWorld(), tags));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_actors_by_type",
            [this](std::string& class_name) -> std::vector<uint64_t> {
                return toUInt64(UnrealClassRegistry::findActorsByType(class_name, getWorld()));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_actors_by_class",
            [this](uint64_t& uclass) -> std::vector<uint64_t> {
                return toUInt64(Unreal::findActorsByClass(getWorld(), toPtr<UClass>(uclass)));
            });

        //
        // Find actors conditionally and return an std::map
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_actors_by_name_as_map",
            [this](std::string& class_name, std::vector<std::string>& actor_names, bool& return_null_if_not_found) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealClassRegistry::findActorsByNameAsMap(class_name, getWorld(), actor_names, return_null_if_not_found));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_actors_by_tag_as_map",
            [this](std::string& class_name, std::string& tag) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealClassRegistry::findActorsByTagAsMap(class_name, getWorld(), tag));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_actors_by_tag_any_as_map",
            [this](std::string& class_name, std::vector<std::string>& tags) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealClassRegistry::findActorsByTagAnyAsMap(class_name, getWorld(), tags));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_actors_by_tag_all_as_map",
            [this](std::string& class_name, std::vector<std::string>& tags) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealClassRegistry::findActorsByTagAllAsMap(class_name, getWorld(), tags));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_actors_by_type_as_map",
            [this](std::string& class_name) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealClassRegistry::findActorsByTypeAsMap(class_name, getWorld()));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_actors_by_class_as_map",
            [this](uint64_t& uclass) -> std::map<std::string, uint64_t> {
                return toUInt64(Unreal::findActorsByClassAsMap(getWorld(), toPtr<UClass>(uclass)));
            });

        //
        // Find actor conditionally
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_actor_by_name",
            [this](std::string& class_name, std::string& actor_name) -> uint64_t {
                return toUInt64(UnrealClassRegistry::findActorByName(class_name, getWorld(), actor_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_actor_by_tag",
            [this](std::string& class_name, std::string& tag) -> uint64_t {
                return toUInt64(UnrealClassRegistry::findActorByTag(class_name, getWorld(), tag));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_actor_by_tag_any",
            [this](std::string& class_name, std::vector<std::string>& tags) -> uint64_t {
                return toUInt64(UnrealClassRegistry::findActorByTagAny(class_name, getWorld(), tags));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_actor_by_tag_all",
            [this](std::string& class_name, std::vector<std::string>& tags) -> uint64_t {
                return toUInt64(UnrealClassRegistry::findActorByTagAll(class_name, getWorld(), tags));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_actor_by_type",
            [this](std::string& class_name) -> uint64_t {
                return toUInt64(UnrealClassRegistry::findActorByType(class_name, getWorld()));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "find_actor_by_class",
            [this](uint64_t& uclass) -> uint64_t {
                return toUInt64(Unreal::findActorByClass(getWorld(), toPtr<UClass>(uclass)));
            });

        //
        // Get components conditionally and return an std::vector
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_components_by_name",
            [this](std::string& class_name, uint64_t& actor, std::vector<std::string>& component_names, bool& include_from_child_actors, bool& return_null_if_not_found) -> std::vector<uint64_t> {
                return toUInt64(UnrealClassRegistry::getComponentsByName(class_name, toPtr<AActor>(actor), component_names, include_from_child_actors, return_null_if_not_found));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_components_by_path",
            [this](std::string& class_name, uint64_t& actor, std::vector<std::string>& component_paths, bool& include_from_child_actors, bool& return_null_if_not_found) -> std::vector<uint64_t> {
                return toUInt64(UnrealClassRegistry::getComponentsByPath(class_name, toPtr<AActor>(actor), component_paths, include_from_child_actors, return_null_if_not_found));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_components_by_tag",
            [this](std::string& class_name, uint64_t& actor, std::string& tag, bool& include_from_child_actors) -> std::vector<uint64_t> {
                return toUInt64(UnrealClassRegistry::getComponentsByTag(class_name, toPtr<AActor>(actor), tag, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_components_by_tag_any",
            [this](std::string& class_name, uint64_t& actor, std::vector<std::string>& tags, bool& include_from_child_actors) -> std::vector<uint64_t> {
                return toUInt64(UnrealClassRegistry::getComponentsByTagAny(class_name, toPtr<AActor>(actor), tags, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_components_by_tag_all",
            [this](std::string& class_name, uint64_t& actor, std::vector<std::string>& tags, bool& include_from_child_actors) -> std::vector<uint64_t> {
                return toUInt64(UnrealClassRegistry::getComponentsByTagAll(class_name, toPtr<AActor>(actor), tags, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_components_by_type",
            [this](std::string& class_name, uint64_t& actor, bool& include_from_child_actors) -> std::vector<uint64_t> {
                return toUInt64(UnrealClassRegistry::getComponentsByType(class_name, toPtr<AActor>(actor), include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_components_by_class",
            [this](uint64_t& actor, uint64_t& uclass, bool& include_from_child_actors) -> std::vector<uint64_t> {
                return toUInt64(Unreal::getComponentsByClass(toPtr<AActor>(actor), toPtr<UClass>(uclass), include_from_child_actors));
            });

        //
        // Get components conditionally and return an std::map
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_components_by_name_as_map",
            [this](std::string& class_name, uint64_t& actor, std::vector<std::string>& component_names, bool& include_from_child_actors, bool& return_null_if_not_found) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealClassRegistry::getComponentsByNameAsMap(class_name, toPtr<AActor>(actor), component_names, include_from_child_actors, return_null_if_not_found));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_components_by_path_as_map",
            [this](std::string& class_name, uint64_t& actor, std::vector<std::string>& component_paths, bool& include_from_child_actors, bool& return_null_if_not_found) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealClassRegistry::getComponentsByPathAsMap(class_name, toPtr<AActor>(actor), component_paths, include_from_child_actors, return_null_if_not_found));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_components_by_tag_as_map",
            [this](std::string& class_name, uint64_t& actor, std::string& tag, bool& include_from_child_actors) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealClassRegistry::getComponentsByTagAsMap(class_name, toPtr<AActor>(actor), tag, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_components_by_tag_any_as_map",
            [this](std::string& class_name, uint64_t& actor, std::vector<std::string>& tags, bool& include_from_child_actors) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealClassRegistry::getComponentsByTagAnyAsMap(class_name, toPtr<AActor>(actor), tags, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_components_by_tag_all_as_map",
            [this](std::string& class_name, uint64_t& actor, std::vector<std::string>& tags, bool& include_from_child_actors) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealClassRegistry::getComponentsByTagAllAsMap(class_name, toPtr<AActor>(actor), tags, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_components_by_type_as_map",
            [this](std::string& class_name, uint64_t& actor, bool& include_from_child_actors) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealClassRegistry::getComponentsByTypeAsMap(class_name, toPtr<AActor>(actor), include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_components_by_class_as_map",
            [this](uint64_t& actor, uint64_t& uclass, bool& include_from_child_actors) -> std::map<std::string, uint64_t> {
                return toUInt64(Unreal::getComponentsByClassAsMap(toPtr<AActor>(actor), toPtr<UClass>(uclass), include_from_child_actors));
            });

        //
        // Get component conditionally
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_component_by_name",
            [this](std::string& class_name, uint64_t& actor, std::string& component_name, bool& include_from_child_actors) -> uint64_t {
                return toUInt64(UnrealClassRegistry::getComponentByName(class_name, toPtr<AActor>(actor), component_name, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_component_by_path",
            [this](std::string& class_name, uint64_t& actor, std::string& component_path, bool& include_from_child_actors) -> uint64_t {
                return toUInt64(UnrealClassRegistry::getComponentByPath(class_name, toPtr<AActor>(actor), component_path, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_component_by_tag",
            [this](std::string& class_name, uint64_t& actor, std::string& tag, bool& include_from_child_actors) -> uint64_t {
                return toUInt64(UnrealClassRegistry::getComponentByTag(class_name, toPtr<AActor>(actor), tag, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_component_by_tag_any",
            [this](std::string& class_name, uint64_t& actor, std::vector<std::string>& tags, bool& include_from_child_actors) -> uint64_t {
                return toUInt64(UnrealClassRegistry::getComponentByTagAny(class_name, toPtr<AActor>(actor), tags, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_component_by_tag_all",
            [this](std::string& class_name, uint64_t& actor, std::vector<std::string>& tags, bool& include_from_child_actors) -> uint64_t {
                return toUInt64(UnrealClassRegistry::getComponentByTagAll(class_name, toPtr<AActor>(actor), tags, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_component_by_type",
            [this](std::string& class_name, uint64_t& actor, bool& include_from_child_actors) -> uint64_t {
                return toUInt64(UnrealClassRegistry::getComponentByType(class_name, toPtr<AActor>(actor), include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_component_by_class",
            [this](uint64_t& actor, uint64_t& uclass, bool& include_from_child_actors) -> uint64_t {
                return toUInt64(Unreal::getComponentByClass(toPtr<AActor>(actor), toPtr<UClass>(uclass), include_from_child_actors));
            });

        //
        // Get children components conditionally from an actor and return an std::vector
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_children_components_by_name_from_actor",
            [this](std::string& class_name, uint64_t& parent, std::vector<std::string>& children_component_names, bool& include_all_descendants, bool& return_null_if_not_found) -> std::vector<uint64_t> {
                return toUInt64(UnrealClassRegistry::getChildrenComponentsByName(class_name, toPtr<AActor>(parent), children_component_names, include_all_descendants, return_null_if_not_found));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_children_components_by_tag_from_actor",
            [this](std::string& class_name, uint64_t& parent, std::string& tag, bool& include_all_descendants) -> std::vector<uint64_t> {
                return toUInt64(UnrealClassRegistry::getChildrenComponentsByTag(class_name, toPtr<AActor>(parent), tag, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_children_components_by_tag_any_from_actor",
            [this](std::string& class_name, uint64_t& parent, std::vector<std::string>& tags, bool& include_all_descendants) -> std::vector<uint64_t> {
                return toUInt64(UnrealClassRegistry::getChildrenComponentsByTagAny(class_name, toPtr<AActor>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_children_components_by_tag_all_from_actor",
            [this](std::string& class_name, uint64_t& parent, std::vector<std::string>& tags, bool& include_all_descendants) -> std::vector<uint64_t> {
                return toUInt64(UnrealClassRegistry::getChildrenComponentsByTagAll(class_name, toPtr<AActor>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_children_components_by_type_from_actor",
            [this](std::string& class_name, uint64_t& parent, bool& include_all_descendants) -> std::vector<uint64_t> {
                return toUInt64(UnrealClassRegistry::getChildrenComponentsByType(class_name, toPtr<AActor>(parent), include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_children_components_by_class_from_actor",
            [this](uint64_t& parent, uint64_t& uclass, bool& include_all_descendants) -> std::vector<uint64_t> {
                return toUInt64(Unreal::getChildrenComponentsByClass(toPtr<AActor>(parent), toPtr<UClass>(uclass), include_all_descendants));
            });

        //
        // Get children components conditionally from an actor and return an std::map
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_children_components_by_name_as_map_from_actor",
            [this](std::string& class_name, uint64_t& parent, std::vector<std::string>& children_component_names, bool& include_all_descendants, bool& return_null_if_not_found) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealClassRegistry::getChildrenComponentsByNameAsMap(class_name, toPtr<AActor>(parent), children_component_names, include_all_descendants, return_null_if_not_found));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_children_components_by_tag_as_map_from_actor",
            [this](std::string& class_name, uint64_t& parent, std::string& tag, bool& include_all_descendants) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealClassRegistry::getChildrenComponentsByTagAsMap(class_name, toPtr<AActor>(parent), tag, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_children_components_by_tag_any_as_map_from_actor",
            [this](std::string& class_name, uint64_t& parent, std::vector<std::string>& tags, bool& include_all_descendants) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealClassRegistry::getChildrenComponentsByTagAnyAsMap(class_name, toPtr<AActor>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_children_components_by_tag_all_as_map_from_actor",
            [this](std::string& class_name, uint64_t& parent, std::vector<std::string>& tags, bool& include_all_descendants) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealClassRegistry::getChildrenComponentsByTagAllAsMap(class_name, toPtr<AActor>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_children_components_by_type_as_map_from_actor",
            [this](std::string& class_name, uint64_t& parent, bool& include_all_descendants) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealClassRegistry::getChildrenComponentsByTypeAsMap(class_name, toPtr<AActor>(parent), include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_children_components_by_class_as_map_from_actor",
            [this](uint64_t& parent, uint64_t& uclass, bool& include_all_descendants) -> std::map<std::string, uint64_t> {
                return toUInt64(Unreal::getChildrenComponentsByClassAsMap(toPtr<AActor>(parent), toPtr<UClass>(uclass), include_all_descendants));
            });

        //
        // Get child component conditionally from an actor
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_child_component_by_name_from_actor",
            [this](std::string& class_name, uint64_t& parent, std::string& child_component_name, bool& include_all_descendants) -> uint64_t {
                return toUInt64(UnrealClassRegistry::getChildComponentByName(class_name, toPtr<AActor>(parent), child_component_name, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_child_component_by_tag_from_actor",
            [this](std::string& class_name, uint64_t& parent, std::string& tag, bool& include_all_descendants) -> uint64_t {
                return toUInt64(UnrealClassRegistry::getChildComponentByTag(class_name, toPtr<AActor>(parent), tag, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_child_component_by_tag_any_from_actor",
            [this](std::string& class_name, uint64_t& parent, std::vector<std::string>& tags, bool& include_all_descendants) -> uint64_t {
                return toUInt64(UnrealClassRegistry::getChildComponentByTagAny(class_name, toPtr<AActor>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_child_component_by_tag_all_from_actor",
            [this](std::string& class_name, uint64_t& parent, std::vector<std::string>& tags, bool& include_all_descendants) -> uint64_t {
                return toUInt64(UnrealClassRegistry::getChildComponentByTagAll(class_name, toPtr<AActor>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_child_component_by_type_from_actor",
            [this](std::string& class_name, uint64_t& parent, bool& include_all_descendants) -> uint64_t {
                return toUInt64(UnrealClassRegistry::getChildComponentByType(class_name, toPtr<AActor>(parent), include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_child_component_by_class_from_actor",
            [this](uint64_t& parent, uint64_t& uclass, bool& include_all_descendants) -> uint64_t {
                return toUInt64(Unreal::getChildComponentByClass(toPtr<AActor>(parent), toPtr<UClass>(uclass), include_all_descendants));
            });

        //
        // Get children components conditionally from a scene component and return an std::vector
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_children_components_by_name_from_scene_component",
            [this](std::string& class_name, uint64_t& parent, std::vector<std::string>& children_component_names, bool& include_all_descendants, bool& return_null_if_not_found) -> std::vector<uint64_t> {
                return toUInt64(UnrealClassRegistry::getChildrenComponentsByName(class_name, toPtr<USceneComponent>(parent), children_component_names, include_all_descendants, return_null_if_not_found));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_children_components_by_tag_from_scene_component",
            [this](std::string& class_name, uint64_t& parent, std::string& tag, bool& include_all_descendants) -> std::vector<uint64_t> {
                return toUInt64(UnrealClassRegistry::getChildrenComponentsByTag(class_name, toPtr<USceneComponent>(parent), tag, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_children_components_by_tag_any_from_scene_component",
            [this](std::string& class_name, uint64_t& parent, std::vector<std::string>& tags, bool& include_all_descendants) -> std::vector<uint64_t> {
                return toUInt64(UnrealClassRegistry::getChildrenComponentsByTagAny(class_name, toPtr<USceneComponent>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_children_components_by_tag_all_from_scene_component",
            [this](std::string& class_name, uint64_t& parent, std::vector<std::string>& tags, bool& include_all_descendants) -> std::vector<uint64_t> {
                return toUInt64(UnrealClassRegistry::getChildrenComponentsByTagAll(class_name, toPtr<USceneComponent>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_children_components_by_type_from_scene_component",
            [this](std::string& class_name, uint64_t& parent, bool& include_all_descendants) -> std::vector<uint64_t> {
                return toUInt64(UnrealClassRegistry::getChildrenComponentsByType(class_name, toPtr<USceneComponent>(parent), include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_children_components_by_class_from_scene_component",
            [this](uint64_t& parent, uint64_t& uclass, bool& include_all_descendants) -> std::vector<uint64_t> {
                return toUInt64(Unreal::getChildrenComponentsByClass(toPtr<USceneComponent>(parent), toPtr<UClass>(uclass), include_all_descendants));
            });

        //
        // Get children components conditionally from a scene component and return an std::map
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_children_components_by_name_as_map_from_scene_component",
            [this](std::string& class_name, uint64_t& parent, std::vector<std::string>& children_component_names, bool& include_all_descendants, bool& return_null_if_not_found) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealClassRegistry::getChildrenComponentsByNameAsMap(class_name, toPtr<USceneComponent>(parent), children_component_names, include_all_descendants, return_null_if_not_found));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_children_components_by_tag_as_map_from_scene_component",
            [this](std::string& class_name, uint64_t& parent, std::string& tag, bool& include_all_descendants) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealClassRegistry::getChildrenComponentsByTagAsMap(class_name, toPtr<USceneComponent>(parent), tag, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_children_components_by_tag_any_as_map_from_scene_component",
            [this](std::string& class_name, uint64_t& parent, std::vector<std::string>& tags, bool& include_all_descendants) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealClassRegistry::getChildrenComponentsByTagAnyAsMap(class_name, toPtr<USceneComponent>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_children_components_by_tag_all_as_map_from_scene_component",
            [this](std::string& class_name, uint64_t& parent, std::vector<std::string>& tags, bool& include_all_descendants) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealClassRegistry::getChildrenComponentsByTagAllAsMap(class_name, toPtr<USceneComponent>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_children_components_by_type_as_map_from_scene_component",
            [this](std::string& class_name, uint64_t& parent, bool& include_all_descendants) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealClassRegistry::getChildrenComponentsByTypeAsMap(class_name, toPtr<USceneComponent>(parent), include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_children_components_by_class_as_map_from_scene_component",
            [this](uint64_t& parent, uint64_t& uclass, bool& include_all_descendants) -> std::map<std::string, uint64_t> {
                return toUInt64(Unreal::getChildrenComponentsByClassAsMap(toPtr<USceneComponent>(parent), toPtr<UClass>(uclass), include_all_descendants));
            });

        //
        // Get child component conditionally from a scene component
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_child_component_by_name_from_scene_component",
            [this](std::string& class_name, uint64_t& parent, std::string& child_component_name, bool& include_all_descendants) -> uint64_t {
                return toUInt64(UnrealClassRegistry::getChildComponentByName(class_name, toPtr<USceneComponent>(parent), child_component_name, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_child_component_by_tag_from_scene_component",
            [this](std::string& class_name, uint64_t& parent, std::string& tag, bool& include_all_descendants) -> uint64_t {
                return toUInt64(UnrealClassRegistry::getChildComponentByTag(class_name, toPtr<USceneComponent>(parent), tag, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_child_component_by_tag_any_from_scene_component",
            [this](std::string& class_name, uint64_t& parent, std::vector<std::string>& tags, bool& include_all_descendants) -> uint64_t {
                return toUInt64(UnrealClassRegistry::getChildComponentByTagAny(class_name, toPtr<USceneComponent>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_child_component_by_tag_all_from_scene_component",
            [this](std::string& class_name, uint64_t& parent, std::vector<std::string>& tags, bool& include_all_descendants) -> uint64_t {
                return toUInt64(UnrealClassRegistry::getChildComponentByTagAll(class_name, toPtr<USceneComponent>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_child_component_by_type_from_scene_component",
            [this](std::string& class_name, uint64_t& parent, bool& include_all_descendants) -> uint64_t {
                return toUInt64(UnrealClassRegistry::getChildComponentByType(class_name, toPtr<USceneComponent>(parent), include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_child_component_by_class_from_scene_component",
            [this](uint64_t& parent, uint64_t& uclass, bool& include_all_descendants) -> uint64_t {
                return toUInt64(Unreal::getChildComponentByClass(toPtr<USceneComponent>(parent), toPtr<UClass>(uclass), include_all_descendants));
            });

        //
        // Spawn actor
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "spawn_actor",
            [this](std::string& class_name, std::string& location_string, std::string& rotation_string, std::string& spawn_parameters_string, std::vector<std::string>& object_flag_strings) -> uint64_t {

                FVector location;
                FRotator rotation;
                FSpActorSpawnParameters sp_actor_spawn_parameters;

                Unreal::setObjectPropertiesFromString(&location, UnrealClassRegistry::getStaticStruct<FVector>(), location_string);
                Unreal::setObjectPropertiesFromString(&rotation, UnrealClassRegistry::getStaticStruct<FRotator>(), rotation_string);
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

                return toUInt64(UnrealClassRegistry::spawnActor(class_name, getWorld(), location, rotation, actor_spawn_parameters));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "spawn_actor_from_class",
            [this](uint64_t& uclass, std::string& location_string, std::string& rotation_string, std::string& spawn_parameters_string, std::vector<std::string>& object_flag_strings) -> uint64_t {

                FVector location;
                FRotator rotation;
                FSpActorSpawnParameters sp_actor_spawn_parameters;

                Unreal::setObjectPropertiesFromString(&location, UnrealClassRegistry::getStaticStruct<FVector>(), location_string);
                Unreal::setObjectPropertiesFromString(&rotation, UnrealClassRegistry::getStaticStruct<FRotator>(), rotation_string);
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

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "destroy_actor",
            [this](uint64_t& actor, bool& net_force, bool& should_modify_level) -> bool {
                SP_ASSERT(actor);
                return toPtr<AActor>(actor)->Destroy(net_force, should_modify_level);
            });

        //
        // Create component
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "create_component_outside_owner_constructor", 
            [this](std::string& class_name, uint64_t& owner, std::string& component_name) -> uint64_t {
                return toUInt64(UnrealClassRegistry::createComponentOutsideOwnerConstructor(class_name, toPtr<AActor>(owner), component_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "create_scene_component_outside_owner_constructor_from_actor",
            [this](std::string& class_name, uint64_t& owner, std::string& scene_component_name) -> uint64_t {
                return toUInt64(UnrealClassRegistry::createSceneComponentOutsideOwnerConstructor(class_name, toPtr<AActor>(owner), scene_component_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "create_scene_component_outside_owner_constructor_from_object",
            [this](std::string& class_name, uint64_t& owner, uint64_t& parent, std::string& scene_component_name) -> uint64_t {
                return toUInt64(UnrealClassRegistry::createSceneComponentOutsideOwnerConstructor(class_name, toPtr<UObject>(owner), toPtr<USceneComponent>(parent), scene_component_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "create_scene_component_outside_owner_constructor_from_component",
            [this](std::string& class_name, uint64_t& owner, std::string& scene_component_name) -> uint64_t {
                return toUInt64(UnrealClassRegistry::createSceneComponentOutsideOwnerConstructor(class_name, toPtr<USceneComponent>(owner), scene_component_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "create_component_outside_owner_constructor_by_class",
            [this](uint64_t& component_class, uint64_t& owner, std::string& component_name) -> uint64_t {
                return toUInt64(Unreal::createComponentOutsideOwnerConstructorByClass(toPtr<UClass>(component_class), toPtr<AActor>(owner), component_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "create_scene_component_outside_owner_constructor_by_class_from_actor",
            [this](uint64_t& scene_component_class, uint64_t& owner, std::string& scene_component_name) -> uint64_t {
                return toUInt64(Unreal::createSceneComponentOutsideOwnerConstructorByClass(toPtr<UClass>(scene_component_class), toPtr<AActor>(owner), scene_component_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "create_scene_component_outside_owner_constructor_by_class_from_object",
            [this](uint64_t& scene_component_class, uint64_t& owner, uint64_t& parent, std::string& scene_component_name) -> uint64_t {
                return toUInt64(Unreal::createSceneComponentOutsideOwnerConstructorByClass(toPtr<UClass>(scene_component_class), toPtr<UObject>(owner), toPtr<USceneComponent>(parent), scene_component_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "create_scene_component_outside_owner_constructor_by_class_from_component",
            [this](uint64_t& scene_component_class, uint64_t& owner, std::string& scene_component_name) -> uint64_t {
                return toUInt64(Unreal::createSceneComponentOutsideOwnerConstructorByClass(toPtr<UClass>(scene_component_class), toPtr<USceneComponent>(owner), scene_component_name));
            });

        //
        // Destroy component
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "destroy_component_outside_owner_constructor",
            [this](uint64_t& component, bool& promote_children) -> void {
                SP_ASSERT(component);
                Unreal::destroyComponentOutsideOwnerConstructor(toPtr<UActorComponent>(component), promote_children);
            });

        //
        // Create new object
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "new_object",
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
                    UnrealClassRegistry::newObject(
                        class_name,
                        outer_ptr,
                        fname,
                        Unreal::getCombinedEnumFlagValueFromStringsAs<EObjectFlags, ESpObjectFlags>(object_flag_strings),
                        toPtr<UObject>(uobject_template),
                        copy_transients_from_class_defaults,
                        toPtr<FObjectInstancingGraph>(in_instance_graph),
                        toPtr<UPackage>(external_package)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "new_object_from_class",
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

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "load_object",
            [this](
                std::string& class_name,
                uint64_t& outer,
                std::string& name,
                std::string& filename,
                std::vector<std::string>& load_flag_strings,
                uint64_t& sandbox,
                uint64_t& instancing_context) -> uint64_t {

                return toUInt64(
                    UnrealClassRegistry::loadObject(
                        class_name,
                        toPtr<UObject>(outer),
                        Unreal::toTCharPtr(name),
                        Unreal::toTCharPtr(filename),
                        Unreal::getCombinedEnumFlagValueFromStringsAs<ELoadFlags, ESpLoadFlags>(load_flag_strings),
                        toPtr<UPackageMap>(sandbox),
                        toPtr<FLinkerInstancingContext>(instancing_context)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "load_class",
            [this](
                std::string& class_name,
                uint64_t& outer,
                std::string& name,
                std::string& filename,
                std::vector<std::string>& load_flag_strings,
                uint64_t& sandbox) -> uint64_t {

                return toUInt64(
                    UnrealClassRegistry::loadClass(
                        class_name,
                        toPtr<UObject>(outer),
                        Unreal::toTCharPtr(name),
                        Unreal::toTCharPtr(filename),
                        Unreal::getCombinedEnumFlagValueFromStringsAs<ELoadFlags, ESpLoadFlags>(load_flag_strings),
                        toPtr<UPackageMap>(sandbox)));
            });

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
                        toPtr<UClass>(in_outer),
                        Unreal::toTCharPtr(name),
                        Unreal::toTCharPtr(filename),
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
                        toPtr<UClass>(in_outer),
                        Unreal::toTCharPtr(name),
                        Unreal::toTCharPtr(filename),
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
            [this](uint64_t& actor) -> bool { return Unreal::hasStableName(toPtr<AActor>(actor)); });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_stable_name_for_actor",
            [this](uint64_t& actor) -> std::string { return Unreal::getStableName(toPtr<AActor>(actor)); });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "try_get_stable_name_for_actor",
            [this](uint64_t& actor) -> std::string { return Unreal::tryGetStableName(toPtr<AActor>(actor)); });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_stable_name_for_component",
            [this](uint64_t& actor_component, bool& include_actor_name) -> std::string { return Unreal::getStableName(toPtr<UActorComponent>(actor_component), include_actor_name); });

        //
        // Get actor and component tags
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_actor_tags",
            [this](uint64_t& actor) -> std::vector<std::string> { return Unreal::getTags(toPtr<AActor>(actor)); });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_component_tags",
            [this](uint64_t& component) -> std::vector<std::string> { return Unreal::getTags(toPtr<UActorComponent>(component)); });
    }
};
