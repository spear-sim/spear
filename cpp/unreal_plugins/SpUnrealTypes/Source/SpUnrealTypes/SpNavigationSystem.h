//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <AI/Navigation/NavigationTypes.h> // FNavDataConfig
#include <HAL/Platform.h>                  // int32, uint8
#include <Kismet/BlueprintFunctionLibrary.h>
#include <NavigationSystem.h>              // ELockRemovalRebuildAction, ENavigationBuildLock
#include <UObject/NameTypes.h>             // FName

#include "SpCore/Assert.h"
#include "SpCore/Unreal.h"

#include "SpNavigationSystem.generated.h"

class ANavigationData;

// UENUM(Flags) decorator is required to obtain an "A | B | C" string representation from a value
UENUM(Flags)
enum class ESpNavigationBuildLock : uint8
{
    NoUpdateInEditor = Unreal::getConstEnumValue(ENavigationBuildLock::NoUpdateInEditor),
    NoUpdateInPIE    = Unreal::getConstEnumValue(ENavigationBuildLock::NoUpdateInPIE),
    InitialLock      = Unreal::getConstEnumValue(ENavigationBuildLock::InitialLock),
    Custom           = Unreal::getConstEnumValue(ENavigationBuildLock::Custom),
    All              = 0xff
};
ENUM_CLASS_FLAGS(ESpNavigationBuildLock); // required if combining values using bitwise operations

UENUM()
enum class ESpLockRemovalRebuildAction
{
    Rebuild              = Unreal::getConstEnumValue(UNavigationSystemV1::ELockRemovalRebuildAction::Rebuild),
    RebuildIfNotInEditor = Unreal::getConstEnumValue(UNavigationSystemV1::ELockRemovalRebuildAction::RebuildIfNotInEditor),
    NoRebuild            = Unreal::getConstEnumValue(UNavigationSystemV1::ELockRemovalRebuildAction::NoRebuild)
};

UCLASS()
class USpNavigationSystemV1 : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()
public:

    //
    // Update lock state
    //

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static void AddNavigationBuildLock(UNavigationSystemV1* NavigationSystem, ESpNavigationBuildLock Flags)
    {
        SP_ASSERT(NavigationSystem);
        uint8 flags = Unreal::getEnumValueAs<uint8>(Flags);
        NavigationSystem->AddNavigationBuildLock(flags);
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static void RemoveNavigationBuildLock(UNavigationSystemV1* NavigationSystem, ESpNavigationBuildLock Flags, ESpLockRemovalRebuildAction RebuildAction = ESpLockRemovalRebuildAction::Rebuild)
    {
        SP_ASSERT(NavigationSystem);
        uint8 flags = Unreal::getEnumValueAs<uint8>(Flags);
        UNavigationSystemV1::ELockRemovalRebuildAction rebuild_action = Unreal::getEnumValueAs<UNavigationSystemV1::ELockRemovalRebuildAction>(RebuildAction);
        NavigationSystem->RemoveNavigationBuildLock(flags, rebuild_action);
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static void SetNavigationOctreeLock(UNavigationSystemV1* NavigationSystem, bool bLock)
    {
        SP_ASSERT(NavigationSystem);
        NavigationSystem->SetNavigationOctreeLock(bLock);
    }

    //
    // Query lock state
    //

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static bool IsNavigationBuildingLocked(UNavigationSystemV1* NavigationSystem, ESpNavigationBuildLock Flags = ESpNavigationBuildLock::All)
    {
        SP_ASSERT(NavigationSystem);
        uint8 flags = Unreal::getEnumValueAs<uint8>(Flags);
        return NavigationSystem->IsNavigationBuildingLocked(flags);
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static bool IsNavigationBuildingPermanentlyLocked(UNavigationSystemV1* NavigationSystem)
    {
        SP_ASSERT(NavigationSystem);
        return NavigationSystem->IsNavigationBuildingPermanentlyLocked();
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static bool IsNavigationOctreeLocked(UNavigationSystemV1* NavigationSystem)
    {
        SP_ASSERT(NavigationSystem);
        return NavigationSystem->IsNavigationOctreeLocked();
    }

    //
    // Build
    //

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static void Build(UNavigationSystemV1* NavigationSystem)
    {
        SP_ASSERT(NavigationSystem);
        NavigationSystem->Build();
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static void CancelBuild(UNavigationSystemV1* NavigationSystem)
    {
        SP_ASSERT(NavigationSystem);
        NavigationSystem->CancelBuild();
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static bool IsNavigationBuildInProgress(UNavigationSystemV1* NavigationSystem)
    {
        SP_ASSERT(NavigationSystem);
        return NavigationSystem->IsNavigationBuildInProgress();
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static int32 GetNumRemainingBuildTasks(UNavigationSystemV1* NavigationSystem)
    {
        SP_ASSERT(NavigationSystem);
        return NavigationSystem->GetNumRemainingBuildTasks();
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static int32 GetNumRunningBuildTasks(UNavigationSystemV1* NavigationSystem)
    {
        SP_ASSERT(NavigationSystem);
        return NavigationSystem->GetNumRunningBuildTasks();
    }

    //
    // Get NavDataConfig objects
    //

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static FNavDataConfig GetDefaultSupportedAgentConfig(UNavigationSystemV1* NavigationSystem)
    {
        SP_ASSERT(NavigationSystem);
        return NavigationSystem->GetDefaultSupportedAgentConfig();
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static TArray<FNavDataConfig> GetSupportedAgents(UNavigationSystemV1* NavigationSystem)
    {
        SP_ASSERT(NavigationSystem);
        return NavigationSystem->GetSupportedAgents();
    }

    //
    // Get ANavigationData objects
    //

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static ANavigationData* GetNavDataForAgentName(UNavigationSystemV1* NavigationSystem, FName AgentName)
    {
        SP_ASSERT(NavigationSystem);
        return NavigationSystem->GetNavDataForAgentName(AgentName);
    }
};
