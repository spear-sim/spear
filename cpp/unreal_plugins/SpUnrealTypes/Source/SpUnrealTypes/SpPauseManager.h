//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Engine/EngineBaseTypes.h> // ETickingGroup
#include <Engine/World.h>
#include <GameFramework/Actor.h>
#include <Kismet/GameplayStatics.h>
#include <UObject/ObjectMacros.h> // GENERATED_BODY, UCLASS, UFUNCTION, UPROPERTY

#include "SpCore/Log.h"

#include "SpPauseManager.generated.h"

UCLASS(ClassGroup="SPEAR", HideCategories=(Actor, Collision, Cooking, DataLayers, HLOD, Input, LevelInstance, Navigation, Networking, Physics, Rendering, Replication, WorldPartition))
class ASpPauseManager : public AActor
{
    GENERATED_BODY()
public: 
    ASpPauseManager()
    {
        SP_LOG_CURRENT_FUNCTION();

        PrimaryActorTick.bCanEverTick = true;
        PrimaryActorTick.bTickEvenWhenPaused = true; // we want to update bIsGamePaused state when paused
        PrimaryActorTick.TickGroup = ETickingGroup::TG_PrePhysics;
    }

    ~ASpPauseManager() override
    {
        SP_LOG_CURRENT_FUNCTION();
    }

    // AActor interface
    void Tick(float delta_time) override
    {
        AActor::Tick(delta_time);

        UWorld* world = GetWorld();
        bIsGamePaused = UGameplayStatics::IsGamePaused(world);
    }

private:

    // Calling UGameplayStatics::SetGamePaused() doesn't synchronize with the default play/pause button in
    // the editor. So we provide a custom button and a custom read-only property to update and visualize the
    // engine-level (as opposed to the editor-level) pause state of the game. All of this functionality is
    // implemented by interacting directly with UGameplayStatics, and is therefore guaranteed to be correctly
    // synchronized with the engine-level state.

    UFUNCTION(BlueprintCallable, CallInEditor, Category="SPEAR")
    void ToggleGamePaused()
    {
        UWorld* world = GetWorld();
        UGameplayStatics::SetGamePaused(world, !UGameplayStatics::IsGamePaused(world));
    }

    UPROPERTY(VisibleAnywhere, Category="SPEAR");
    bool bIsGamePaused = false;
};
