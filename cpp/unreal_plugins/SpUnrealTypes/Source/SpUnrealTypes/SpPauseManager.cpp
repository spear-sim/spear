//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpUnrealTypes/SpPauseManager.h"

#include <Engine/EngineBaseTypes.h> // ETickingGroup
#include <Engine/World.h>
#include <GameFramework/Actor.h>
#include <Kismet/GameplayStatics.h>

#include "SpCore/Log.h"

ASpPauseManager::ASpPauseManager()
{
    SP_LOG_CURRENT_FUNCTION();

    PrimaryActorTick.bCanEverTick = true;
    PrimaryActorTick.bTickEvenWhenPaused = true; // we want to update bIsGamePaused state when paused
    PrimaryActorTick.TickGroup = ETickingGroup::TG_PrePhysics;
}

ASpPauseManager::~ASpPauseManager()
{
    SP_LOG_CURRENT_FUNCTION();
}

void ASpPauseManager::Tick(float delta_time)
{
    AActor::Tick(delta_time);

    UWorld* world = GetWorld();
    bIsGamePaused = UGameplayStatics::IsGamePaused(world);
}

void ASpPauseManager::ToggleGamePaused()
{
    UWorld* world = GetWorld();
    UGameplayStatics::SetGamePaused(world, !UGameplayStatics::IsGamePaused(world));
}
