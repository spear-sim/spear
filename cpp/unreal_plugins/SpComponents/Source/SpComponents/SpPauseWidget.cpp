//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpComponents/SpPauseWidget.h"

#include <Engine/EngineBaseTypes.h> // ETickingGroup
#include <GameFramework/Actor.h>
#include <Kismet/GameplayStatics.h>

#include "SpCore/Log.h"

ASpPauseWidget::ASpPauseWidget()
{
    SP_LOG_CURRENT_FUNCTION();

    PrimaryActorTick.bCanEverTick = true;
    PrimaryActorTick.bTickEvenWhenPaused = true; // we want to update bIsGamePaused state when paused
    PrimaryActorTick.TickGroup = ETickingGroup::TG_PrePhysics;
}

ASpPauseWidget::~ASpPauseWidget()
{
    SP_LOG_CURRENT_FUNCTION();
}

void ASpPauseWidget::Tick(float delta_time)
{
    AActor::Tick(delta_time);
    bIsGamePaused = UGameplayStatics::IsGamePaused(GetWorld());
}

void ASpPauseWidget::ToggleGamePaused()
{
    UGameplayStatics::SetGamePaused(GetWorld(), !UGameplayStatics::IsGamePaused(GetWorld()));
}
