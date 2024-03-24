//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpearSim/SpSpectatorPawn.h"

#include <string>

#include <Engine/EngineBaseTypes.h> // ETickingGroup
#include <GameFramework/SpectatorPawn.h>
#include <GameFramework/SpectatorPawnMovement.h>
#include <GenericPlatform/GenericPlatformMisc.h>
#include <Kismet/GameplayStatics.h>

#include "SpCore/Log.h"
#include "SpCore/StableNameComponent.h"
#include "SpCore/Unreal.h"

ASpSpectatorPawn::ASpSpectatorPawn()
{
    SP_LOG_CURRENT_FUNCTION();

    // Disable collision so the user can fly through walls by default.
    SetActorEnableCollision(false);

    // Need to enable these, otherwise mouse movements will not work when the game is paused in the editor.
    bUseControllerRotationPitch = true;
    bUseControllerRotationYaw = true;
    bUseControllerRotationRoll = true;

    // Need to set this to be true because the logic in our Tick(...) function depends on being called even when the game is paused.
    PrimaryActorTick.bCanEverTick = true;
    PrimaryActorTick.bTickEvenWhenPaused = true;
    PrimaryActorTick.TickGroup = ETickingGroup::TG_PrePhysics;

    // UStableNameComponent
    StableNameComponent = Unreal::createComponentInsideOwnerConstructor<UStableNameComponent>(this, "stable_name");
    SP_ASSERT(StableNameComponent);

    // USpectatorPawnMovement
    SpectatorPawnMovement = dynamic_cast<USpectatorPawnMovement*>(GetMovementComponent());
    SP_ASSERT(SpectatorPawnMovement);

    // Need to set this to true, otherwise keyboard input will not be processed when attempting to move the camera when the game is paused.
    SpectatorPawnMovement->PrimaryComponentTick.bCanEverTick = true;
    SpectatorPawnMovement->PrimaryComponentTick.bTickEvenWhenPaused = true;
    SpectatorPawnMovement->PrimaryComponentTick.TickGroup = ETickingGroup::TG_PrePhysics;
}

ASpSpectatorPawn::~ASpSpectatorPawn()
{
    SP_LOG_CURRENT_FUNCTION();

    SP_ASSERT(SpectatorPawnMovement);
    SpectatorPawnMovement = nullptr;

    SP_ASSERT(StableNameComponent);
    StableNameComponent = nullptr;
}

void ASpSpectatorPawn::Tick(float delta_time)
{
    ASpectatorPawn::Tick(delta_time);

    // When in standalone mode, we need to adjust the speed of the camera when the game is paused, otherwise it will be too jittery.
    #if !WITH_EDITOR // defined in an auto-generated header
        bool is_paused = UGameplayStatics::IsGamePaused(GetWorld());
        if (is_paused_ != is_paused) {
            if (is_paused) {
                // cache current values
                spectator_pawn_movement_ignore_time_dilation_ = SpectatorPawnMovement->bIgnoreTimeDilation;
                spectator_pawn_movement_max_speed_ = SpectatorPawnMovement->MaxSpeed;

                // set new values
                SpectatorPawnMovement->bIgnoreTimeDilation = true;
                SpectatorPawnMovement->MaxSpeed = spectator_pawn_movement_max_speed_ * 0.1;
            } else {
                // restore previous values
                SpectatorPawnMovement->bIgnoreTimeDilation = spectator_pawn_movement_ignore_time_dilation_;
                SpectatorPawnMovement->MaxSpeed = spectator_pawn_movement_max_speed_;
            }
            is_paused_ = is_paused;
        }
    #endif
}
