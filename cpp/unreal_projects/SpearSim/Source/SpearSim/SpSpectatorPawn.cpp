//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpearSim/SpSpectatorPawn.h"

#include <chrono>
#include <numeric>
#include <string>

#include <Engine/EngineBaseTypes.h> // ETickingGroup
#include <GameFramework/SpectatorPawn.h>
#include <GameFramework/SpectatorPawnMovement.h>
#include <GenericPlatform/GenericPlatformMisc.h>
#include <Kismet/GameplayStatics.h>
#include <Misc/App.h>

#include "SpCore/Assert.h"
#include "SpCore/Boost.h"
#include "SpCore/Log.h"
#include "SpCore/StableNameComponent.h"
#include "SpCore/Unreal.h"

ASpSpectatorPawn::ASpSpectatorPawn()
{
    SP_LOG_CURRENT_FUNCTION();

    // Disable collision so the user can fly through walls by default.
    SetActorEnableCollision(false);

    // Need to set these to true, otherwise mouse movements will not behave as expected when the game is paused.
    bUseControllerRotationPitch = true;
    bUseControllerRotationYaw = true;
    bUseControllerRotationRoll = true;

    // We want to execute the logic in our Tick(...) function to execute even when the game is paused.
    PrimaryActorTick.bCanEverTick = true;
    PrimaryActorTick.bTickEvenWhenPaused = true;
    PrimaryActorTick.TickGroup = ETickingGroup::TG_PrePhysics;

    // UStableNameComponent
    StableNameComponent = Unreal::createComponentInsideOwnerConstructor<UStableNameComponent>(this, "stable_name");
    SP_ASSERT(StableNameComponent);

    // USpectatorPawnMovement
    SpectatorPawnMovement = dynamic_cast<USpectatorPawnMovement*>(GetMovementComponent());
    SP_ASSERT(SpectatorPawnMovement);

    // We want this pawn to be able to move even when the game is paused.
    SpectatorPawnMovement->PrimaryComponentTick.bCanEverTick = true;
    SpectatorPawnMovement->PrimaryComponentTick.bTickEvenWhenPaused = true;
    SpectatorPawnMovement->PrimaryComponentTick.TickGroup = ETickingGroup::TG_PrePhysics;
    SpectatorPawnMovement->bIgnoreTimeDilation = true;
}

ASpSpectatorPawn::~ASpSpectatorPawn()
{
    SP_LOG_CURRENT_FUNCTION();

    SP_ASSERT(SpectatorPawnMovement);
    SpectatorPawnMovement = nullptr;

    SP_ASSERT(StableNameComponent);
    StableNameComponent = nullptr;
}

void ASpSpectatorPawn::BeginPlay()
{
    ASpectatorPawn::BeginPlay();

    int num_samples = 200;
    previous_time_deltas_.set_capacity(num_samples);
    previous_time_deltas_.resize(num_samples, 0.0);
    previous_time_point_ = std::chrono::high_resolution_clock::now();
    previous_is_benchmarking_ = false;
    previous_max_speed_ = SpectatorPawnMovement->MaxSpeed;
}

void ASpSpectatorPawn::Tick(float delta_time)
{
    ASpectatorPawn::Tick(delta_time);

    // If we're in "benchmarking" mode, i.e., if we're using a fixed time step, then scale SpectatorPawnMovement's max speed
    // to undo the effect of the fixed time step. Note that GetWorld()->GetRealTimeSeconds() actually returns whatever fixed
    // time step we set previously using FApp::SetFixedDeltaTime(...), not the wall-clock time, so we use std::chrono to
    // obtain the wall-clock time. We use a circular buffer to compute a moving average of recent time deltas because it
    // results in slightly smoother camera behavior.

    std::chrono::time_point current_time_point = std::chrono::high_resolution_clock::now();
    double time_delta_seconds = std::chrono::duration<double>(current_time_point - previous_time_point_).count();
    previous_time_deltas_.push_back(time_delta_seconds);
    previous_time_point_ = current_time_point;

    bool current_is_benchmarking = FApp::IsBenchmarking();
    if (current_is_benchmarking) {
        if (current_is_benchmarking != previous_is_benchmarking_) {
            previous_max_speed_ = SpectatorPawnMovement->MaxSpeed;
        }
        double time_delta_average_seconds = std::accumulate(previous_time_deltas_.begin(), previous_time_deltas_.end(), 0.0) / previous_time_deltas_.size();
        SpectatorPawnMovement->MaxSpeed = previous_max_speed_*(time_delta_average_seconds/FApp::GetFixedDeltaTime());
    } else {
        if (current_is_benchmarking != previous_is_benchmarking_) {
            SpectatorPawnMovement->MaxSpeed = previous_max_speed_;
        }
    }
    previous_is_benchmarking_ = current_is_benchmarking;
}
