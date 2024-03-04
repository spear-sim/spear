//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <GameFramework/SpectatorPawn.h>
#include <UObject/ObjectMacros.h> // GENERATED_BODY, UCLASS

#include "SpSpectatorPawn.generated.h"

class UInputActionComponent;
class USpectatorPawnMovement;

UCLASS()
class ASpSpectatorPawn : public ASpectatorPawn
{
    GENERATED_BODY()
public:
    ASpSpectatorPawn();
    ~ASpSpectatorPawn();

    // ASpectatorPawn interface
    void BeginPlay() override;
    void Tick(float delta_time) override;

private:
    UInputActionComponent* input_action_component_ = nullptr;
    USpectatorPawnMovement* spectator_pawn_movement_ = nullptr;

    float spectator_pawn_movement_max_speed_ = 0.0f;
    bool spectator_pawn_movement_ignore_time_dilation_ = false;
    bool is_paused_ = false;
};
