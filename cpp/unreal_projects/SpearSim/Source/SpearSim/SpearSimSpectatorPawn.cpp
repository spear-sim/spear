//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpearSim/SpearSimSpectatorPawn.h"

#include <Components/InputComponent.h>
#include <GameFramework/PlayerInput.h>
#include <GameFramework/SpectatorPawn.h>
#include <GenericPlatform/GenericPlatformMisc.h>

#include "CoreUtils/Config.h"
#include "CoreUtils/PlayerInputComponent.h"
#include "CoreUtils/Unreal.h"

ASpearSimSpectatorPawn::ASpearSimSpectatorPawn()
{
    SP_LOG_CURRENT_FUNCTION();
}

ASpearSimSpectatorPawn::~ASpearSimSpectatorPawn()
{
    SP_LOG_CURRENT_FUNCTION();
}

void ASpearSimSpectatorPawn::SetupPlayerInputComponent(UInputComponent* input_component)
{
    ASpectatorPawn::SetupPlayerInputComponent(input_component);

    SP_LOG_CURRENT_FUNCTION();

    SP_ASSERT(input_component);
    UPlayerInput* player_input = GetWorld()->GetFirstPlayerController()->PlayerInput;
    SP_ASSERT(player_input);
    player_input->AddAxisMapping(FInputAxisKeyMapping(Unreal::toFName("Exit"), FKey(Unreal::toFName("Escape")), 1.0f));
    input_component->BindAxis(Unreal::toFName("Exit"));
}

void ASpearSimSpectatorPawn::Tick(float delta_time)
{
    ASpectatorPawn::Tick(delta_time);

    SP_ASSERT(InputComponent);

    float axis_value = InputComponent->GetAxisValue(Unreal::toFName("Exit"));
    if (axis_value == 1.0f) {
        bool force = false;
        FGenericPlatformMisc::RequestExit(force);
    }
}
