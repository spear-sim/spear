//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpearSim/SpearSimSpectatorPawn.h"

#include <iostream>

#include <Components/InputComponent.h>
#include <GameFramework/PlayerInput.h>
#include <GameFramework/SpectatorPawn.h>
#include <GenericPlatform/GenericPlatformMisc.h>

#include "CoreUtils/Config.h"
#include "CoreUtils/Unreal.h"

ASpearSimSpectatorPawn::ASpearSimSpectatorPawn(const FObjectInitializer& object_initializer) : ASpectatorPawn(object_initializer)
{
    std::cout << "[SPEAR | SpearSimSpectatorPawn.cpp] ASpearSimSpectatorPawn::ASpearSimSpectatorPawn" << std::endl;

    if (!Config::s_initialized_) {
        return;
    }
}

ASpearSimSpectatorPawn::~ASpearSimSpectatorPawn()
{
    std::cout << "[SPEAR | SpearSimSpectatorPawn.cpp] ASpearSimSpectatorPawn::~ASpearSimSpectatorPawn" << std::endl;
}

void ASpearSimSpectatorPawn::SetupPlayerInputComponent(UInputComponent* input_component)
{
    ASpectatorPawn::SetupPlayerInputComponent(input_component);

    UPlayerInput* player_input = GetWorld()->GetFirstPlayerController()->PlayerInput;
    player_input->AddAxisMapping(FInputAxisKeyMapping(Unreal::toFName("Exit"), FKey(Unreal::toFName("Escape")), 1.0f));
    input_component->BindAxis("Exit", this, &ASpearSimSpectatorPawn::exit);
}

void ASpearSimSpectatorPawn::exit(float value)
{
    if (value == 1.0f) {
        bool force = false;
        FGenericPlatformMisc::RequestExit(force);
    }
}
