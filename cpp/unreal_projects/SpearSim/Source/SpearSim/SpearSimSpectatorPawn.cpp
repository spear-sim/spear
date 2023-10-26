//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpearSim/SpearSimSpectatorPawn.h"

#include <GameFramework/SpectatorPawn.h>
#include <GenericPlatform/GenericPlatformMisc.h>

#include "CoreUtils/PlayerInputComponent.h"
#include "CoreUtils/Unreal.h"

ASpearSimSpectatorPawn::ASpearSimSpectatorPawn()
{
    SP_LOG_CURRENT_FUNCTION();

    // UPlayerInputComponent
    player_input_component_ = CreateDefaultSubobject<UPlayerInputComponent>(Unreal::toFName("player_input_component"));
    SP_ASSERT(player_input_component_);
    // Need to explicitly set this up so that the component hierarchy is well-defined.
    player_input_component_->SetupAttachment(GetRootComponent());
}

ASpearSimSpectatorPawn::~ASpearSimSpectatorPawn()
{
    SP_LOG_CURRENT_FUNCTION();

    SP_ASSERT(player_input_component_);
    player_input_component_ = nullptr;
}

void ASpearSimSpectatorPawn::BeginPlay()
{
    ASpectatorPawn::BeginPlay();

    SP_LOG_CURRENT_FUNCTION();

    std::map<std::string, float> input_actions{ {"Escape", 1.0f} };
    player_input_component_->bindInputActions(input_actions);
    player_input_component_->apply_action_func_ = [](const PlayerInputActionDesc& player_input_action_desc, float axis_value) -> void {
        if (axis_value == 1.0f) {
            bool force = false;
            FGenericPlatformMisc::RequestExit(force);
        }
    };
}
