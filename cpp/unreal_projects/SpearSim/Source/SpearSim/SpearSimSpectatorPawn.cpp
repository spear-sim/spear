//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpearSim/SpearSimSpectatorPawn.h"

#include <string>

#include <GameFramework/SpectatorPawn.h>
#include <GenericPlatform/GenericPlatformMisc.h>

#include "CoreUtils/InputActionComponent.h"
#include "CoreUtils/Unreal.h"

ASpearSimSpectatorPawn::ASpearSimSpectatorPawn()
{
    SP_LOG_CURRENT_FUNCTION();

    // UInputActionComponent
    input_action_component_ = Unreal::createComponentInsideOwnerConstructor<UInputActionComponent>(this, "input_action_component", GetRootComponent());
    SP_ASSERT(input_action_component_);
}

ASpearSimSpectatorPawn::~ASpearSimSpectatorPawn()
{
    SP_LOG_CURRENT_FUNCTION();

    SP_ASSERT(input_action_component_);
    input_action_component_ = nullptr;
}

void ASpearSimSpectatorPawn::BeginPlay()
{
    ASpectatorPawn::BeginPlay();

    std::map<std::string, float> input_actions{{"Escape", 1.0f}};
    input_action_component_->bindInputActions(input_actions);
    input_action_component_->apply_input_action_func_ = [](const std::string& key) -> void {
        bool force = false;
        FGenericPlatformMisc::RequestExit(force);
    };
}
