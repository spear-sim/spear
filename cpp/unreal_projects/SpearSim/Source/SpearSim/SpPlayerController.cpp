//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpearSim/SpPlayerController.h"

#include "SpCore/Log.h"
#include "SpCore/Unreal.h"
#include "SpCore/UserInputComponent.h"

ASpPlayerController::ASpPlayerController()
{
    SP_LOG_CURRENT_FUNCTION();

    // Need to set this to true to enable moving the camera when the game is paused.
    bShouldPerformFullTickWhenPaused = true;

    // Need to set this to true so the mouse cursor doesn't disappear when hovering over the game viewport in standalone mode.
    // However, setting this to true makes rendering ever-so-slightly choppier during click-and-drag events, but we think it
    // is worth it.
    bShowMouseCursor = true;

    // UUserInputComponent
    user_input_component_ = Unreal::createComponentInsideOwnerConstructor<UUserInputComponent>(this, GetRootComponent(), "user_input");
    SP_ASSERT(user_input_component_);

    // We want to enable handling custom user input so pressing escape always exits the application.
    user_input_component_->bEnableHandleUserInput = true;
}

ASpPlayerController::~ASpPlayerController()
{
    SP_LOG_CURRENT_FUNCTION();

    user_input_component_ = nullptr;
}

void ASpPlayerController::BeginPlay()
{
    APlayerController::BeginPlay();

    user_input_component_->subscribeToUserInputs({"Escape"});
    user_input_component_->setHandleUserInputFunc([](const std::string& key, float axis_value) -> void {
        bool force = false;
        FGenericPlatformMisc::RequestExit(force);
    });
}
