//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpComponents/SpPlayerController.h"

#include <Engine/EngineTypes.h> // EEndPlayReason
#include <GenericPlatform/GenericPlatformMisc.h>

#include "SpCore/Assert.h"
#include "SpCore/Log.h"
#include "SpCore/Unreal.h"

#include "SpComponents/SpUserInputComponent.h"

ASpPlayerController::ASpPlayerController()
{
    SP_LOG_CURRENT_FUNCTION();

    // Need to set this to true to enable moving the camera when the game is paused.
    bShouldPerformFullTickWhenPaused = true;

    // Need to set this to true so the mouse cursor doesn't disappear when hovering over the game viewport in standalone mode.
    // However, setting this to true makes rendering ever-so-slightly choppier during click-and-drag events, but we think it
    // is worth it.
    bShowMouseCursor = true;

    // USpUserInputComponent
    SpUserInputComponent = Unreal::createComponentInsideOwnerConstructor<USpUserInputComponent>(this, "sp_user_input_component");
    SP_ASSERT(SpUserInputComponent);

    SpUserInputComponent->bHandleUserInput = true; // SpUserInputComponents need to be explicitly enabled
    SpUserInputComponent->PrimaryComponentTick.bTickEvenWhenPaused = true; // we want to exit even when paused
    SpUserInputComponent->PrimaryComponentTick.TickGroup = ETickingGroup::TG_PrePhysics;
}

ASpPlayerController::~ASpPlayerController()
{
    SP_LOG_CURRENT_FUNCTION();
}

void ASpPlayerController::BeginPlay()
{
    SP_LOG_CURRENT_FUNCTION();

    APlayerController::BeginPlay();

    // Need to set this to true to avoid blurry visual artifacts in the editor when the game is paused.
    GetWorld()->bIsCameraMoveableWhenPaused = true;

    SpUserInputComponent->subscribeToUserInputs({"Escape"});
    SpUserInputComponent->setHandleUserInputFunc([](const std::string& key, float axis_value) -> void {
        bool force = false;
        FGenericPlatformMisc::RequestExit(force);
    });
}

void ASpPlayerController::EndPlay(const EEndPlayReason::Type end_play_reason)
{
    SP_LOG_CURRENT_FUNCTION();

    APlayerController::EndPlay(end_play_reason);

    SpUserInputComponent->setHandleUserInputFunc(nullptr);
    SpUserInputComponent->unsubscribeFromUserInputs({"Escape"});
}
