//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpearSim/SpPlayerController.h"

#include "SpCore/Log.h"

ASpPlayerController::ASpPlayerController()
{
    SP_LOG_CURRENT_FUNCTION();

    // Need to set this to true to enable moving the camera when the game is paused.
    bShouldPerformFullTickWhenPaused = true;

    // Need to set this to true so the mouse cursor doesn't disappear when hovering over the game viewport in standalone mode.
    // However, setting this to true makes rendering ever-so-slightly choppier during click-and-drag events, but we think it
    // is worth it.
    bShowMouseCursor = true;
}

ASpPlayerController::~ASpPlayerController()
{
    SP_LOG_CURRENT_FUNCTION();
}
