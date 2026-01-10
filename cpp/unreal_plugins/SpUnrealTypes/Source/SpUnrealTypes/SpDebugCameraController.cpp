//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpUnrealTypes/SpDebugCameraController.h"

#include <Engine/World.h>         // FActorSpawnParameters
#include <UObject/ObjectMacros.h> // EObjectFlags

#include "SpCore/Log.h"

#include "SpUnrealTypes/SpDebugCameraHUD.h"

ASpDebugCameraController::ASpDebugCameraController()
{
    SP_LOG_CURRENT_FUNCTION();

    // Need to set this to true so the mouse cursor doesn't disappear when hovering over the game viewport in standalone mode.
    // However, setting this to true makes rendering ever-so-slightly choppier during click-and-drag events, but we think it
    // is worth it.
    bShowMouseCursor = true;
}

ASpDebugCameraController::~ASpDebugCameraController()
{
    SP_LOG_CURRENT_FUNCTION();
}

void ASpDebugCameraController::PostInitializeComponents()
{
    // skip ADebugCameraController::PostInitializeComponents() because we don't want to initialize the default debug camera HUD
    APlayerController::PostInitializeComponents();

    if (MyHUD) {
        MyHUD->Destroy();
    }

    FActorSpawnParameters actor_spawn_parameters;
    actor_spawn_parameters.Owner = this;
    actor_spawn_parameters.Instigator = GetInstigator();
    actor_spawn_parameters.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    actor_spawn_parameters.ObjectFlags |= EObjectFlags::RF_Transient; // we never want these to save into a map
    MyHUD = GetWorld()->SpawnActor<ASpDebugCameraHUD>(actor_spawn_parameters);

    ChangeState(NAME_Inactive);
}
