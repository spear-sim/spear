//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpUnrealTypes/SpUnrealTypes.h"

#include <Modules/ModuleManager.h> // FDefaultGameModuleImpl, FDefaultModuleImpl, IMPLEMENT_GAME_MODULE, IMPLEMENT_MODULE

#include "SpCore/AssertModuleLoaded.h"
#include "SpCore/Log.h"
#include "SpCore/UnrealClassRegistrar.h"

// SpUnrealTypes classes
#include "SpUnrealTypes/SpBasicKeyboardControlComponent.h"
#include "SpUnrealTypes/SpDebugManager.h"
#include "SpUnrealTypes/SpDummyComponent.h"
#include "SpUnrealTypes/SpHitEventManager.h"
#include "SpUnrealTypes/SpInitializeWorldManager.h"
#include "SpUnrealTypes/SpMessageQueueManager.h"
#include "SpUnrealTypes/SpPauseManager.h"
#include "SpUnrealTypes/SpPlayerController.h"
#include "SpUnrealTypes/SpSceneCaptureComponent2D.h"
#include "SpUnrealTypes/SpSpectatorPawn.h"
#include "SpUnrealTypes/SpUpdateTransformComponent.h"
#include "SpUnrealTypes/SpUserInputComponent.h"

void SpUnrealTypes::StartupModule()
{
    SP_ASSERT_MODULE_LOADED("SpCore");
    SP_LOG_CURRENT_FUNCTION();

    registerClasses();
}

void SpUnrealTypes::ShutdownModule()
{
    SP_LOG_CURRENT_FUNCTION();

    unregisterClasses();
}

// Normally we would do the operations in registerClasses() and unregisterClasses(...) in the opposite order.
// But we make an exception here (i.e., we do the operations in the same order) to make it easier and less
// error-prone to register classes.

void SpUnrealTypes::registerClasses()
{
    // SpUnrealTypes classes
    UnrealClassRegistrar::registerActorClass<ASpDebugManager>("ASpDebugManager");
    UnrealClassRegistrar::registerActorClass<ASpHitEventManager>("ASpHitEventManager");
    UnrealClassRegistrar::registerActorClass<ASpInitializeWorldManager>("ASpInitializeWorldManager");
    UnrealClassRegistrar::registerActorClass<ASpMessageQueueManager>("ASpMessageQueueManager");
    UnrealClassRegistrar::registerActorClass<ASpPauseManager>("ASpPauseManager");
    UnrealClassRegistrar::registerActorClass<ASpPlayerController>("ASpPlayerController");
    UnrealClassRegistrar::registerActorClass<ASpSpectatorPawn>("ASpSpectatorPawn");
    UnrealClassRegistrar::registerComponentClass<USpBasicKeyboardControlComponent>("USpBasicKeyboardControlComponent");
    UnrealClassRegistrar::registerComponentClass<USpDummyComponent>("USpDummyComponent");
    UnrealClassRegistrar::registerComponentClass<USpSceneCaptureComponent2D>("USpSceneCaptureComponent2D");
    UnrealClassRegistrar::registerComponentClass<USpUpdateTransformComponent>("USpUpdateTransformComponent");
    UnrealClassRegistrar::registerComponentClass<USpUserInputComponent>("USpUserInputComponent");
}

void SpUnrealTypes::unregisterClasses()
{
    // SpUnrealTypes classes
    UnrealClassRegistrar::unregisterActorClass<ASpDebugManager>("ASpDebugManager");
    UnrealClassRegistrar::unregisterActorClass<ASpHitEventManager>("ASpHitEventManager");
    UnrealClassRegistrar::unregisterActorClass<ASpInitializeWorldManager>("ASpInitializeWorldManager");
    UnrealClassRegistrar::unregisterActorClass<ASpMessageQueueManager>("ASpMessageQueueManager");
    UnrealClassRegistrar::unregisterActorClass<ASpPauseManager>("ASpPauseManager");
    UnrealClassRegistrar::unregisterActorClass<ASpPlayerController>("ASpPlayerController");
    UnrealClassRegistrar::unregisterActorClass<ASpSpectatorPawn>("ASpSpectatorPawn");
    UnrealClassRegistrar::unregisterComponentClass<USpBasicKeyboardControlComponent>("USpBasicKeyboardControlComponent");
    UnrealClassRegistrar::unregisterComponentClass<USpDummyComponent>("USpDummyComponent");
    UnrealClassRegistrar::unregisterComponentClass<USpSceneCaptureComponent2D>("USpSceneCaptureComponent2D");
    UnrealClassRegistrar::unregisterComponentClass<USpUpdateTransformComponent>("USpUpdateTransformComponent");
    UnrealClassRegistrar::unregisterComponentClass<USpUserInputComponent>("USpUserInputComponent");
}

// use IMPLEMENT_GAME_MODULE if module implements Unreal classes, use IMPLEMENT_MODULE otherwise
IMPLEMENT_GAME_MODULE(SpUnrealTypes, SpUnrealTypes);
