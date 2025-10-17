//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpUnrealTypes/SpUnrealTypes.h"

#include <Modules/ModuleManager.h> // FDefaultGameModuleImpl, FDefaultModuleImpl, IMPLEMENT_GAME_MODULE, IMPLEMENT_MODULE

#include "SpCore/AssertModuleLoaded.h"
#include "SpCore/Log.h"
#include "SpCore/UnrealClassRegistry.h"

// SpUnrealTypes classes
#include "SpUnrealTypes/SpAssetCompilingManager.h"
#include "SpUnrealTypes/SpBasicKeyboardControlComponent.h"
#include "SpUnrealTypes/SpCoreDelegates.h"
#include "SpUnrealTypes/SpDebugCameraController.h"
#include "SpUnrealTypes/SpDebugCameraHUD.h"
#include "SpUnrealTypes/SpDebugManager.h"
#include "SpUnrealTypes/SpDummyComponent.h"
#include "SpUnrealTypes/SpGameMode.h"
#include "SpUnrealTypes/SpGameEngine.h"
#include "SpUnrealTypes/SpGameViewportClient.h"
#include "SpUnrealTypes/SpHitEventManager.h"
#include "SpUnrealTypes/SpInitializeWorldManager.h"
#include "SpUnrealTypes/SpLevelStreaming.h"
#include "SpUnrealTypes/SpMessageQueueManager.h"
#include "SpUnrealTypes/SpPauseManager.h"
#include "SpUnrealTypes/SpPlayerController.h"
#include "SpUnrealTypes/SpSceneCaptureComponent2D.h"
#include "SpUnrealTypes/SpSpectatorPawn.h"
#include "SpUnrealTypes/SpUpdateTransformComponent.h"
#include "SpUnrealTypes/SpUserInputComponent.h"
#include "SpUnrealTypes/SpWorld.h"

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

void SpUnrealTypes::registerClasses() const
{
    // SpUnrealTypes classes
    UnrealClassRegistry::registerActorClass<ASpDebugCameraController>("ASpDebugCameraController");
    UnrealClassRegistry::registerActorClass<ASpDebugCameraHUD>("ASpDebugCameraHUD");
    UnrealClassRegistry::registerActorClass<ASpDebugManager>("ASpDebugManager");
    UnrealClassRegistry::registerActorClass<ASpGameMode>("ASpGameMode");
    UnrealClassRegistry::registerActorClass<ASpHitEventManager>("ASpHitEventManager");
    UnrealClassRegistry::registerActorClass<ASpInitializeWorldManager>("ASpInitializeWorldManager");
    UnrealClassRegistry::registerActorClass<ASpPauseManager>("ASpPauseManager");
    UnrealClassRegistry::registerActorClass<ASpPlayerController>("ASpPlayerController");
    UnrealClassRegistry::registerActorClass<ASpSpectatorPawn>("ASpSpectatorPawn");
    UnrealClassRegistry::registerComponentClass<USpBasicKeyboardControlComponent>("USpBasicKeyboardControlComponent");
    UnrealClassRegistry::registerComponentClass<USpDummyComponent>("USpDummyComponent");
    UnrealClassRegistry::registerComponentClass<USpSceneCaptureComponent2D>("USpSceneCaptureComponent2D");
    UnrealClassRegistry::registerComponentClass<USpUpdateTransformComponent>("USpUpdateTransformComponent");
    UnrealClassRegistry::registerComponentClass<USpUserInputComponent>("USpUserInputComponent");
    UnrealClassRegistry::registerClass<USpAssetCompilingManager>("USpAssetCompilingManager");
    UnrealClassRegistry::registerClass<USpCoreDelegates>("USpCoreDelegates");
    UnrealClassRegistry::registerClass<USpGameEngine>("USpGameEngine");
    UnrealClassRegistry::registerClass<USpGameViewportClient>("USpGameViewportClient");
    UnrealClassRegistry::registerClass<USpLevelStreaming>("USpLevelStreaming");
    UnrealClassRegistry::registerClass<USpMessageQueueManager>("USpMessageQueueManager");
    UnrealClassRegistry::registerClass<USpWorld>("USpWorld");
}

void SpUnrealTypes::unregisterClasses() const
{
    // SpUnrealTypes classes
    UnrealClassRegistry::unregisterActorClass<ASpDebugCameraController>("ASpDebugCameraController");
    UnrealClassRegistry::unregisterActorClass<ASpDebugCameraHUD>("ASpDebugCameraHUD");
    UnrealClassRegistry::unregisterActorClass<ASpDebugManager>("ASpDebugManager");
    UnrealClassRegistry::unregisterActorClass<ASpGameMode>("ASpGameMode");
    UnrealClassRegistry::unregisterActorClass<ASpHitEventManager>("ASpHitEventManager");
    UnrealClassRegistry::unregisterActorClass<ASpInitializeWorldManager>("ASpInitializeWorldManager");
    UnrealClassRegistry::unregisterActorClass<ASpPauseManager>("ASpPauseManager");
    UnrealClassRegistry::unregisterActorClass<ASpPlayerController>("ASpPlayerController");
    UnrealClassRegistry::unregisterActorClass<ASpSpectatorPawn>("ASpSpectatorPawn");
    UnrealClassRegistry::unregisterComponentClass<USpBasicKeyboardControlComponent>("USpBasicKeyboardControlComponent");
    UnrealClassRegistry::unregisterComponentClass<USpDummyComponent>("USpDummyComponent");
    UnrealClassRegistry::unregisterComponentClass<USpSceneCaptureComponent2D>("USpSceneCaptureComponent2D");
    UnrealClassRegistry::unregisterComponentClass<USpUpdateTransformComponent>("USpUpdateTransformComponent");
    UnrealClassRegistry::unregisterComponentClass<USpUserInputComponent>("USpUserInputComponent");
    UnrealClassRegistry::unregisterClass<USpAssetCompilingManager>("USpAssetCompilingManager");
    UnrealClassRegistry::unregisterClass<USpCoreDelegates>("USpCoreDelegates");
    UnrealClassRegistry::unregisterClass<USpGameEngine>("USpGameEngine");
    UnrealClassRegistry::unregisterClass<USpGameViewportClient>("USpGameViewportClient");
    UnrealClassRegistry::unregisterClass<USpLevelStreaming>("USpLevelStreaming");
    UnrealClassRegistry::unregisterClass<USpMessageQueueManager>("USpMessageQueueManager");
    UnrealClassRegistry::unregisterClass<USpWorld>("USpWorld");
}

// use IMPLEMENT_GAME_MODULE if module implements Unreal classes, use IMPLEMENT_MODULE otherwise
IMPLEMENT_GAME_MODULE(SpUnrealTypes, SpUnrealTypes);
