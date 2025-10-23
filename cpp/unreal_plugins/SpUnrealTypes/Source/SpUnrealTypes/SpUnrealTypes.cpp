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
#include "SpUnrealTypes/SpGameEngine.h"
#include "SpUnrealTypes/SpGameMode.h"
#include "SpUnrealTypes/SpGameViewportClient.h"
#include "SpUnrealTypes/SpHitEventManager.h"
#include "SpUnrealTypes/SpInitializeWorldManager.h"
#include "SpUnrealTypes/SpLevelSequence.h"
#include "SpUnrealTypes/SpLevelStreaming.h"
#include "SpUnrealTypes/SpMessageQueueManager.h"
#include "SpUnrealTypes/SpNavigationData.h"
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

    SP_REGISTER_ACTOR_CLASS(ASpDebugCameraController);
    SP_REGISTER_ACTOR_CLASS(ASpDebugCameraHUD);
    SP_REGISTER_ACTOR_CLASS(ASpDebugManager);
    SP_REGISTER_ACTOR_CLASS(ASpGameMode);
    SP_REGISTER_ACTOR_CLASS(ASpHitEventManager);
    SP_REGISTER_ACTOR_CLASS(ASpInitializeWorldManager);
    SP_REGISTER_ACTOR_CLASS(ASpPauseManager);
    SP_REGISTER_ACTOR_CLASS(ASpPlayerController);
    SP_REGISTER_ACTOR_CLASS(ASpSpectatorPawn);

    SP_REGISTER_COMPONENT_CLASS(USpBasicKeyboardControlComponent);
    SP_REGISTER_COMPONENT_CLASS(USpDummyComponent);
    SP_REGISTER_COMPONENT_CLASS(USpSceneCaptureComponent2D);
    SP_REGISTER_COMPONENT_CLASS(USpUpdateTransformComponent);
    SP_REGISTER_COMPONENT_CLASS(USpUserInputComponent);

    SP_REGISTER_CLASS(USpAssetCompilingManager);
    SP_REGISTER_CLASS(USpCoreDelegates);
    SP_REGISTER_CLASS(USpGameEngine);
    SP_REGISTER_CLASS(USpGameViewportClient);
    SP_REGISTER_CLASS(USpLevelSequence);
    SP_REGISTER_CLASS(USpLevelStreaming);
    SP_REGISTER_CLASS(USpMessageQueueManager);
    SP_REGISTER_CLASS(USpNavigationData);
    SP_REGISTER_CLASS(USpWorld);
}

void SpUnrealTypes::unregisterClasses() const
{
    // SpUnrealTypes classes

    SP_UNREGISTER_ACTOR_CLASS(ASpDebugCameraController);
    SP_UNREGISTER_ACTOR_CLASS(ASpDebugCameraHUD);
    SP_UNREGISTER_ACTOR_CLASS(ASpDebugManager);
    SP_UNREGISTER_ACTOR_CLASS(ASpGameMode);
    SP_UNREGISTER_ACTOR_CLASS(ASpHitEventManager);
    SP_UNREGISTER_ACTOR_CLASS(ASpInitializeWorldManager);
    SP_UNREGISTER_ACTOR_CLASS(ASpPauseManager);
    SP_UNREGISTER_ACTOR_CLASS(ASpPlayerController);
    SP_UNREGISTER_ACTOR_CLASS(ASpSpectatorPawn);

    SP_UNREGISTER_COMPONENT_CLASS(USpBasicKeyboardControlComponent);
    SP_UNREGISTER_COMPONENT_CLASS(USpDummyComponent);
    SP_UNREGISTER_COMPONENT_CLASS(USpSceneCaptureComponent2D);
    SP_UNREGISTER_COMPONENT_CLASS(USpUpdateTransformComponent);
    SP_UNREGISTER_COMPONENT_CLASS(USpUserInputComponent);

    SP_UNREGISTER_CLASS(USpAssetCompilingManager);
    SP_UNREGISTER_CLASS(USpCoreDelegates);
    SP_UNREGISTER_CLASS(USpGameEngine);
    SP_UNREGISTER_CLASS(USpGameViewportClient);
    SP_UNREGISTER_CLASS(USpLevelSequence);
    SP_UNREGISTER_CLASS(USpLevelStreaming);
    SP_UNREGISTER_CLASS(USpMessageQueueManager);
    SP_UNREGISTER_CLASS(USpNavigationData);
    SP_UNREGISTER_CLASS(USpWorld);
}

// use IMPLEMENT_GAME_MODULE if module implements Unreal classes, use IMPLEMENT_MODULE otherwise
IMPLEMENT_GAME_MODULE(SpUnrealTypes, SpUnrealTypes);
