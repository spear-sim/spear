//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpComponents/SpComponents.h"

#include <Modules/ModuleManager.h> // FDefaultGameModuleImpl, FDefaultModuleImpl, IMPLEMENT_GAME_MODULE, IMPLEMENT_MODULE

#include "SpCore/AssertModuleLoaded.h"
#include "SpCore/Log.h"
#include "SpCore/UnrealClassRegistrar.h"

// SpComponents classes
#include "SpComponents/SpDebugManager.h"
#include "SpComponents/SpHitEventManager.h"
#include "SpComponents/SpInitializeWorldManager.h"
#include "SpComponents/SpMessageQueueManager.h"
#include "SpComponents/SpSceneCaptureComponent2D.h"

void SpComponents::StartupModule()
{
    SP_ASSERT_MODULE_LOADED("SpCore");
    SP_LOG_CURRENT_FUNCTION();

    registerClasses();
}

void SpComponents::ShutdownModule()
{
    SP_LOG_CURRENT_FUNCTION();

    unregisterClasses();
}

// Normally we would do the operations in registerClasses() and unregisterClasses(...) in the opposite order.
// But we make an exception here (i.e., we do the operations in the same order) to make it easier and less
// error-prone to register classes.

void SpComponents::registerClasses()
{
    // SpComponents classes
    UnrealClassRegistrar::registerActorClass<ASpDebugManager>("ASpDebugManager");
    UnrealClassRegistrar::registerActorClass<ASpHitEventManager>("ASpHitEventManager");
    UnrealClassRegistrar::registerActorClass<ASpInitializeWorldManager>("ASpInitializeWorldManager");
    UnrealClassRegistrar::registerActorClass<ASpMessageQueueManager>("ASpMessageQueueManager");
    UnrealClassRegistrar::registerComponentClass<USpSceneCaptureComponent2D>("USpSceneCaptureComponent2D");
}

void SpComponents::unregisterClasses()
{
    // SpComponents classes
    UnrealClassRegistrar::unregisterActorClass<ASpDebugManager>("ASpDebugManager");
    UnrealClassRegistrar::unregisterActorClass<ASpHitEventManager>("ASpHitEventManager");
    UnrealClassRegistrar::unregisterActorClass<ASpInitializeWorldManager>("ASpInitializeWorldManager");
    UnrealClassRegistrar::unregisterActorClass<ASpMessageQueueManager>("ASpMessageQueueManager");
    UnrealClassRegistrar::unregisterComponentClass<USpSceneCaptureComponent2D>("USpSceneCaptureComponent2D");
}

// use IMPLEMENT_GAME_MODULE if module implements Unreal classes, use IMPLEMENT_MODULE otherwise
IMPLEMENT_GAME_MODULE(SpComponents, SpComponents);
