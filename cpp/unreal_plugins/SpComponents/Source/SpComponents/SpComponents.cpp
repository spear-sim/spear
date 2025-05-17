//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpComponents/SpComponents.h"

#include <Modules/ModuleManager.h> // IMPLEMENT_GAME_MODULE, IMPLEMENT_MODULE

#include "SpCore/AssertModuleLoaded.h"
#include "SpCore/Log.h"
#include "SpCore/UnrealClassRegistrar.h"

// SpComponents classes
#include "SpComponents/SpDebugManager.h"
#include "SpComponents/SpHitEventManager.h"
#include "SpComponents/SpInitializeWorldManager.h"
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
    UnrealClassRegistrar::registerActorClass<ASpInitializeWorldManager>("ASpInitializeWorldManager");
    UnrealClassRegistrar::registerActorClass<ASpHitEventManager>("ASpHitEventManager");
    UnrealClassRegistrar::registerComponentClass<USpSceneCaptureComponent2D>("USpSceneCaptureComponent2D");
}

void SpComponents::unregisterClasses()
{
    // SpComponents classes
    UnrealClassRegistrar::unregisterActorClass<ASpDebugManager>("ASpDebugManager");
    UnrealClassRegistrar::unregisterActorClass<ASpInitializeWorldManager>("ASpInitializeWorldManager");
    UnrealClassRegistrar::unregisterActorClass<ASpHitEventManager>("ASpHitEventManager");
    UnrealClassRegistrar::unregisterComponentClass<USpSceneCaptureComponent2D>("USpSceneCaptureComponent2D");
}

// use if module does not implement any Unreal classes
// IMPLEMENT_MODULE(SpComponents, SpComponents);

// use if module implements any Unreal classes
IMPLEMENT_GAME_MODULE(SpComponents, SpComponents);
