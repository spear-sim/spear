//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpComponents/SpComponents.h"

#include <Modules/ModuleManager.h> // IMPLEMENT_GAME_MODULE, IMPLEMENT_MODULE

#include "SpCore/AssertModuleLoaded.h"
#include "SpCore/Log.h"
#include "SpCore/UnrealClassRegistrar.h"

#include "SpComponents/SpDebugManager.h"
#include "SpComponents/SpHitEventManager.h"
#include "SpComponents/SpSceneCaptureComponent2D.h"

void SpComponents::StartupModule()
{
    SP_ASSERT_MODULE_LOADED("SpCore");
    SP_LOG_CURRENT_FUNCTION();

    UnrealClassRegistrar::registerActorClass<ASpDebugManager>("ASpDebugManager");
    UnrealClassRegistrar::registerActorClass<ASpHitEventManager>("ASpHitEventManager");
    UnrealClassRegistrar::registerComponentClass<USpSceneCaptureComponent2D>("USpSceneCaptureComponent2D");
}

void SpComponents::ShutdownModule()
{
    SP_LOG_CURRENT_FUNCTION();

    UnrealClassRegistrar::unregisterActorClass<ASpDebugManager>("ASpDebugManager");
    UnrealClassRegistrar::unregisterActorClass<ASpHitEventManager>("ASpHitEventManager");
    UnrealClassRegistrar::unregisterComponentClass<USpSceneCaptureComponent2D>("USpSceneCaptureComponent2D");
}

// use if module does not implement any Unreal classes
// IMPLEMENT_MODULE(SpComponents, SpComponents);

// use if module implements any Unreal classes
IMPLEMENT_GAME_MODULE(SpComponents, SpComponents);
