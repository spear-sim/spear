//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpComponents/SpComponents.h"

#include <Modules/ModuleManager.h> // IMPLEMENT_GAME_MODULE, IMPLEMENT_MODULE

#include "SpCore/AssertModuleLoaded.h"
#include "SpCore/Log.h"
#include "SpCore/UnrealClassRegistrar.h"

#include "SpComponents/SpHitEventManager.h"

void SpComponents::StartupModule()
{
    SP_ASSERT_MODULE_LOADED("SpCore");
    SP_LOG_CURRENT_FUNCTION();

    UnrealClassRegistrar::registerActorClass<ASpHitEventManager>("ASpHitEventManager");
}

void SpComponents::ShutdownModule()
{
    SP_LOG_CURRENT_FUNCTION();

    UnrealClassRegistrar::unregisterActorClass<ASpHitEventManager>("ASpHitEventManager");
}

// use if module does not implement any Unreal classes
// IMPLEMENT_MODULE(SpComponents, SpComponents);

// use if module implements any Unreal classes
IMPLEMENT_GAME_MODULE(SpComponents, SpComponents);
