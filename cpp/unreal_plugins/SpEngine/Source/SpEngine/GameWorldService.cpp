//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpEngine/GameWorldService.h"

#include <string>
#include <vector>

#include <Delegates/IDelegateInstance.h> // FDelegateHandle
#include <Engine/Engine.h>               // GEngine
#include <Engine/World.h>                // UWorld

#include "SpCore/Assert.h"
#include "SpCore/Log.h"
#include "SpEngine/EngineService.h"


void GameWorldService::postWorldInitializationHandler(UWorld* world, const UWorld::InitializationValues initialization_values)
{
    SP_LOG_CURRENT_FUNCTION();
    SP_ASSERT(world);

#if WITH_EDITOR // defined in an auto-generated header
    bool world_is_ready = world->IsGameWorld();
#else
    bool world_is_ready = GEngine->GetWorldContextFromWorld(world) != nullptr;
#endif

    if (world_is_ready) {
        // we expect worldCleanupHandler(...) to be called before a new world is created
        SP_ASSERT(!world_);

        // cache local reference to the UWorld
        world_ = world;
    }
}

void GameWorldService::worldCleanupHandler(UWorld* world, bool session_ended, bool cleanup_resources)
{
    SP_LOG_CURRENT_FUNCTION();
    SP_ASSERT(world);

    // We only need to perform any additional steps if the world being cleaned up is the world we cached in our world_ member variable.
    if (world == world_) {
        // clear cached world_ pointer
        world_ = nullptr;
    }
}
