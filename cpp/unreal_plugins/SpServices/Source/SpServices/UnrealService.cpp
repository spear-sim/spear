//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpServices/UnrealService.h"

#include <string>
#include <vector>

#include <Engine/Engine.h> // GEngine
#include <Engine/World.h>

#include "SpCore/Assert.h"
#include "SpCore/Log.h"

void UnrealService::postWorldInitializationHandler(UWorld* world, const UWorld::InitializationValues initialization_values)
{
    SP_LOG_CURRENT_FUNCTION();
    SP_ASSERT(world);
    if (world->IsGameWorld() && GEngine->GetWorldContextFromWorld(world)) {
        SP_ASSERT(!world_);
        world_ = world;
    }
}

void UnrealService::worldCleanupHandler(UWorld* world, bool session_ended, bool cleanup_resources)
{
    SP_LOG_CURRENT_FUNCTION();
    SP_ASSERT(world);
    if (world == world_) {
        world_ = nullptr;
    }
}
