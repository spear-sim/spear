//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpEngine/GameWorldService.h"

#include <string>
#include <vector>

#include <Engine/Engine.h> // GEngine
#include <Engine/World.h>

#include "SpCore/Assert.h"
#include "SpCore/Log.h"

void GameWorldService::postWorldInitializationHandler(UWorld* world, const UWorld::InitializationValues initialization_values)
{
    SP_LOG_CURRENT_FUNCTION();
    SP_ASSERT(world);

    if (world->IsGameWorld() && GEngine->GetWorldContextFromWorld(world)) {
        SP_ASSERT(!world_);
        world_ = world;
    }
}

void GameWorldService::worldCleanupHandler(UWorld* world, bool session_ended, bool cleanup_resources)
{
    SP_LOG_CURRENT_FUNCTION();
    SP_ASSERT(world);

    if (world == world_) {
        world_ = nullptr;
    }
}

GameWorldServicePropertyDesc GameWorldService::toPropertyDesc(const Unreal::PropertyDesc& property_desc)
{
    GameWorldServicePropertyDesc game_world_property_desc;
    game_world_property_desc.property_ = reinterpret_cast<uint64_t>(property_desc.property_);
    game_world_property_desc.value_ptr_ = reinterpret_cast<uint64_t>(property_desc.value_ptr_);
    return game_world_property_desc;
}

Unreal::PropertyDesc GameWorldService::toPropertyDesc(const GameWorldServicePropertyDesc& game_world_property_desc)
{
    Unreal::PropertyDesc property_desc;
    property_desc.property_ = reinterpret_cast<FProperty*>(game_world_property_desc.property_);
    property_desc.value_ptr_ = reinterpret_cast<void*>(game_world_property_desc.value_ptr_);
    return property_desc;
}
