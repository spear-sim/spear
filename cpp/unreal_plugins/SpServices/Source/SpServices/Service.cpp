//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpServices/Service.h"

#include <Delegates/IDelegateInstance.h> // FDelegateHandle
#include <Engine/Engine.h>               // GEngine
#include <Engine/World.h>                // FWorldDelegates, UWorld::InitializationValues

#include "SpCore/Assert.h"
#include "SpCore/Log.h"
#include "SpCore/Unreal.h"

Service::Service()
{
    post_world_initialization_handle_ = FWorldDelegates::OnPostWorldInitialization.AddRaw(this, &Service::postWorldInitializationHandler);
    world_cleanup_handle_ = FWorldDelegates::OnWorldCleanup.AddRaw(this, &Service::worldCleanupHandler);
}

Service::~Service()
{
    FWorldDelegates::OnWorldCleanup.Remove(world_cleanup_handle_);
    FWorldDelegates::OnPostWorldInitialization.Remove(post_world_initialization_handle_);

    world_cleanup_handle_.Reset();
    post_world_initialization_handle_.Reset();
}

UWorld* Service::getWorld()
{
    SP_ASSERT(world_);
    return world_;
}

void Service::postWorldInitialization(UWorld* world, const UWorld::InitializationValues initialization_values)
{
    SP_ASSERT(world);
    if (world->IsGameWorld() && GEngine->GetWorldContextFromWorld(world)) {
        SP_ASSERT(!world_);
        world_ = world;
        world_begin_play_handle_ = world_->OnWorldBeginPlay.AddRaw(this, &Service::worldBeginPlayHandler);
    }
}

void Service::worldCleanup(UWorld* world, bool session_ended, bool cleanup_resources)
{
    SP_ASSERT(world);
    if (world == world_) {
        world_->OnWorldBeginPlay.Remove(world_begin_play_handle_);
        world_begin_play_handle_.Reset();
        world_ = nullptr;
    }
}

void Service::postWorldInitializationHandler(UWorld* world, const UWorld::InitializationValues initialization_values)
{
    SP_LOG_CURRENT_FUNCTION();
    postWorldInitialization(world, initialization_values);
}

void Service::worldCleanupHandler(UWorld* world, bool session_ended, bool cleanup_resources)
{
    SP_LOG_CURRENT_FUNCTION();
    worldCleanup(world, session_ended, cleanup_resources);
}

void Service::worldBeginPlayHandler()
{
    SP_LOG_CURRENT_FUNCTION();
    worldBeginPlay();
}
