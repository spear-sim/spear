//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpServices/Service.h"

#include <Delegates/IDelegateInstance.h> // FDelegateHandle
#include <Engine/Engine.h>               // GEngine
#include <Engine/World.h>                // FWorldDelegates, UWorld::InitializationValues
#include <Misc/CoreDelegates.h>

#include "SpCore/Assert.h"
#include "SpCore/Log.h"
#include "SpCore/Unreal.h"

Service::Service()
{
    post_world_initialization_handle_ = FWorldDelegates::OnPostWorldInitialization.AddRaw(this, &Service::postWorldInitializationHandler);
    world_cleanup_handle_ = FWorldDelegates::OnWorldCleanup.AddRaw(this, &Service::worldCleanupHandler);
    begin_frame_handle_ = FCoreDelegates::OnBeginFrame.AddRaw(this, &Service::beginFrameHandler);
    end_frame_handle_ = FCoreDelegates::OnEndFrame.AddRaw(this, &Service::endFrameHandler);
}

Service::~Service()
{
    FCoreDelegates::OnEndFrame.Remove(end_frame_handle_);
    FCoreDelegates::OnBeginFrame.Remove(begin_frame_handle_);
    FWorldDelegates::OnWorldCleanup.Remove(world_cleanup_handle_);
    FWorldDelegates::OnPostWorldInitialization.Remove(post_world_initialization_handle_);

    end_frame_handle_.Reset();
    begin_frame_handle_.Reset();
    world_cleanup_handle_.Reset();
    post_world_initialization_handle_.Reset();
}

void Service::postWorldInitialization(UWorld* world, const UWorld::InitializationValues initialization_values)
{
    SP_ASSERT(GEngine);
    SP_ASSERT(world);

    SP_LOG("World: ", Unreal::toStdString(world->GetName()));

    if (world->IsGameWorld() && GEngine->GetWorldContextFromWorld(world)) {
        SP_LOG("Caching world...");
        SP_ASSERT(!world_);
        world_ = world;
        world_begin_play_handle_ = world_->OnWorldBeginPlay.AddRaw(this, &Service::worldBeginPlayHandler);
    }
}

void Service::worldCleanup(UWorld* world, bool session_ended, bool cleanup_resources)
{
    SP_ASSERT(world);

    SP_LOG("World: ", Unreal::toStdString(world->GetName()));

    if (world == world_) {
        SP_LOG("Clearing cached world...");
        world_->OnWorldBeginPlay.Remove(world_begin_play_handle_);
        world_begin_play_handle_.Reset();
        world_ = nullptr;
    }
}

void Service::worldBeginPlay()
{
    SP_ASSERT(world_);
}

UWorld* Service::getWorld()
{
    return world_;
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

void Service::beginFrameHandler()
{
    beginFrame();
}

void Service::endFrameHandler()
{
    endFrame();
}
