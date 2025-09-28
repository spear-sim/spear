//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpServices/Service.h"

#include <string>

#include <Delegates/IDelegateInstance.h> // FDelegateHandle
#include <Engine/Engine.h>               // GEngine
#include <Engine/World.h>                // FWorldDelegates, UWorld::InitializationValues
#include <Misc/CoreDelegates.h>

#include "SpCore/Assert.h"
#include "SpCore/Log.h"
#include "SpCore/Unreal.h"

Service::Service(std::string name)
{
    name_ = name;

    post_world_initialization_handle_ = FWorldDelegates::OnPostWorldInitialization.AddRaw(this, &Service::postWorldInitializationHandler);
    world_cleanup_handle_ = FWorldDelegates::OnWorldCleanup.AddRaw(this, &Service::worldCleanupHandler);
    begin_frame_handle_ = FCoreDelegates::OnBeginFrame.AddRaw(this, &Service::beginFrameHandler);
    end_frame_handle_ = FCoreDelegates::OnEndFrame.AddRaw(this, &Service::endFrameHandler);
}

Service::Service(std::string name, WorldFilter* world_filter) : Service(name)
{
    world_filter_ = world_filter;
}

Service::~Service()
{
    world_filter_ = nullptr;

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
    SP_LOG_CURRENT_FUNCTION();
    SP_ASSERT(world);
    SP_ASSERT(world_filter_);
    SP_LOG("    Service:      ", name_);
    SP_LOG("    World filter: ", world_filter_->getName());
    SP_LOG("    World:        ", Unreal::toStdString(world->GetName()));
    SP_LOG("    Caching world...");
    world_ = world;
    world_begin_play_handle_ = world->OnWorldBeginPlay.AddRaw(this, &Service::worldBeginPlayHandler);
}

void Service::worldCleanup(UWorld* world, bool session_ended, bool cleanup_resources)
{
    SP_LOG_CURRENT_FUNCTION();
    SP_ASSERT(world);
    SP_ASSERT(world_filter_);
    SP_LOG("    Service:      ", name_);
    SP_LOG("    World filter: ", world_filter_->getName());
    SP_LOG("    World:        ", Unreal::toStdString(world->GetName()));
    SP_LOG("    Clearing cached world...");
    world->OnWorldBeginPlay.Remove(world_begin_play_handle_);
    world_begin_play_handle_.Reset();
    world_ = nullptr;
}

void Service::worldBeginPlay()
{
    SP_LOG_CURRENT_FUNCTION();
    SP_ASSERT(world_);
    SP_ASSERT(world_filter_);
    SP_LOG("    Service:      ", name_);
    SP_LOG("    World filter: ", world_filter_->getName());
    SP_LOG("    World:        ", Unreal::toStdString(world_->GetName()));
}

void Service::beginFrame() {}
void Service::endFrame() {}

void Service::postWorldInitializationHandler(UWorld* world, const UWorld::InitializationValues initialization_values)
{
    SP_LOG_CURRENT_FUNCTION();

    if (world_filter_ && world_filter_->isValid(world)) {
        postWorldInitialization(world, initialization_values);
    }
}

void Service::worldCleanupHandler(UWorld* world, bool session_ended, bool cleanup_resources)
{
    SP_LOG_CURRENT_FUNCTION();

    if (world == world_) {
        worldCleanup(world, session_ended, cleanup_resources);
    }
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
