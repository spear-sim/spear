//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <string>
#include <vector>

#include <Delegates/IDelegateInstance.h> // FDelegateHandle
#include <Engine/Engine.h>               // GEngine
#include <Engine/World.h>                // UWorld
#include <Kismet/GameplayStatics.h>

#include "SpCore/Assert.h"
#include "SpCore/Log.h"
#include "SpCore/Unreal.h"
#include "SpEngine/EntryPointBinder.h"

class GameWorldService {
public:
    GameWorldService() = delete;
    GameWorldService(CUnrealEntryPointBinder auto* unreal_entry_point_binder)
    {
        SP_ASSERT(unreal_entry_point_binder);

        post_world_initialization_handle_ = FWorldDelegates::OnPostWorldInitialization.AddRaw(this, &GameWorldService::postWorldInitializationHandler);
        world_cleanup_handle_ = FWorldDelegates::OnWorldCleanup.AddRaw(this, &GameWorldService::worldCleanupHandler);

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "pause_game", [this]() -> void {
            SP_ASSERT(world_);
            UGameplayStatics::SetGamePaused(world_, true);
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "unpause_game", [this]() -> void {
            SP_ASSERT(world_);
            UGameplayStatics::SetGamePaused(world_, false);
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "open_level", [this](const std::string& desired_level_name) -> void {
            SP_ASSERT(world_);
            SP_LOG("Opening level: ", desired_level_name);
            UGameplayStatics::OpenLevel(world_, Unreal::toFName(desired_level_name));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_current_level_name", [this]() -> std::string {
            SP_ASSERT(world_);
            return Unreal::toStdString(world_->GetName());
        });
    }

    ~GameWorldService()
    {
        FWorldDelegates::OnWorldCleanup.Remove(world_cleanup_handle_);
        FWorldDelegates::OnPostWorldInitialization.Remove(post_world_initialization_handle_);

        world_cleanup_handle_.Reset();
        post_world_initialization_handle_.Reset();
    }

    void postWorldInitializationHandler(UWorld* world, const UWorld::InitializationValues initialization_values)
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

    void worldCleanupHandler(UWorld* world, bool session_ended, bool cleanup_resources)
    {
        SP_LOG_CURRENT_FUNCTION();
        SP_ASSERT(world);

        // We only need to perform any additional steps if the world being cleaned up is the world we cached in our world_ member variable.
        if (world == world_) {
            // clear cached world_ pointer
            world_ = nullptr;
        }
    }

private:

    // FDelegateHandle objects corresponding to each event handler defined in this class
    FDelegateHandle post_world_initialization_handle_;
    FDelegateHandle world_cleanup_handle_;

    // store a local reference to the game world
    UWorld* world_ = nullptr;
};
