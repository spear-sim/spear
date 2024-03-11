//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <concepts> // std::same_as
#include <string>
#include <vector>

#include <Delegates/IDelegateInstance.h> // FDelegateHandle
#include <Engine/Engine.h>               // GEngine
#include <Engine/World.h>                // UWorld
#include <Kismet/GameplayStatics.h>

#include <SpCore/Assert.h>
#include <SpCore/Config.h>
#include <SpCore/Log.h>
#include <SpCore/Unreal.h>
#include <SpEngine/EngineService.h> // CEntryPointBinder

class GameWorldService {
public:
    GameWorldService() = delete;
	GameWorldService(CEntryPointBinder auto* entry_point_binder)
	{
        post_world_initialization_handle_ = FWorldDelegates::OnPostWorldInitialization.AddRaw(this, &GameWorldService::postWorldInitializationEventHandler);
        world_cleanup_handle_ = FWorldDelegates::OnWorldCleanup.AddRaw(this, &GameWorldService::worldCleanupEventHandler);

		entry_point_binder->bind("game_world_service", "pause_game", [this]() -> void {
            SP_ASSERT(world_);
            SP_LOG("Pausing the game...");
            UGameplayStatics::SetGamePaused(world_, true);
	    });

        entry_point_binder->bind("game_world_service", "unpause_game", [this](int a, int b) -> void {
            SP_ASSERT(world_);
            SP_LOG("Unpausing the game...");
            UGameplayStatics::SetGamePaused(world_, false);
        });

        //entry_point_binder->bind("game_world_service", "open_level", [this](std::string scene_id="", std::string map_id="") -> void {
        //    SP_ASSERT(world_);
        //    std::string desired_world_path_name;
        //    std::string desired_level_name;
        //    if (scene_id != "") {
        //        if (map_id == "") {
        //            map_id = scene_id;
        //        }
        //        desired_world_path_name = "/Game/Scenes/" + scene_id + "/Maps/" + map_id + "." + map_id;
        //        desired_level_name = "/Game/Scenes/" + scene_id + "/Maps/" + map_id;
        //    }

        //    // if the current world is not the desired one, open the desired one
        //    bool open_level = desired_world_path_name != "" && desired_world_path_name != Unreal::toStdString(world_->GetPathName());

        //    SP_LOG("scene_id:                ", scene_id);
        //    SP_LOG("map_id:                  ", map_id);
        //    SP_LOG("desired_world_path_name: ", desired_world_path_name);
        //    SP_LOG("desired_level_name:      ", desired_level_name);
        //    SP_LOG("world_->GetPathName():   ", Unreal::toStdString(world_->GetPathName()));
        //    SP_LOG("open_level:              ", open_level);

        //    if (open_level) {
        //        SP_LOG("Opening level: ", desired_level_name);

        //        // if we're at this line of code and OpenLevel is already pending, it means we failed
        //        SP_ASSERT(!open_level_pending_);

        //        UGameplayStatics::OpenLevel(world_, Unreal::toFName(desired_level_name));

        //        open_level_pending_ = true;
        //    } else {
        //        SP_LOG("Level:", desired_level_name, "is currently open, so will not try to open it again.");
        //        open_level_pending_ = false;
        //    }
        //});
	}

    ~GameWorldService()
    {
        FWorldDelegates::OnWorldCleanup.Remove(world_cleanup_handle_);
        FWorldDelegates::OnPostWorldInitialization.Remove(post_world_initialization_handle_);

        world_cleanup_handle_.Reset();
        post_world_initialization_handle_.Reset();
    }

    void postWorldInitializationEventHandler(UWorld* world, const UWorld::InitializationValues initialization_values)
    {
        SP_LOG_CURRENT_FUNCTION();
        SP_ASSERT(world);

#if WITH_EDITOR // defined in an auto-generated header
        bool world_is_ready = world->IsGameWorld();
#else
        bool world_is_ready = world->IsGameWorld() && GEngine->GetWorldContextFromWorld(world);
#endif

        if (world_is_ready) {
            // we expect worldCleanupEventHandler(...) to be called before a new world is created
            SP_ASSERT(!world_);

            // cache local reference to the UWorld
            world_ = world;
        }
    }

    void worldCleanupEventHandler(UWorld* world, bool session_ended, bool cleanup_resources)
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

    // Unreal lifecycle state
    bool open_level_pending_ = false;
};
