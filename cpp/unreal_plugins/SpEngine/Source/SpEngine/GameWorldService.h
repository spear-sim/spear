//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <concepts>  // std::same_as

#include <Delegates/IDelegateInstance.h> // FDelegateHandle
#include <Engine/Engine.h>               // GEngine
#include <Engine/World.h>                // UWorld
#include <Kismet/GameplayStatics.h>
#include <Misc/App.h>
#include <PhysicsEngine/PhysicsSettings.h>

#include <SpCore/Assert.h>
#include <SpCore/Config.h>
#include <SpCore/Log.h>
#include <SpCore/Unreal.h>
#include <SpEngine/EngineService.h> // CEntryPointBinder

class GameWorldService {
public:
	GameWorldService(CEntryPointBinder auto* entry_point_binder)
	{
        post_world_initialization_handle_ = FWorldDelegates::OnPostWorldInitialization.AddRaw(this, &GameWorldService::postWorldInitializationEventHandler);
        world_cleanup_handle_ = FWorldDelegates::OnWorldCleanup.AddRaw(this, &GameWorldService::worldCleanupEventHandler);

		entry_point_binder->bind("game_world_service", "pause_game", [this]() -> void {
            SP_LOG("Pausing the game...");
            UGameplayStatics::SetGamePaused(world_, true);
	    });

        entry_point_binder->bind("game_world_service", "unpause_game", [this]() -> void {
            SP_LOG("Unpausing the game...");
            UGameplayStatics::SetGamePaused(world_, false);
        });
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

            auto scene_id = Config::get<std::string>("SP_ENGINE.SCENE_ID");
            auto map_id = Config::get<std::string>("SP_ENGINE.MAP_ID");

            std::string desired_world_path_name;
            std::string desired_level_name;
            if (scene_id != "") {
                if (map_id == "") {
                    map_id = scene_id;
                }
                desired_world_path_name = "/Game/Scenes/" + scene_id + "/Maps/" + map_id + "." + map_id;
                desired_level_name = "/Game/Scenes/" + scene_id + "/Maps/" + map_id;
            }

            // if the current world is not the desired one, open the desired one
            bool open_level = desired_world_path_name != "" && desired_world_path_name != Unreal::toStdString(world->GetPathName());

            SP_LOG("scene_id:                ", scene_id);
            SP_LOG("map_id:                  ", map_id);
            SP_LOG("desired_world_path_name: ", desired_world_path_name);
            SP_LOG("desired_level_name:      ", desired_level_name);
            SP_LOG("world->GetPathName():    ", Unreal::toStdString(world->GetPathName()));
            SP_LOG("open_level:              ", open_level);

            if (open_level) {
                SP_LOG("Opening level: ", desired_level_name);

                // if we're at this line of code and OpenLevel is already pending, it means we failed
                SP_ASSERT(!open_level_pending_);

                UGameplayStatics::OpenLevel(world, Unreal::toFName(desired_level_name));
                open_level_pending_ = true;

            } else {
                open_level_pending_ = false;

                // we expect worldCleanupEventHandler(...) to be called before a new world is created
                SP_ASSERT(!world_);

                // cache local reference to the UWorld
                world_ = world;

                // We need to defer initializing this handler until after we have a valid world_ pointer,
                // and we defer the rest of our initialization code until the OnWorldBeginPlay event. We
                // wrap this code in an if block to enable interactive navigation mode, which will potentially
                // need to load a new map via the config system, but should not initialize the rest of our
                // code.
                //if (Config::get<std::string>("SP_ENGINE.INTERACTION_MODE") == "programmatic") {
                    world_begin_play_handle_ = world_->OnWorldBeginPlay.AddRaw(this, &GameWorldService::worldBeginPlayEventHandler);
                //}
            }
        }
    }

    void worldBeginPlayEventHandler()
    {
        SP_LOG_CURRENT_FUNCTION();

        // execute optional console commands from python client
        for (auto& command : Config::get<std::vector<std::string>>("SP_ENGINE.CUSTOM_UNREAL_CONSOLE_COMMANDS")) {
            GEngine->Exec(world_, *Unreal::toFString(command));
        }

        // set physics parameters
        UPhysicsSettings* physics_settings = UPhysicsSettings::Get();
        physics_settings->bEnableEnhancedDeterminism = Config::get<bool>("SP_ENGINE.PHYSICS.ENABLE_ENHANCED_DETERMINISM");
        physics_settings->bSubstepping = Config::get<bool>("SP_ENGINE.PHYSICS.ENABLE_SUBSTEPPING");
        physics_settings->MaxSubstepDeltaTime = Config::get<float>("SP_ENGINE.PHYSICS.MAX_SUBSTEP_DELTA_TIME");
        physics_settings->MaxSubsteps = Config::get<int32>("SP_ENGINE.PHYSICS.MAX_SUBSTEPS");

        // Check that the physics substepping parameters match our desired simulation step time.
        // See https://carla.readthedocs.io/en/latest/adv_synchrony_timestep for more details.
        float step_time = Config::get<float>("SP_ENGINE.PHYSICS.SIMULATION_STEP_TIME");
        if (physics_settings->bSubstepping) {
            float max_step_time_allowed_when_substepping = physics_settings->MaxSubstepDeltaTime * physics_settings->MaxSubsteps;
            SP_ASSERT(step_time <= max_step_time_allowed_when_substepping);
        }

        // set fixed simulation step time in seconds, FApp::SetBenchmarking(true) is also needed to enable
        FApp::SetBenchmarking(true);
        FApp::SetFixedDeltaTime(Config::get<double>("SP_ENGINE.PHYSICS.SIMULATION_STEP_TIME"));

        // pause the game
        UGameplayStatics::SetGamePaused(world_, true);

        has_world_begin_play_executed_ = true;
    }

    void worldCleanupEventHandler(UWorld* world, bool session_ended, bool cleanup_resources)
    {
        SP_LOG_CURRENT_FUNCTION();
        SP_ASSERT(world);

        // We only need to perform any additional steps if the world being cleaned up is the world we cached in our world_ member variable.
        if (world == world_) {

            // The worldCleanupEventHandler(...) function is called for all worlds, but some local state (such as rpc_server_ and agent_)
            // is initialized only when worldBeginPlayEventHandler(...) is called for a particular world. So we check if worldBeginPlayEventHandler(...)
            // has been executed.
            if (has_world_begin_play_executed_) {
                has_world_begin_play_executed_ = false;
            }

            // remove event handlers bound to this world before world gets cleaned up
            //if (Config::get<std::string>("SP_ENGINE.INTERACTION_MODE") == "programmatic") {
                world_->OnWorldBeginPlay.Remove(world_begin_play_handle_);
                world_begin_play_handle_.Reset();
            //}

            // clear cached world_ pointer
            world_ = nullptr;
        }
    }

    ~GameWorldService()
    {
        // If an object of this class is destroyed in the middle of simulation for some reason, raise an error.
        // We expect worldCleanUpEvenHandler(...) to be called before ~GameWorldService().
        SP_ASSERT(!world_begin_play_handle_.IsValid());

        FWorldDelegates::OnWorldCleanup.Remove(world_cleanup_handle_);
        FWorldDelegates::OnPostWorldInitialization.Remove(post_world_initialization_handle_);

        world_cleanup_handle_.Reset();
        post_world_initialization_handle_.Reset();
    }

private:

    // FDelegateHandle objects corresponding to each event handler defined in this class
    FDelegateHandle post_world_initialization_handle_;
    FDelegateHandle world_begin_play_handle_;
    FDelegateHandle world_cleanup_handle_;

    // store a local reference to the game world
    UWorld* world_ = nullptr;

    // Unreal lifecycle state
    bool has_world_begin_play_executed_ = false;
    bool open_level_pending_ = false;
};
