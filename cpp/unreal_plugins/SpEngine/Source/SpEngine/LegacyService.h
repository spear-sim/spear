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
#include <Misc/App.h>
#include <PhysicsEngine/PhysicsSettings.h>

#include <SpCore/Assert.h>
#include <SpCore/Config.h>
#include <SpCore/Log.h>
#include <SpCore/Unreal.h>
#include <SpEngine/EngineService.h> // CEntryPointBinder
#include <SpEngine/NavMesh.h>

class LegacyService {
public:
	LegacyService() = delete;
	LegacyService(CEntryPointBinder auto* entry_point_binder)
	{
        post_world_initialization_handle_ = FWorldDelegates::OnPostWorldInitialization.AddRaw(this, &LegacyService::postWorldInitializationEventHandler);
        world_cleanup_handle_ = FWorldDelegates::OnWorldCleanup.AddRaw(this, &LegacyService::worldCleanupEventHandler);

        entry_point_binder->bind("legacy_service", "get_random_points",
            [this](const int& num_points) -> std::vector<double> {
                return nav_mesh_.getRandomPoints(num_points);
        });

        entry_point_binder->bind("legacy_service", "get_random_reachable_points_in_radius",
            [this](const std::vector<double>& initial_points, const float& radius) -> std::vector<double> {
                return nav_mesh_.getRandomReachablePointsInRadius(initial_points, radius);
        });

        entry_point_binder->bind("legacy_service", "get_paths",
            [this](const std::vector<double>& initial_points, const std::vector<double>& goal_points) -> std::vector<std::vector<double>> {
                return nav_mesh_.getPaths(initial_points, goal_points);
        });
	}

    ~LegacyService()
    {
        // If an object of this class is destroyed in the middle of simulation for some reason, raise an error.
        // We expect worldCleanUpEvenHandler(...) to be called before ~LegacyService().
        SP_ASSERT(!world_begin_play_handle_.IsValid());

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

            // We need to defer initializing this handler until after we have a valid world_ pointer,
            // and we defer the rest of our initialization code until the OnWorldBeginPlay event. We
            // wrap this code in an if block to enable interactive navigation mode, which will potentially
            // need to load a new map via the config system, but should not initialize the rest of our
            // code.
            //if (Config::get<std::string>("SP_ENGINE.INTERACTION_MODE") == "programmatic") {
            world_begin_play_handle_ = world_->OnWorldBeginPlay.AddRaw(this, &LegacyService::worldBeginPlayEventHandler);
            //}
        }
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

                nav_mesh_.cleanUpObjectReferences();
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

    void worldBeginPlayEventHandler()
    {
        SP_LOG_CURRENT_FUNCTION();
        SP_ASSERT(world_);

        // execute optional console commands from python client
        if (Config::s_initialized_) {
            for (auto& command : Config::get<std::vector<std::string>>("SP_ENGINE.CUSTOM_UNREAL_CONSOLE_COMMANDS")) {
                GEngine->Exec(world_, *Unreal::toFString(command));
            }
        }

        // set physics parameters
        UPhysicsSettings* physics_settings = UPhysicsSettings::Get();
        if (Config::s_initialized_) {
            physics_settings->bEnableEnhancedDeterminism = Config::get<bool>("SP_ENGINE.PHYSICS.ENABLE_ENHANCED_DETERMINISM");
            physics_settings->bSubstepping = Config::get<bool>("SP_ENGINE.PHYSICS.ENABLE_SUBSTEPPING");
            physics_settings->MaxSubstepDeltaTime = Config::get<float>("SP_ENGINE.PHYSICS.MAX_SUBSTEP_DELTA_TIME");
            physics_settings->MaxSubsteps = Config::get<int32>("SP_ENGINE.PHYSICS.MAX_SUBSTEPS");
        }
        else {
            physics_settings->bEnableEnhancedDeterminism = true;
            physics_settings->bSubstepping = true;
            physics_settings->MaxSubstepDeltaTime = 0.01;
            physics_settings->MaxSubsteps = 100;
        }

        // Check that the physics substepping parameters match our desired simulation step time.
        // See https://carla.readthedocs.io/en/latest/adv_synchrony_timestep for more details.
        float step_time = 0.05;
        if (Config::s_initialized_) {
            step_time = Config::get<float>("SP_ENGINE.PHYSICS.SIMULATION_STEP_TIME");
        }

        if (physics_settings->bSubstepping) {
            float max_step_time_allowed_when_substepping = physics_settings->MaxSubstepDeltaTime * physics_settings->MaxSubsteps;
            SP_ASSERT(step_time <= max_step_time_allowed_when_substepping);
        }

        // set fixed simulation step time in seconds, FApp::SetBenchmarking(true) is also needed to enable
        FApp::SetBenchmarking(true);
        FApp::SetFixedDeltaTime(step_time);

        // pause the game
        UGameplayStatics::SetGamePaused(world_, true);

        // deferred initialization
        //agent_->findObjectReferences(world_);
        //task_->findObjectReferences(world_);
        nav_mesh_.findObjectReferences(world_);

        has_world_begin_play_executed_ = true;
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

    NavMeshV2 nav_mesh_;
};
