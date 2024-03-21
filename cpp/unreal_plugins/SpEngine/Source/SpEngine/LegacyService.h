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
#include <Misc/App.h>
#include <PhysicsEngine/PhysicsSettings.h>

#include <SpCore/Assert.h>
#include <SpCore/Config.h>
#include <SpCore/Log.h>
#include <SpCore/Unreal.h>
#include <SpEngine/EngineService.h> // CEntryPointBinder
#include "SpEngine/Legacy/Agent.h"
#include "SpEngine/Legacy/ClassRegistrationUtils.h"
#include "SpEngine/Legacy/ImitationLearningTask.h"
#include "SpEngine/Legacy/NavMesh.h"
#include "SpEngine/Legacy/NullAgent.h"
#include "SpEngine/Legacy/NullTask.h"

class LegacyService {
public:
	LegacyService() = delete;
	LegacyService(CEntryPointBinder auto* entry_point_binder)
	{
        post_world_initialization_handle_ = FWorldDelegates::OnPostWorldInitialization.AddRaw(this, &LegacyService::postWorldInitializationHandler);
        world_cleanup_handle_ = FWorldDelegates::OnWorldCleanup.AddRaw(this, &LegacyService::worldCleanupHandler);

        entry_point_binder->bind_func("legacy_service", "get_action_space", [this]() -> std::map<std::string, ArrayDesc> {
            SP_ASSERT(agent_);
            return agent_->getActionSpace();
        });

        entry_point_binder->bind_func("legacy_service", "get_observation_space", [this]() -> std::map<std::string, ArrayDesc> {
            SP_ASSERT(agent_);
            return agent_->getObservationSpace();
        });

        entry_point_binder->bind_func("legacy_service", "get_agent_step_info_space", [this]() -> std::map<std::string, ArrayDesc> {
            SP_ASSERT(agent_);
            return agent_->getStepInfoSpace();
        });

        entry_point_binder->bind_func("legacy_service", "get_task_step_info_space", [this]() -> std::map<std::string, ArrayDesc> {
            SP_ASSERT(task_);
            return task_->getStepInfoSpace();
        });

        entry_point_binder->bind_func_wrapped("legacy_service", "apply_action", [this](const std::map<std::string, std::vector<uint8_t>>& action) -> void {
            SP_ASSERT(agent_);
            agent_->applyAction(action);
        });

        entry_point_binder->bind_func_wrapped("legacy_service", "get_observation", [this]() -> std::map<std::string, std::vector<uint8_t>> {
            SP_ASSERT(agent_);
            return agent_->getObservation();
        });

        entry_point_binder->bind_func_wrapped("legacy_service", "get_reward", [this]() -> float {
            SP_ASSERT(task_);
            return task_->getReward();
        });

        entry_point_binder->bind_func_wrapped("legacy_service", "is_episode_done", [this]() -> bool {
            SP_ASSERT(task_);
            return task_->isEpisodeDone();
        });

        entry_point_binder->bind_func_wrapped("legacy_service", "get_agent_step_info", [this]() -> std::map<std::string, std::vector<uint8_t>> {
            SP_ASSERT(agent_);
            return agent_->getStepInfo();
        });

        entry_point_binder->bind_func_wrapped("legacy_service", "get_task_step_info", [this]() -> std::map<std::string, std::vector<uint8_t>> {
            SP_ASSERT(task_);
            return task_->getStepInfo();
        });

        entry_point_binder->bind_func_wrapped("legacy_service", "reset_agent", [this]() -> void {
            SP_ASSERT(agent_);
            agent_->reset();
        });

        entry_point_binder->bind_func_wrapped("legacy_service", "reset_task", [this]() -> void {
            SP_ASSERT(task_);
            task_->reset();
        });

        entry_point_binder->bind_func_wrapped("legacy_service", "is_agent_ready", [this]() -> bool {
            SP_ASSERT(agent_);
            return agent_->isReady();
        });

        entry_point_binder->bind_func_wrapped("legacy_service", "is_task_ready", [this]() -> bool {
            SP_ASSERT(task_);
            return task_->isReady();
        });

        entry_point_binder->bind_func_wrapped("legacy_service", "get_random_points",
            [this](const int& num_points) -> std::vector<double> {
                SP_ASSERT(nav_mesh_);
                return nav_mesh_->getRandomPoints(num_points);
        });

        entry_point_binder->bind_func_wrapped("legacy_service", "get_random_reachable_points_in_radius",
            [this](const std::vector<double>& initial_points, const float& radius) -> std::vector<double> {
                SP_ASSERT(nav_mesh_);
                return nav_mesh_->getRandomReachablePointsInRadius(initial_points, radius);
        });

        entry_point_binder->bind_func_wrapped("legacy_service", "get_paths",
            [this](const std::vector<double>& initial_points, const std::vector<double>& goal_points) -> std::vector<std::vector<double>> {
                SP_ASSERT(nav_mesh_);
                return nav_mesh_->getPaths(initial_points, goal_points);
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

    void postWorldInitializationHandler(UWorld* world, const UWorld::InitializationValues initialization_values)
    {
        SP_LOG_CURRENT_FUNCTION();
        SP_ASSERT(world);

        #if WITH_EDITOR // defined in an auto-generated header
            bool world_is_ready = world->IsGameWorld();
        #else
            bool world_is_ready = world->IsGameWorld() && GEngine->GetWorldContextFromWorld(world);
        #endif

        if (world_is_ready) {

            std::string scene_id = "apartment_0000";
            std::string map_id = "apartment_0000";
            if (Config::s_initialized_) {
                scene_id = Config::get<std::string>("SP_ENGINE.SCENE_ID");
                map_id = Config::get<std::string>("SP_ENGINE.MAP_ID");
            }

            std::string desired_world_path_name = "";
            std::string desired_level_name = "";
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

                // we expect worldCleanupHandler(...) to be called before a new world is created
                SP_ASSERT(!world_);

                // cache local reference to the UWorld
                world_ = world;

                // We need to defer initializing this handler until after we have a valid world_ pointer,
                // and we defer the rest of our initialization code until the OnWorldBeginPlay event. We
                // wrap this code in an if block to enable interactive navigation mode, which will potentially
                // need to load a new map via the config system, but should not initialize the rest of our
                // code.
                //if (Config::s_initialized_ && Config::get<std::string>("SP_ENGINE.INTERACTION_MODE") == "programmatic") {
                    world_begin_play_handle_ = world_->OnWorldBeginPlay.AddRaw(this, &LegacyService::worldBeginPlayEventHandler);
                //}
            }
        }
    }

    void worldCleanupHandler(UWorld* world, bool session_ended, bool cleanup_resources)
    {
        SP_LOG_CURRENT_FUNCTION();
        SP_ASSERT(world);

        // We only need to perform any additional steps if the world being cleaned up is the world we cached in our world_ member variable.
        if (world == world_) {

            // The worldCleanupHandler(...) function is called for all worlds, but some local state (such as rpc_server_ and agent_)
            // is initialized only when worldBeginPlayEventHandler(...) is called for a particular world. So we check if worldBeginPlayEventHandler(...)
            // has been executed.
            if (has_world_begin_play_executed_) {
                has_world_begin_play_executed_ = false;

                SP_ASSERT(nav_mesh_);
                nav_mesh_->cleanUpObjectReferences();
                nav_mesh_ = nullptr;

                SP_ASSERT(task_);
                task_->cleanUpObjectReferences();
                task_ = nullptr;

                SP_ASSERT(agent_);
                agent_->cleanUpObjectReferences();
                agent_ = nullptr;
            }

            // remove event handlers bound to this world before world gets cleaned up
            //if (Config::s_initialized_ && Config::get<std::string>("SP_ENGINE.INTERACTION_MODE") == "programmatic") {
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
            physics_settings->bSubstepping               = Config::get<bool>("SP_ENGINE.PHYSICS.ENABLE_SUBSTEPPING");
            physics_settings->MaxSubstepDeltaTime        = Config::get<float>("SP_ENGINE.PHYSICS.MAX_SUBSTEP_DELTA_TIME");
            physics_settings->MaxSubsteps                = Config::get<int32>("SP_ENGINE.PHYSICS.MAX_SUBSTEPS");
        } else {
            physics_settings->bEnableEnhancedDeterminism = true;
            physics_settings->bSubstepping               = true;
            physics_settings->MaxSubstepDeltaTime        = 0.01;
            physics_settings->MaxSubsteps                = 100;
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

        if (Config::s_initialized_) {
            // create Agent
            agent_ = std::unique_ptr<Agent>(ClassRegistrationUtils::create(Agent::s_class_registrar_, Config::get<std::string>("SP_ENGINE.AGENT"), world_));

            // create Task
            if (Config::get<std::string>("SP_ENGINE.TASK") == "NullTask") {
                task_ = std::make_unique<NullTask>();
            } else if (Config::get<std::string>("SP_ENGINE.TASK") == "ImitationLearningTask") {
                task_ = std::make_unique<ImitationLearningTask>(world_);
            } else {
                SP_ASSERT(false);
            }
        } else {
            agent_ = std::unique_ptr<Agent>(ClassRegistrationUtils::create(Agent::s_class_registrar_, "NullAgent", world_));
            task_  = std::make_unique<NullTask>();
        }
        SP_ASSERT(agent_);
        SP_ASSERT(task_);

        // create NavMesh
        nav_mesh_ = std::make_unique<NavMesh>();
        SP_ASSERT(nav_mesh_);

        // deferred initialization
        agent_->findObjectReferences(world_);
        task_->findObjectReferences(world_);
        nav_mesh_->findObjectReferences(world_);

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

    std::unique_ptr<Agent> agent_ = nullptr;
    std::unique_ptr<Task> task_ = nullptr;
    std::unique_ptr<NavMesh> nav_mesh_ = nullptr;
};
