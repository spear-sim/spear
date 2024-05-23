//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <string>
#include <vector>

#include <Delegates/IDelegateInstance.h> // FDelegateHandle
#include <Engine/World.h>                // FWorldDelegates

#include "SpCore/Assert.h"

#include "SpServices/EntryPointBinder.h"
#include "SpServices/Legacy/Agent.h"
#include "SpServices/Legacy/NavMesh.h"
#include "SpServices/Legacy/Task.h"


class LegacyService {
public:
    LegacyService() = delete;
    LegacyService(CUnrealEntryPointBinder auto* unreal_entry_point_binder)
    {
        post_world_initialization_handle_ = FWorldDelegates::OnPostWorldInitialization.AddRaw(this, &LegacyService::postWorldInitializationHandler);
        world_cleanup_handle_ = FWorldDelegates::OnWorldCleanup.AddRaw(this, &LegacyService::worldCleanupHandler);

        unreal_entry_point_binder->bindFuncNoUnreal("legacy_service", "get_action_space", [this]() -> std::map<std::string, ArrayDesc> {
            SP_ASSERT(agent_);
            return agent_->getActionSpace();
        });

        unreal_entry_point_binder->bindFuncNoUnreal("legacy_service", "get_observation_space", [this]() -> std::map<std::string, ArrayDesc> {
            SP_ASSERT(agent_);
            return agent_->getObservationSpace();
        });

        unreal_entry_point_binder->bindFuncNoUnreal("legacy_service", "get_agent_step_info_space", [this]() -> std::map<std::string, ArrayDesc> {
            SP_ASSERT(agent_);
            return agent_->getStepInfoSpace();
        });

        unreal_entry_point_binder->bindFuncNoUnreal("legacy_service", "get_task_step_info_space", [this]() -> std::map<std::string, ArrayDesc> {
            SP_ASSERT(task_);
            return task_->getStepInfoSpace();
        });

        unreal_entry_point_binder->bindFuncUnreal("legacy_service", "apply_action", [this](const std::map<std::string, std::vector<uint8_t>>& action) -> void {
            SP_ASSERT(agent_);
            agent_->applyAction(action);
        });

        unreal_entry_point_binder->bindFuncUnreal("legacy_service", "get_observation", [this]() -> std::map<std::string, std::vector<uint8_t>> {
            SP_ASSERT(agent_);
            return agent_->getObservation();
        });

        unreal_entry_point_binder->bindFuncUnreal("legacy_service", "get_reward", [this]() -> float {
            SP_ASSERT(task_);
            return task_->getReward();
        });

        unreal_entry_point_binder->bindFuncUnreal("legacy_service", "is_episode_done", [this]() -> bool {
            SP_ASSERT(task_);
            return task_->isEpisodeDone();
        });

        unreal_entry_point_binder->bindFuncUnreal("legacy_service", "get_agent_step_info", [this]() -> std::map<std::string, std::vector<uint8_t>> {
            SP_ASSERT(agent_);
            return agent_->getStepInfo();
        });

        unreal_entry_point_binder->bindFuncUnreal("legacy_service", "get_task_step_info", [this]() -> std::map<std::string, std::vector<uint8_t>> {
            SP_ASSERT(task_);
            return task_->getStepInfo();
        });

        unreal_entry_point_binder->bindFuncUnreal("legacy_service", "reset_agent", [this]() -> void {
            SP_ASSERT(agent_);
            agent_->reset();
        });

        unreal_entry_point_binder->bindFuncUnreal("legacy_service", "reset_task", [this]() -> void {
            SP_ASSERT(task_);
            task_->reset();
        });

        unreal_entry_point_binder->bindFuncUnreal("legacy_service", "is_agent_ready", [this]() -> bool {
            SP_ASSERT(agent_);
            return agent_->isReady();
        });

        unreal_entry_point_binder->bindFuncUnreal("legacy_service", "is_task_ready", [this]() -> bool {
            SP_ASSERT(task_);
            return task_->isReady();
        });

        unreal_entry_point_binder->bindFuncUnreal("legacy_service", "get_random_points",
            [this](const int& num_points) -> std::vector<double> {
                SP_ASSERT(nav_mesh_);
                return nav_mesh_->getRandomPoints(num_points);
        });

        unreal_entry_point_binder->bindFuncUnreal("legacy_service", "get_random_reachable_points_in_radius",
            [this](const std::vector<double>& initial_points, const float& radius) -> std::vector<double> {
                SP_ASSERT(nav_mesh_);
                return nav_mesh_->getRandomReachablePointsInRadius(initial_points, radius);
        });

        unreal_entry_point_binder->bindFuncUnreal("legacy_service", "get_paths",
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

    void postWorldInitializationHandler(UWorld* world, const UWorld::InitializationValues initialization_values);
    void worldCleanupHandler(UWorld* world, bool session_ended, bool cleanup_resources);
    void worldBeginPlayHandler();

private:
    FDelegateHandle post_world_initialization_handle_;
    FDelegateHandle world_begin_play_handle_;
    FDelegateHandle world_cleanup_handle_;

    UWorld* world_ = nullptr;

    // Unreal life cycle state
    bool has_world_begin_play_executed_ = false;
    bool open_level_pending_ = false;

    // OpenAI Gym helper objects
    std::unique_ptr<Agent> agent_ = nullptr;
    std::unique_ptr<Task> task_ = nullptr;

    // Navmesh helper object
    std::unique_ptr<NavMesh> nav_mesh_ = nullptr;
};
