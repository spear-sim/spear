//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SimulationController/ImitationLearningTask.h"

#include <fstream>
#include <map>
#include <string>
#include <vector>

#include <Components/SceneComponent.h>
#include <Delegates/IDelegateInstance.h>
#include <DrawDebugHelpers.h>
#include <Engine/EngineTypes.h>
#include <Engine/World.h>
#include <GameFramework/Actor.h>
#include <Math/Rotator.h>
#include <Math/Vector.h>

#include "CoreUtils/ArrayDesc.h"
#include "CoreUtils/Assert.h"
#include "CoreUtils/Config.h"
#include "CoreUtils/Std.h"
#include "CoreUtils/Unreal.h"
#include "SimulationController/ActorHitEventComponent.h"

ImitationLearningTask::ImitationLearningTask(UWorld* world)
{
    FActorSpawnParameters actor_spawn_params;
    actor_spawn_params.Name = Unreal::toFName(Config::get<std::string>("SIMULATION_CONTROLLER.IMITATION_LEARNING_TASK.GOAL_ACTOR_NAME"));
    actor_spawn_params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    goal_actor_ = world->SpawnActor<AActor>(FVector::ZeroVector, FRotator::ZeroRotator, actor_spawn_params);
    SP_ASSERT(goal_actor_);

    auto scene_component = NewObject<USceneComponent>(goal_actor_);
    scene_component->SetMobility(EComponentMobility::Movable);
    goal_actor_->SetRootComponent(scene_component);

    parent_actor_ = world->SpawnActor<AActor>();
    SP_ASSERT(parent_actor_);

    // Create UActorHitEvent but don't subscribe to any actors yet
    actor_hit_event_component_ = NewObject<UActorHitEventComponent>(parent_actor_);
    SP_ASSERT(actor_hit_event_component_);
    actor_hit_event_component_->RegisterComponent();
    actor_hit_event_handle_ = actor_hit_event_component_->delegate_.AddRaw(this, &ImitationLearningTask::actorHitEventHandler);

    // Get start and end goal locations from a file
    if (Config::get<bool>("SIMULATION_CONTROLLER.IMITATION_LEARNING_TASK.LOAD_TRAJECTORY_FROM_FILE")) {
        getPositionsFromFile();
    }
}

ImitationLearningTask::~ImitationLearningTask()
{
    if (Config::get<bool>("SIMULATION_CONTROLLER.IMITATION_LEARNING_TASK.LOAD_TRAJECTORY_FROM_FILE")) {
        clearPositions();
    }
    
    SP_ASSERT(actor_hit_event_component_);
    actor_hit_event_component_->delegate_.Remove(actor_hit_event_handle_);
    actor_hit_event_handle_.Reset();
    actor_hit_event_component_->DestroyComponent();
    actor_hit_event_component_ = nullptr;

    SP_ASSERT(parent_actor_);
    parent_actor_->Destroy();
    parent_actor_ = nullptr;

    SP_ASSERT(goal_actor_);
    goal_actor_->Destroy();
    goal_actor_ = nullptr;
}

void ImitationLearningTask::findObjectReferences(UWorld* world)
{
    agent_actor_ = Unreal::findActorByName(world, Config::get<std::string>("SIMULATION_CONTROLLER.IMITATION_LEARNING_TASK.AGENT_ACTOR_NAME"));
    SP_ASSERT(agent_actor_);

    bool return_null_if_not_found = false;
    obstacle_ignore_actors_ = Unreal::findActorsByName(
        world, Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.IMITATION_LEARNING_TASK.OBSTACLE_IGNORE_ACTOR_NAMES"), return_null_if_not_found);

    // Subscribe to the agent actor now that we have obtained a reference to it
    actor_hit_event_component_->subscribeToActor(agent_actor_);
}

void ImitationLearningTask::cleanUpObjectReferences()
{
    SP_ASSERT(actor_hit_event_component_);
    actor_hit_event_component_->unsubscribeFromActor(agent_actor_);

    obstacle_ignore_actors_.clear();

    SP_ASSERT(agent_actor_);
    agent_actor_ = nullptr;
}

void ImitationLearningTask::beginFrame()
{
    hit_goal_ = false;
    hit_obstacle_ = false;
}

void ImitationLearningTask::endFrame() {}

float ImitationLearningTask::getReward() const
{
    return -std::numeric_limits<float>::infinity();
}

bool ImitationLearningTask::isEpisodeDone() const
{
    return hit_goal_ || hit_obstacle_;
}

std::map<std::string, ArrayDesc> ImitationLearningTask::getStepInfoSpace() const
{
    std::map<std::string, ArrayDesc> step_info_space;
    ArrayDesc array_desc;

    array_desc.low_ = 0.0;
    array_desc.high_ = 1.0;
    array_desc.shape_ = {1};
    array_desc.datatype_ = DataType::UInteger8;
    step_info_space["hit_goal"] = std::move(array_desc);

    array_desc.low_ = 0.0;
    array_desc.high_ = 1.0;
    array_desc.shape_ = {1};
    array_desc.datatype_ = DataType::UInteger8;
    step_info_space["hit_obstacle"] = std::move(array_desc);

    return step_info_space;
}

std::map<std::string, std::vector<uint8_t>> ImitationLearningTask::getStepInfo() const
{
    std::map<std::string, std::vector<uint8_t>> step_info;

    step_info["hit_goal"] = std::vector<uint8_t>{hit_goal_};
    step_info["hit_obstacle"] = std::vector<uint8_t>{hit_obstacle_};

    return step_info;
}

void ImitationLearningTask::reset()
{
    // Set agent and goal positions
    bool sweep = false;
    FHitResult* hit_result = nullptr;

    if (Config::get<bool>("SIMULATION_CONTROLLER.IMITATION_LEARNING_TASK.LOAD_TRAJECTORY_FROM_FILE")) {
        agent_actor_->SetActorLocationAndRotation(
            agent_initial_positions_.at(position_index_), FRotator::ZeroRotator, sweep, hit_result, ETeleportType::TeleportPhysics);
        goal_actor_->SetActorLocationAndRotation(
            agent_goal_positions_.at(position_index_), FRotator::ZeroRotator, sweep, hit_result, ETeleportType::TeleportPhysics);
    }

    // Increment position_index_
    if (position_index_ < agent_goal_positions_.size() - 1) { 
        position_index_++;
    }  else {
        position_index_ = 0;
    }
}

bool ImitationLearningTask::isReady() const
{
    return true;
}

void ImitationLearningTask::actorHitEventHandler(AActor* self_actor, AActor* other_actor, FVector normal_impulse, const FHitResult& hit_result)
{
    SP_ASSERT(self_actor == agent_actor_);

    if (other_actor == goal_actor_) {
        hit_goal_ = true;
    } else if (!Std::contains(obstacle_ignore_actors_, other_actor)) {
        hit_obstacle_ = true;
    }
}

void ImitationLearningTask::getPositionsFromFile()
{
    agent_initial_positions_.clear();
    agent_goal_positions_.clear();
    position_index_ = -1;

    // Create an input filestream 
    std::ifstream fs(Config::get<std::string>("SIMULATION_CONTROLLER.IMITATION_LEARNING_TASK.TRAJECTORY_LOCATIONS_FILE")); 
    SP_ASSERT(fs.is_open());

    // Read file data, line-by-line in the format:
    // scene_id, start_location_x, start_location_y, start_location_z, end_location_x, end_location_y, end_location_z
    std::string line;
    std::getline(fs, line); // header
    while (std::getline(fs, line)) {        
        std::vector<std::string> tokens = Std::tokenize(line, ",");
        SP_ASSERT(tokens.size() == 7);
        std::string scene_id = tokens.at(0);
        FVector init(std::stod(tokens.at(1)), std::stod(tokens.at(2)), std::stod(tokens.at(3)));
        FVector goal(std::stod(tokens.at(4)), std::stod(tokens.at(5)), std::stod(tokens.at(6)));

        // If the scene id matches the currently opened map, then add to our list of positions
        if(scene_id == Config::get<std::string>("SIMULATION_CONTROLLER.SCENE_ID")) {
            agent_initial_positions_.push_back(init);
            agent_goal_positions_.push_back(goal);
        }
    }

    // Close file
    fs.close();

    position_index_ = 0;
}

void ImitationLearningTask::clearPositions()
{
    agent_initial_positions_.clear();
    agent_goal_positions_.clear();
    position_index_ = -1;
}
