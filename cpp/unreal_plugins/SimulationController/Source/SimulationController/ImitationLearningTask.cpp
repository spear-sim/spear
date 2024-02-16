//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SimulationController/ImitationLearningTask.h"

#include <stdint.h> // uint8_t

#include <fstream> // std::ifstream
#include <limits>  // std::numeric_limits
#include <map>
#include <memory>  // std::make_unique
#include <string>  // std::getline, std::stod
#include <utility> // std::move
#include <vector>

#include <Components/SceneComponent.h>
#include <Engine/World.h>           // FActorSpawnParameters
#include <GameFramework/Actor.h>
#include <Math/Rotator.h>
#include <Math/Vector.h>

#include "SimulationController/ActorHitEventComponent.h"
#include "SimulationController/StandaloneComponent.h"
#include "SpCore/ArrayDesc.h"
#include "SpCore/Assert.h"
#include "SpCore/Config.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"

struct FHitResult;

ImitationLearningTask::ImitationLearningTask(UWorld* world)
{
    FActorSpawnParameters actor_spawn_parameters;
    actor_spawn_parameters.Name = Unreal::toFName(Config::get<std::string>("SIMULATION_CONTROLLER.IMITATION_LEARNING_TASK.GOAL_ACTOR_NAME"));
    actor_spawn_parameters.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    goal_actor_ = world->SpawnActor<AActor>(FVector::ZeroVector, FRotator::ZeroRotator, actor_spawn_parameters);
    SP_ASSERT(goal_actor_);

    // Although scene_component appears to not being used anywhere, it is required here to make goal_actor_ movable.
    auto scene_component = Unreal::createComponentOutsideOwnerConstructor<USceneComponent>(goal_actor_, "scene_component");
    SP_ASSERT(scene_component);
    scene_component->SetMobility(EComponentMobility::Movable);

    // Create UActorHitEventComponent but don't subscribe to any actors yet
    actor_hit_event_component_ = std::make_unique<StandaloneComponent<UActorHitEventComponent>>(world, "actor_hit_event_component");
    SP_ASSERT(actor_hit_event_component_);
    SP_ASSERT(actor_hit_event_component_->component_);

    // Get initial and goal locations of all episodes in the following format:
    //    scene_id, initial_location_x, initial_location_y, initial_location_z, goal_location_x, goal_location_y, goal_location_z
    agent_initial_locations_.clear();
    agent_goal_locations_.clear();
    episode_index_ = -1;

    // Read file data, line-by-line in the format:
    // scene_id, initial_location_x, initial_location_y, initial_location_z, goal_location_x, goal_location_y, goal_location_z
    std::ifstream fs(Config::get<std::string>("SIMULATION_CONTROLLER.IMITATION_LEARNING_TASK.EPISODES_FILE"));
    SP_ASSERT(fs.is_open());
    std::string line;
    std::getline(fs, line); // read header
    std::vector<std::string> tokens = Std::tokenize(line, ",");
    SP_ASSERT(tokens.size() == 7);
    SP_ASSERT(tokens.at(0) == "scene_id");
    SP_ASSERT(tokens.at(1) == "initial_location_x");
    SP_ASSERT(tokens.at(2) == "initial_location_y");
    SP_ASSERT(tokens.at(3) == "initial_location_z");
    SP_ASSERT(tokens.at(4) == "goal_location_x");
    SP_ASSERT(tokens.at(5) == "goal_location_y");
    SP_ASSERT(tokens.at(6) == "goal_location_z");    
    while (std::getline(fs, line)) {
        tokens = Std::tokenize(line, ",");
        SP_ASSERT(tokens.size() == 7);
        std::string scene_id = tokens.at(0);
        FVector initial_location = {std::stod(tokens.at(1)), std::stod(tokens.at(2)), std::stod(tokens.at(3))};
        FVector goal_location = {std::stod(tokens.at(4)), std::stod(tokens.at(5)), std::stod(tokens.at(6))};

        // If scene_id matches the currently opened map, then add to our list of episodes.
        // TODO (MR): Maybe scene_id should be passed in, because currently this lower-level code is
        // reading a config parameter that belongs to a higher-level system, which we usually avoid.
        // I think this is ok for now though, because we intend to migrate this code to Python soon.
        if (scene_id == Config::get<std::string>("SIMULATION_CONTROLLER.SCENE_ID")) {
            agent_initial_locations_.push_back(initial_location);
            agent_goal_locations_.push_back(goal_location);
        }
    }
    fs.close();

    episode_index_ = 0;
}

ImitationLearningTask::~ImitationLearningTask()
{
    agent_initial_locations_.clear();
    agent_goal_locations_.clear();
    episode_index_ = -1;

    SP_ASSERT(actor_hit_event_component_);
    actor_hit_event_component_ = nullptr;

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

    actor_hit_event_component_->component_->subscribe(agent_actor_);
    actor_hit_event_component_->component_->actor_hit_func_ =
        [this](AActor* self_actor, AActor* other_actor, FVector normal_impulse, const FHitResult& hit_result) -> void {
            SP_ASSERT(self_actor == agent_actor_);
            if (other_actor == goal_actor_) {
                hit_goal_ = true;
            } else if (!Std::contains(obstacle_ignore_actors_, other_actor)) {
                hit_obstacle_ = true;
            }
        };
}

void ImitationLearningTask::cleanUpObjectReferences()
{
    SP_ASSERT(actor_hit_event_component_);
    actor_hit_event_component_->component_->actor_hit_func_ = nullptr;
    actor_hit_event_component_->component_->unsubscribe(agent_actor_);

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
    Std::insert(step_info_space, "hit_goal", std::move(array_desc));

    array_desc.low_ = 0.0;
    array_desc.high_ = 1.0;
    array_desc.shape_ = {1};
    array_desc.datatype_ = DataType::UInteger8;
    Std::insert(step_info_space, "hit_obstacle", std::move(array_desc));

    return step_info_space;
}

std::map<std::string, std::vector<uint8_t>> ImitationLearningTask::getStepInfo() const
{
    std::map<std::string, std::vector<uint8_t>> step_info;

    Std::insert(step_info, "hit_goal", {hit_goal_});
    Std::insert(step_info, "hit_obstacle", {hit_obstacle_});

    return step_info;
}

void ImitationLearningTask::reset()
{
    FVector offset_location = {
        Config::get<double>("SIMULATION_CONTROLLER.IMITATION_LEARNING_TASK.AGENT_SPAWN_OFFSET_LOCATION_X"),
        Config::get<double>("SIMULATION_CONTROLLER.IMITATION_LEARNING_TASK.AGENT_SPAWN_OFFSET_LOCATION_Y"),
        Config::get<double>("SIMULATION_CONTROLLER.IMITATION_LEARNING_TASK.AGENT_SPAWN_OFFSET_LOCATION_Z")
    };
    FVector agent_initial_location = agent_initial_locations_.at(episode_index_) + offset_location;

    bool sweep = false;
    FHitResult* hit_result = nullptr;
    agent_actor_->SetActorLocationAndRotation(
        agent_initial_location, FRotator::ZeroRotator, sweep, hit_result, ETeleportType::TeleportPhysics);
    goal_actor_->SetActorLocationAndRotation(
        agent_goal_locations_.at(episode_index_), FRotator::ZeroRotator, sweep, hit_result, ETeleportType::TeleportPhysics);

    if (episode_index_ < agent_goal_locations_.size() - 1) { 
        episode_index_++;
    }  else {
        episode_index_ = 0;
    }
}

bool ImitationLearningTask::isReady() const
{
    return true;
}
