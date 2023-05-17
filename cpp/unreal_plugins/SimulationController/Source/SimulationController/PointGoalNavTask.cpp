//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SimulationController/PointGoalNavTask.h"

#include <Delegates/IDelegateInstance.h>
#include <Engine/EngineTypes.h>
#include <Engine/StaticMesh.h>
#include <Engine/StaticMeshActor.h>
#include <Engine/World.h>
#include <Materials/Material.h>
#include <Math/Rotator.h>
#include <Math/Vector.h>

#include "CoreUtils/ArrayDesc.h"
#include "CoreUtils/Assert.h"
#include "CoreUtils/Config.h"
#include "CoreUtils/Std.h"
#include "CoreUtils/Unreal.h"
#include "SimulationController/ActorHitEventComponent.h"

PointGoalNavTask::PointGoalNavTask(UWorld* world)
{
    // spawn actor
    FActorSpawnParameters actor_spawn_params;
    actor_spawn_params.Name = Unreal::toFName(Config::get<std::string>("SIMULATION_CONTROLLER.POINT_GOAL_NAV_TASK.GOAL_ACTOR_NAME"));
    actor_spawn_params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    goal_actor_ = world->SpawnActor<AStaticMeshActor>(FVector::ZeroVector, FRotator::ZeroRotator, actor_spawn_params);
    SP_ASSERT(goal_actor_);

    goal_actor_->SetMobility(EComponentMobility::Movable);

    FVector scale =
        FVector(
            Config::get<double>("SIMULATION_CONTROLLER.POINT_GOAL_NAV_TASK.GOAL_SCALE"),
            Config::get<double>("SIMULATION_CONTROLLER.POINT_GOAL_NAV_TASK.GOAL_SCALE"),
            Config::get<double>("SIMULATION_CONTROLLER.POINT_GOAL_NAV_TASK.GOAL_SCALE"));
    goal_actor_->SetActorScale3D(scale);

    // load mesh and material
    UStaticMesh* goal_mesh = LoadObject<UStaticMesh>(
        nullptr, *Unreal::toFString(Config::get<std::string>("SIMULATION_CONTROLLER.POINT_GOAL_NAV_TASK.GOAL_MESH")));
    SP_ASSERT(goal_mesh);
    UMaterial* goal_material = LoadObject<UMaterial>(
        nullptr, *Unreal::toFString(Config::get<std::string>("SIMULATION_CONTROLLER.POINT_GOAL_NAV_TASK.GOAL_MATERIAL")));
    SP_ASSERT(goal_material);

    // configure static mesh component
    UStaticMeshComponent* goal_mesh_component = goal_actor_->GetStaticMeshComponent();
    goal_mesh_component->SetStaticMesh(goal_mesh);
    goal_mesh_component->SetMaterial(0, goal_material);

    // create UActorHitEvent but don't subscribe to any actors yet
    parent_actor_ = world->SpawnActor<AActor>();
    SP_ASSERT(parent_actor_);

    actor_hit_event_component_ = NewObject<UActorHitEventComponent>(parent_actor_);
    SP_ASSERT(actor_hit_event_component_);
    actor_hit_event_component_->RegisterComponent();
    actor_hit_event_handle_ = actor_hit_event_component_->delegate_.AddRaw(this, &PointGoalNavTask::actorHitEventHandler);

    minstd_rand_ = std::minstd_rand(Config::get<int>("SIMULATION_CONTROLLER.POINT_GOAL_NAV_TASK.RANDOM_SEED"));

    hit_goal_ = false;
    hit_obstacle_ = false;
}

PointGoalNavTask::~PointGoalNavTask()
{
    hit_obstacle_ = false;
    hit_goal_ = false;

    minstd_rand_ = std::minstd_rand();

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

void PointGoalNavTask::findObjectReferences(UWorld* world)
{
    agent_actor_ = Unreal::findActorByName(world, Config::get<std::string>("SIMULATION_CONTROLLER.POINT_GOAL_NAV_TASK.AGENT_ACTOR_NAME"));
    SP_ASSERT(agent_actor_);

    bool return_null_if_not_found = false;
    obstacle_ignore_actors_ = Unreal::findActorsByName(
        world, Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.POINT_GOAL_NAV_TASK.OBSTACLE_IGNORE_ACTOR_NAMES"), return_null_if_not_found);

    // Subscribe to the agent actor now that we have obtained a reference to it
    actor_hit_event_component_->subscribeToActor(agent_actor_);
}

void PointGoalNavTask::cleanUpObjectReferences()
{
    SP_ASSERT(actor_hit_event_component_);
    actor_hit_event_component_->unsubscribeFromActor(agent_actor_);

    obstacle_ignore_actors_.clear();

    SP_ASSERT(agent_actor_);
    agent_actor_ = nullptr;
}

void PointGoalNavTask::beginFrame()
{
    hit_goal_ = false;
    hit_obstacle_ = false;
}

void PointGoalNavTask::endFrame() {}

float PointGoalNavTask::getReward() const
{
    float reward;

    if (hit_goal_) {
        reward = Config::get<float>("SIMULATION_CONTROLLER.POINT_GOAL_NAV_TASK.REWARD.HIT_GOAL");
    } else if (hit_obstacle_) {
        reward = Config::get<float>("SIMULATION_CONTROLLER.POINT_GOAL_NAV_TASK.REWARD.HIT_OBSTACLE");
    } else {
        FVector agent_to_goal = goal_actor_->GetActorLocation() - agent_actor_->GetActorLocation();
        reward = -agent_to_goal.Size();
    }

    return reward;
}

bool PointGoalNavTask::isEpisodeDone() const
{
    return hit_goal_ || hit_obstacle_;
}

std::map<std::string, ArrayDesc> PointGoalNavTask::getStepInfoSpace() const
{
    std::map<std::string, ArrayDesc> step_info_space;
    ArrayDesc array_desc;
    
    array_desc.low_ = 0.0f;
    array_desc.high_ = 1.0f;
    array_desc.shape_ = {1};
    array_desc.datatype_ = DataType::UInteger8;
    step_info_space["hit_goal"] = std::move(array_desc);

    array_desc.low_ = 0.0f;
    array_desc.high_ = 1.0f;
    array_desc.shape_ = {1};
    array_desc.datatype_ = DataType::UInteger8;
    step_info_space["hit_obstacle"] = std::move(array_desc);

    return step_info_space;
}

std::map<std::string, std::vector<uint8_t>> PointGoalNavTask::getStepInfo() const
{
    std::map<std::string, std::vector<uint8_t>> step_info;

    step_info["hit_goal"] = std::vector<uint8_t>{hit_goal_};
    step_info["hit_obstacle"] = std::vector<uint8_t>{hit_obstacle_};

    return step_info;
}

void PointGoalNavTask::reset()
{
    FVector agent_position;
    FVector goal_position;

    while ((agent_position - goal_position).Size() < Config::get<float>("SIMULATION_CONTROLLER.POINT_GOAL_NAV_TASK.EPISODE_BEGIN.SPAWN_DISTANCE_THRESHOLD")) {

        std::uniform_real_distribution distribution_agent_position_x(
            Config::get<float>("SIMULATION_CONTROLLER.POINT_GOAL_NAV_TASK.EPISODE_BEGIN.AGENT_POSITION_X_MIN"),
            Config::get<float>("SIMULATION_CONTROLLER.POINT_GOAL_NAV_TASK.EPISODE_BEGIN.AGENT_POSITION_X_MAX"));
        std::uniform_real_distribution distribution_agent_position_y(
            Config::get<float>("SIMULATION_CONTROLLER.POINT_GOAL_NAV_TASK.EPISODE_BEGIN.AGENT_POSITION_Y_MIN"),
            Config::get<float>("SIMULATION_CONTROLLER.POINT_GOAL_NAV_TASK.EPISODE_BEGIN.AGENT_POSITION_Y_MAX"));
        std::uniform_real_distribution distribution_goal_position_x(
            Config::get<float>("SIMULATION_CONTROLLER.POINT_GOAL_NAV_TASK.EPISODE_BEGIN.GOAL_POSITION_X_MIN"),
            Config::get<float>("SIMULATION_CONTROLLER.POINT_GOAL_NAV_TASK.EPISODE_BEGIN.GOAL_POSITION_X_MAX"));
        std::uniform_real_distribution distribution_goal_position_y(
            Config::get<float>("SIMULATION_CONTROLLER.POINT_GOAL_NAV_TASK.EPISODE_BEGIN.GOAL_POSITION_Y_MIN"),
            Config::get<float>("SIMULATION_CONTROLLER.POINT_GOAL_NAV_TASK.EPISODE_BEGIN.GOAL_POSITION_Y_MAX"));

        agent_position = FVector(
            distribution_agent_position_x(minstd_rand_),
            distribution_agent_position_y(minstd_rand_),
            Config::get<float>("SIMULATION_CONTROLLER.POINT_GOAL_NAV_TASK.EPISODE_BEGIN.AGENT_POSITION_Z"));
        goal_position = FVector(
            distribution_goal_position_x(minstd_rand_),
            distribution_goal_position_y(minstd_rand_),
            Config::get<float>("SIMULATION_CONTROLLER.POINT_GOAL_NAV_TASK.EPISODE_BEGIN.GOAL_POSITION_Z"));
    }

    bool sweep = false;
    FHitResult* hit_result = nullptr;
    agent_actor_->SetActorLocationAndRotation(agent_position, FRotator::ZeroRotator, sweep, hit_result, ETeleportType::TeleportPhysics);
    goal_actor_->SetActorLocationAndRotation(goal_position, FRotator::ZeroRotator, sweep, hit_result, ETeleportType::TeleportPhysics);
}

bool PointGoalNavTask::isReady() const
{
    return true;
}

void PointGoalNavTask::actorHitEventHandler(AActor* self_actor, AActor* other_actor, FVector normal_impulse, const FHitResult& hit_result)
{
    SP_ASSERT(self_actor == agent_actor_);

    if (other_actor == goal_actor_) {
        hit_goal_ = true;
    } else if (!Std::contains(obstacle_ignore_actors_, other_actor)) {
        hit_obstacle_ = true;
    }
};
