//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SimulationController/PointGoalNavTask.h"

#include <Delegates/IDelegateInstance.h>
#include <EngineUtils.h>
#include <Engine/StaticMesh.h>
#include <Engine/StaticMeshActor.h>
#include <Materials/Material.h>
#include <Math/RandomStream.h>
#include <Math/Vector.h>
#include <UObject/UObjectGlobals.h>

#include "CoreUtils/Assert.h"
#include "CoreUtils/Box.h"
#include "CoreUtils/Config.h"
#include "CoreUtils/Std.h"
#include "CoreUtils/Unreal.h"
#include "SimulationController/ActorHitEvent.h"

PointGoalNavTask::PointGoalNavTask(UWorld* world)
{
    // spawn goal actor
    FActorSpawnParameters actor_spawn_params;
    actor_spawn_params.Name = Unreal::toFName(Config::get<std::string>("SIMULATION_CONTROLLER.POINT_GOAL_NAV_TASK.GOAL_ACTOR_NAME"));
    actor_spawn_params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    goal_actor_ = world->SpawnActor<AStaticMeshActor>(FVector::ZeroVector, FRotator::ZeroRotator, actor_spawn_params);
    ASSERT(goal_actor_);

    goal_actor_->SetMobility(EComponentMobility::Movable);

    UStaticMeshComponent* goal_mesh_component = goal_actor_->GetStaticMeshComponent();

    UStaticMesh* goal_mesh = LoadObject<UStaticMesh>(
        nullptr, *Unreal::toFString(Config::get<std::string>("SIMULATION_CONTROLLER.POINT_GOAL_NAV_TASK.GOAL_MESH")));
    ASSERT(goal_mesh);
    UMaterial* goal_material = LoadObject<UMaterial>(
        nullptr, *Unreal::toFString(Config::get<std::string>("SIMULATION_CONTROLLER.POINT_GOAL_NAV_TASK.GOAL_MATERIAL")));
    ASSERT(goal_material);

    goal_mesh_component->SetStaticMesh(goal_mesh);
    goal_mesh_component->SetMaterial(0, goal_material);
    goal_actor_->SetActorScale3D(FVector(
        Config::get<float>("SIMULATION_CONTROLLER.POINT_GOAL_NAV_TASK.GOAL_SCALE"),
        Config::get<float>("SIMULATION_CONTROLLER.POINT_GOAL_NAV_TASK.GOAL_SCALE"),
        Config::get<float>("SIMULATION_CONTROLLER.POINT_GOAL_NAV_TASK.GOAL_SCALE")));

    parent_actor_ = world->SpawnActor<AActor>();
    ASSERT(parent_actor_);

    // create UActorHitEvent but don't subscribe to any actors yet
    actor_hit_event_ = NewObject<UActorHitEvent>(parent_actor_);
    ASSERT(actor_hit_event_);
    actor_hit_event_->RegisterComponent();
    actor_hit_event_delegate_handle_ = actor_hit_event_->delegate_.AddRaw(this, &PointGoalNavTask::actorHitEventHandler);

    random_stream_.Initialize(Config::get<int>("SIMULATION_CONTROLLER.POINT_GOAL_NAV_TASK.RANDOM_SEED"));
    hit_goal_ = false;
    hit_obstacle_ = false;
}

PointGoalNavTask::~PointGoalNavTask()
{
    hit_obstacle_ = false;
    hit_goal_ = false;
    random_stream_.Reset();

    ASSERT(actor_hit_event_);
    actor_hit_event_->delegate_.Remove(actor_hit_event_delegate_handle_);
    actor_hit_event_delegate_handle_.Reset();
    actor_hit_event_->DestroyComponent();
    actor_hit_event_ = nullptr;

    ASSERT(parent_actor_);
    parent_actor_->Destroy();
    parent_actor_ = nullptr;

    ASSERT(goal_actor_);
    goal_actor_->Destroy();
    goal_actor_ = nullptr;    
}

void PointGoalNavTask::findObjectReferences(UWorld* world)
{
    agent_actor_ = Unreal::findActorByName(world, Config::get<std::string>("SIMULATION_CONTROLLER.POINT_GOAL_NAV_TASK.AGENT_ACTOR_NAME"));
    ASSERT(agent_actor_);

    bool return_null_if_not_found = false;
    obstacle_ignore_actors_ = Unreal::findActorsByName(
        world, Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.POINT_GOAL_NAV_TASK.OBSTACLE_IGNORE_ACTOR_NAMES"), return_null_if_not_found);

    // Subscribe to the agent actor now that we have obtained a reference to it
    actor_hit_event_->subscribeToActor(agent_actor_);
}

void PointGoalNavTask::cleanUpObjectReferences()
{
    ASSERT(actor_hit_event_);
    actor_hit_event_->unsubscribeFromActor(agent_actor_);

    obstacle_ignore_actors_.clear();

    ASSERT(agent_actor_);
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
        reward = -agent_to_goal.Size() * Config::get<float>("SIMULATION_CONTROLLER.POINT_GOAL_NAV_TASK.REWARD.DISTANCE_TO_GOAL_SCALE");
    }

    return reward;
}

bool PointGoalNavTask::isEpisodeDone() const
{
    return hit_goal_ || hit_obstacle_;
}

std::map<std::string, Box> PointGoalNavTask::getStepInfoSpace() const
{
    std::map<std::string, Box> step_info_space;
    Box box;
    
    box.low_ = 0.0f;
    box.high_ = 1.0f;
    box.shape_ = {1};
    box.datatype_ = DataType::Boolean;
    step_info_space["hit_goal"] = std::move(box);

    box.low_ = 0.0f;
    box.high_ = 1.0f;
    box.shape_ = {1};
    box.datatype_ = DataType::Boolean;
    step_info_space["hit_obstacle"] = std::move(box);

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
    float position_x, position_y, position_z;
    FVector agent_position(0), goal_position(0);

    while ((agent_position - goal_position).Size() < Config::get<float>("SIMULATION_CONTROLLER.POINT_GOAL_NAV_TASK.EPISODE_BEGIN.SPAWN_DISTANCE_THRESHOLD")) {
        position_x = random_stream_.FRandRange(
            Config::get<float>("SIMULATION_CONTROLLER.POINT_GOAL_NAV_TASK.EPISODE_BEGIN.AGENT_POSITION_X_MIN"),
            Config::get<float>("SIMULATION_CONTROLLER.POINT_GOAL_NAV_TASK.EPISODE_BEGIN.AGENT_POSITION_X_MAX"));
        position_y = random_stream_.FRandRange(
            Config::get<float>("SIMULATION_CONTROLLER.POINT_GOAL_NAV_TASK.EPISODE_BEGIN.AGENT_POSITION_Y_MIN"),
            Config::get<float>("SIMULATION_CONTROLLER.POINT_GOAL_NAV_TASK.EPISODE_BEGIN.AGENT_POSITION_Y_MAX"));
        position_z = Config::get<float>("SIMULATION_CONTROLLER.POINT_GOAL_NAV_TASK.EPISODE_BEGIN.AGENT_POSITION_Z");

        agent_position = FVector(position_x, position_y, position_z);

        position_x = random_stream_.FRandRange(
            Config::get<float>("SIMULATION_CONTROLLER.POINT_GOAL_NAV_TASK.EPISODE_BEGIN.GOAL_POSITION_X_MIN"),
            Config::get<float>("SIMULATION_CONTROLLER.POINT_GOAL_NAV_TASK.EPISODE_BEGIN.GOAL_POSITION_X_MAX"));
        position_y = random_stream_.FRandRange(
            Config::get<float>("SIMULATION_CONTROLLER.POINT_GOAL_NAV_TASK.EPISODE_BEGIN.GOAL_POSITION_Y_MIN"),
            Config::get<float>("SIMULATION_CONTROLLER.POINT_GOAL_NAV_TASK.EPISODE_BEGIN.GOAL_POSITION_Y_MAX"));
        position_z = Config::get<float>("SIMULATION_CONTROLLER.POINT_GOAL_NAV_TASK.EPISODE_BEGIN.GOAL_POSITION_Z");

        goal_position = FVector(position_x, position_y, position_z);
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
    ASSERT(self_actor == agent_actor_);

    if (other_actor == goal_actor_) {
        hit_goal_ = true;
    } else if (!Std::contains(obstacle_ignore_actors_, other_actor)) {
        hit_obstacle_ = true;
    }
};
