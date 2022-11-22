#include "PointGoalNavTask.h"

#include <algorithm>

#include <EngineUtils.h>
#include <Engine/StaticMesh.h>
#include <Engine/StaticMeshActor.h>
#include <Materials/Material.h>
#include <UObject/UObjectGlobals.h>

#include "ActorHitEvent.h"
#include "Assert/Assert.h"
#include "Box.h"
#include "Config.h"

PointGoalNavTask::PointGoalNavTask(UWorld* world)
{
    // spawn goal actor
    FActorSpawnParameters goal_spawn_params;
    goal_spawn_params.Name = FName(Config::getValue<std::string>({"SIMULATION_CONTROLLER", "POINT_GOAL_NAV_TASK", "GOAL_ACTOR_NAME"}).c_str());
    goal_spawn_params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

    goal_actor_ = world->SpawnActor<AStaticMeshActor>(AStaticMeshActor::StaticClass(), FVector::ZeroVector, FRotator::ZeroRotator, goal_spawn_params);
    ASSERT(goal_actor_);

    goal_actor_->SetMobility(EComponentMobility::Movable);

    UStaticMeshComponent* goal_mesh_component = goal_actor_->GetStaticMeshComponent();

    UStaticMesh* goal_mesh   = LoadObject<UStaticMesh>(nullptr, UTF8_TO_TCHAR(Config::getValue<std::string>({"SIMULATION_CONTROLLER", "POINT_GOAL_NAV_TASK", "GOAL_MESH"}).c_str()));
    ASSERT(goal_mesh);
    UMaterial* goal_material = LoadObject<UMaterial>  (nullptr, UTF8_TO_TCHAR(Config::getValue<std::string>({"SIMULATION_CONTROLLER", "POINT_GOAL_NAV_TASK", "GOAL_MATERIAL"}).c_str()));
    ASSERT(goal_material);

    goal_mesh_component->SetStaticMesh(goal_mesh);
    goal_mesh_component->SetMaterial(0, goal_material);
    goal_actor_->SetActorScale3D(FVector(Config::getValue<float>({"SIMULATION_CONTROLLER", "POINT_GOAL_NAV_TASK", "GOAL_SCALE"}),
                                         Config::getValue<float>({"SIMULATION_CONTROLLER", "POINT_GOAL_NAV_TASK", "GOAL_SCALE"}),
                                         Config::getValue<float>({"SIMULATION_CONTROLLER", "POINT_GOAL_NAV_TASK", "GOAL_SCALE"})));

    new_object_parent_actor_ = world->SpawnActor<AActor>();
    ASSERT(new_object_parent_actor_);

    // create UActorHitEvent but don't subscribe to any actors yet
    actor_hit_event_ = NewObject<UActorHitEvent>(new_object_parent_actor_, TEXT("ActorHitEvent"));
    ASSERT(actor_hit_event_);
    actor_hit_event_->RegisterComponent();
    actor_hit_event_delegate_handle_ = actor_hit_event_->delegate_.AddRaw(this, &PointGoalNavTask::actorHitEventHandler);

    random_stream_.Initialize(Config::getValue<int>({"SIMULATION_CONTROLLER", "POINT_GOAL_NAV_TASK", "RANDOM_SEED"}));
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

    ASSERT(new_object_parent_actor_);
    new_object_parent_actor_->Destroy();
    new_object_parent_actor_ = nullptr;

    ASSERT(goal_actor_);
    goal_actor_->Destroy();
    goal_actor_ = nullptr;    
}

void PointGoalNavTask::findObjectReferences(UWorld* world)
{
    auto obstacle_ignore_actor_names = Config::getValue<std::vector<std::string>>({"SIMULATION_CONTROLLER", "POINT_GOAL_NAV_TASK", "OBSTACLE_IGNORE_ACTOR_NAMES"});
    for (TActorIterator<AActor> actor_itr(world); actor_itr; ++actor_itr) {
        std::string actor_name = TCHAR_TO_UTF8(*((*actor_itr)->GetName()));
        if (actor_name == Config::getValue<std::string>({"SIMULATION_CONTROLLER", "POINT_GOAL_NAV_TASK", "AGENT_ACTOR_NAME"})) {
            ASSERT(!agent_actor_);
            agent_actor_ = *actor_itr;
        } else if (std::find(obstacle_ignore_actor_names.begin(), obstacle_ignore_actor_names.end(), actor_name) != obstacle_ignore_actor_names.end()) {
            obstacle_ignore_actors_.emplace_back(*actor_itr);
        }
    }
    ASSERT(agent_actor_);

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

void PointGoalNavTask::endFrame()
{
}

float PointGoalNavTask::getReward() const
{
    float reward;

    if (hit_goal_) {
        reward = Config::getValue<float>({"SIMULATION_CONTROLLER", "POINT_GOAL_NAV_TASK", "REWARD", "HIT_GOAL"});
    } else if (hit_obstacle_) {
        reward = Config::getValue<float>({"SIMULATION_CONTROLLER", "POINT_GOAL_NAV_TASK", "REWARD", "HIT_OBSTACLE"});
    } else {
        FVector agent_to_goal = goal_actor_->GetActorLocation() - agent_actor_->GetActorLocation();
        reward = -agent_to_goal.Size() * Config::getValue<float>({"SIMULATION_CONTROLLER", "POINT_GOAL_NAV_TASK", "REWARD", "DISTANCE_TO_GOAL_SCALE"});
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
    
    box.low = 0;
    box.high = 1;
    box.shape = {1};
    box.dtype = DataType::Boolean;
    step_info_space["hit_goal"] = std::move(box);

    box.low = 0;
    box.high = 1;
    box.shape = {1};
    box.dtype = DataType::Boolean;
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

    while ((agent_position - goal_position).Size() < Config::getValue<float>({"SIMULATION_CONTROLLER", "POINT_GOAL_NAV_TASK", "EPISODE_BEGIN", "SPAWN_DISTANCE_THRESHOLD"})) {
        position_x = random_stream_.FRandRange(
            Config::getValue<float>({"SIMULATION_CONTROLLER", "POINT_GOAL_NAV_TASK", "EPISODE_BEGIN", "AGENT_POSITION_X_MIN"}),
            Config::getValue<float>({"SIMULATION_CONTROLLER", "POINT_GOAL_NAV_TASK", "EPISODE_BEGIN", "AGENT_POSITION_X_MAX"}));
        position_y = random_stream_.FRandRange(
            Config::getValue<float>({"SIMULATION_CONTROLLER", "POINT_GOAL_NAV_TASK", "EPISODE_BEGIN", "AGENT_POSITION_Y_MIN"}),
            Config::getValue<float>({"SIMULATION_CONTROLLER", "POINT_GOAL_NAV_TASK", "EPISODE_BEGIN", "AGENT_POSITION_Y_MAX"}));
        position_z = Config::getValue<float>({"SIMULATION_CONTROLLER", "POINT_GOAL_NAV_TASK", "EPISODE_BEGIN", "AGENT_POSITION_Z"});

        agent_position = FVector(position_x, position_y, position_z);

        position_x = random_stream_.FRandRange(
            Config::getValue<float>({"SIMULATION_CONTROLLER", "POINT_GOAL_NAV_TASK", "EPISODE_BEGIN", "GOAL_POSITION_X_MIN"}),
            Config::getValue<float>({"SIMULATION_CONTROLLER", "POINT_GOAL_NAV_TASK", "EPISODE_BEGIN", "GOAL_POSITION_X_MAX"}));
        position_y = random_stream_.FRandRange(
            Config::getValue<float>({"SIMULATION_CONTROLLER", "POINT_GOAL_NAV_TASK", "EPISODE_BEGIN", "GOAL_POSITION_Y_MIN"}),
            Config::getValue<float>({"SIMULATION_CONTROLLER", "POINT_GOAL_NAV_TASK", "EPISODE_BEGIN", "GOAL_POSITION_Y_MAX"}));
        position_z = Config::getValue<float>({"SIMULATION_CONTROLLER", "POINT_GOAL_NAV_TASK", "EPISODE_BEGIN", "GOAL_POSITION_Z"});

        goal_position = FVector(position_x, position_y, position_z);
    }

    agent_actor_->SetActorLocation(agent_position);
    goal_actor_->SetActorLocation(goal_position);
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
    } else if (std::find(obstacle_ignore_actors_.begin(), obstacle_ignore_actors_.end(), other_actor) == obstacle_ignore_actors_.end()) {
        hit_obstacle_ = true;
    }
};
