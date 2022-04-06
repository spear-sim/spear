#include "PointGoalNavTask.h"

#include <algorithm>

#include <EngineUtils.h>
#include <UObject/UObjectGlobals.h>

#include "ActorHitEvent.h"
#include "Assert.h"
#include "Config.h"

PointGoalNavTask::PointGoalNavTask(UWorld* world)
{
    // append all actors that need to be ignored during collision check
    std::vector<std::string> obstacle_ignore_actor_names = Config::getValue<std::vector<std::string>>({"SIMULATION_CONTROLLER", "POINT_GOAL_NAV_TASK", "OBSTACLE_IGNORE_ACTOR_NAMES"});

    for (TActorIterator<AActor> actor_itr(world, AActor::StaticClass()); actor_itr; ++actor_itr) {
        std::string actor_name = TCHAR_TO_UTF8(*(*actor_itr)->GetName());

        if (actor_name == Config::getValue<std::string>({"SIMULATION_CONTROLLER", "POINT_GOAL_NAV_TASK", "ACTOR_NAME"})) { 
            ASSERT(!agent_actor_);
            agent_actor_ = (*actor_itr);
        } else if (actor_name == Config::getValue<std::string>({"SIMULATION_CONTROLLER", "POINT_GOAL_NAV_TASK", "GOAL_NAME"})) {
            ASSERT(!goal_actor_);
            goal_actor_ = *actor_itr;
        } else if (std::find(obstacle_ignore_actor_names.begin(), obstacle_ignore_actor_names.end(), actor_name) != obstacle_ignore_actor_names.end()) {
            obstacle_ignore_actors_.emplace_back(*actor_itr);
        }
    }
    ASSERT(agent_actor_);
    ASSERT(goal_actor_);
    ASSERT(obstacle_ignore_actors_.size() == obstacle_ignore_actor_names.size());

    // read config value for random stream initialization
    random_stream_.Initialize(Config::getValue<int>({"SIMULATION_CONTROLLER", "POINT_GOAL_NAV_TASK", "RANDOM_SEED"}));

    // create and initialize actor hit handler
    actor_hit_event_ = NewObject<UActorHitEvent>(agent_actor_, TEXT("ActorHitEvent"));
    ASSERT(actor_hit_event_);
    actor_hit_event_->subscribeToActor(agent_actor_);
    actor_hit_event_delegate_handle_ = actor_hit_event_->delegate_.AddRaw(this, &PointGoalNavTask::ActorHitEventHandler);
}

PointGoalNavTask::~PointGoalNavTask()
{
    ASSERT(actor_hit_event_);
    actor_hit_event_->delegate_.Remove(actor_hit_event_delegate_handle_);
    actor_hit_event_delegate_handle_.Reset();
    actor_hit_event_->unsubscribeFromActor(agent_actor_);

    random_stream_.Reset();

    ASSERT(goal_actor_);    
    goal_actor_ = nullptr;

    ASSERT(agent_actor_);
    agent_actor_ = nullptr;
}

void PointGoalNavTask::beginFrame()
{
    // reset hit states
    hit_goal_ = false;
    hit_obstacle_ = false;
}

float PointGoalNavTask::getReward()
{
    float reward;

    if (hit_goal_) {
        reward = Config::getValue<float>({"SIMULATION_CONTROLLER", "POINT_GOAL_NAV_TASK", "REWARD", "HIT_GOAL"});
    } else if (hit_obstacle_) {
        reward = Config::getValue<float>({"SIMULATION_CONTROLLER", "POINT_GOAL_NAV_TASK", "REWARD", "HIT_OBSTACLE"});
    } else {
        const FVector sphere_to_cone = goal_actor_->GetActorLocation() - agent_actor_->GetActorLocation();
        reward = -sphere_to_cone.Size() / Config::getValue<float>({"SIMULATION_CONTROLLER", "POINT_GOAL_NAV_TASK", "REWARD", "DISTANCE_TO_GOAL_SCALE"});
    }

    return reward;
}

bool PointGoalNavTask::isEpisodeDone() const
{
    return hit_goal_ or hit_obstacle_;
}

void PointGoalNavTask::reset()
{
    float position_x, position_y, position_z;
    FVector sphere_position(0), cone_position(0);

    while ((sphere_position - cone_position).Size() < Config::getValue<float>({"SIMULATION_CONTROLLER", "POINT_GOAL_NAV_TASK", "EPISODE_BEGIN", "SPAWN_DISTANCE_THRESHOLD"})) {
        position_x = random_stream_.FRandRange(
            Config::getValue<float>({"SIMULATION_CONTROLLER", "POINT_GOAL_NAV_TASK", "EPISODE_BEGIN", "SPHERE_POSITION_X_MIN"}),
            Config::getValue<float>({"SIMULATION_CONTROLLER", "POINT_GOAL_NAV_TASK", "EPISODE_BEGIN", "SPHERE_POSITION_X_MAX"}));
        position_y = random_stream_.FRandRange(
            Config::getValue<float>({"SIMULATION_CONTROLLER", "POINT_GOAL_NAV_TASK", "EPISODE_BEGIN", "SPHERE_POSITION_Y_MIN"}),
            Config::getValue<float>({"SIMULATION_CONTROLLER", "POINT_GOAL_NAV_TASK", "EPISODE_BEGIN", "SPHERE_POSITION_Y_MAX"}));
        position_z = Config::getValue<float>({"SIMULATION_CONTROLLER", "POINT_GOAL_NAV_TASK", "EPISODE_BEGIN", "SPHERE_POSITION_Z"});

        sphere_position = FVector(position_x, position_y, position_z);

        position_x = random_stream_.FRandRange(
            Config::getValue<float>({"SIMULATION_CONTROLLER", "POINT_GOAL_NAV_TASK", "EPISODE_BEGIN", "CONE_POSITION_X_MIN"}),
            Config::getValue<float>({"SIMULATION_CONTROLLER", "POINT_GOAL_NAV_TASK", "EPISODE_BEGIN", "CONE_POSITION_X_MAX"}));
        position_y = random_stream_.FRandRange(
            Config::getValue<float>({"SIMULATION_CONTROLLER", "POINT_GOAL_NAV_TASK", "EPISODE_BEGIN", "CONE_POSITION_Y_MIN"}),
            Config::getValue<float>({"SIMULATION_CONTROLLER", "POINT_GOAL_NAV_TASK", "EPISODE_BEGIN", "CONE_POSITION_Y_MAX"}));
        position_z = Config::getValue<float>({"SIMULATION_CONTROLLER", "POINT_GOAL_NAV_TASK", "EPISODE_BEGIN", "CONE_POSITION_Z"});

        cone_position = FVector(position_x, position_y, position_z);
    }

    agent_actor_->SetActorLocation(sphere_position);
    goal_actor_->SetActorLocation(cone_position);

    UStaticMeshComponent* static_mesh_component = Cast<UStaticMeshComponent>(agent_actor_->GetRootComponent());
    static_mesh_component->SetPhysicsLinearVelocity(FVector(0), false);
    static_mesh_component->SetPhysicsAngularVelocityInRadians(FVector(0), false);
    static_mesh_component->GetBodyInstance()->ClearTorques();
    static_mesh_component->GetBodyInstance()->ClearForces();
}

void PointGoalNavTask::ActorHitEventHandler(AActor* self_actor, AActor* other_actor, FVector normal_impulse, const FHitResult& hit)
{
    if (self_actor == agent_actor_ && other_actor != nullptr && other_actor == goal_actor_) {
        hit_goal_ = true;
    } else if (self_actor == agent_actor_ && other_actor != nullptr && !other_actor->GetName().Equals(TEXT("Architecture_SMid0_PMid0_INSTid1227_obj19"), ESearchCase::IgnoreCase) && !other_actor->GetName().Equals(TEXT("6KAPD2ZVAZSUSPTUKE888888_SMid116_PMid1009_INSTid1080_obj0_0"), ESearchCase::IgnoreCase)) {
        hit_obstacle_ = true;
    }
};
