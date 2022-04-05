#include "PointGoalNavTask.h"

#include <UObject/UObjectGlobals.h>

#include "ActorHitDummyHandler.h"
#include "Assert.h"
#include "Config.h"

PointGoalNavTask::PointGoalNavTask(UWorld* world)
{
    agent_actor_ = Cast<AActor>(StaticFindObject(AActor::StaticClass(), world, UTF8_TO_TCHAR(Config::getValue<std::string>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "ACTOR_NAME"}).c_str()), true));
    ASSERT(agent_actor_);

    observation_camera_actor_ = Cast<AActor>(StaticFindObject(AActor::StaticClass(), world, UTF8_TO_TCHAR(Config::getValue<std::string>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "OBSERVATION_CAMERA_NAME"}).c_str()), true));
    ASSERT(observation_camera_actor_);
    
    goal_actor_ = Cast<AActor>(StaticFindObject(AActor::StaticClass(), world, UTF8_TO_TCHAR(Config::getValue<std::string>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "GOAL_NAME"}).c_str()), true));
    ASSERT(goal_actor_);

    // read config value for random stream initialization
    random_stream_.Initialize(Config::getValue<int>({"SIMULATION_CONTROLLER", "POINT_GOAL_NAV_TASK", "RANDOM_SEED"}));

    // create and initialize actor hit dummy handler
    actor_hit_dummy_handler_ = NewObject<UActorHitDummyHandler>(world, TEXT("ActorHitDummyHandler"));
    ASSERT(actor_hit_dummy_handler_);
    actor_hit_dummy_handler_->initialize(agent_actor_, this);
}

PointGoalNavTask::~PointGoalNavTask()
{
    ASSERT(actor_hit_dummy_handler_);
    actor_hit_dummy_handler_->terminate(agent_actor_);

    random_stream_.Reset();

    ASSERT(goal_actor_);    
    goal_actor_ = nullptr;

    ASSERT(observation_camera_actor_);
    observation_camera_actor_ = nullptr;

    ASSERT(agent_actor_);
    agent_actor_ = nullptr;
}

float PointGoalNavTask::getReward()
{
    if (hit_goal_) {
        reward_ = Config::getValue<float>({"SIMULATION_CONTROLLER", "POINT_GOAL_NAV_TASK", "REWARD", "HIT_GOAL"});
    } else if (hit_other_) {
        reward_ = Config::getValue<float>({"SIMULATION_CONTROLLER", "POINT_GOAL_NAV_TASK", "REWARD", "HIT_OBSTACLE"});
    } else {
        const FVector sphere_to_cone = goal_actor_->GetActorLocation() - agent_actor_->GetActorLocation();
        reward_ = -sphere_to_cone.Size() / Config::getValue<float>({"SIMULATION_CONTROLLER", "POINT_GOAL_NAV_TASK", "REWARD", "DISTANCE_TO_GOAL_SCALE"});
    }

    return reward_;
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

    const FVector observation_camera_position(
        agent_actor_->GetActorLocation() +
        FVector(Config::getValue<float>({"SIMULATION_CONTROLLER", "POINT_GOAL_NAV_TASK", "EPISODE_BEGIN", "OBSERVATION_CAMERA_POSITION_OFFSET_X"}),
                Config::getValue<float>({"SIMULATION_CONTROLLER", "POINT_GOAL_NAV_TASK", "EPISODE_BEGIN", "OBSERVATION_CAMERA_POSITION_OFFSET_Y"}),
                Config::getValue<float>({"SIMULATION_CONTROLLER", "POINT_GOAL_NAV_TASK", "EPISODE_BEGIN", "OBSERVATION_CAMERA_POSITION_OFFSET_Z"})));
    observation_camera_actor_->SetActorLocation(observation_camera_position);

    Cast<UStaticMeshComponent>(agent_actor_->GetRootComponent())->SetPhysicsLinearVelocity(FVector(0), false);
    Cast<UStaticMeshComponent>(agent_actor_->GetRootComponent())->SetPhysicsAngularVelocityInRadians(FVector(0), false);
    Cast<UStaticMeshComponent>(agent_actor_->GetRootComponent())->GetBodyInstance()->ClearTorques();
    Cast<UStaticMeshComponent>(agent_actor_->GetRootComponent())->GetBodyInstance()->ClearForces();
}

bool PointGoalNavTask::isEpisodeDone() const
{
    return end_episode_;
}

void PointGoalNavTask::ActorHitEventHandler(AActor* self_actor, AActor* other_actor, FVector normal_impulse, const FHitResult& hit)
{
    // reset hit state
    hit_goal_ = false;
    hit_other_ = false;

    // reset end episode state
    end_episode_ = false;

    if (other_actor != nullptr && other_actor == goal_actor_) {
        hit_goal_ = true;
        end_episode_ = true;
    } else if (other_actor != nullptr && !other_actor->GetName().Equals(TEXT("Architecture_SMid0_PMid0_INSTid1227_obj19"), ESearchCase::IgnoreCase) && !other_actor->GetName().Equals(TEXT("6KAPD2ZVAZSUSPTUKE888888_SMid116_PMid1009_INSTid1080_obj0_0"), ESearchCase::IgnoreCase)) {
        hit_other_ = true;
        end_episode_ = true;
    }
};
