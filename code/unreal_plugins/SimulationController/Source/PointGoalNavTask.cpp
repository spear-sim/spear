#include "PointGoalNavTask.h"

#include "SphereAgentController.h"

PointGoalNavTask::PointGoalNavTask(SphereAgentController* agent_controller)
{
    ASSERT(agent_controller);
    agent_controller_ = agent_controller;
    agent_controller_->getSphereActor()->OnActorHit.AddDynamic(this, &PointGoalNavTask::ActorHitEventHandler);

    random_stream_.Initialize(Config::getValue<int>({"INTERIORSIM", "RANDOM_SEED"}));
}

float PointGoalNavTask::getReward()
{
    end_episode_ = false;

    if (hit_goal_) {
        reward_ = Config::getValue<float>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "REWARD", "HIT_GOAL"});
        end_episode_ = true;
    } else if (hit_other_) {
        reward_ = Config::getValue<float>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "REWARD", "HIT_OBSTACLE"});
        end_episode_ = true;
    } else {
        const FVector sphere_to_cone = agent_controller_->getConeActor()->GetActorLocation() - agent_controller_->getSphereActor()->GetActorLocation();
        reward_ = -sphere_to_cone.Size() / Config::getValue<float>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "REWARD", "DISTANCE_TO_GOAL_SCALE"});
    }

    // reset hit state
    hit_goal_ = false;
    hit_other_ = false;

    return reward_;
}

void PointGoalNavTask::reset()
{
    float position_x, position_y, position_z;
    FVector sphere_position(0), cone_position(0);

    while ((sphere_position - cone_position).Size() < Config::getValue<float>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "EPISODE_BEGIN", "SPAWN_DISTANCE_THRESHOLD"})) {
        position_x = random_stream_.FRandRange(
            Config::getValue<float>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "EPISODE_BEGIN", "SPHERE_POSITION_X_MIN"}),
            Config::getValue<float>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "EPISODE_BEGIN", "SPHERE_POSITION_X_MAX"}));
        position_y = random_stream_.FRandRange(
            Config::getValue<float>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "EPISODE_BEGIN", "SPHERE_POSITION_Y_MIN"}),
            Config::getValue<float>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "EPISODE_BEGIN", "SPHERE_POSITION_Y_MAX"}));
        position_z = Config::getValue<float>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "EPISODE_BEGIN", "SPHERE_POSITION_Z"});

        sphere_position = FVector(position_x, position_y, position_z);

        position_x = random_stream_.FRandRange(
            Config::getValue<float>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "EPISODE_BEGIN", "CONE_POSITION_X_MIN"}),
            Config::getValue<float>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "EPISODE_BEGIN", "CONE_POSITION_X_MAX"}));
        position_y = random_stream_.FRandRange(
            Config::getValue<float>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "EPISODE_BEGIN", "CONE_POSITION_Y_MIN"}),
            Config::getValue<float>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "EPISODE_BEGIN", "CONE_POSITION_Y_MAX"}));
        position_z = Config::getValue<float>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "EPISODE_BEGIN", "CONE_POSITION_Z"});

        cone_position = FVector(position_x, position_y, position_z);
    }

    agent_controller_->getSphereActor()->SetActorLocation(sphere_position);
    agent_controller_->getConeActor()->SetActorLocation(cone_position);

    const FVector observation_camera_position(
        agent_controller_->getSphereActor()->GetActorLocation() +
        FVector(Config::getValue<float>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "OBSERVATION_CAMERA", "POSITION_OFFSET_X"}),
                Config::getValue<float>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "OBSERVATION_CAMERA", "POSITION_OFFSET_Y"}),
                Config::getValue<float>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "OBSERVATION_CAMERA", "POSITION_OFFSET_Z"})));
    agent_controller_->getObservationCameraActor()->SetActorLocation(observation_camera_position);

    Cast<UStaticMeshComponent>(agent_controller_->getSphereActor()->GetRootComponent())->SetPhysicsLinearVelocity(FVector(0), false);
    Cast<UStaticMeshComponent>(agent_controller_->getSphereActor()->GetRootComponent())->SetPhysicsAngularVelocityInRadians(FVector(0), false);
    Cast<UStaticMeshComponent>(agent_controller_->getSphereActor()->GetRootComponent())->GetBodyInstance()->ClearTorques();
    Cast<UStaticMeshComponent>(agent_controller_->getSphereActor()->GetRootComponent())->GetBodyInstance()->ClearForces();
}

void PointGoalNavTask::ActorHitEventHandler(AActor* self_actor, AActor* other_actor, FVector normal_impulse, const FHitResult& hit)
{
    if (other_actor != nullptr && other_actor->GetName().Equals(Config::getValue<std::string>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "CONE_NAME"}).c_str(), ESearchCase::IgnoreCase)) {
        hit_goal_ = true;
    } else if (other_actor != nullptr && !other_actor->GetName().Equals(TEXT("Architecture_SMid0_PMid0_INSTid1227_obj19"), ESearchCase::IgnoreCase) && !other_actor->GetName().Equals(TEXT("6KAPD2ZVAZSUSPTUKE888888_SMid116_PMid1009_INSTid1080_obj0_0"), ESearchCase::IgnoreCase)) {
        hit_other_ = true;
    }
};
