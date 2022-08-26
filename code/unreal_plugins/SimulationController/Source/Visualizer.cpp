#pragma once

#include <Engine/World.h>
#include <GameFramework/Actor.h>

void Visualizer::findObjectReferences(UWorld* world)
{
    for (TActorIterator<AActor> actor_itr(world, AActor::StaticClass()); actor_itr; ++actor_itr) {
        std::string actor_name = TCHAR_TO_UTF8(*(*actor_itr)->GetName());
        if (actor_name == Config::getValue<std::string>({"SIMULATION_CONTROLLER", "VISUALIZER", "CAMERA_ACTOR_NAME"})) {
            ASSERT(!camera_actor_);
            camera_actor_ = *actor_itr;
            break;
        }
    }
    ASSERT(camera_actor_);

    // set active camera
    APlayerController* Controller = world->GetFirstPlayerController();
    ASSERT(Controller);
    Controller->SetViewTarget(camera_actor_);

    // setup camera pose
    if (Config::getValue<bool>({"SIMULATION_CONTROLLER", "VISUALIZER", "SET_CAMERA_POSE"})) {
        const FVector camera_pose(
            Config::getValue<float>({"SIMULATION_CONTROLLER", "VISUALIZER", "CAMERA_POSITION_X"}),
            Config::getValue<float>({"SIMULATION_CONTROLLER", "VISUALIZER", "CAMERA_POSITION_Y"}),
            Config::getValue<float>({"SIMULATION_CONTROLLER", "VISUALIZER", "CAMERA_POSITION_Z"}));
        camera_actor_->SetActorLocation(camera_pose);
        camera_actor_->SetActorRotation(FRotator(
            Config::getValue<float>({"SIMULATION_CONTROLLER", "VISUALIZER", "CAMERA_PITCH"}),
            Config::getValue<float>({"SIMULATION_CONTROLLER", "VISUALIZER", "CAMERA_YAW"}),
            Config::getValue<float>({"SIMULATION_CONTROLLER", "VISUALIZER", "CAMERA_ROLL"})));
    }
}

void Visualizer::cleanUpObjectReferences()
{
    ASSERT(camera_actor_);
    camera_actor_ = nullptr;
}
