//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SimulationController/Visualizer.h"

#include <string>

#include <Camera/CameraActor.h>
#include <Engine/World.h>
#include <GameFramework/Actor.h>
#include <GameFramework/PlayerController.h>
#include <Math/Rotator.h>
#include <Math/Vector.h>

#include "CoreUtils/Assert.h"
#include "CoreUtils/Config.h"
#include "CoreUtils/Unreal.h"

Visualizer::Visualizer(UWorld* world)
{
    if (Config::get<bool>("SIMULATION_CONTROLLER.VISUALIZER.SPAWN_CAMERA")) {
        FVector location(
            Config::get<double>("SIMULATION_CONTROLLER.VISUALIZER.SPAWN_CAMERA_POSITION_X"),
            Config::get<double>("SIMULATION_CONTROLLER.VISUALIZER.SPAWN_CAMERA_POSITION_Y"),
            Config::get<double>("SIMULATION_CONTROLLER.VISUALIZER.SPAWN_CAMERA_POSITION_Z"));
        FRotator rotation(
            Config::get<double>("SIMULATION_CONTROLLER.VISUALIZER.SPAWN_CAMERA_ORIENTATION_PITCH"),
            Config::get<double>("SIMULATION_CONTROLLER.VISUALIZER.SPAWN_CAMERA_ORIENTATION_YAW"),
            Config::get<double>("SIMULATION_CONTROLLER.VISUALIZER.SPAWN_CAMERA_ORIENTATION_ROLL"));

        FActorSpawnParameters actor_spawn_params;
        actor_spawn_params.Name = Unreal::toFName(Config::get<std::string>("SIMULATION_CONTROLLER.VISUALIZER.ACTOR_NAME"));
        actor_spawn_params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
        actor_ = world->SpawnActor<ACameraActor>(location, rotation, actor_spawn_params);
        SP_ASSERT(actor_);

        APlayerController* player_controller = world->GetFirstPlayerController();
        SP_ASSERT(player_controller);
        player_controller->SetViewTarget(actor_);
    }
}

Visualizer::~Visualizer()
{
    if (Config::get<bool>("SIMULATION_CONTROLLER.VISUALIZER.SPAWN_CAMERA")) {
        SP_ASSERT(actor_);
        actor_->Destroy();
        actor_ = nullptr;
    }
}

void Visualizer::findObjectReferences(UWorld* world)
{
    if (!Config::get<bool>("SIMULATION_CONTROLLER.VISUALIZER.SPAWN_CAMERA")) {
        actor_ = Unreal::findActorByName(world, Config::get<std::string>("SIMULATION_CONTROLLER.VISUALIZER.ACTOR_NAME"));
        SP_ASSERT(actor_);

        APlayerController* player_controller = world->GetFirstPlayerController();
        SP_ASSERT(player_controller);
        player_controller->SetViewTarget(actor_);
    }
}

void Visualizer::cleanUpObjectReferences()
{
    if (!Config::get<bool>("SIMULATION_CONTROLLER.VISUALIZER.SPAWN_CAMERA")) {
        SP_ASSERT(actor_);
        actor_ = nullptr;
    }
}
