//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SimulationController/Visualizer.h"

#include <string>

#include <Camera/CameraActor.h>
#include <Containers/StringConv.h>
#include <EngineUtils.h>
#include <GameFramework/Actor.h>
#include <GameFramework/PlayerController.h>

#include "CoreUtils/Config.h"
#include "CoreUtils/UnrealUtils.h"

Visualizer::Visualizer(UWorld* world)
{
    if (Config::getValue<bool>({"SIMULATION_CONTROLLER", "VISUALIZER", "SPAWN_CAMERA"})) {

        FVector location(
            Config::getValue<float>({"SIMULATION_CONTROLLER", "VISUALIZER", "SPAWN_CAMERA_POSITION_X"}),
            Config::getValue<float>({"SIMULATION_CONTROLLER", "VISUALIZER", "SPAWN_CAMERA_POSITION_Y"}),
            Config::getValue<float>({"SIMULATION_CONTROLLER", "VISUALIZER", "SPAWN_CAMERA_POSITION_Z"}));
        FRotator orientation(
            Config::getValue<float>({"SIMULATION_CONTROLLER", "VISUALIZER", "SPAWN_CAMERA_ORIENTATION_PITCH"}),
            Config::getValue<float>({"SIMULATION_CONTROLLER", "VISUALIZER", "SPAWN_CAMERA_ORIENTATION_YAW"}),
            Config::getValue<float>({"SIMULATION_CONTROLLER", "VISUALIZER", "SPAWN_CAMERA_ORIENTATION_ROLL"}));

        FActorSpawnParameters actor_spawn_params;
        actor_spawn_params.Name = UnrealUtils::toFName(Config::getValue<std::string>({"SIMULATION_CONTROLLER", "VISUALIZER", "ACTOR_NAME"}));
        actor_spawn_params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
        actor_ = world->SpawnActor<ACameraActor>(location, orientation, actor_spawn_params);
        ASSERT(actor_);

        APlayerController* player_controller = world->GetFirstPlayerController();
        ASSERT(player_controller);
        player_controller->SetViewTarget(actor_);
    }
}

Visualizer::~Visualizer()
{
    if (Config::getValue<bool>({"SIMULATION_CONTROLLER", "VISUALIZER", "SPAWN_CAMERA"})) {
        ASSERT(actor_);
        actor_->Destroy();
        actor_ = nullptr;
    }
}

void Visualizer::findObjectReferences(UWorld* world)
{
    if (!Config::getValue<bool>({"SIMULATION_CONTROLLER", "VISUALIZER", "SPAWN_CAMERA"})) {
        actor_ = UnrealUtils::findActorByName(world, Config::getValue<std::string>({"SIMULATION_CONTROLLER", "VISUALIZER", "ACTOR_NAME"}));
        ASSERT(actor_);

        APlayerController* player_controller = world->GetFirstPlayerController();
        ASSERT(player_controller);
        player_controller->SetViewTarget(actor_);
    }
}

void Visualizer::cleanUpObjectReferences()
{
    if (!Config::getValue<bool>({"SIMULATION_CONTROLLER", "VISUALIZER", "SPAWN_CAMERA"})) {
        ASSERT(actor_);
        actor_ = nullptr;
    }
}
