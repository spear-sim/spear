#include "Visualizer.h"

#include <string>

#include <Camera/CameraActor.h>
#include <Containers/StringConv.h>
#include <EngineUtils.h>
#include <GameFramework/Actor.h>
#include <GameFramework/PlayerController.h>

#include "Config.h"

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

        FActorSpawnParameters spawn_params;
        spawn_params.Name = FName(Config::getValue<std::string>({"SIMULATION_CONTROLLER", "VISUALIZER", "ACTOR_NAME"}).c_str());
        spawn_params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
        actor_ = world->SpawnActor<ACameraActor>(location, orientation, spawn_params);
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

        for (TActorIterator<AActor> actor_itr(world); actor_itr; ++actor_itr) {
            std::string actor_name = TCHAR_TO_UTF8(*((*actor_itr)->GetName()));
            if (actor_name == Config::getValue<std::string>({"SIMULATION_CONTROLLER", "VISUALIZER", "ACTOR_NAME"})) {
                ASSERT(!actor_);
                actor_ = *actor_itr;
                break;
            }
        }
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
