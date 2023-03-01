//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SimulationController/UrdfBotAgent.h"

#include <map>

#include <Components/SceneCaptureComponent2D.h>
#include <Components/BoxComponent.h>
#include <Components/PrimitiveComponent.h>
#include <Components/StaticMeshComponent.h>
#include <Engine/TextureRenderTarget2D.h>
#include <Engine/World.h>
#include <EngineUtils.h>
#include <GameFramework/Actor.h>
#include <Kismet/GameplayStatics.h>
#include <UObject/UObjectGlobals.h>

#include "CoreUtils/Assert.h"
#include "CoreUtils/Config.h"
#include "CoreUtils/Std.h"
#include "CoreUtils/Unreal.h"
#include "UrdfBot/UrdfBotPawn.h"
#include "SimulationController/Box.h"
#include "SimulationController/CameraSensor.h"
#include "SimulationController/ImuSensor.h"
#include "SimulationController/SonarSensor.h"

UrdfBotAgent::UrdfBotAgent(UWorld* world)
{
    FActorSpawnParameters actor_spawn_params;
    actor_spawn_params.Name = Unreal::toFName(Config::get<std::string>("SIMULATION_CONTROLLER.URDFBOT_AGENT.URDFBOT_ACTOR_NAME"));
    actor_spawn_params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    urdf_bot_pawn_ = world->SpawnActor<AUrdfBotPawn>(FVector::ZeroVector, FRotator::ZeroRotator, actor_spawn_params);
    ASSERT(urdf_bot_pawn_);
}

UrdfBotAgent::~UrdfBotAgent()
{
    auto observation_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.URDFBOT_AGENT.OBSERVATION_COMPONENTS");

    ASSERT(urdf_bot_pawn_);
    urdf_bot_pawn_->Destroy();
    urdf_bot_pawn_ = nullptr;
}

void UrdfBotAgent::findObjectReferences(UWorld* world)
{
    auto step_info_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.URDFBOT_AGENT.STEP_INFO_COMPONENTS");
}

void UrdfBotAgent::cleanUpObjectReferences()
{
    auto step_info_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.URDFBOT_AGENT.STEP_INFO_COMPONENTS");
}

std::map<std::string, Box> UrdfBotAgent::getActionSpace() const
{

    std::map<std::string, Box> action_space;
    auto action_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.URDFBOT_AGENT.ACTION_COMPONENTS");

    return action_space;
}

std::map<std::string, Box> UrdfBotAgent::getObservationSpace() const
{
    std::map<std::string, Box> observation_space;
    Box box;

    auto observation_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.URDFBOT_AGENT.OBSERVATION_COMPONENTS");

    return observation_space;
}

std::map<std::string, Box> UrdfBotAgent::getStepInfoSpace() const
{
    std::map<std::string, Box> step_info_space;

    return step_info_space;
}

void UrdfBotAgent::applyAction(const std::map<std::string, std::vector<uint8_t>>& action)
{
    auto action_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.URDFBOT_AGENT.ACTION_COMPONENTS");
}

std::map<std::string, std::vector<uint8_t>> UrdfBotAgent::getObservation() const
{
    std::map<std::string, std::vector<uint8_t>> observation;

    return observation;
}

std::map<std::string, std::vector<uint8_t>> UrdfBotAgent::getStepInfo() const
{
    std::map<std::string, std::vector<uint8_t>> step_info;

    return step_info;
}

void UrdfBotAgent::reset()
{
    // Compute a new trajectory for step_info["trajectory_data"]
    auto step_info_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.URDFBOT_AGENT.STEP_INFO_COMPONENTS");
}

bool UrdfBotAgent::isReady() const
{
    return urdf_bot_pawn_->GetVelocity().Size() <= Config::get<float>("SIMULATION_CONTROLLER.URDFBOT_AGENT.IS_READY_VELOCITY_THRESHOLD");
}
