////------ BEGIN UE5 MIGRATION ------////
//// Uncomment this file when UrdfBot is supported in UE5.
/*
//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SimulationController/UrdfBotAgent.h"

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <Components/SceneCaptureComponent2D.h>
#include <Engine/World.h>
#include <Math/Rotator.h>
#include <Math/Vector.h>

#include "CoreUtils/ArrayDesc.h"
#include "CoreUtils/Assert.h"
#include "CoreUtils/Config.h"
#include "CoreUtils/Std.h"
#include "CoreUtils/Unreal.h"
#include "SimulationController/CameraSensor.h"
#include "SimulationController/ImuSensor.h"
#include "SimulationController/SonarSensor.h"
#include "UrdfBot/UrdfBotPawn.h"
#include "UrdfBot/UrdfRobotComponent.h"

UrdfBotAgent::UrdfBotAgent(UWorld* world)
{
    FVector spawn_location = FVector::ZeroVector;
    FRotator spawn_rotation = FRotator::ZeroRotator;
    std::string spawn_mode = Config::get<std::string>("SIMULATION_CONTROLLER.URDFBOT_AGENT.SPAWN_MODE");
    if (spawn_mode == "specify_existing_actor") {
        AActor* spawn_actor = Unreal::findActorByName(world, Config::get<std::string>("SIMULATION_CONTROLLER.URDFBOT_AGENT.SPAWN_ACTOR_NAME"));
        ASSERT(spawn_actor);
        spawn_location = spawn_actor->GetActorLocation();
        spawn_rotation = spawn_actor->GetActorRotation();
    } else if (spawn_mode == "specify_pose") {
        spawn_location = FVector(
            Config::get<float>("SIMULATION_CONTROLLER.URDFBOT_AGENT.SPAWN_POSITION_X"),
            Config::get<float>("SIMULATION_CONTROLLER.URDFBOT_AGENT.SPAWN_POSITION_Y"),
            Config::get<float>("SIMULATION_CONTROLLER.URDFBOT_AGENT.SPAWN_POSITION_Z"));
        spawn_rotation = FRotator(
            Config::get<float>("SIMULATION_CONTROLLER.URDFBOT_AGENT.SPAWN_PITCH"),
            Config::get<float>("SIMULATION_CONTROLLER.URDFBOT_AGENT.SPAWN_YAW"),
            Config::get<float>("SIMULATION_CONTROLLER.URDFBOT_AGENT.SPAWN_ROLL"));
    } else {
        ASSERT(false);
    }
    FActorSpawnParameters actor_spawn_params;
    actor_spawn_params.Name = Unreal::toFName(Config::get<std::string>("SIMULATION_CONTROLLER.URDFBOT_AGENT.URDFBOT_ACTOR_NAME"));
    actor_spawn_params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    urdf_bot_pawn_ = world->SpawnActor<AUrdfBotPawn>(spawn_location, spawn_rotation, actor_spawn_params);
    ASSERT(urdf_bot_pawn_);

    auto observation_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.URDFBOT_AGENT.OBSERVATION_COMPONENTS");

    if (Std::contains(observation_components, "camera")) {
        camera_sensor_ = std::make_unique<CameraSensor>(
            urdf_bot_pawn_->camera_component_,
            Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.URDFBOT_AGENT.CAMERA.RENDER_PASSES"),
            Config::get<unsigned int>("SIMULATION_CONTROLLER.URDFBOT_AGENT.CAMERA.IMAGE_WIDTH"),
            Config::get<unsigned int>("SIMULATION_CONTROLLER.URDFBOT_AGENT.CAMERA.IMAGE_HEIGHT"));
        ASSERT(camera_sensor_);

        // update FOV
        for (auto& pass : Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.URDFBOT_AGENT.CAMERA.RENDER_PASSES")) {
            camera_sensor_->render_passes_.at(pass).scene_capture_component_->FOVAngle =
                Config::get<float>("SIMULATION_CONTROLLER.URDFBOT_AGENT.CAMERA.FOV");
        }
    }
}

UrdfBotAgent::~UrdfBotAgent()
{
    auto observation_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.URDFBOT_AGENT.OBSERVATION_COMPONENTS");

    if (Std::contains(observation_components, "camera")) {
        ASSERT(camera_sensor_);
        camera_sensor_ = nullptr;
    }

    ASSERT(urdf_bot_pawn_);
    urdf_bot_pawn_->Destroy();
    urdf_bot_pawn_ = nullptr;
}

void UrdfBotAgent::findObjectReferences(UWorld* world) {}

void UrdfBotAgent::cleanUpObjectReferences() {}

std::map<std::string, ArrayDesc> UrdfBotAgent::getActionSpace() const
{
    std::map<std::string, ArrayDesc> action_space;
    auto action_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.URDFBOT_AGENT.ACTION_COMPONENTS");

    std::map<std::string, ArrayDesc> robot_component_action_space = urdf_bot_pawn_->urdf_robot_component_->getActionSpace(action_components);
    for (auto& robot_component_observation_action_component : robot_component_action_space) {
        action_space[robot_component_observation_action_component.first] = std::move(robot_component_observation_action_component.second);
    }

    return action_space;
}

std::map<std::string, ArrayDesc> UrdfBotAgent::getObservationSpace() const
{
    std::map<std::string, ArrayDesc> observation_space;

    auto observation_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.URDFBOT_AGENT.OBSERVATION_COMPONENTS");

    std::map<std::string, ArrayDesc> robot_component_observation_space = urdf_bot_pawn_->urdf_robot_component_->getObservationSpace(observation_components);
    for (auto& robot_component_observation_space_component : robot_component_observation_space) {
        observation_space[robot_component_observation_space_component.first] = std::move(robot_component_observation_space_component.second);
    }

    std::map<std::string, ArrayDesc> camera_sensor_observation_space = camera_sensor_->getObservationSpace(observation_components);
    for (auto& camera_sensor_observation_space_component : camera_sensor_observation_space) {
        observation_space[camera_sensor_observation_space_component.first] = std::move(camera_sensor_observation_space_component.second);
    }

    return observation_space;
}

std::map<std::string, ArrayDesc> UrdfBotAgent::getStepInfoSpace() const
{
    return {};
}

void UrdfBotAgent::applyAction(const std::map<std::string, std::vector<uint8_t>>& actions)
{
    auto action_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.URDFBOT_AGENT.ACTION_COMPONENTS");

    if (Std::contains(action_components, "control_joints")) {
        urdf_bot_pawn_->urdf_robot_component_->applyAction(actions);
    }
}

std::map<std::string, std::vector<uint8_t>> UrdfBotAgent::getObservation() const
{
    std::map<std::string, std::vector<uint8_t>> observation;

    auto observation_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.URDFBOT_AGENT.OBSERVATION_COMPONENTS");

    std::map<std::string, std::vector<uint8_t>> robot_component_observation = urdf_bot_pawn_->urdf_robot_component_->getObservation(observation_components);
    for (auto& robot_component_observation_component : robot_component_observation) {
        observation[robot_component_observation_component.first] = std::move(robot_component_observation_component.second);
    }

    std::map<std::string, std::vector<uint8_t>> camera_sensor_observation = camera_sensor_->getObservation(observation_components);
    for (auto& camera_sensor_observation_component : camera_sensor_observation) {
        observation[camera_sensor_observation_component.first] = std::move(camera_sensor_observation_component.second);
    }

    return observation;
}

std::map<std::string, std::vector<uint8_t>> UrdfBotAgent::getStepInfo() const
{
    return {};
}

void UrdfBotAgent::reset() {}

bool UrdfBotAgent::isReady() const
{
    return urdf_bot_pawn_->urdf_robot_component_->GetComponentVelocity().Size() <=
           Config::get<float>("SIMULATION_CONTROLLER.URDFBOT_AGENT.IS_READY_VELOCITY_THRESHOLD");
}
*/
////------ END UE5 MIGRATION ------////