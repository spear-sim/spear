//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SimulationController/UrdfBotAgent.h"

#include <Components/SceneCaptureComponent2D.h>

#include "CoreUtils/Assert.h"
#include "CoreUtils/Config.h"
#include "CoreUtils/Std.h"
#include "CoreUtils/Unreal.h"
#include "SimulationController/Box.h"
#include "SimulationController/CameraSensor.h"
#include "SimulationController/ImuSensor.h"
#include "SimulationController/SonarSensor.h"
#include "UrdfBot/UrdfBotPawn.h"
#include "UrdfBot/UrdfJointComponent.h"
#include "UrdfBot/UrdfLinkComponent.h"
#include "UrdfBot/UrdfRobotComponent.h"
#include "UrdfBot/UrdfParser.h"

UrdfBotAgent::UrdfBotAgent(UWorld* world)
{
    FActorSpawnParameters actor_spawn_params;
    actor_spawn_params.Name = Unreal::toFName(Config::get<std::string>("SIMULATION_CONTROLLER.URDFBOT_AGENT.URDFBOT_ACTOR_NAME"));
    actor_spawn_params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    urdf_bot_pawn_ = world->SpawnActor<AUrdfBotPawn>(FVector::ZeroVector, FRotator::ZeroRotator, actor_spawn_params);
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

std::map<std::string, Box> UrdfBotAgent::getActionSpace() const
{
    std::map<std::string, Box> action_space;
    auto action_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.URDFBOT_AGENT.ACTION_COMPONENTS");

    if (Std::contains(action_components, "joint")) {
        for (auto& joint_component : urdf_bot_pawn_->urdf_robot_component_->joint_components_) {
            if (joint_component.second->control_type_ != UrdfJointControlType::Invalid) {
                Box box;
                box.low_ = std::numeric_limits<float>::lowest();
                box.high_ = std::numeric_limits<float>::max();
                box.shape_ = {1};
                box.datatype_ = DataType::Float32;
                action_space[joint_component.first] = std::move(box);
            }
        }
    }

    return action_space;
}

std::map<std::string, Box> UrdfBotAgent::getObservationSpace() const
{
    std::map<std::string, Box> observation_space;

    auto observation_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.URDFBOT_AGENT.OBSERVATION_COMPONENTS");

    if (Std::contains(observation_components, "link_state")) {
        for (auto& link_component : urdf_bot_pawn_->urdf_robot_component_->link_components_) {
            Box box;
            box.low_ = std::numeric_limits<float>::lowest();
            box.high_ = std::numeric_limits<float>::max();
            box.shape_ = {6};
            box.datatype_ = DataType::Float32;
            observation_space[link_component.first] = std::move(box);
        }
    }

    std::map<std::string, Box> camera_sensor_observation_space = camera_sensor_->getObservationSpace(observation_components);
    for (auto& camera_sensor_observation_space_component : camera_sensor_observation_space) {
        observation_space[camera_sensor_observation_space_component.first] = std::move(camera_sensor_observation_space_component.second);
    }

    return observation_space;
}

std::map<std::string, Box> UrdfBotAgent::getStepInfoSpace() const
{
    return {};
}

void UrdfBotAgent::applyAction(const std::map<std::string, std::vector<uint8_t>>& actions)
{
    auto action_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.URDFBOT_AGENT.ACTION_COMPONENTS");

    if (Std::contains(action_components, "joint")) {
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
