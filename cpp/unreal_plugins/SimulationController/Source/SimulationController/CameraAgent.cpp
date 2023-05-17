//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SimulationController/CameraAgent.h"

#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Camera/CameraActor.h>
#include <Camera/CameraComponent.h>
#include <Engine/EngineTypes.h>
#include <Engine/World.h>
#include <GameFramework/Actor.h>
#include <Math/Rotator.h>
#include <Math/Vector.h>

#include "CoreUtils/ArrayDesc.h"
#include "CoreUtils/Assert.h"
#include "CoreUtils/Config.h"
#include "CoreUtils/Std.h"
#include "CoreUtils/Unreal.h"
#include "SimulationController/CameraSensor.h"

struct FHitResult;

CameraAgent::CameraAgent(UWorld* world)
{
    FVector spawn_location = FVector::ZeroVector;
    FRotator spawn_rotation = FRotator::ZeroRotator;
    std::string spawn_mode = Config::get<std::string>("SIMULATION_CONTROLLER.CAMERA_AGENT.SPAWN_MODE");
    if (spawn_mode == "specify_existing_actor") {
        AActor* spawn_actor = Unreal::findActorByName(world, Config::get<std::string>("SIMULATION_CONTROLLER.CAMERA_AGENT.SPAWN_ACTOR_NAME"));
        SP_ASSERT(spawn_actor);
        spawn_location = spawn_actor->GetActorLocation();
        spawn_rotation = spawn_actor->GetActorRotation();
    } else if (spawn_mode == "specify_pose") {
        spawn_location = FVector(
            Config::get<double>("SIMULATION_CONTROLLER.CAMERA_AGENT.SPAWN_POSITION_X"),
            Config::get<double>("SIMULATION_CONTROLLER.CAMERA_AGENT.SPAWN_POSITION_Y"),
            Config::get<double>("SIMULATION_CONTROLLER.CAMERA_AGENT.SPAWN_POSITION_Z"));
        spawn_rotation = FRotator(
            Config::get<double>("SIMULATION_CONTROLLER.CAMERA_AGENT.SPAWN_PITCH"),
            Config::get<double>("SIMULATION_CONTROLLER.CAMERA_AGENT.SPAWN_YAW"),
            Config::get<double>("SIMULATION_CONTROLLER.CAMERA_AGENT.SPAWN_ROLL"));
    } else {
        SP_ASSERT(false);
    }
    FActorSpawnParameters actor_spawn_params;
    actor_spawn_params.Name = Unreal::toFName(Config::get<std::string>("SIMULATION_CONTROLLER.CAMERA_AGENT.CAMERA_ACTOR_NAME"));
    actor_spawn_params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    camera_actor_ = world->SpawnActor<ACameraActor>(spawn_location, spawn_rotation, actor_spawn_params);
    SP_ASSERT(camera_actor_);

    camera_actor_->GetCameraComponent()->AspectRatio =
        Config::get<float>("SIMULATION_CONTROLLER.CAMERA_AGENT.CAMERA.IMAGE_WIDTH") /
        Config::get<float>("SIMULATION_CONTROLLER.CAMERA_AGENT.CAMERA.IMAGE_HEIGHT");

    auto observation_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.CAMERA_AGENT.OBSERVATION_COMPONENTS");

    if (Std::contains(observation_components, "camera")) {
        camera_sensor_ = std::make_unique<CameraSensor>(
            camera_actor_->GetCameraComponent(),
            Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.CAMERA_AGENT.CAMERA.RENDER_PASSES"),
            Config::get<unsigned int>("SIMULATION_CONTROLLER.CAMERA_AGENT.CAMERA.IMAGE_WIDTH"),
            Config::get<unsigned int>("SIMULATION_CONTROLLER.CAMERA_AGENT.CAMERA.IMAGE_HEIGHT"),
            Config::get<float>("SIMULATION_CONTROLLER.CAMERA_AGENT.CAMERA.FOV"));
        SP_ASSERT(camera_sensor_);
    }
}

CameraAgent::~CameraAgent()
{
    auto observation_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.CAMERA_AGENT.OBSERVATION_COMPONENTS");

    if (Std::contains(observation_components, "camera")) {
        SP_ASSERT(camera_sensor_);
        camera_sensor_ = nullptr;
    }

    SP_ASSERT(camera_actor_);
    camera_actor_->Destroy();
    camera_actor_ = nullptr;
}

void CameraAgent::findObjectReferences(UWorld* world) {}
void CameraAgent::cleanUpObjectReferences() {}

std::map<std::string, ArrayDesc> CameraAgent::getActionSpace() const
{
    std::map<std::string, ArrayDesc> action_space;
    auto action_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.CAMERA_AGENT.ACTION_COMPONENTS");

    if (Std::contains(action_components, "set_position")) {
        ArrayDesc array_desc;
        array_desc.low_ = std::numeric_limits<float>::lowest();
        array_desc.high_ = std::numeric_limits<float>::max();
        array_desc.shape_ = {3};
        array_desc.datatype_ = DataType::Float64;
        action_space["set_position"] = std::move(array_desc);
    }

    if (Std::contains(action_components, "set_rotation")) {
        ArrayDesc array_desc;
        array_desc.low_ = std::numeric_limits<float>::lowest();
        array_desc.high_ = std::numeric_limits<float>::max();
        array_desc.shape_ = {3};
        array_desc.datatype_ = DataType::Float64;
        action_space["set_rotation"] = std::move(array_desc);
    }

    return action_space;
}

std::map<std::string, ArrayDesc> CameraAgent::getObservationSpace() const
{
    std::map<std::string, ArrayDesc> observation_space;
    auto observation_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.CAMERA_AGENT.OBSERVATION_COMPONENTS");

    std::map<std::string, ArrayDesc> camera_sensor_observation_space = camera_sensor_->getObservationSpace(observation_components);
    for (auto& camera_sensor_observation_space_component : camera_sensor_observation_space) {
        observation_space[camera_sensor_observation_space_component.first] = std::move(camera_sensor_observation_space_component.second);
    }

    return observation_space;
}

std::map<std::string, ArrayDesc> CameraAgent::getStepInfoSpace() const
{
    return {};
}

void CameraAgent::applyAction(const std::map<std::string, std::vector<uint8_t>>& action)
{
    auto action_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.CAMERA_AGENT.ACTION_COMPONENTS");

    if (Std::contains(action_components, "set_position")) {
        std::vector<double> component_data = Std::reinterpretAs<double>(action.at("set_position"));
        FVector agent_location(component_data.at(0), component_data.at(1), component_data.at(2));
        bool sweep = false;
        FHitResult* hit_result = nullptr;
        camera_actor_->SetActorLocation(agent_location, sweep, hit_result, ETeleportType::ResetPhysics);
    }

    if (Std::contains(action_components, "set_rotation")) {
        std::vector<double> component_data = Std::reinterpretAs<double>(action.at("set_rotation"));
        FRotator agent_rotation(component_data.at(0), component_data.at(1), component_data.at(2));
        camera_actor_->SetActorRotation(agent_rotation, ETeleportType::ResetPhysics);
    }
}

std::map<std::string, std::vector<uint8_t>> CameraAgent::getObservation() const
{
    std::map<std::string, std::vector<uint8_t>> observation;
    auto observation_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.CAMERA_AGENT.OBSERVATION_COMPONENTS");

    std::map<std::string, std::vector<uint8_t>> camera_sensor_observation = camera_sensor_->getObservation(observation_components);
    for (auto& camera_sensor_observation_component : camera_sensor_observation) {
        observation[camera_sensor_observation_component.first] = std::move(camera_sensor_observation_component.second);
    }

    return observation;
}

std::map<std::string, std::vector<uint8_t>> CameraAgent::getStepInfo() const
{
    return {};
}

void CameraAgent::reset() {}

bool CameraAgent::isReady() const
{
    return true;
}
