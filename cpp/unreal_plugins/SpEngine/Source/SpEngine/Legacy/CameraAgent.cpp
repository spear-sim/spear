//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpEngine/Legacy/CameraAgent.h"

#include <stdint.h> // uint8_t

#include <limits>  // std::numeric_limits
#include <map>
#include <memory>  // std::make_unique
#include <string>
#include <utility> // std::move
#include <vector>

#include <Camera/CameraActor.h>
#include <Camera/CameraComponent.h> // UCameraComponent::AspectRatio, UCameraComponent::FieldOfView
#include <Engine/World.h>           // FActorSpawnParameters
#include <GameFramework/Actor.h>
#include <Math/Rotator.h>
#include <Math/Vector.h>

#include "SpCore/ArrayDesc.h" // DataType
#include "SpCore/Assert.h"
#include "SpCore/Config.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"
#include "SpEngine/Legacy/CameraSensor.h"

struct FHitResult;

CameraAgent::CameraAgent(UWorld* world)
{
    FVector spawn_location = FVector::ZeroVector;
    FRotator spawn_rotation = FRotator::ZeroRotator;
    std::string spawn_mode = Config::get<std::string>("SP_ENGINE.LEGACY.CAMERA_AGENT.SPAWN_MODE");
    if (spawn_mode == "specify_existing_actor") {
        AActor* spawn_actor = Unreal::findActorByName(world, Config::get<std::string>("SP_ENGINE.LEGACY.CAMERA_AGENT.SPAWN_ACTOR_NAME"));
        SP_ASSERT(spawn_actor);
        spawn_location = spawn_actor->GetActorLocation();
        spawn_rotation = spawn_actor->GetActorRotation();
    } else if (spawn_mode == "specify_pose") {
        spawn_location = FVector(
            Config::get<double>("SP_ENGINE.LEGACY.CAMERA_AGENT.SPAWN_LOCATION_X"),
            Config::get<double>("SP_ENGINE.LEGACY.CAMERA_AGENT.SPAWN_LOCATION_Y"),
            Config::get<double>("SP_ENGINE.LEGACY.CAMERA_AGENT.SPAWN_LOCATION_Z"));
        spawn_rotation = FRotator(
            Config::get<double>("SP_ENGINE.LEGACY.CAMERA_AGENT.SPAWN_ROTATION_PITCH"),
            Config::get<double>("SP_ENGINE.LEGACY.CAMERA_AGENT.SPAWN_ROTATION_YAW"),
            Config::get<double>("SP_ENGINE.LEGACY.CAMERA_AGENT.SPAWN_ROTATION_ROLL"));
    } else {
        SP_ASSERT(false);
    }
    FActorSpawnParameters actor_spawn_parameters;
    actor_spawn_parameters.Name = Unreal::toFName(Config::get<std::string>("SP_ENGINE.LEGACY.CAMERA_AGENT.CAMERA_ACTOR_NAME"));
    actor_spawn_parameters.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    camera_actor_ = world->SpawnActor<ACameraActor>(spawn_location, spawn_rotation, actor_spawn_parameters);
    SP_ASSERT(camera_actor_);

    camera_actor_->GetCameraComponent()->FieldOfView =
        Config::get<float>("SP_ENGINE.LEGACY.CAMERA_AGENT.CAMERA.FOV");
    camera_actor_->GetCameraComponent()->AspectRatio =
        Config::get<float>("SP_ENGINE.LEGACY.CAMERA_AGENT.CAMERA.IMAGE_WIDTH") /
        Config::get<float>("SP_ENGINE.LEGACY.CAMERA_AGENT.CAMERA.IMAGE_HEIGHT");

    auto observation_components = Config::get<std::vector<std::string>>("SP_ENGINE.LEGACY.CAMERA_AGENT.OBSERVATION_COMPONENTS");

    if (Std::contains(observation_components, "camera")) {
        camera_sensor_ = std::make_unique<CameraSensor>(
            camera_actor_->GetCameraComponent(),
            Config::get<std::vector<std::string>>("SP_ENGINE.LEGACY.CAMERA_AGENT.CAMERA.RENDER_PASSES"),
            Config::get<unsigned int>("SP_ENGINE.LEGACY.CAMERA_AGENT.CAMERA.IMAGE_WIDTH"),
            Config::get<unsigned int>("SP_ENGINE.LEGACY.CAMERA_AGENT.CAMERA.IMAGE_HEIGHT"),
            Config::get<float>("SP_ENGINE.LEGACY.CAMERA_AGENT.CAMERA.FOV"));
        SP_ASSERT(camera_sensor_);
    }
}

CameraAgent::~CameraAgent()
{
    auto observation_components = Config::get<std::vector<std::string>>("SP_ENGINE.LEGACY.CAMERA_AGENT.OBSERVATION_COMPONENTS");

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
    auto action_components = Config::get<std::vector<std::string>>("SP_ENGINE.LEGACY.CAMERA_AGENT.ACTION_COMPONENTS");

    if (Std::contains(action_components, "set_location")) {
        ArrayDesc array_desc;
        array_desc.low_ = std::numeric_limits<float>::lowest();
        array_desc.high_ = std::numeric_limits<float>::max();
        array_desc.shape_ = {3};
        array_desc.datatype_ = DataType::Float64;
        Std::insert(action_space, "set_location", std::move(array_desc));
    }

    if (Std::contains(action_components, "set_rotation")) {
        ArrayDesc array_desc;
        array_desc.low_ = std::numeric_limits<float>::lowest();
        array_desc.high_ = std::numeric_limits<float>::max();
        array_desc.shape_ = {3};
        array_desc.datatype_ = DataType::Float64;
        Std::insert(action_space, "set_rotation", std::move(array_desc));
    }

    return action_space;
}

std::map<std::string, ArrayDesc> CameraAgent::getObservationSpace() const
{
    std::map<std::string, ArrayDesc> observation_space;
    auto observation_components = Config::get<std::vector<std::string>>("SP_ENGINE.LEGACY.CAMERA_AGENT.OBSERVATION_COMPONENTS");

    if (Std::contains(observation_components, "camera")) {
        Std::insert(observation_space, camera_sensor_->getObservationSpace());
    }

    return observation_space;
}

std::map<std::string, ArrayDesc> CameraAgent::getStepInfoSpace() const
{
    return {};
}

void CameraAgent::applyAction(const std::map<std::string, std::vector<uint8_t>>& action)
{
    auto action_components = Config::get<std::vector<std::string>>("SP_ENGINE.LEGACY.CAMERA_AGENT.ACTION_COMPONENTS");

    if (Std::contains(action_components, "set_location")) {
        std::span<const double> action_component_data = Std::reinterpretAsSpanOf<const double>(action.at("set_location"));
        bool sweep = false;
        FHitResult* hit_result = nullptr;
        camera_actor_->SetActorLocation(
            {Std::at(action_component_data, 0), Std::at(action_component_data, 1), Std::at(action_component_data, 2)}, sweep, hit_result, ETeleportType::ResetPhysics);
    }

    if (Std::contains(action_components, "set_rotation")) {
        std::span<const double> action_component_data = Std::reinterpretAsSpanOf<const double>(action.at("set_rotation"));
        camera_actor_->SetActorRotation({Std::at(action_component_data, 0), Std::at(action_component_data, 1), Std::at(action_component_data, 2)}, ETeleportType::ResetPhysics);
    }
}

std::map<std::string, std::vector<uint8_t>> CameraAgent::getObservation() const
{
    std::map<std::string, std::vector<uint8_t>> observation;
    auto observation_components = Config::get<std::vector<std::string>>("SP_ENGINE.LEGACY.CAMERA_AGENT.OBSERVATION_COMPONENTS");

    if (Std::contains(observation_components, "camera")) {
        Std::insert(observation, camera_sensor_->getObservation());
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
