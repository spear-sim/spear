//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpEngine/Legacy/VehicleAgent.h"

#include <stdint.h> // uint8_t

#include <limits>  // std::numeric_limits
#include <map>
#include <memory>  // std::make_unique
#include <string>
#include <utility> // std::move
#include <vector>

#include <Camera/CameraComponent.h>  // UCameraComponent::AspectRatio, UCameraComponent::FieldOfView
#include <Components/BoxComponent.h> // UBoxComponent, UPrimitiveComponent
#include <Engine/World.h>            // FActorSpawnParameters
#include <GameFramework/Actor.h>
#include <Math/Rotator.h>
#include <Math/Vector.h>

#include "SpEngine/Legacy/CameraSensor.h"
#include "SpEngine/Legacy/ImuSensor.h"
#include "SpCore/ArrayDesc.h"
#include "SpCore/Assert.h"
#include "SpCore/Config.h"
#include "SpCore/Log.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"
#include "Vehicle/VehicleMovementComponent.h"
#include "Vehicle/VehiclePawn.h"

VehicleAgent::VehicleAgent(UWorld* world)
{
    FVector spawn_location = FVector::ZeroVector;
    FRotator spawn_rotation = FRotator::ZeroRotator;
    auto spawn_mode = Config::get<std::string>("SIMULATION_CONTROLLER.VEHICLE_AGENT.SPAWN_MODE");
    if (spawn_mode == "specify_existing_actor") {
        AActor* spawn_actor = Unreal::findActorByName(world, Config::get<std::string>("SIMULATION_CONTROLLER.VEHICLE_AGENT.SPAWN_ACTOR_NAME"));
        SP_ASSERT(spawn_actor);
        spawn_location = spawn_actor->GetActorLocation();
        spawn_rotation = spawn_actor->GetActorRotation();
    } else if (spawn_mode == "specify_pose") {
        spawn_location = FVector(
            Config::get<double>("SIMULATION_CONTROLLER.VEHICLE_AGENT.SPAWN_LOCATION_X"),
            Config::get<double>("SIMULATION_CONTROLLER.VEHICLE_AGENT.SPAWN_LOCATION_Y"),
            Config::get<double>("SIMULATION_CONTROLLER.VEHICLE_AGENT.SPAWN_LOCATION_Z"));
        spawn_rotation = FRotator(
            Config::get<double>("SIMULATION_CONTROLLER.VEHICLE_AGENT.SPAWN_ROTATION_PITCH"),
            Config::get<double>("SIMULATION_CONTROLLER.VEHICLE_AGENT.SPAWN_ROTATION_YAW"),
            Config::get<double>("SIMULATION_CONTROLLER.VEHICLE_AGENT.SPAWN_ROTATION_ROLL"));
    } else {
        SP_ASSERT(false);
    }
    FActorSpawnParameters actor_spawn_parameters;
    actor_spawn_parameters.Name = Unreal::toFName(Config::get<std::string>("SIMULATION_CONTROLLER.VEHICLE_AGENT.VEHICLE_ACTOR_NAME"));
    actor_spawn_parameters.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    vehicle_pawn_ = world->SpawnActor<AVehiclePawn>(spawn_location, spawn_rotation, actor_spawn_parameters);
    SP_ASSERT(vehicle_pawn_);

    vehicle_pawn_->CameraComponent->FieldOfView =
        Config::get<float>("SIMULATION_CONTROLLER.VEHICLE_AGENT.CAMERA.FOV");
    vehicle_pawn_->CameraComponent->AspectRatio =
        Config::get<float>("SIMULATION_CONTROLLER.VEHICLE_AGENT.CAMERA.IMAGE_WIDTH") /
        Config::get<float>("SIMULATION_CONTROLLER.VEHICLE_AGENT.CAMERA.IMAGE_HEIGHT");

    // We don't normally cache config values in member variables, but we make an exception in this case
    // because we want ACTION_COMPONENTS and OBSERVATION_COMPONENTS to be defined in VEHICLE_AGENT, but
    // we don't want to pass these arrays around every time we need to apply an action or get an observation.
    vehicle_pawn_->setActionComponents(Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.VEHICLE_AGENT.ACTION_COMPONENTS"));
    vehicle_pawn_->setObservationComponents(Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.VEHICLE_AGENT.OBSERVATION_COMPONENTS"));

    auto observation_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.VEHICLE_AGENT.OBSERVATION_COMPONENTS");

    if (Std::contains(observation_components, "camera")) {
        camera_sensor_ = std::make_unique<CameraSensor>(
            vehicle_pawn_->CameraComponent,
            Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.VEHICLE_AGENT.CAMERA.RENDER_PASSES"),
            Config::get<unsigned int>("SIMULATION_CONTROLLER.VEHICLE_AGENT.CAMERA.IMAGE_WIDTH"),
            Config::get<unsigned int>("SIMULATION_CONTROLLER.VEHICLE_AGENT.CAMERA.IMAGE_HEIGHT"),
            Config::get<float>("SIMULATION_CONTROLLER.VEHICLE_AGENT.CAMERA.FOV"));
        SP_ASSERT(camera_sensor_);
    }

    if (Std::contains(observation_components, "imu")) {
        imu_sensor_ = std::make_unique<ImuSensor>(vehicle_pawn_->ImuComponent);
        SP_ASSERT(imu_sensor_);
    }
}

VehicleAgent::~VehicleAgent()
{
    SP_LOG_CURRENT_FUNCTION();

    auto observation_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.VEHICLE_AGENT.OBSERVATION_COMPONENTS");

    if (Std::contains(observation_components, "imu")) {
        SP_ASSERT(imu_sensor_);
        imu_sensor_ = nullptr;
    }

    if (Std::contains(observation_components, "camera")) {
        SP_ASSERT(camera_sensor_);
        camera_sensor_ = nullptr;
    }

    SP_ASSERT(vehicle_pawn_);
    vehicle_pawn_->Destroy();
    vehicle_pawn_ = nullptr;
}

void VehicleAgent::findObjectReferences(UWorld* world) {}
void VehicleAgent::cleanUpObjectReferences() {}

std::map<std::string, ArrayDesc> VehicleAgent::getActionSpace() const
{
    return vehicle_pawn_->getActionSpace();
}

std::map<std::string, ArrayDesc> VehicleAgent::getObservationSpace() const
{
    std::map<std::string, ArrayDesc> observation_space;
    auto observation_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.VEHICLE_AGENT.OBSERVATION_COMPONENTS");

    Std::insert(observation_space, vehicle_pawn_->getObservationSpace());

    if (Std::contains(observation_components, "camera")) {
        Std::insert(observation_space, camera_sensor_->getObservationSpace());
    }

    if (Std::contains(observation_components, "imu")) {
        Std::insert(observation_space, imu_sensor_->getObservationSpace());
    }

    return observation_space;
}

std::map<std::string, ArrayDesc> VehicleAgent::getStepInfoSpace() const
{
    return {};
}

void VehicleAgent::applyAction(const std::map<std::string, std::vector<uint8_t>>& action)
{
    vehicle_pawn_->applyAction(action);
}

std::map<std::string, std::vector<uint8_t>> VehicleAgent::getObservation() const
{
    std::map<std::string, std::vector<uint8_t>> observation;
    auto observation_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.VEHICLE_AGENT.OBSERVATION_COMPONENTS");

    Std::insert(observation, vehicle_pawn_->getObservation());

    if (Std::contains(observation_components, "camera")) {
        Std::insert(observation, camera_sensor_->getObservation());
    }

    if (Std::contains(observation_components, "imu")) {
        Std::insert(observation, imu_sensor_->getObservation());
    }

    return observation;
}

std::map<std::string, std::vector<uint8_t>> VehicleAgent::getStepInfo() const
{
    return {};
}

void VehicleAgent::reset()
{
    vehicle_pawn_->GetVehicleMovementComponent()->ResetVehicle();
}

bool VehicleAgent::isReady() const
{
    return vehicle_pawn_->GetVelocity().Size() <= Config::get<double>("SIMULATION_CONTROLLER.VEHICLE_AGENT.IS_READY_VELOCITY_THRESHOLD");
}
