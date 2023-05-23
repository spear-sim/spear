//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SimulationController/WheeledVehicleAgent.h"

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <Components/BoxComponent.h>
#include <Components/PrimitiveComponent.h>
#include <Components/SceneCaptureComponent2D.h>
#include <Components/StaticMeshComponent.h>
#include <Engine/World.h>
#include <EngineUtils.h>
#include <GameFramework/Actor.h>

#include "CoreUtils/ArrayDesc.h"
#include "CoreUtils/Assert.h"
#include "CoreUtils/Config.h"
#include "CoreUtils/Log.h"
#include "CoreUtils/Std.h"
#include "CoreUtils/Unreal.h"
#include "SimulationController/CameraSensor.h"
#include "SimulationController/ImuSensor.h"
#include "SimulationController/SonarSensor.h"
#include "WheeledVehicle/VehiclePawn.h"

WheeledVehicleAgent::WheeledVehicleAgent(UWorld* world)
{
    SP_LOG_CURRENT_FUNCTION();

    FVector spawn_location = FVector::ZeroVector;
    FRotator spawn_rotation = FRotator::ZeroRotator;
    std::string spawn_mode = Config::get<std::string>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.SPAWN_MODE");
    if (spawn_mode == "specify_existing_actor") {
        AActor* spawn_actor = Unreal::findActorByName(world, Config::get<std::string>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.SPAWN_ACTOR_NAME"));
        SP_ASSERT(spawn_actor);
        spawn_location = spawn_actor->GetActorLocation();
        spawn_rotation = spawn_actor->GetActorRotation();
    }
    else if (spawn_mode == "specify_pose") {
        spawn_location = FVector(
            Config::get<float>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.SPAWN_POSITION_X"),
            Config::get<float>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.SPAWN_POSITION_Y"),
            Config::get<float>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.SPAWN_POSITION_Z"));
        spawn_rotation = FRotator(
            Config::get<float>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.SPAWN_PITCH"),
            Config::get<float>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.SPAWN_YAW"),
            Config::get<float>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.SPAWN_ROLL"));
    }
    else {
        SP_ASSERT(false);
    }
    FActorSpawnParameters actor_spawn_params;
    actor_spawn_params.Name = Unreal::toFName(Config::get<std::string>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.WHEELED_VEHICLE_ACTOR_NAME"));
    actor_spawn_params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    wheeled_vehicle_pawn_ = world->SpawnActor<AVehiclePawn>(spawn_location, spawn_rotation, actor_spawn_params);
    SP_ASSERT(wheeled_vehicle_pawn_);

    auto observation_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.OBSERVATION_COMPONENTS");

    if (Std::contains(observation_components, "camera")) {
        camera_sensor_ = std::make_unique<CameraSensor>(
            wheeled_vehicle_pawn_->camera_component_,
            Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.CAMERA.RENDER_PASSES"),
            Config::get<unsigned int>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.CAMERA.IMAGE_WIDTH"),
            Config::get<unsigned int>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.CAMERA.IMAGE_HEIGHT"),
            Config::get<float>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.CAMERA.FOV"));
        SP_ASSERT(camera_sensor_);
    }

    if (Std::contains(observation_components, "imu")) {
        imu_sensor_ = std::make_unique<ImuSensor>(wheeled_vehicle_pawn_->imu_component_);
        SP_ASSERT(imu_sensor_);
    }

    if (Std::contains(observation_components, "sonar")) {
        sonar_sensor_ = std::make_unique<SonarSensor>(wheeled_vehicle_pawn_->sonar_component_);
        SP_ASSERT(sonar_sensor_);
    }
}

WheeledVehicleAgent::~WheeledVehicleAgent()
{
    SP_LOG_CURRENT_FUNCTION();

    auto observation_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.OBSERVATION_COMPONENTS");

    if (Std::contains(observation_components, "sonar")) {
        SP_ASSERT(sonar_sensor_);
        sonar_sensor_ = nullptr;
    }

    if (Std::contains(observation_components, "imu")) {
        SP_ASSERT(imu_sensor_);
        imu_sensor_ = nullptr;
    }

    if (Std::contains(observation_components, "camera")) {
        SP_ASSERT(camera_sensor_);
        camera_sensor_ = nullptr;
    }

    SP_ASSERT(wheeled_vehicle_pawn_);
    wheeled_vehicle_pawn_->Destroy();
    wheeled_vehicle_pawn_ = nullptr;
}

void WheeledVehicleAgent::findObjectReferences(UWorld* world) {}

void WheeledVehicleAgent::cleanUpObjectReferences() {}

std::map<std::string, ArrayDesc> WheeledVehicleAgent::getActionSpace() const
{
    std::map<std::string, ArrayDesc> action_space;
    auto action_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.ACTION_COMPONENTS");

    if (Std::contains(action_components, "apply_wheel_torques")) {
        ArrayDesc array_desc;
        array_desc.low_ = std::numeric_limits<float>::lowest();;
        array_desc.high_ = std::numeric_limits<float>::max();;
        array_desc.shape_ = { 4 };
        array_desc.datatype_ = DataType::Float64;
        action_space["apply_wheel_torques"] = std::move(array_desc);
    }

    if (Std::contains(action_components, "set_position_xyz_centimeters")) {
        ArrayDesc array_desc;
        array_desc.low_ = std::numeric_limits<float>::lowest();
        array_desc.high_ = std::numeric_limits<float>::max();
        array_desc.shape_ = { 3 };
        array_desc.datatype_ = DataType::Float64;
        action_space["set_position_xyz_centimeters"] = std::move(array_desc);
    }

    if (Std::contains(action_components, "set_orientation_pyr_radians")) {
        ArrayDesc array_desc;
        array_desc.low_ = std::numeric_limits<float>::lowest();
        array_desc.high_ = std::numeric_limits<float>::max();
        array_desc.shape_ = { 3 };
        array_desc.datatype_ = DataType::Float64;
        action_space["set_orientation_pyr_radians"] = std::move(array_desc);
    }

    return action_space;
}

std::map<std::string, ArrayDesc> WheeledVehicleAgent::getObservationSpace() const
{
    std::map<std::string, ArrayDesc> observation_space;
    auto observation_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.OBSERVATION_COMPONENTS");

    if (Std::contains(observation_components, "state_data")) {
        ArrayDesc array_desc;
        array_desc.low_ = std::numeric_limits<float>::lowest();
        array_desc.high_ = std::numeric_limits<float>::max();
        array_desc.datatype_ = DataType::Float64;
        array_desc.shape_ = { 6 };
        observation_space["state_data"] = std::move(array_desc); // position (X, Y, Z) and orientation (Roll, Pitch, Yaw) of the agent relative to the world frame.
    }

    if (Std::contains(observation_components, "encoder")) {
        ArrayDesc array_desc;
        array_desc.low_ = std::numeric_limits<float>::lowest();
        array_desc.high_ = std::numeric_limits<float>::max();
        array_desc.datatype_ = DataType::Float64;
        array_desc.shape_ = { 4 };
        observation_space["encoder"] = std::move(array_desc); // FL, FR, RL, RR
    }

    if (Std::contains(observation_components, "imu")) {
        ArrayDesc array_desc;
        array_desc.low_ = std::numeric_limits<float>::lowest();
        array_desc.high_ = std::numeric_limits<float>::max();
        array_desc.datatype_ = DataType::Float64;
        array_desc.shape_ = { 6 };
        observation_space["imu"] = std::move(array_desc); // a_x, a_y, a_z, g_x, g_y, g_z
    }

    if (Std::contains(observation_components, "sonar")) {
        ArrayDesc array_desc;
        array_desc.low_ = std::numeric_limits<float>::lowest();
        array_desc.high_ = std::numeric_limits<float>::max();
        array_desc.datatype_ = DataType::Float64;
        array_desc.shape_ = { 1 };
        observation_space["sonar"] = std::move(array_desc); // Front obstacle distance in [m]
    }

    std::map<std::string, ArrayDesc> camera_sensor_observation_space = camera_sensor_->getObservationSpace(observation_components);
    for (auto& camera_sensor_observation_space_component : camera_sensor_observation_space) {
        observation_space[camera_sensor_observation_space_component.first] = std::move(camera_sensor_observation_space_component.second);
    }

    return observation_space;
}

std::map<std::string, ArrayDesc> WheeledVehicleAgent::getStepInfoSpace() const
{
    return {};
}

void WheeledVehicleAgent::applyAction(const std::map<std::string, std::vector<uint8_t>>& action)
{
    auto action_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.ACTION_COMPONENTS");

    if (Std::contains(action_components, "apply_wheel_torques")) {
        std::vector<double> component_data = Std::reinterpretAs<double>(action.at("apply_wheel_torques"));
        wheeled_vehicle_pawn_->setDriveTorques(Eigen::Vector4d(
            component_data.at(0), component_data.at(1), component_data.at(2), component_data.at(3)));
        wheeled_vehicle_pawn_->setBrakeTorques(Eigen::Vector4d(0.0f, 0.0f, 0.0f, 0.0f));
    }

    if (Std::contains(action_components, "set_position_xyz_centimeters")) {
        std::vector<double> component_data = Std::reinterpretAs<double>(action.at("set_position_xyz_centimeters"));
        FVector location(component_data.at(0), component_data.at(1), component_data.at(2));
        bool sweep = false;
        FHitResult* hit_result = nullptr;
        wheeled_vehicle_pawn_->SetActorLocation(location, sweep, hit_result, ETeleportType::TeleportPhysics);
        wheeled_vehicle_pawn_->setBrakeTorques(Eigen::Vector4d(1000.0f, 1000.0f, 1000.0f, 1000.0f)); // TODO: get value from the config system
    }

    if (Std::contains(action_components, "set_orientation_pyr_radians")) {
        std::vector<double> component_data = Std::reinterpretAs<double>(action.at("set_orientation_pyr_radians"));
        FRotator rotation(
            FMath::RadiansToDegrees(component_data.at(0)),
            FMath::RadiansToDegrees(component_data.at(1)),
            FMath::RadiansToDegrees(component_data.at(2)));
        wheeled_vehicle_pawn_->SetActorRotation(rotation, ETeleportType::TeleportPhysics);
        wheeled_vehicle_pawn_->setBrakeTorques(Eigen::Vector4d(1000.0f, 1000.0f, 1000.0f, 1000.0f)); // TODO: get value from the config system
    }
}

std::map<std::string, std::vector<uint8_t>> WheeledVehicleAgent::getObservation() const
{
    std::map<std::string, std::vector<uint8_t>> observation;

    auto observation_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.OBSERVATION_COMPONENTS");

    if (Std::contains(observation_components, "state_data")) {
        FVector location = wheeled_vehicle_pawn_->GetActorLocation();
        FRotator rotation = wheeled_vehicle_pawn_->GetActorRotation();
        observation["state_data"] = Std::reinterpretAs<uint8_t>(std::vector<double>{
            location.X,
            location.Y,
            location.Z,
            FMath::DegreesToRadians(rotation.Pitch),
            FMath::DegreesToRadians(rotation.Yaw),
            FMath::DegreesToRadians(rotation.Roll)});
    }

    if (Std::contains(observation_components, "encoder")) {
        Eigen::Vector4d wheel_rotation_speeds = wheeled_vehicle_pawn_->getWheelRotationSpeeds();
        observation["encoder"] = Std::reinterpretAs<uint8_t>(std::vector<double>{
            wheel_rotation_speeds(0),
            wheel_rotation_speeds(1),
            wheel_rotation_speeds(2),
            wheel_rotation_speeds(3)});
    }

    if (Std::contains(observation_components, "imu")) {
        observation["imu"] = Std::reinterpretAs<uint8_t>(std::vector<double>{
            imu_sensor_->linear_acceleration_body_.X,
            imu_sensor_->linear_acceleration_body_.Y,
            imu_sensor_->linear_acceleration_body_.Z,
            imu_sensor_->angular_velocity_body_.X,
            imu_sensor_->angular_velocity_body_.Y,
            imu_sensor_->angular_velocity_body_.Z});
    }

    if (Std::contains(observation_components, "sonar")) {
        observation["sonar"] = Std::reinterpretAs<uint8_t>(std::vector<double>{sonar_sensor_->range_});
    }

    std::map<std::string, std::vector<uint8_t>> camera_sensor_observation = camera_sensor_->getObservation(observation_components);
    for (auto& camera_sensor_observation_component : camera_sensor_observation) {
        observation[camera_sensor_observation_component.first] = std::move(camera_sensor_observation_component.second);
    }

    return observation;
}

std::map<std::string, std::vector<uint8_t>> WheeledVehicleAgent::getStepInfo() const
{
    return {};
}

void WheeledVehicleAgent::reset()
{
    // For some reason, the pose of AOpenBotPawn needs to be set using ETeleportType::TeleportPhysics, which maintains
    // velocity information across calls to SetActorPositionAndRotation(...). Since tasks are supposed to be implemented
    // in a general way, they must therefore use ETeleportType::TeleportPhysics to set the pose of actors, because the
    // actor they're attempting to reset might be an AOpenBotPawn. But this means that our velocity will be maintained
    // unless we explicitly reset it, so we reset our velocity here.
    wheeled_vehicle_pawn_->skeletal_mesh_component_->SetPhysicsLinearVelocity(FVector::ZeroVector, false);
    wheeled_vehicle_pawn_->skeletal_mesh_component_->SetPhysicsAngularVelocityInRadians(FVector::ZeroVector, false);
    wheeled_vehicle_pawn_->skeletal_mesh_component_->GetBodyInstance()->ClearTorques();
    wheeled_vehicle_pawn_->skeletal_mesh_component_->GetBodyInstance()->ClearForces();

    // Reset vehicle
    wheeled_vehicle_pawn_->resetVehicle();
    wheeled_vehicle_pawn_->setBrakeTorques(Eigen::Vector4d(1000.0f, 1000.0f, 1000.0f, 1000.0f)); // TODO: get value from the config system
}

bool WheeledVehicleAgent::isReady() const
{
    return wheeled_vehicle_pawn_->GetVelocity().Size() <= Config::get<float>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.IS_READY_VELOCITY_THRESHOLD");
}
