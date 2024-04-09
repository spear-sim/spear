//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "Vehicle/VehiclePawn.h"

#include <stdint.h> // uint8_t

#include <map>
#include <string>
#include <utility> // std::move
#include <vector>

#include <Animation/AnimInstance.h>
#include <Camera/CameraComponent.h>
#include <Components/BoxComponent.h>
#include <Components/SkeletalMeshComponent.h>
#include <Engine/CollisionProfile.h>
#include <Engine/SkeletalMesh.h>
#include <Templates/Casts.h>
#include <UObject/ConstructorHelpers.h>
#include <UObject/Object.h>         // CreateDefaultSubobject
#include <UObject/UObjectGlobals.h> // FObjectInitializer
#include <WheeledVehiclePawn.h>

#include "SpCore/ArrayDesc.h"
#include "SpCore/Assert.h"
#include "SpCore/Config.h"
#include "SpCore/Log.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"
#include "SpCore/StableNameComponent.h"
#include "SpCore/UserInputComponent.h"
#include "Vehicle/VehicleMovementComponent.h"

const std::map<std::string, std::map<std::string, std::vector<double>>> DEFAULT_USER_INPUT_ACTIONS = {
    {"One",   {{"set_drive_torques", { 0.1f,  0.1f,  0.1f,  0.1f}}}},
    {"Two",   {{"set_drive_torques", { 0.1f, -0.1f,  0.1f, -0.1f}}}},
    {"Three", {{"set_drive_torques", {-0.1f,  0.1f, -0.1f,  0.1f}}}},
    {"Four",  {{"set_drive_torques", {-0.1f, -0.1f, -0.1f, -0.1f}}}},
    {"Five",  {{"set_drive_torques", { 0.0f,  0.0f,  0.0f,  0.0f}}}}
};

// Calling the AWheeledVehiclePawn constructor in this way is necessary to override the UChaosWheeledVehicleMovementComponent
// class used by AWheeledVehiclePawn. See the following link for details:
//     https://docs.unrealengine.com/5.2/en-US/API/Plugins/ChaosVehicles/AWheeledVehiclePawn
AVehiclePawn::AVehiclePawn(const FObjectInitializer& object_initializer) :
    AWheeledVehiclePawn(object_initializer.SetDefaultSubobjectClass<UVehicleMovementComponent>(AWheeledVehiclePawn::VehicleMovementComponentName))
{
    SP_LOG_CURRENT_FUNCTION();

    // UStableNameComponent
    StableNameComponent = Unreal::createComponentInsideOwnerConstructor<UStableNameComponent>(this, "stable_name");
    SP_ASSERT(StableNameComponent);

    // UUserInputComponent
    UserInputComponent = Unreal::createComponentInsideOwnerConstructor<UUserInputComponent>(this, GetMesh(), "user_input");
    SP_ASSERT(UserInputComponent);

    // USkeletalMeshComponent
    std::string skeletal_mesh_str;
    std::string anim_instance_str;
    if (Config::isInitialized()) {
        skeletal_mesh_str = Config::get<std::string>("VEHICLE.VEHICLE_PAWN.SKELETAL_MESH");
        anim_instance_str = Config::get<std::string>("VEHICLE.VEHICLE_PAWN.ANIM_INSTANCE");
    } else {
        // OpenBot defaults, see python/spear/config/default_config.vehicle.yaml
        skeletal_mesh_str = "/Vehicle/OpenBot/Meshes/SK_OpenBot.SK_OpenBot";
        anim_instance_str = "/Vehicle/OpenBot/Meshes/ABP_OpenBot.ABP_OpenBot_C";
    }

    ConstructorHelpers::FObjectFinder<USkeletalMesh> skeletal_mesh(*Unreal::toFString(skeletal_mesh_str));
    SP_ASSERT(skeletal_mesh.Succeeded());

    ConstructorHelpers::FClassFinder<UAnimInstance> anim_instance(*Unreal::toFString(anim_instance_str));
    SP_ASSERT(anim_instance.Succeeded());

    GetMesh()->SetSkeletalMesh(skeletal_mesh.Object);
    GetMesh()->SetAnimClass(anim_instance.Class);

    // The AWheeledVehiclePawn constructor sets this parameter to false, but we want it set to true.
    // We choose to exactly undo the behavior of the AWheeledVehiclePawn constructor and set this bool
    // directly, rather than calling GetMesh()->SetSimulatePhysics(true), to avoid any other possible
    // side effects.
    GetMesh()->BodyInstance.bSimulatePhysics = true;

    // UCameraComponent
    FVector camera_location;
    FRotator camera_rotation;
    float field_of_view;
    float aspect_ratio;
    if (Config::isInitialized()) {
        camera_location = {
            Config::get<double>("VEHICLE.VEHICLE_PAWN.CAMERA_COMPONENT.LOCATION_X"),
            Config::get<double>("VEHICLE.VEHICLE_PAWN.CAMERA_COMPONENT.LOCATION_Y"),
            Config::get<double>("VEHICLE.VEHICLE_PAWN.CAMERA_COMPONENT.LOCATION_Z")};
        camera_rotation = {
            Config::get<double>("VEHICLE.VEHICLE_PAWN.CAMERA_COMPONENT.ROTATION_PITCH"),
            Config::get<double>("VEHICLE.VEHICLE_PAWN.CAMERA_COMPONENT.ROTATION_YAW"),
            Config::get<double>("VEHICLE.VEHICLE_PAWN.CAMERA_COMPONENT.ROTATION_ROLL")};
        field_of_view = Config::get<float>("VEHICLE.VEHICLE_PAWN.CAMERA_COMPONENT.FOV");
        aspect_ratio = Config::get<float>("VEHICLE.VEHICLE_PAWN.CAMERA_COMPONENT.ASPECT_RATIO");
    } else {
        // OpenBot defaults, see python/spear/config/default_config.vehicle.yaml
        camera_location = {9.0, 5.0, 10.0};
        camera_rotation = FRotator::ZeroRotator;
        field_of_view = 70.0f;
        aspect_ratio = 1.333333f;
    }

    CameraComponent = Unreal::createComponentInsideOwnerConstructor<UCameraComponent>(this, GetMesh(), "camera");
    SP_ASSERT(CameraComponent);
    CameraComponent->SetRelativeLocationAndRotation(camera_location, camera_rotation);
    CameraComponent->bUsePawnControlRotation = false;
    CameraComponent->FieldOfView = field_of_view;
    CameraComponent->AspectRatio = aspect_ratio;

    // UBoxComponent
    FVector imu_location;
    FRotator imu_rotation;
    if (Config::isInitialized()) {
        imu_location = {
            Config::get<double>("VEHICLE.VEHICLE_PAWN.IMU_COMPONENT.LOCATION_X"),
            Config::get<double>("VEHICLE.VEHICLE_PAWN.IMU_COMPONENT.LOCATION_Y"),
            Config::get<double>("VEHICLE.VEHICLE_PAWN.IMU_COMPONENT.LOCATION_Z")};
        imu_rotation = {
            Config::get<double>("VEHICLE.VEHICLE_PAWN.IMU_COMPONENT.ROTATION_PITCH"),
            Config::get<double>("VEHICLE.VEHICLE_PAWN.IMU_COMPONENT.ROTATION_YAW"),
            Config::get<double>("VEHICLE.VEHICLE_PAWN.IMU_COMPONENT.ROTATION_ROLL")};
    } else {
        // OpenBot defaults, see python/spear/config/default_config.vehicle.yaml
        imu_location = {8.0, 0.0, 10.0};
        imu_rotation = FRotator::ZeroRotator;
    }

    ImuComponent = Unreal::createComponentInsideOwnerConstructor<UBoxComponent>(this, GetMesh(), "imu");
    SP_ASSERT(ImuComponent);
    ImuComponent->SetRelativeLocationAndRotation(imu_location, imu_rotation);

    // UVehicleMovementComponent
    MovementComponent = Cast<UVehicleMovementComponent>(GetVehicleMovementComponent()); // no RTTI available, so use Cast instead of dynamic_cast
    SP_ASSERT(MovementComponent);
}

AVehiclePawn::~AVehiclePawn()
{
    SP_LOG_CURRENT_FUNCTION();
}

void AVehiclePawn::BeginPlay()
{
    AWheeledVehiclePawn::BeginPlay();

    // Get player input actions from the config system if it is initialized, otherwise use hard-coded keyboard actions, which
    // can be useful for debugging.
    std::map<std::string, std::map<std::string, std::vector<double>>> user_input_actions;
    if (Config::isInitialized()) {
        user_input_actions = Config::get<std::map<std::string, std::map<std::string, std::vector<double>>>>("VEHICLE.VEHICLE_PAWN.USER_INPUT_ACTIONS");
    } else {
        user_input_actions = DEFAULT_USER_INPUT_ACTIONS;
    }

    UserInputComponent->subscribeToUserInputs(Std::keys(user_input_actions));
    UserInputComponent->setHandleUserInputFunc([this, user_input_actions](const std::string& key, float axis_value) -> void {
        applyAction(user_input_actions.at(key));
    });
}

void AVehiclePawn::setActionComponents(const std::vector<std::string>& action_components)
{
    action_components_ = action_components;
}

void AVehiclePawn::setObservationComponents(const std::vector<std::string>& observation_components)
{
    observation_components_ = observation_components;
}

std::map<std::string, ArrayDesc> AVehiclePawn::getActionSpace() const
{
    std::map<std::string, ArrayDesc> action_space;

    if (Std::contains(action_components_, "set_brake_torques")) {
        ArrayDesc array_desc;
        array_desc.low_ = std::numeric_limits<double>::lowest();
        array_desc.high_ = std::numeric_limits<double>::max();
        array_desc.shape_ = {MovementComponent->WheelSetups.Num()};
        array_desc.datatype_ = DataType::Float64;
        Std::insert(action_space, "set_brake_torques", std::move(array_desc));
    }

    if (Std::contains(action_components_, "set_drive_torques")) {
        ArrayDesc array_desc;
        array_desc.low_ = std::numeric_limits<double>::lowest();
        array_desc.high_ = std::numeric_limits<double>::max();
        array_desc.shape_ = {MovementComponent->WheelSetups.Num()};
        array_desc.datatype_ = DataType::Float64;
        Std::insert(action_space, "set_drive_torques", std::move(array_desc));
    }

    return action_space;
}

std::map<std::string, ArrayDesc> AVehiclePawn::getObservationSpace() const
{
    std::map<std::string, ArrayDesc> observation_space;

    if (Std::contains(observation_components_, "location")) {
        ArrayDesc array_desc;
        array_desc.low_ = std::numeric_limits<double>::lowest();
        array_desc.high_ = std::numeric_limits<double>::max();
        array_desc.datatype_ = DataType::Float64;
        array_desc.shape_ = {3};
        Std::insert(observation_space, "location", std::move(array_desc)); // x, y, z in [cm] of the agent relative to the world frame
    }

    if (Std::contains(observation_components_, "rotation")) {
        ArrayDesc array_desc;
        array_desc.low_ = std::numeric_limits<double>::lowest();
        array_desc.high_ = std::numeric_limits<double>::max();
        array_desc.datatype_ = DataType::Float64;
        array_desc.shape_ = {3};
        Std::insert(observation_space, "rotation", std::move(array_desc)); // pitch, yaw, roll in [deg] of the agent relative to the world frame
    }

    if (Std::contains(observation_components_, "wheel_rotation_speeds")) {
        ArrayDesc array_desc;
        array_desc.low_ = std::numeric_limits<double>::lowest();
        array_desc.high_ = std::numeric_limits<double>::max();
        array_desc.datatype_ = DataType::Float64;
        array_desc.shape_ = {MovementComponent->WheelSetups.Num()};
        Std::insert(observation_space, "wheel_rotation_speeds", std::move(array_desc)); // [rad/s]
    }

    return observation_space;
}

void AVehiclePawn::applyAction(const std::map<std::string, std::vector<uint8_t>>& action)
{
    // Apply torques in [N.m] to the vehicle wheels. The torques are persistent, i.e., if you call SetDriveTorque,
    // the torque will remain in effect until you call SetDriveTorque again.

    if (Std::contains(action_components_, "set_brake_torques")) {
        SP_ASSERT(Std::containsKey(action, "set_brake_torques"));
        std::span<const double> action_component_data = Std::reinterpretAsSpanOf<const double>(action.at("set_brake_torques"));
        for (int i = 0; i < MovementComponent->WheelSetups.Num(); i++) {
            MovementComponent->SetBrakeTorque(Std::at(action_component_data, i), i);
        }
    }

    if (Std::contains(action_components_, "set_drive_torques")) {
        SP_ASSERT(Std::containsKey(action, "set_drive_torques"));
        std::span<const double> action_component_data = Std::reinterpretAsSpanOf<const double>(action.at("set_drive_torques"));
        for (int i = 0; i < MovementComponent->WheelSetups.Num(); i++) {
            MovementComponent->SetDriveTorque(Std::at(action_component_data, i), i);
        }
    }
}

std::map<std::string, std::vector<uint8_t>> AVehiclePawn::getObservation() const
{
    std::map<std::string, std::vector<uint8_t>> observation;

    if (Std::contains(observation_components_, "location")) {
        FVector location = GetActorLocation();
        Std::insert(observation, "location", Std::reinterpretAsVector<uint8_t, double>({location.X, location.Y, location.Z}));
    }

    if (Std::contains(observation_components_, "rotation")) {
        FRotator rotation = GetActorRotation();
        Std::insert(observation, "rotation", Std::reinterpretAsVector<uint8_t, double>({rotation.Pitch, rotation.Yaw, rotation.Roll}));
    }

    if (Std::contains(observation_components_, "wheel_rotation_speeds")) {
        std::vector<double> wheel_rotation_speeds;        
        for (int i = 0; i < MovementComponent->WheelSetups.Num(); i++) {
            wheel_rotation_speeds.push_back(MovementComponent->VehicleSimulationPT->PVehicle->GetWheel(i).GetAngularVelocity());
        }
        Std::insert(observation, "wheel_rotation_speeds", Std::reinterpretAsVectorOf<uint8_t>(wheel_rotation_speeds));
    }

    return observation;
}

void AVehiclePawn::applyAction(const std::map<std::string, std::vector<double>>& action)
{
    if (Std::containsKey(action, "set_brake_torques")) {
        std::vector<double> action_component_data = action.at("set_brake_torques");
        for (int i = 0; i < MovementComponent->WheelSetups.Num(); i++) {
            MovementComponent->SetBrakeTorque(action_component_data.at(i), i);
        }
    }

    if (Std::containsKey(action, "set_drive_torques")) {
        std::vector<double> action_component_data = action.at("set_drive_torques");
        for (int i = 0; i < MovementComponent->WheelSetups.Num(); i++) {
            MovementComponent->SetDriveTorque(action_component_data.at(i), i);
        }
    }
}
