//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "WheeledVehicle/VehiclePawn.h"

#include <vector>

#include <Animation/AnimInstance.h>
#include <Camera/CameraComponent.h>
#include <Components/InputComponent.h>
#include <Components/SkeletalMeshComponent.h>
#include <Engine/CollisionProfile.h>
#include <UObject/ConstructorHelpers.h>

#include "CoreUtils/Assert.h"
#include "CoreUtils/Config.h"
#include "CoreUtils/Log.h"
#include "CoreUtils/Unreal.h"
#include "WheeledVehicle/VehicleMovementComponent.h"

AVehiclePawn::AVehiclePawn(const FObjectInitializer& object_initializer) : APawn(object_initializer)
{
    SP_LOG_CURRENT_FUNCTION();

    if (!Config::s_initialized_) {
        return;
    }

    ConstructorHelpers::FObjectFinder<USkeletalMesh> skeletal_mesh(*Unreal::toFString(Config::get<std::string>("WHEELED_VEHICLE.VEHICLE_PAWN.SKELETAL_MESH")));
    SP_ASSERT(skeletal_mesh.Succeeded());

    ConstructorHelpers::FClassFinder<UAnimInstance> anim_instance(*Unreal::toFString(Config::get<std::string>("WHEELED_VEHICLE.VEHICLE_PAWN.ANIM_INSTANCE")));
    SP_ASSERT(anim_instance.Succeeded());

    skeletal_mesh_component_ = CreateDefaultSubobject<USkeletalMeshComponent>(TEXT("AVehiclePawn::skeletal_mesh_component_"));
    SP_ASSERT(skeletal_mesh_component_);
    skeletal_mesh_component_->SetSkeletalMesh(skeletal_mesh.Object);
    skeletal_mesh_component_->SetAnimClass(anim_instance.Class);
    skeletal_mesh_component_->SetCollisionProfileName(UCollisionProfile::Vehicle_ProfileName);
    skeletal_mesh_component_->BodyInstance.bSimulatePhysics = true;
    skeletal_mesh_component_->BodyInstance.bNotifyRigidBodyCollision = true;
    skeletal_mesh_component_->BodyInstance.bUseCCD = true;
    skeletal_mesh_component_->bBlendPhysics = true;
    skeletal_mesh_component_->SetGenerateOverlapEvents(true);
    skeletal_mesh_component_->SetCanEverAffectNavigation(false);

    RootComponent = skeletal_mesh_component_;

    // Setup vehicle movement
    vehicle_movement_component_ = CreateDefaultSubobject<UVehicleMovementComponent>(TEXT("AVehiclePawn::vehicle_movement_component_"));
    SP_ASSERT(vehicle_movement_component_);
    vehicle_movement_component_->SetIsReplicated(true); // Enable replication by default
    vehicle_movement_component_->UpdatedComponent = skeletal_mesh_component_;
    // this ensures that the body doesn't ever sleep. Need this to bypass a Chaos bug that doesn't take torque inputs to wheels into consideration for determining the sleep state of the body.
    vehicle_movement_component_->SleepThreshold = 0;

    // Setup camera
    FVector camera_location(
        Config::get<float>("WHEELED_VEHICLE.VEHICLE_PAWN.CAMERA_COMPONENT.POSITION_X"),
        Config::get<float>("WHEELED_VEHICLE.VEHICLE_PAWN.CAMERA_COMPONENT.POSITION_Y"),
        Config::get<float>("WHEELED_VEHICLE.VEHICLE_PAWN.CAMERA_COMPONENT.POSITION_Z"));

    FRotator camera_orientation(
        Config::get<float>("WHEELED_VEHICLE.VEHICLE_PAWN.CAMERA_COMPONENT.PITCH"),
        Config::get<float>("WHEELED_VEHICLE.VEHICLE_PAWN.CAMERA_COMPONENT.YAW"),
        Config::get<float>("WHEELED_VEHICLE.VEHICLE_PAWN.CAMERA_COMPONENT.ROLL"));

    camera_component_ = CreateDefaultSubobject<UCameraComponent>(TEXT("AVehiclePawn::camera_component_"));
    SP_ASSERT(camera_component_);

    camera_component_->SetRelativeLocationAndRotation(camera_location, camera_orientation);
    camera_component_->SetupAttachment(skeletal_mesh_component_);
    camera_component_->bUsePawnControlRotation = false;
    camera_component_->FieldOfView = Config::get<float>("WHEELED_VEHICLE.VEHICLE_PAWN.CAMERA_COMPONENT.FOV");
}

AVehiclePawn::~AVehiclePawn()
{
    SP_LOG_CURRENT_FUNCTION();
}

void AVehiclePawn::SetupPlayerInputComponent(UInputComponent* input_component)
{
    SP_ASSERT(input_component);
    APawn::SetupPlayerInputComponent(input_component);

    input_component->BindAxis("MoveForward", this, &AVehiclePawn::moveForward);
    input_component->BindAxis("MoveRight", this, &AVehiclePawn::moveRight);
}

void AVehiclePawn::moveForward(float forward)
{
    float torque = forward * 10.0;
    vehicle_movement_component_->SetDriveTorque(torque, 0);
    vehicle_movement_component_->SetDriveTorque(torque, 1);
    vehicle_movement_component_->SetDriveTorque(torque, 2);
    vehicle_movement_component_->SetDriveTorque(torque, 3);
}

void AVehiclePawn::moveRight(float right)
{
    float torque = right * 10.0;
    vehicle_movement_component_->SetDriveTorque(torque, 0);
    vehicle_movement_component_->SetDriveTorque(torque, 1);
    vehicle_movement_component_->SetDriveTorque(torque, 2);
    vehicle_movement_component_->SetDriveTorque(torque, 3);
}

void AVehiclePawn::setWheelTorques(const std::vector<double>& wheel_torques)
{
    SP_ASSERT(wheel_torques.size() == 4);

    vehicle_movement_component_->SetDriveTorque(wheel_torques.at(0), 0);
    vehicle_movement_component_->SetDriveTorque(wheel_torques.at(1), 1);
    vehicle_movement_component_->SetDriveTorque(wheel_torques.at(2), 2);
    vehicle_movement_component_->SetDriveTorque(wheel_torques.at(3), 3);
}