//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "Vehicle/VehiclePawn.h"

#include <string>
#include <vector>

#include <Animation/AnimInstance.h>
#include <Camera/CameraComponent.h>
#include <Components/BoxComponent.h>
#include <Components/SkeletalMeshComponent.h>
#include <Engine/CollisionProfile.h>
#include <UObject/ConstructorHelpers.h>

#include "CoreUtils/Assert.h"
#include "CoreUtils/Config.h"
#include "CoreUtils/Log.h"
#include "CoreUtils/Unreal.h"
#include "Vehicle/VehicleMovementComponent.h"

// Calling the AWheeledVehiclePawn constructor in this way is necessary to override the UChaosWheeledVehicleMovementComponent
// class used by AWheeledVehiclePawn. See the following link for details:
//     https://docs.unrealengine.com/5.2/en-US/API/Plugins/ChaosVehicles/AWheeledVehiclePawn
AVehiclePawn::AVehiclePawn(const FObjectInitializer& object_initializer) :
    AWheeledVehiclePawn(object_initializer.SetDefaultSubobjectClass<UVehicleMovementComponent>(AWheeledVehiclePawn::VehicleMovementComponentName))
{
    SP_LOG_CURRENT_FUNCTION();

    if (!Config::s_initialized_) {
        return;
    }

    ConstructorHelpers::FObjectFinder<USkeletalMesh> skeletal_mesh(*Unreal::toFString(Config::get<std::string>("VEHICLE.VEHICLE_PAWN.SKELETAL_MESH")));
    SP_ASSERT(skeletal_mesh.Succeeded());

    ConstructorHelpers::FClassFinder<UAnimInstance> anim_instance(*Unreal::toFString(Config::get<std::string>("VEHICLE.VEHICLE_PAWN.ANIM_INSTANCE")));
    SP_ASSERT(anim_instance.Succeeded());

    GetMesh()->SetSkeletalMesh(skeletal_mesh.Object);
    GetMesh()->SetAnimClass(anim_instance.Class);

    // The AWheeledVehiclePawn constructor sets this parameter to false, but we want it set to true.
    // We choose to exactly undo the behavior of the AWheeledVehiclePawn constructor and set this bool
    // directly, rather than calling GetMesh()->SetSimulatePhysics(true), to avoid any other possible
    // side effects.
    GetMesh()->BodyInstance.bSimulatePhysics = true;

    // Setup camera
    camera_component_ = CreateDefaultSubobject<UCameraComponent>(Unreal::toFName("camera_component"));
    SP_ASSERT(camera_component_);

    FVector camera_location(
        Config::get<double>("VEHICLE.VEHICLE_PAWN.CAMERA_COMPONENT.LOCATION_X"),
        Config::get<double>("VEHICLE.VEHICLE_PAWN.CAMERA_COMPONENT.LOCATION_Y"),
        Config::get<double>("VEHICLE.VEHICLE_PAWN.CAMERA_COMPONENT.LOCATION_Z"));

    FRotator camera_rotation(
        Config::get<double>("VEHICLE.VEHICLE_PAWN.CAMERA_COMPONENT.ROTATION_PITCH"),
        Config::get<double>("VEHICLE.VEHICLE_PAWN.CAMERA_COMPONENT.ROTATION_YAW"),
        Config::get<double>("VEHICLE.VEHICLE_PAWN.CAMERA_COMPONENT.ROTATION_ROLL"));

    camera_component_->SetRelativeLocationAndRotation(camera_location, camera_rotation);
    camera_component_->SetupAttachment(GetMesh());
    camera_component_->bUsePawnControlRotation = false;
    camera_component_->FieldOfView = Config::get<float>("VEHICLE.VEHICLE_PAWN.CAMERA_COMPONENT.FOV");
    camera_component_->AspectRatio = Config::get<float>("VEHICLE.VEHICLE_PAWN.CAMERA_COMPONENT.ASPECT_RATIO");
    // no need to call RegisterComponent() in the constructor

    // Setup IMU sensor
    imu_component_ = CreateDefaultSubobject<UBoxComponent>(Unreal::toFName("imu_component"));
    SP_ASSERT(imu_component_);

    FVector imu_location(
        Config::get<float>("VEHICLE.VEHICLE_PAWN.IMU_COMPONENT.LOCATION_X"),
        Config::get<float>("VEHICLE.VEHICLE_PAWN.IMU_COMPONENT.LOCATION_Y"),
        Config::get<float>("VEHICLE.VEHICLE_PAWN.IMU_COMPONENT.LOCATION_Z"));

    FRotator imu_rotation(
        Config::get<float>("VEHICLE.VEHICLE_PAWN.IMU_COMPONENT.ROTATION_PITCH"),
        Config::get<float>("VEHICLE.VEHICLE_PAWN.IMU_COMPONENT.ROTATION_YAW"),
        Config::get<float>("VEHICLE.VEHICLE_PAWN.IMU_COMPONENT.ROTATION_ROLL"));

    imu_component_->SetRelativeLocationAndRotation(imu_location, imu_rotation);
    imu_component_->SetupAttachment(GetMesh());
    // no need to call RegisterComponent() in the constructor
}

AVehiclePawn::~AVehiclePawn()
{
    SP_LOG_CURRENT_FUNCTION();

    // Pawns don't need to be cleaned up explicitly.
}
