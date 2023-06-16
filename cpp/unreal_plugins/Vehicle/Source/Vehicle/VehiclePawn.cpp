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

AVehiclePawn::AVehiclePawn(const FObjectInitializer& object_initializer) : AWheeledVehiclePawn(object_initializer.SetDefaultSubobjectClass<UVehicleMovementComponent>(TEXT("VehicleMovementComp")))
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
    GetMesh()->BodyInstance.bSimulatePhysics = true;

    // Setup camera
    FVector camera_location(
        Config::get<float>("VEHICLE.VEHICLE_PAWN.CAMERA_COMPONENT.POSITION_X"),
        Config::get<float>("VEHICLE.VEHICLE_PAWN.CAMERA_COMPONENT.POSITION_Y"),
        Config::get<float>("VEHICLE.VEHICLE_PAWN.CAMERA_COMPONENT.POSITION_Z"));

    FRotator camera_orientation(
        Config::get<float>("VEHICLE.VEHICLE_PAWN.CAMERA_COMPONENT.PITCH"),
        Config::get<float>("VEHICLE.VEHICLE_PAWN.CAMERA_COMPONENT.YAW"),
        Config::get<float>("VEHICLE.VEHICLE_PAWN.CAMERA_COMPONENT.ROLL"));

    camera_component_ = CreateDefaultSubobject<UCameraComponent>(TEXT("camera_component"));
    SP_ASSERT(camera_component_);

    camera_component_->SetRelativeLocationAndRotation(camera_location, camera_orientation);
    camera_component_->SetupAttachment(GetMesh());
    camera_component_->bUsePawnControlRotation = false;
    camera_component_->FieldOfView = Config::get<float>("VEHICLE.VEHICLE_PAWN.CAMERA_COMPONENT.FOV");

    // Setup IMU sensor
    FVector imu_location(Config::get<float>("VEHICLE.VEHICLE_PAWN.IMU_COMPONENT.POSITION_X"),
        Config::get<float>("VEHICLE.VEHICLE_PAWN.IMU_COMPONENT.POSITION_Y"),
        Config::get<float>("VEHICLE.VEHICLE_PAWN.IMU_COMPONENT.POSITION_Z"));

    FRotator imu_orientation(
        Config::get<float>("VEHICLE.VEHICLE_PAWN.IMU_COMPONENT.PITCH"),
        Config::get<float>("VEHICLE.VEHICLE_PAWN.IMU_COMPONENT.YAW"),
        Config::get<float>("VEHICLE.VEHICLE_PAWN.IMU_COMPONENT.ROLL"));

    imu_component_ = CreateDefaultSubobject<UBoxComponent>(TEXT("imu_component"));
    ASSERT(imu_component_);

    imu_component_->SetRelativeLocationAndRotation(imu_location, imu_orientation);
    imu_component_->SetupAttachment(GetMesh());
}

AVehiclePawn::~AVehiclePawn()
{
    SP_LOG_CURRENT_FUNCTION();
}
