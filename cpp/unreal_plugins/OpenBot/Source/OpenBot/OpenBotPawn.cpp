//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "OpenBot/OpenBotPawn.h"

#include <iostream>

#include <Animation/AnimInstance.h>
#include <Camera/CameraComponent.h>
#include <Components/BoxComponent.h>
#include <Components/InputComponent.h>
#include <Components/SkeletalMeshComponent.h>
#include <Engine/CollisionProfile.h>
#include <Math/Vector.h>
#include <PhysicsPublic.h>
#include <PhysXIncludes.h>
#include <PhysXPublic.h>
#include <PhysXVehicleManager.h>
#include <SimpleWheeledVehicleMovementComponent.h>
#include <UObject/ConstructorHelpers.h>

#include "CoreUtils/IgnoreCompilerWarnings.h"
#include "CoreUtils/Config.h"
#include "CoreUtils/Std.h"
#include "CoreUtils/Unreal.h"
#include "OpenBot/OpenBotWheel.h"

BEGIN_IGNORE_COMPILER_WARNINGS
AOpenBotPawn::AOpenBotPawn(const FObjectInitializer& object_initializer): APawn(object_initializer)
{
    std::cout << "[SPEAR | OpenBotPawn.cpp] AOpenBotPawn::AOpenBotPawn" << std::endl;

    ConstructorHelpers::FObjectFinder<USkeletalMesh> skeletal_mesh(*Unreal::toFString(Config::get<std::string>("OPENBOT.OPENBOT_PAWN.SKELETAL_MESH")));
    ASSERT(skeletal_mesh.Succeeded());

    ConstructorHelpers::FClassFinder<UAnimInstance> anim_instance(*Unreal::toFString(Config::get<std::string>("OPENBOT.OPENBOT_PAWN.ANIM_INSTANCE")));
    ASSERT(anim_instance.Succeeded());

    skeletal_mesh_component_ = CreateDefaultSubobject<USkeletalMeshComponent>(TEXT("AOpenBotPawn::skeletal_mesh_component_"));
    ASSERT(skeletal_mesh_component_); 
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
    vehicle_movement_component_ = CreateDefaultSubobject<USimpleWheeledVehicleMovementComponent>(TEXT("AOpenBotPawn::vehicle_movement_component_"));
    ASSERT(vehicle_movement_component_); 
    vehicle_movement_component_->SetIsReplicated(true); // Enable replication by default
    vehicle_movement_component_->UpdatedComponent = skeletal_mesh_component_;

    UClass* wheel_class = UOpenBotWheel::StaticClass();

    vehicle_movement_component_->WheelSetups.SetNum(4);

    vehicle_movement_component_->WheelSetups[0].WheelClass = wheel_class;
    vehicle_movement_component_->WheelSetups[0].BoneName = FName("FL");
    vehicle_movement_component_->WheelSetups[0].AdditionalOffset = FVector::ZeroVector; // Offset the wheel from the bone's location

    vehicle_movement_component_->WheelSetups[1].WheelClass = wheel_class;
    vehicle_movement_component_->WheelSetups[1].BoneName = FName("FR");
    vehicle_movement_component_->WheelSetups[1].AdditionalOffset = FVector::ZeroVector; // Offset the wheel from the bone's location

    vehicle_movement_component_->WheelSetups[2].WheelClass = wheel_class;
    vehicle_movement_component_->WheelSetups[2].BoneName = FName("RL");
    vehicle_movement_component_->WheelSetups[2].AdditionalOffset = FVector::ZeroVector; // Offset the wheel from the bone's location

    vehicle_movement_component_->WheelSetups[3].WheelClass = wheel_class;
    vehicle_movement_component_->WheelSetups[3].BoneName = FName("RR");
    vehicle_movement_component_->WheelSetups[3].AdditionalOffset = FVector::ZeroVector; // Offset the wheel from the bone's location

    vehicle_movement_component_->Mass               = Config::get<float>("OPENBOT.OPENBOT_PAWN.VEHICLE_COMPONENT.MASS");
    vehicle_movement_component_->InertiaTensorScale = FVector::OneVector;
    vehicle_movement_component_->DragCoefficient    = Config::get<float>("OPENBOT.OPENBOT_PAWN.VEHICLE_COMPONENT.DRAG_COEFFICIENT");
    vehicle_movement_component_->ChassisWidth       = Config::get<float>("OPENBOT.OPENBOT_PAWN.VEHICLE_COMPONENT.CHASSIS_WIDTH");
    vehicle_movement_component_->ChassisHeight      = Config::get<float>("OPENBOT.OPENBOT_PAWN.VEHICLE_COMPONENT.CHASSIS_HEIGHT");
    vehicle_movement_component_->MaxEngineRPM       = Config::get<float>("OPENBOT.OPENBOT_PAWN.VEHICLE_COMPONENT.MOTOR_MAX_RPM");

    // Setup camera
    FVector camera_location(
        Config::get<float>("OPENBOT.OPENBOT_PAWN.CAMERA_COMPONENT.POSITION_X"),
        Config::get<float>("OPENBOT.OPENBOT_PAWN.CAMERA_COMPONENT.POSITION_Y"),
        Config::get<float>("OPENBOT.OPENBOT_PAWN.CAMERA_COMPONENT.POSITION_Z"));

    FRotator camera_orientation(
        Config::get<float>("OPENBOT.OPENBOT_PAWN.CAMERA_COMPONENT.PITCH"),
        Config::get<float>("OPENBOT.OPENBOT_PAWN.CAMERA_COMPONENT.YAW"),
        Config::get<float>("OPENBOT.OPENBOT_PAWN.CAMERA_COMPONENT.ROLL"));

    camera_component_ = CreateDefaultSubobject<UCameraComponent>(TEXT("AOpenBotPawn::camera_component_"));
    ASSERT(camera_component_); 

    camera_component_->SetRelativeLocationAndRotation(camera_location, camera_orientation);
    camera_component_->SetupAttachment(skeletal_mesh_component_);
    camera_component_->bUsePawnControlRotation = false;
    camera_component_->FieldOfView = Config::get<float>("OPENBOT.OPENBOT_PAWN.CAMERA_COMPONENT.FOV");

    // Setup IMU sensor
    FVector imu_location(
        Config::get<float>("OPENBOT.OPENBOT_PAWN.IMU_COMPONENT.POSITION_X"), 
        Config::get<float>("OPENBOT.OPENBOT_PAWN.IMU_COMPONENT.POSITION_Y"), 
        Config::get<float>("OPENBOT.OPENBOT_PAWN.IMU_COMPONENT.POSITION_Z"));
    
    FRotator imu_orientation(
        Config::get<float>("OPENBOT.OPENBOT_PAWN.IMU_COMPONENT.PITCH"), 
        Config::get<float>("OPENBOT.OPENBOT_PAWN.IMU_COMPONENT.YAW"), 
        Config::get<float>("OPENBOT.OPENBOT_PAWN.IMU_COMPONENT.ROLL"));
    
    imu_component_ = CreateDefaultSubobject<UBoxComponent>(TEXT("AOpenBotPawn::imu_component_")); 
    ASSERT(imu_component_ );
    
    imu_component_->SetRelativeLocationAndRotation(imu_location, imu_orientation); 
    imu_component_->SetupAttachment(skeletal_mesh_component_);

    // Setup Sonar sensor
    FVector sonar_location(
        Config::get<float>("OPENBOT.OPENBOT_PAWN.SONAR_COMPONENT.POSITION_X"), 
        Config::get<float>("OPENBOT.OPENBOT_PAWN.SONAR_COMPONENT.POSITION_Y"), 
        Config::get<float>("OPENBOT.OPENBOT_PAWN.SONAR_COMPONENT.POSITION_Z"));
        
    FRotator sonar_orientation(
        Config::get<float>("OPENBOT.OPENBOT_PAWN.SONAR_COMPONENT.PITCH"), 
        Config::get<float>("OPENBOT.OPENBOT_PAWN.SONAR_COMPONENT.YAW"), 
        Config::get<float>("OPENBOT.OPENBOT_PAWN.SONAR_COMPONENT.ROLL"));
    
    sonar_component_ = CreateDefaultSubobject<UBoxComponent>(TEXT("AOpenBotPawn::sonar_component_")); 
    ASSERT(sonar_component_); 

    sonar_component_->SetRelativeLocationAndRotation(sonar_location, sonar_orientation); 
    sonar_component_->SetupAttachment(skeletal_mesh_component_);

    // Initialize duty cycle
    duty_cycle_.setZero();
}
END_IGNORE_COMPILER_WARNINGS

void AOpenBotPawn::SetupPlayerInputComponent(class UInputComponent* input_component)
{
    ASSERT(input_component);
    Super::SetupPlayerInputComponent(input_component);

    input_component->BindAxis("MoveForward", this, &AOpenBotPawn::moveForward);
    input_component->BindAxis("MoveRight", this, &AOpenBotPawn::moveRight);
}

void AOpenBotPawn::Tick(float delta_time)
{
    Super::Tick(delta_time);
    setDriveTorquesFromDutyCycle();
}

BEGIN_IGNORE_COMPILER_WARNINGS
void AOpenBotPawn::resetWheels()
{
    PxRigidDynamic* rigid_body_dynamic_actor = vehicle_movement_component_->PVehicle->getRigidDynamicActor();
    ASSERT(rigid_body_dynamic_actor);

    // We want to reset the physics state of OpenBot, so we are inlining the below code from
    // Engine/Source/ThirdParty/PhysX3/PhysX_3.4/Source/PhysXVehicle/src/PxVehicleDrive.cpp::setToRestState(), and
    // Engine/Source/ThirdParty/PhysX3/PhysX_3.4/Source/PhysXVehicle/src/PxVehicleWheels.cpp::setToRestState(), because these functions are protected.
    if (!(rigid_body_dynamic_actor->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC)) {
        rigid_body_dynamic_actor->setLinearVelocity(PxVec3(0.0f, 0.0f, 0.0f));
        rigid_body_dynamic_actor->setAngularVelocity(PxVec3(0.0f, 0.0f, 0.0f));
        rigid_body_dynamic_actor->clearForce(PxForceMode::eACCELERATION);
        rigid_body_dynamic_actor->clearForce(PxForceMode::eVELOCITY_CHANGE);
        rigid_body_dynamic_actor->clearTorque(PxForceMode::eACCELERATION);
        rigid_body_dynamic_actor->clearTorque(PxForceMode::eVELOCITY_CHANGE);
    }
    vehicle_movement_component_->PVehicle->mWheelsDynData.setToRestState();

    // In our experience, it is not possible to call vehicle_movement_component->PVehicleDrive->mDriveDynData.setToRestState()
    // because PVehicleDrive is NULL. So we do not attempt to call PVehicleDrive->mDriveDynData.setToRestState(), and we assert
    // that PVehicleDrive is NULL.
    // vehicle_movement_component->PVehicleDrive->mDriveDynData.setToRestState();
    ASSERT(!vehicle_movement_component_->PVehicleDrive);
}
END_IGNORE_COMPILER_WARNINGS

BEGIN_IGNORE_COMPILER_WARNINGS
void AOpenBotPawn::setBrakeTorques(const Eigen::Vector4f& brake_torques)
{
    // Torque applied to the brakes, expressed in [N.m]. The applied torque persists until the next call to SetBrakeTorque.
    vehicle_movement_component_->SetBrakeTorque(brake_torques(0), 0);
    vehicle_movement_component_->SetBrakeTorque(brake_torques(1), 1);
    vehicle_movement_component_->SetBrakeTorque(brake_torques(2), 2);
    vehicle_movement_component_->SetBrakeTorque(brake_torques(3), 3);
}
END_IGNORE_COMPILER_WARNINGS

Eigen::Vector4f AOpenBotPawn::getDutyCycle() const
{
    return duty_cycle_;
}

void AOpenBotPawn::setDutyCycle(const Eigen::Vector4f& duty_cycle)
{
    // duty_cycle describe the percentage of input voltage to be applied to each motors by the H-bridge controller: 1 = 100%, -1 = reverse 100%

    // First make sure the duty cycle is not getting above 100%. This is done similarly on the real OpenBot.
    // https://github.com/isl-org/OpenBot/blob/master/android/app/src/main/java/org/openbot/vehicle/Control.java

    duty_cycle_ = duty_cycle.cwiseMin(1.f).cwiseMax(-1.f);
}

BEGIN_IGNORE_COMPILER_WARNINGS
Eigen::Vector4f AOpenBotPawn::getWheelRotationSpeeds() const
{
    Eigen::Vector4f wheel_rotation_speeds;
    wheel_rotation_speeds(0) = vehicle_movement_component_->PVehicle->mWheelsDynData.getWheelRotationSpeed(0); // Expressed in [RPM]
    wheel_rotation_speeds(1) = vehicle_movement_component_->PVehicle->mWheelsDynData.getWheelRotationSpeed(1); // Expressed in [RPM]
    wheel_rotation_speeds(2) = vehicle_movement_component_->PVehicle->mWheelsDynData.getWheelRotationSpeed(2); // Expressed in [RPM]
    wheel_rotation_speeds(3) = vehicle_movement_component_->PVehicle->mWheelsDynData.getWheelRotationSpeed(3); // Expressed in [RPM]
    return rpmToRadSec(wheel_rotation_speeds); // Expressed in [rad/s]
}
END_IGNORE_COMPILER_WARNINGS

void AOpenBotPawn::moveForward(float forward)
{
    // forward describes the percentage of input voltage to be applied to the
    // motor by the H-bridge controller: 1.0 = 100%, -1.0 = reverse 100%

    duty_cycle_(0) += forward; // in [%]
    duty_cycle_(1) += forward; // in [%]
    duty_cycle_(2) += forward; // in [%]
    duty_cycle_(3) += forward; // in [%]
    duty_cycle_ = duty_cycle_.cwiseMin(1.f).cwiseMax(-1.f);
}

void AOpenBotPawn::moveRight(float right)
{
    // right describes the percentage of input voltage to be applied to the
    // motor by the H-bridge controller: 1.0 = 100%, -1.0 = reverse 100%

    duty_cycle_(0) += right; // in [%]
    duty_cycle_(1) -= right; // in [%]
    duty_cycle_(2) += right; // in [%]
    duty_cycle_(3) -= right; // in [%]
    duty_cycle_ = duty_cycle_.cwiseMin(1.f).cwiseMax(-1.f);
}

BEGIN_IGNORE_COMPILER_WARNINGS
void AOpenBotPawn::setDriveTorquesFromDutyCycle()
{
    // Motor torque: 1200 gf.cm (gram force centimeter) == 0.1177 N.m
    // Gear ratio: 50
    // Max wheel torque: 5.88399 N.m
    // https://www.conrad.de/de/p/joy-it-com-motor01-getriebemotor-gelb-schwarz-passend-fuer-einplatinen-computer-arduino-banana-pi-cubieboard-raspbe-1573543.html

    auto motor_velocity_constant = Config::get<float>("OPENBOT.OPENBOT_PAWN.MOTOR_VELOCITY_CONSTANT"); // Motor torque constant in [N.m/A]
    auto gear_ratio              = Config::get<float>("OPENBOT.OPENBOT_PAWN.GEAR_RATIO");              // Gear ratio of the OpenBot motors
    auto motor_torque_constant   = 1.f / motor_velocity_constant;                                      // Motor torque constant in [rad/s/V]

    auto battery_voltage         = Config::get<float>("OPENBOT.OPENBOT_PAWN.BATTERY_VOLTAGE");         // Voltage of the battery powering the OpenBot [V]
    auto control_dead_zone       = Config::get<float>("OPENBOT.OPENBOT_PAWN.CONTROL_DEAD_ZONE");       // Absolute duty cycle (in the ACTION_SCALE range)
                                                                                                       // below which a command does not produces any torque
                                                                                                       // on the vehicle
    auto motor_torque_max        = Config::get<float>("OPENBOT.OPENBOT_PAWN.MOTOR_TORQUE_MAX");        // Motor maximal torque [N.m]
    auto electrical_resistance   = Config::get<float>("OPENBOT.OPENBOT_PAWN.ELECTRICAL_RESISTANCE");   // Motor winding electrical resistance [Ohms]
    auto electrical_inductance   = Config::get<float>("OPENBOT.OPENBOT_PAWN.ELECTRICAL_INDUCTANCE");   // Motor winding electrical inductance [Henry]

    // Speed multiplier defined in the OpenBot to map a [-1,1] action to a suitable command to
    // be processed by the low-level microcontroller. For more details, feel free to check the
    // "speedMultiplier" command in the OpenBot code.
    // https://github.com/isl-org/OpenBot/blob/master/android/app/src/main/java/org/openbot/vehicle/Vehicle.java#L375
    
    auto action_scale = Config::get<float>("OPENBOT.OPENBOT_PAWN.ACTION_SCALE");

    // Acquire the ground truth motor and wheel velocity for motor counter-electromotive force
    // computation purposes (or alternatively friction computation purposes).

    // The ground truth velocity of the robot wheels in [RPM]
    Eigen::Vector4f wheel_rotation_speeds;
    wheel_rotation_speeds(0) = vehicle_movement_component_->PVehicle->mWheelsDynData.getWheelRotationSpeed(0); // Expressed in [RPM]
    wheel_rotation_speeds(1) = vehicle_movement_component_->PVehicle->mWheelsDynData.getWheelRotationSpeed(1); // Expressed in [RPM]
    wheel_rotation_speeds(2) = vehicle_movement_component_->PVehicle->mWheelsDynData.getWheelRotationSpeed(2); // Expressed in [RPM]
    wheel_rotation_speeds(3) = vehicle_movement_component_->PVehicle->mWheelsDynData.getWheelRotationSpeed(3); // Expressed in [RPM]

    // The ground truth rotation speed of the motors in [rad/s]
    Eigen::Vector4f motor_speed = gear_ratio * rpmToRadSec(wheel_rotation_speeds); // Expressed in [rad/s]

    // Compute the counter electromotive force using the motor torque constant
    Eigen::Vector4f counter_electromotive_force = motor_torque_constant * motor_speed; // Expressed in [V]

    // The electrical current allowed to circulate in the motor is the result of the
    // difference between the applied voltage and the counter electromotive force
    Eigen::Vector4f motor_winding_current = ((battery_voltage * duty_cycle_) - counter_electromotive_force) / electrical_resistance; // Expressed in [A]

    // The torque is then obtained using the torque coefficient of the motor in [N.m]
    Eigen::Vector4f motor_torque = motor_torque_constant * motor_winding_current;

    // Motor torque is saturated to match the motor limits
    motor_torque = motor_torque.cwiseMin(motor_torque_max).cwiseMax(-motor_torque_max);

    // The torque applied to the robot wheels is finally computed accounting for the gear ratio
    Eigen::Vector4f wheel_torque = gear_ratio * motor_torque;

    // Control dead zone at near-zero speed. This is a simplified but reliable way to deal with
    // the friction behavior observed on the real vehicle in the low-speed/low-duty-cycle regime.
    for (int i = 0; i < duty_cycle_.size(); i++) {
        // TODO: get value from the config system
        if (std::abs(motor_speed(i)) < 1e-5f && std::abs(duty_cycle_(i)) <= control_dead_zone / action_scale) {
            wheel_torque(i) = 0.0f;
        }
    }

    // Apply the drive torque in [N.m] to the vehicle wheels. The applied driveTorque persists until the
    // next call to SetDriveTorque. Note that the SetDriveTorque command can be found in the code of the
    // Unreal Engine at the following location:
    //     Engine/Plugins/Runtime/PhysXVehicles/Source/PhysXVehicles/Public/SimpleWheeledVehicleMovementComponent.h
    //
    // This file also contains a bunch of useful functions such as SetBrakeTorque or SetSteerAngle.
    // Please take a look if you want to modify the way the simulated vehicle is being controlled.
    
    vehicle_movement_component_->SetDriveTorque(wheel_torque(0), 0);
    vehicle_movement_component_->SetDriveTorque(wheel_torque(1), 1);
    vehicle_movement_component_->SetDriveTorque(wheel_torque(2), 2);
    vehicle_movement_component_->SetDriveTorque(wheel_torque(3), 3);
}
END_IGNORE_COMPILER_WARNINGS

// Rev per minute to rad/s
Eigen::VectorXf AOpenBotPawn::rpmToRadSec(Eigen::VectorXf rpm)
{
    return rpm * PI / 30.f;
}

// rad/s to rev per minute
Eigen::VectorXf AOpenBotPawn::radSecToRpm(Eigen::VectorXf omega)
{
    return omega * 30.f / PI;
}
