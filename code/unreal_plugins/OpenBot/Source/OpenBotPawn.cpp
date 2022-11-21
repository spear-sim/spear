#include "OpenBotPawn.h"

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

#include "CompilerWarningUtils.h"
#include "Config.h"
#include "OpenBotWheel.h"

BEGIN_IGNORE_COMPILER_WARNINGS
AOpenBotPawn::AOpenBotPawn(const FObjectInitializer& object_initializer): APawn(object_initializer)
{
    // Setup skeletal mesh
    ConstructorHelpers::FObjectFinder<USkeletalMesh> openbot_mesh_finder(UTF8_TO_TCHAR(Config::getValue<std::string>({"OPENBOT", "OPENBOT_PAWN", "SKELETAL_MESH"}).c_str()));
    ASSERT(openbot_mesh_finder.Succeeded());

    ConstructorHelpers::FClassFinder<UAnimInstance> openbot_animation_finder(UTF8_TO_TCHAR(Config::getValue<std::string>({"OPENBOT", "OPENBOT_PAWN", "ANIM_INSTANCE"}).c_str()));
    ASSERT(openbot_animation_finder.Succeeded());

    skeletal_mesh_component_ = CreateDefaultSubobject<USkeletalMeshComponent>(TEXT("SkeletalMeshComponent"));
    ASSERT(skeletal_mesh_component_); 
    skeletal_mesh_component_->SetSkeletalMesh(openbot_mesh_finder.Object);
    skeletal_mesh_component_->SetAnimClass(openbot_animation_finder.Class);
    skeletal_mesh_component_->SetCollisionProfileName(UCollisionProfile::Vehicle_ProfileName);
    skeletal_mesh_component_->BodyInstance.bSimulatePhysics = true;
    skeletal_mesh_component_->BodyInstance.bNotifyRigidBodyCollision = true;
    skeletal_mesh_component_->BodyInstance.bUseCCD = true;
    skeletal_mesh_component_->bBlendPhysics = true;
    skeletal_mesh_component_->SetGenerateOverlapEvents(true);
    skeletal_mesh_component_->SetCanEverAffectNavigation(false);

    RootComponent = skeletal_mesh_component_;

    // Setup vehicle movement
    vehicle_movement_component_ = CreateDefaultSubobject<USimpleWheeledVehicleMovementComponent>(TEXT("SimpleWheeledVehicleMovementComponent"));
    ASSERT(vehicle_movement_component_); 
    vehicle_movement_component_->SetIsReplicated(true); // Enable replication by default
    vehicle_movement_component_->UpdatedComponent = skeletal_mesh_component_;

    UClass* wheel_class = UOpenBotWheel::StaticClass();

    vehicle_movement_component_->WheelSetups.SetNum(4);

    vehicle_movement_component_->WheelSetups[0].WheelClass = wheel_class;
    vehicle_movement_component_->WheelSetups[0].BoneName = FName("FL");
    vehicle_movement_component_->WheelSetups[0].AdditionalOffset = FVector(0.f, 0.f, 0.f); // offset the wheel from the bone's location

    vehicle_movement_component_->WheelSetups[1].WheelClass = wheel_class;
    vehicle_movement_component_->WheelSetups[1].BoneName = FName("FR");
    vehicle_movement_component_->WheelSetups[1].AdditionalOffset = FVector(0.f, 0.f, 0.f); // offset the wheel from the bone's location

    vehicle_movement_component_->WheelSetups[2].WheelClass = wheel_class;
    vehicle_movement_component_->WheelSetups[2].BoneName = FName("RL");
    vehicle_movement_component_->WheelSetups[2].AdditionalOffset = FVector(0.f, 0.f, 0.f); // offset the wheel from the bone's location

    vehicle_movement_component_->WheelSetups[3].WheelClass = wheel_class;
    vehicle_movement_component_->WheelSetups[3].BoneName = FName("RR");
    vehicle_movement_component_->WheelSetups[3].AdditionalOffset = FVector(0.f, 0.f, 0.f); // offset the wheel from the bone's location

    vehicle_movement_component_->Mass = Config::getValue<float>({"OPENBOT", "OPENBOT_PAWN", "VEHICLE_COMPONENT", "MASS"}); // Mass of the vehicle chassis
    vehicle_movement_component_->InertiaTensorScale = FVector{1.f, 1.f, 1.f};
    vehicle_movement_component_->DragCoefficient = Config::getValue<float>({"OPENBOT", "OPENBOT_PAWN", "VEHICLE_COMPONENT", "DRAG_COEFFICIENT"}); // DragCoefficient of the vehicle chassis
    vehicle_movement_component_->ChassisWidth    = Config::getValue<float>({"OPENBOT", "OPENBOT_PAWN", "VEHICLE_COMPONENT", "CHASSIS_WIDTH"});    // Chassis width used for drag force computation in [cm]
    vehicle_movement_component_->ChassisHeight   = Config::getValue<float>({"OPENBOT", "OPENBOT_PAWN", "VEHICLE_COMPONENT", "CHASSIS_HEIGHT"});   // Chassis height used for drag force computation in [cm]
    vehicle_movement_component_->MaxEngineRPM    = Config::getValue<float>({"OPENBOT", "OPENBOT_PAWN", "VEHICLE_COMPONENT", "MOTOR_MAX_RPM"});    // Max RPM for engine
    
    // Setup camera
    FVector camera_location(
        Config::getValue<float>({"OPENBOT", "OPENBOT_PAWN", "CAMERA_COMPONENT", "POSITION_X"}),
        Config::getValue<float>({"OPENBOT", "OPENBOT_PAWN", "CAMERA_COMPONENT", "POSITION_Y"}),
        Config::getValue<float>({"OPENBOT", "OPENBOT_PAWN", "CAMERA_COMPONENT", "POSITION_Z"}));

    FRotator camera_orientation(
        Config::getValue<float>({"OPENBOT", "OPENBOT_PAWN", "CAMERA_COMPONENT", "PITCH"}),
        Config::getValue<float>({"OPENBOT", "OPENBOT_PAWN", "CAMERA_COMPONENT", "YAW"}),
        Config::getValue<float>({"OPENBOT", "OPENBOT_PAWN", "CAMERA_COMPONENT", "ROLL"}));

    camera_component_ = CreateDefaultSubobject<UCameraComponent>(TEXT("CameraComponent"));
    ASSERT(camera_component_); 

    camera_component_->SetRelativeLocationAndRotation(camera_location, camera_orientation);
    camera_component_->SetupAttachment(skeletal_mesh_component_);
    camera_component_->bUsePawnControlRotation = false;
    camera_component_->FieldOfView = Config::getValue<float>({"OPENBOT", "OPENBOT_PAWN", "CAMERA_COMPONENT", "FOV"});

    // Setup IMU sensor component
    FVector imu_location(
        Config::getValue<float>({"OPENBOT", "OPENBOT_PAWN", "IMU_COMPONENT", "POSITION_X"}), 
        Config::getValue<float>({"OPENBOT", "OPENBOT_PAWN", "IMU_COMPONENT", "POSITION_Y"}), 
        Config::getValue<float>({"OPENBOT", "OPENBOT_PAWN", "IMU_COMPONENT", "POSITION_Z"}));
    
    FRotator imu_orientation(
        Config::getValue<float>({"OPENBOT", "OPENBOT_PAWN", "IMU_COMPONENT", "PITCH"}), 
        Config::getValue<float>({"OPENBOT", "OPENBOT_PAWN", "IMU_COMPONENT", "YAW"}), 
        Config::getValue<float>({"OPENBOT", "OPENBOT_PAWN", "IMU_COMPONENT", "ROLL"}));
    
    imu_component_ = CreateDefaultSubobject<UBoxComponent>(TEXT("ImuComponent")); 
    ASSERT(imu_component_ );
    
    imu_component_->SetRelativeLocationAndRotation(imu_location, imu_orientation); 
    imu_component_->SetupAttachment(skeletal_mesh_component_);

    // Setup Sonar sensor component
    FVector sonar_location(
        Config::getValue<float>({"OPENBOT", "OPENBOT_PAWN", "SONAR_COMPONENT", "POSITION_X"}), 
        Config::getValue<float>({"OPENBOT", "OPENBOT_PAWN", "SONAR_COMPONENT", "POSITION_Y"}), 
        Config::getValue<float>({"OPENBOT", "OPENBOT_PAWN", "SONAR_COMPONENT", "POSITION_Z"}));
        
    FRotator sonar_orientation(
        Config::getValue<float>({"OPENBOT", "OPENBOT_PAWN", "SONAR_COMPONENT", "PITCH"}), 
        Config::getValue<float>({"OPENBOT", "OPENBOT_PAWN", "SONAR_COMPONENT", "YAW"}), 
        Config::getValue<float>({"OPENBOT", "OPENBOT_PAWN", "SONAR_COMPONENT", "ROLL"}));
    
    sonar_component_ = CreateDefaultSubobject<UBoxComponent>(TEXT("SonarComponent")); 
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
    setDriveTorques(delta_time);
}

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

void AOpenBotPawn::setDutyCycleAndClamp(const Eigen::Vector4f& duty_cycle)
{
    // duty_cycle describe the percentage of input voltage to be applied to each motors by the H-bridge controller: 1 = 100%, -1 = reverse 100%

    // First make sure the duty cycle is not getting above 100%. This is done similarly on the real OpenBot:
    // https://github.com/isl-org/OpenBot/blob/master/android/app/src/main/java/org/openbot/vehicle/Control.java

    duty_cycle_ = duty_cycle.cwiseMin(1.f).cwiseMax(-1.f);
}

Eigen::Vector4f AOpenBotPawn::getDutyCycle() const
{
    return duty_cycle_;
}

BEGIN_IGNORE_COMPILER_WARNINGS
void AOpenBotPawn::resetPhysicsState()
{
    PxRigidDynamic* rigid_body_dynamic_actor = vehicle_movement_component_->PVehicle->getRigidDynamicActor();
    ASSERT(rigid_body_dynamic_actor);

    // We want to reset the physics state of OpenBot, so we are inlining the below code from
    // Engine/Source/ThirdParty/PhysX3/PhysX_3.4/Source/PhysXVehicle/src/PxVehicleDrive.cpp::setToRestState(), and
    // Engine/Source/ThirdParty/PhysX3/PhysX_3.4/Source/PhysXVehicle/src/PxVehicleWheels.cpp::setToRestState(), because these functions are protected.
    if (!(rigid_body_dynamic_actor->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC)) {
        rigid_body_dynamic_actor->setLinearVelocity(PxVec3(0, 0, 0));
        rigid_body_dynamic_actor->setAngularVelocity(PxVec3(0, 0, 0));
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
void AOpenBotPawn::setDriveTorques(float delta_time)
{
    // Openbot motor torque: 1200 gf.cm (gram force centimeter) = 0.1177 N.m
    // https://www.conrad.de/de/p/joy-it-com-motor01-getriebemotor-gelb-schwarz-passend-fuer-einplatinen-computer-arduino-banana-pi-cubieboard-raspbe-1573543.html)
    // Gear ratio: 50 => max. wheel torque = 5.88399 N.m

    auto motor_velocity_constant = Config::getValue<float>({"OPENBOT", "OPENBOT_PAWN", "MOTOR_VELOCITY_CONSTANT"}); // Motor torque constant in [N.m/A]
    auto gear_ratio              = Config::getValue<float>({"OPENBOT", "OPENBOT_PAWN", "GEAR_RATIO"});              // Gear ratio of the OpenBot motors
    auto motor_torque_constant   = 1.f / motor_velocity_constant;                                                   // Motor torque constant in [rad/s/V]

    auto battery_voltage         = Config::getValue<float>({"OPENBOT", "OPENBOT_PAWN", "BATTERY_VOLTAGE"});         // Voltage of the battery powering the OpenBot [V]
    auto control_dead_zone       = Config::getValue<float>({"OPENBOT", "OPENBOT_PAWN", "CONTROL_DEAD_ZONE"});       // Absolute duty cycle (in the ACTION_SCALE range) below which a command does not produces any torque on the vehicle
    auto motor_torque_max        = Config::getValue<float>({"OPENBOT", "OPENBOT_PAWN", "MOTOR_TORQUE_MAX"});        // Motor maximal torque [N.m]
    auto electrical_resistance   = Config::getValue<float>({"OPENBOT", "OPENBOT_PAWN", "ELECTRICAL_RESISTANCE"});   // Motor winding electrical resistance [Ohms]
    auto electrical_inductance   = Config::getValue<float>({"OPENBOT", "OPENBOT_PAWN", "ELECTRICAL_INDUCTANCE"});   // Motor winding electrical inductance [Henry]

    // Speed multiplier defined in the OpenBot to map a [-1,1] action to a suitable command to
    // be processed by the low-level microcontroller. For more details, feel free to check the
    // "speedMultiplier" command in the OpenBot code:
    // https://github.com/isl-org/OpenBot/blob/master/android/app/src/main/java/org/openbot/vehicle/Vehicle.java#L375
    
    auto action_scale = Config::getValue<float>({"OPENBOT", "OPENBOT_PAWN", "ACTION_SCALE"});

    // Acquire the ground truth motor and wheel velocity for motor counter-electromotive force computation purposes
    // (or alternatively friction computation purposes)

    // The ground truth velocity of the robot wheels in [RPM]
    Eigen::Vector4f wheel_rotation_speed;
    wheel_rotation_speed(0) = vehicle_movement_component_->PVehicle->mWheelsDynData.getWheelRotationSpeed(0); // Expressed in [RPM]
    wheel_rotation_speed(1) = vehicle_movement_component_->PVehicle->mWheelsDynData.getWheelRotationSpeed(1); // Expressed in [RPM]
    wheel_rotation_speed(2) = vehicle_movement_component_->PVehicle->mWheelsDynData.getWheelRotationSpeed(2); // Expressed in [RPM]
    wheel_rotation_speed(3) = vehicle_movement_component_->PVehicle->mWheelsDynData.getWheelRotationSpeed(3); // Expressed in [RPM]

    // The ground truth velocity of the motors in [rad/s]
    Eigen::Vector4f motor_velocity = gear_ratio * rpmToRadSec(wheel_rotation_speed); // Expressed in [rad/s]

    // Compute the counter electromotive force using the motor torque constant
    Eigen::Vector4f counter_electromotive_force = motor_torque_constant * motor_velocity; // Expressed in [V]

    // The electrical current allowed to circulate in the motor is the result of the
    // difference between the applied voltage and the counter electromotive force
    Eigen::Vector4f motor_winding_current = ((battery_voltage * duty_cycle_) - counter_electromotive_force) / electrical_resistance; // Expressed in [A]

    // motor_winding_current = motor_winding_current *
    // (1-(electrical_resistance/electrical_inductance)*delta_time) +
    // ((batteryVoltage_*duty_cycle_)-counter_electromotive_force)*delta_time/electrical_inductance;
    // If delta_time is "small enouth" (which is definitely not the case here)

    // The torque is then obtained using the torque coefficient of the motor in [N.m]
    Eigen::Vector4f motor_torque = motor_torque_constant * motor_winding_current;

    // Motor torque is saturated to match the motor limits
    motor_torque = motor_torque.cwiseMin(motor_torque_max).cwiseMax(-motor_torque_max);

    // The torque applied to the robot wheels is finally computed accounting for the gear ratio
    Eigen::Vector4f wheel_torque = gear_ratio * motor_torque;

    // Control dead zone at near-zero velocity. Note: this is a simplified but reliable way to deal with
    // the friction behavior observed on the real vehicle in the low-velocities/low-duty-cycle dommain.
    for (int i = 0; i < duty_cycle_.size(); i++) {
        // set torque zeror if the motor is "nearly" stopped
        if (std::abs(motor_velocity(i)) < 1e-5 and std::abs(duty_cycle_(i)) <= control_dead_zone / action_scale) {
            wheel_torque(i) = 0.f;
        }
    }

    // Apply the drive torque in [N.m] to the vehicle wheels. Note that the "SetDriveTorque" command
    // can be found in the code of the Unreal Engine at the following location:
    // Engine/Plugins/Runtime/PhysXVehicles/Source/PhysXVehicles/Public/SimpleWheeledVehicleMovementComponent.h
    //
    // This file also contains a bunch of useful functions such as "SetBrakeTorque" or "SetSteerAngle".
    // Please take a look if you want to modify the way the simulated vehicle is being controlled.
    vehicle_movement_component_->SetDriveTorque(wheel_torque(0), 0); // Torque applied to the wheel, expressed in [N.m]. The applied driveTorque persists until the next call to SetDriveTorque
    vehicle_movement_component_->SetDriveTorque(wheel_torque(1), 1); // Torque applied to the wheel, expressed in [N.m]. The applied driveTorque persists until the next call to SetDriveTorque
    vehicle_movement_component_->SetDriveTorque(wheel_torque(2), 2); // Torque applied to the wheel, expressed in [N.m]. The applied driveTorque persists until the next call to SetDriveTorque
    vehicle_movement_component_->SetDriveTorque(wheel_torque(3), 3); // Torque applied to the wheel, expressed in [N.m]. The applied driveTorque persists until the next call to SetDriveTorque
}
END_IGNORE_COMPILER_WARNINGS

BEGIN_IGNORE_COMPILER_WARNINGS
void AOpenBotPawn::activateBrakes()
{
    vehicle_movement_component_->SetDriveTorque(0.f, 0); // Torque applied to the wheel, expressed in [N.m]. The applied driveTorque persists until the next call to setDriveTorque.
    vehicle_movement_component_->SetDriveTorque(0.f, 1); // Torque applied to the wheel, expressed in [N.m]. The applied driveTorque persists until the next call to setDriveTorque.
    vehicle_movement_component_->SetDriveTorque(0.f, 2); // Torque applied to the wheel, expressed in [N.m]. The applied driveTorque persists until the next call to setDriveTorque.
    vehicle_movement_component_->SetDriveTorque(0.f, 3); // Torque applied to the wheel, expressed in [N.m]. The applied driveTorque persists until the next call to setDriveTorque.

    vehicle_movement_component_->SetBrakeTorque(1000.f, 0); // Torque applied to the wheel, expressed in [N.m]. The applied driveTorque persists until the next call to setDriveTorque.
    vehicle_movement_component_->SetBrakeTorque(1000.f, 1); // Torque applied to the wheel, expressed in [N.m]. The applied driveTorque persists until the next call to setDriveTorque.
    vehicle_movement_component_->SetBrakeTorque(1000.f, 2); // Torque applied to the wheel, expressed in [N.m]. The applied driveTorque persists until the next call to setDriveTorque.
    vehicle_movement_component_->SetBrakeTorque(1000.f, 3); // Torque applied to the wheel, expressed in [N.m]. The applied driveTorque persists until the next call to setDriveTorque.
}
END_IGNORE_COMPILER_WARNINGS

BEGIN_IGNORE_COMPILER_WARNINGS
void AOpenBotPawn::deactivateBrakes()
{
    vehicle_movement_component_->SetBrakeTorque(0.f, 0); // Torque applied to the wheel, expressed in [N.m]. The applied driveTorque persists until the next call to setDriveTorque.
    vehicle_movement_component_->SetBrakeTorque(0.f, 1); // Torque applied to the wheel, expressed in [N.m]. The applied driveTorque persists until the next call to setDriveTorque.
    vehicle_movement_component_->SetBrakeTorque(0.f, 2); // Torque applied to the wheel, expressed in [N.m]. The applied driveTorque persists until the next call to setDriveTorque.
    vehicle_movement_component_->SetBrakeTorque(0.f, 3); // Torque applied to the wheel, expressed in [N.m]. The applied driveTorque persists until the next call to setDriveTorque.
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
