//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "OpenBot/OpenBotPawn.h"

#include <iostream>
#include <conio.h>

#include <Eigen/Dense>

#include <Animation/AnimInstance.h>
#include <Camera/CameraComponent.h>
#include <Components/InputComponent.h>
#include <Components/SkeletalMeshComponent.h>
#include <Engine/CollisionProfile.h>
#include <UObject/ConstructorHelpers.h>

#include "CoreUtils/Config.h"
#include "CoreUtils/Unreal.h"
#include "OpenBot/OpenBotMovementComponent.h"

AOpenBotPawn::AOpenBotPawn(const FObjectInitializer& object_initializer) : APawn(object_initializer)
{
    std::cout << "[SPEAR | OpenBotPawn.cpp] AOpenBotPawn::AOpenBotPawn" << std::endl;

    if (!Config::s_initialized_) {
        return;
    }

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
    vehicle_movement_component_ = CreateDefaultSubobject<UOpenBotMovementComponent>(TEXT("AOpenBotPawn::vehicle_movement_component_"));
    ASSERT(vehicle_movement_component_);
    vehicle_movement_component_->SetIsReplicated(true); // Enable replication by default
    vehicle_movement_component_->UpdatedComponent = skeletal_mesh_component_;
    // this ensures that the body doesn't ever sleep. Need this to bypass a Chaos bug that doesn't take torque inputs to wheels into consideration for determining the sleep state of the body.
    vehicle_movement_component_->SleepThreshold = 0;

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

    // Initialize duty cycle
    duty_cycle_.setZero();
}

AOpenBotPawn::~AOpenBotPawn()
{
    std::cout << "[SPEAR | OpenBotPawn.cpp] AOpenBotPawn::~AOpenBotPawn" << std::endl;
}

void AOpenBotPawn::SetupPlayerInputComponent(UInputComponent* input_component)
{
    ASSERT(input_component);
    APawn::SetupPlayerInputComponent(input_component);

    input_component->BindAxis("MoveForward", this, &AOpenBotPawn::moveForward);
    input_component->BindAxis("MoveRight", this, &AOpenBotPawn::moveRight);
}

void AOpenBotPawn::BeginPlay()
{
    APawn::BeginPlay();
}

void AOpenBotPawn::Tick(float delta_time)
{
    APawn::Tick(delta_time);

    setDriveTorquesFromDutyCycle();
}

void AOpenBotPawn::moveForward(float forward)
{
    duty_cycle_(0) += forward; // in [%]
    duty_cycle_(1) += forward; // in [%]
    duty_cycle_(2) += forward; // in [%]
    duty_cycle_(3) += forward; // in [%]
    duty_cycle_ = duty_cycle_.cwiseMin(1.f).cwiseMax(-1.f);
}

void AOpenBotPawn::moveRight(float right)
{
    duty_cycle_(0) += right; // in [%]
    duty_cycle_(1) -= right; // in [%]
    duty_cycle_(2) += right; // in [%]
    duty_cycle_(3) -= right; // in [%]
    duty_cycle_ = duty_cycle_.cwiseMin(1.f).cwiseMax(-1.f);
}

void AOpenBotPawn::setDriveTorquesFromDutyCycle()
{    
    vehicle_movement_component_->SetSleeping(false);

    //UE_LOG(LogTemp, Warning, TEXT("OpenBotPawn.cpp::setDriveTorquesFromDutyCycle(), duty_cycle_(0) = %f"), duty_cycle_(0));
    //UE_LOG(LogTemp, Warning, TEXT("OpenBotPawn.cpp::setDriveTorquesFromDutyCycle(), duty_cycle_(1) = %f"), duty_cycle_(1));
    //UE_LOG(LogTemp, Warning, TEXT("OpenBotPawn.cpp::setDriveTorquesFromDutyCycle(), duty_cycle_(2) = %f"), duty_cycle_(2));
    //UE_LOG(LogTemp, Warning, TEXT("OpenBotPawn.cpp::setDriveTorquesFromDutyCycle(), duty_cycle_(3) = %f"), duty_cycle_(3));

    // Motor torque: 1200 gf.cm (gram force centimeter) == 0.1177 N.m
    // Gear ratio: 50
    // Max wheel torque: 5.88399 N.m
    // https://www.conrad.de/de/p/joy-it-com-motor01-getriebemotor-gelb-schwarz-passend-fuer-einplatinen-computer-arduino-banana-pi-cubieboard-raspbe-1573543.html

    auto motor_velocity_constant = Config::get<float>("OPENBOT.OPENBOT_PAWN.MOTOR_VELOCITY_CONSTANT"); // Motor torque constant in [N.m/A]
    auto gear_ratio = Config::get<float>("OPENBOT.OPENBOT_PAWN.GEAR_RATIO");              // Gear ratio of the OpenBot motors
    auto motor_torque_constant = 1.f / motor_velocity_constant;                                      // Motor torque constant in [rad/s/V]

    auto battery_voltage = Config::get<float>("OPENBOT.OPENBOT_PAWN.BATTERY_VOLTAGE");         // Voltage of the battery powering the OpenBot [V]
    auto control_dead_zone = Config::get<float>("OPENBOT.OPENBOT_PAWN.CONTROL_DEAD_ZONE");     // Absolute duty cycle (in the ACTION_SCALE range)
                                                                                               // below which a command does not produces any torque
                                                                                               // on the vehicle
    auto motor_torque_max = Config::get<float>("OPENBOT.OPENBOT_PAWN.MOTOR_TORQUE_MAX");        // Motor maximal torque [N.m]
    auto electrical_resistance = Config::get<float>("OPENBOT.OPENBOT_PAWN.ELECTRICAL_RESISTANCE");   // Motor winding electrical resistance [Ohms]
    auto electrical_inductance = Config::get<float>("OPENBOT.OPENBOT_PAWN.ELECTRICAL_INDUCTANCE");   // Motor winding electrical inductance [Henry]

    // Speed multiplier defined in the OpenBot to map a [-1,1] action to a suitable command to
    // be processed by the low-level microcontroller. For more details, feel free to check the
    // "speedMultiplier" command in the OpenBot code.
    // https://github.com/isl-org/OpenBot/blob/master/android/app/src/main/java/org/openbot/vehicle/Vehicle.java#L375

    auto action_scale = Config::get<float>("OPENBOT.OPENBOT_PAWN.ACTION_SCALE");

    // Acquire the ground truth motor and wheel velocity for motor counter-electromotive force
    // computation purposes (or alternatively friction computation purposes).

    // The ground truth velocity of the robot wheels in [RPM]
    Eigen::Vector4f wheel_rotation_speeds = vehicle_movement_component_->getWheelRotationSpeeds();

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
    Eigen::Vector4f wheel_torque = 0.001 * gear_ratio * motor_torque;

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
    //     Engine/Plugins/Experimental/ChaosVehiclesPlugin/Source/ChaosVehicles/Private/ChaosWheeledVehicleMovementComponent.h
    //
    // This file also contains a bunch of useful functions such as SetBrakeTorque or SetSteerAngle.
    // Please take a look if you want to modify the way the simulated vehicle is being controlled.

    //UE_LOG(LogTemp, Warning, TEXT("AOpenBotPawn.cpp::setDriveTorquesFromDutyCycle(), setting wheel_torque(0) = %f"), wheel_torque(0));
    //UE_LOG(LogTemp, Warning, TEXT("AOpenBotPawn.cpp::setDriveTorquesFromDutyCycle(), setting wheel_torque(1) = %f"), wheel_torque(1));
    //UE_LOG(LogTemp, Warning, TEXT("AOpenBotPawn.cpp::setDriveTorquesFromDutyCycle(), setting wheel_torque(2) = %f"), wheel_torque(2));
    //UE_LOG(LogTemp, Warning, TEXT("AOpenBotPawn.cpp::setDriveTorquesFromDutyCycle(), setting wheel_torque(3) = %f"), wheel_torque(3));

    vehicle_movement_component_->SetDriveTorque(wheel_torque(0), 0);
    vehicle_movement_component_->SetDriveTorque(wheel_torque(1), 1);
    vehicle_movement_component_->SetDriveTorque(wheel_torque(2), 2);
    vehicle_movement_component_->SetDriveTorque(wheel_torque(3), 3);
}

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
