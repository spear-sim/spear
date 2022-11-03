#include "OpenBotPawn.h"

#include <iostream>

#include <Camera/CameraComponent.h>
#include <Engine/CollisionProfile.h>
#include <PhysicsPublic.h>
#include <PhysXIncludes.h>
#include <PhysXPublic.h>
#include <PhysXVehicleManager.h>
#include <SimpleWheeledVehicleMovementComponent.h>

#include "Config.h"
#include "IgnoreCompilerWarnings.h"
#include "OpenBotWheel.h"

AOpenBotPawn::AOpenBotPawn(const FObjectInitializer& object_initializer): APawn(object_initializer)
{
    // To create components, you can use CreateDefaultSubobject<Type>("InternalName").
    skeletal_mesh_component_ = CreateDefaultSubobject<USkeletalMeshComponent>(TEXT("VehicleMesh"));

    // Setup skeletal mesh
    std::string mesh_name = Config::getValue<std::string>({"OPENBOT", "OPENBOT_PAWN", "MESH"});
    ConstructorHelpers::FObjectFinder<USkeletalMesh> openbot_mesh_finder(UTF8_TO_TCHAR(mesh_name.c_str()));
    ASSERT(openbot_mesh_finder.Succeeded());
    skeletal_mesh_component_->SetSkeletalMesh(openbot_mesh_finder.Object);

    // Setup animation
    std::string animation_name = Config::getValue<std::string>({"OPENBOT", "OPENBOT_PAWN", "ANIMATION"});
    ConstructorHelpers::FClassFinder<UAnimInstance> openbot_animation_finder(UTF8_TO_TCHAR(animation_name.c_str()));
    ASSERT(openbot_animation_finder.Succeeded());
    skeletal_mesh_component_->SetAnimClass(openbot_animation_finder.Class);

    skeletal_mesh_component_->SetCollisionProfileName(UCollisionProfile::Vehicle_ProfileName);
    skeletal_mesh_component_->BodyInstance.bSimulatePhysics = true;
    skeletal_mesh_component_->BodyInstance.bNotifyRigidBodyCollision = true;
    skeletal_mesh_component_->BodyInstance.bUseCCD = true;
    skeletal_mesh_component_->bBlendPhysics = true;
    skeletal_mesh_component_->SetGenerateOverlapEvents(true);
    skeletal_mesh_component_->SetCanEverAffectNavigation(false);

    RootComponent = skeletal_mesh_component_;

BEGIN_IGNORE_COMPILER_WARNINGS
    vehicle_movement_component_ = CreateDefaultSubobject<USimpleWheeledVehicleMovementComponent>(TEXT("SimpleWheeledVehicleMovement"));
END_IGNORE_COMPILER_WARNINGS

    vehicle_movement_component_->SetIsReplicated(true); // Enable replication by default
    vehicle_movement_component_->UpdatedComponent = skeletal_mesh_component_;

    // Setup wheels:
    vehicle_movement_component_->WheelSetups.SetNum(4);

    // TODO dynamic tire?
    UClass* wheel_class = UOpenBotWheel::StaticClass();

    // Wheels to create:
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

    duty_cycle_.setZero();

    // Vehicle dynamics:
    vehicle_movement_component_->DragCoefficient = Config::getValue<float>({"OPENBOT", "VEHICLE_COMPONENT", "DRAG_COEFFICIENT"}); // DragCoefficient of the vehicle chassis.
    vehicle_movement_component_->ChassisWidth = Config::getValue<float>({"OPENBOT", "VEHICLE_COMPONENT", "CHASSIS_WIDTH"});       // Chassis width used for drag force computation in [cm]
    vehicle_movement_component_->ChassisHeight = Config::getValue<float>({"OPENBOT", "VEHICLE_COMPONENT", "CHASSIS_HEIGHT"});     // Chassis height used for drag force computation in [cm]
    vehicle_movement_component_->MaxEngineRPM = Config::getValue<float>({"OPENBOT", "VEHICLE_COMPONENT", "MOTOR_MAX_RPM"});       // Max RPM for engine

    // Create camera component
    FVector camera_pose(Config::getValue<float>({"OPENBOT", "CAMERA_COMPONENT", "POSITION_X"}), Config::getValue<float>({"OPENBOT", "CAMERA_COMPONENT", "POSITION_Y"}), Config::getValue<float>({"OPENBOT", "CAMERA_COMPONENT", "POSITION_Z"}));
    FRotator camera_orientation(Config::getValue<float>({"OPENBOT", "CAMERA_COMPONENT", "PITCH"}), Config::getValue<float>({"OPENBOT", "CAMERA_COMPONENT", "YAW"}), Config::getValue<float>({"OPENBOT", "CAMERA_COMPONENT", "ROLL"}));
    camera_component_ = CreateDefaultSubobject<UCameraComponent>(TEXT("OpenBotSmartPhoneCamera"));
    camera_component_->SetRelativeLocationAndRotation(camera_pose, camera_orientation);
    camera_component_->SetupAttachment(skeletal_mesh_component_);
    camera_component_->bUsePawnControlRotation = false;
    camera_component_->FieldOfView = Config::getValue<float>({"OPENBOT", "CAMERA_COMPONENT", "FOV"});
}

void AOpenBotPawn::SetupPlayerInputComponent(class UInputComponent* input_component)
{
    ASSERT(input_component);
    Super::SetupPlayerInputComponent(input_component);

    input_component->BindAxis("MoveForward", this, &AOpenBotPawn::moveForward);
    input_component->BindAxis("MoveRight", this, &AOpenBotPawn::moveRight);
}

// Called every simulator update
void AOpenBotPawn::Tick(float delta_time)
{
    Super::Tick(delta_time);
    setDriveTorques(delta_time);
}

// This command is meant to be bound to keyboard input. It will be executed at
// each press or unpress event.
void AOpenBotPawn::moveForward(float forward)
{
    // forward describes the percentage of input voltage to be applied to the
    // motor by the H-bridge controller: 1.0 = 100%, -1.0 = reverse 100%.

    duty_cycle_(0) += forward; // in [%]
    duty_cycle_(1) += forward; // in [%]
    duty_cycle_(2) += forward; // in [%]
    duty_cycle_(3) += forward; // in [%]
    duty_cycle_ = clamp(duty_cycle_, -Eigen::Vector4f::Ones(), Eigen::Vector4f::Ones());
}

// This command is meant to be bound to keyboard input. It will be executed at
// each press or unpress event.
void AOpenBotPawn::moveRight(float right)
{
    // right describes the percentage of input voltage to be applied to the
    // motor by the H-bridge controller: 1.0 = 100%, -1.0 = reverse 100%.

    duty_cycle_(0) += right; // in [%]
    duty_cycle_(1) -= right; // in [%]
    duty_cycle_(2) += right; // in [%]
    duty_cycle_(3) -= right; // in [%]
    duty_cycle_ = clamp(duty_cycle_, -Eigen::Vector4f::Ones(), Eigen::Vector4f::Ones());
}

// This command is meant to be used by the python client interface.
void AOpenBotPawn::setDutyCycleAndClamp(Eigen::Vector4f duty_cycle)
{
    // duty_cycle describe the percentage of input voltage to be applied to each motors by the H-bridge controller: 1 = 100%, -1 = reverse 100%.

    // First make sure the duty cycle is not getting above 100%. This is done similarly on the real OpenBot:
    // (c.f. https://github.com/isl-org/OpenBot/blob/master/android/app/src/main/java/org/openbot/vehicle/Control.java)
    duty_cycle_ = clamp(duty_cycle, -Eigen::Vector4f::Ones(), Eigen::Vector4f::Ones());
}

// Provides feedback on the action executed by the robot. This action can either
// be defined through the python client or by keyboard/game controller input.
Eigen::Vector4f AOpenBotPawn::getDutyCycle()
{
    return duty_cycle_;
}

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
    // PVehicleDrive is not intiliazed, and so PVehicleDrive->mDriveDynData.setToRestState() is commented out. We want to know if this changes at some point.
    ASSERT(!vehicle_movement_component_->PVehicleDrive);
    // vehicle_movement_component->PVehicleDrive->mDriveDynData.setToRestState(); // throws seg fault
}

void AOpenBotPawn::setDriveTorques(float delta_time)
{
    // Openbot motor torque: 1200 gf.cm (gram force centimeter) = 0.1177 N.m
    // https://www.conrad.de/de/p/joy-it-com-motor01-getriebemotor-gelb-schwarz-passend-fuer-einplatinen-computer-arduino-banana-pi-cubieboard-raspbe-1573543.html)
    // Gear ratio: 50 => max. wheel torque = 5.88399 N.m

    float gear_ratio = Config::getValue<float>({"OPENBOT", "OPENBOT_PAWN", "GEAR_RATIO"});                           // Gear ratio of the OpenBot motors.
    float motor_velocity_constant = Config::getValue<float>({"OPENBOT", "OPENBOT_PAWN", "MOTOR_VELOCITY_CONSTANT"}); // Motor torque constant in [N.m/A]
    float motor_torque_constant = 1 / motor_velocity_constant;                                       // Motor torque constant in [rad/s/V]
    float battery_voltage = Config::getValue<float>({"OPENBOT", "OPENBOT_PAWN", "BATTERY_VOLTAGE"});                 // Expressed in [V]

    float control_dead_zone = Config::getValue<float>({"OPENBOT", "OPENBOT_PAWN", "CONTROL_DEAD_ZONE"});         // Below this command threshold, the torque is set to zero if the motor velocity is "small enough"
    float motor_torque_max = Config::getValue<float>({"OPENBOT", "OPENBOT_PAWN", "MOTOR_TORQUE_MAX"});           // Maximum ammount of torque an OpenBot motor can generate [N.m].
    float electrical_resistance = Config::getValue<float>({"OPENBOT", "OPENBOT_PAWN", "ELECTRICAL_RESISTANCE"}); // Electrical resistance of the DC motor windings in [Ohms]
    float electrical_inductance = Config::getValue<float>({"OPENBOT", "OPENBOT_PAWN", "ELECTRICAL_INDUCTANCE"}); // Electrical inductance of the DC motor windings in [Henry]
    // Speed multiplier defined in the OpenBot to map a [-1,1] action to a suitable command to
    // be processed by the low-level microcontroller. For more details, feel free
    // to check the "speedMultiplier" command in the OpenBot code:
    // https://github.com/isl-org/OpenBot/blob/d4362e688435155f6c20dfdb756e55556fc12cc8/android/app/src/main/java/org/openbot/vehicle/Vehicle.java#L375
    float action_scale = Config::getValue<float>({"OPENBOT", "OPENBOT_PAWN", "ACTION_SCALE"});

    // The ground truth velocity of the robot wheels in [RPM]
    Eigen::Vector4f wheel_rotation_speed;
    // Acquire the ground truth motor and wheel velocity for motor counter-electromotive force computation purposes
    // (or alternatively friction computation purposes):
    wheel_rotation_speed(0) = vehicle_movement_component_->PVehicle->mWheelsDynData.getWheelRotationSpeed(0); // Expressed in [RPM]
    wheel_rotation_speed(1) = vehicle_movement_component_->PVehicle->mWheelsDynData.getWheelRotationSpeed(1); // Expressed in [RPM]
    wheel_rotation_speed(2) = vehicle_movement_component_->PVehicle->mWheelsDynData.getWheelRotationSpeed(2); // Expressed in [RPM]
    wheel_rotation_speed(3) = vehicle_movement_component_->PVehicle->mWheelsDynData.getWheelRotationSpeed(3); // Expressed in [RPM]

    // The ground truth velocity of the motors in [rad/s]
    Eigen::Vector4f motor_velocity = gear_ratio * rpmToRadSec(wheel_rotation_speed); // Expressed in [rad/s]

    // Compute the counter electromotive force using the motor torque constant:
    Eigen::Vector4f counter_electromotive_force = motor_torque_constant * motor_velocity; // Expressed in [V]

    // The electrical current allowed to circulate in the motor is the result of the
    // difference between the applied voltage and the counter electromotive force:
    Eigen::Vector4f motor_winding_current = ((battery_voltage * duty_cycle_) - counter_electromotive_force) / electrical_resistance; // Expressed in [A]

    // motor_winding_current = motor_winding_current *
    // (1-(electrical_resistance/electrical_inductance)*delta_time) +
    // ((batteryVoltage_*duty_cycle_)-counter_electromotive_force)*delta_time/electrical_inductance;
    // If delta_time is "small enouth" (which is definitely not the case here)

    // The torque is then obtained using the torque coefficient of the motor in [N.m]:
    Eigen::Vector4f motor_torque = motor_torque_constant * motor_winding_current;

    // Motor torque is saturated to match the motor limits:
    motor_torque = clamp(motor_torque, Eigen::Vector4f::Constant(-motor_torque_max), Eigen::Vector4f::Constant(motor_torque_max));

    // The torque applied to the robot wheels is finally computed accounting for the gear ratio:
    Eigen::Vector4f wheel_torque = gear_ratio * motor_torque;

    // Control dead zone at near-zero velocity:
    // Note: this is a simplified but reliable way to deal with the friction
    // behavior observed on the real vehicle in the low-velocities/low-duty-cycle dommain.
    for (int i = 0; i < duty_cycle_.size(); i++) {
        // set torque zeror if the motor is "nearly" stopped
        if (std::abs(motor_velocity(i)) < 1e-5 and std::abs(duty_cycle_(i)) <= control_dead_zone / action_scale) {
            wheel_torque(i) = 0.f;
        }
    }

    // Apply the drive torque in [N.m] to the vehicle wheels.
    // Note that the "SetDriveTorque" command can be found in the code of the unreal engine at the following location:
    // UnrealEngine-4.26.2-release/Engine/Plugins/Runtime/PhysXVehicles/Source/PhysXVehicles/Public/SimpleWheeledVehicleMovementComponent.h.
    // This file also contains a bunch of useful functions such as "SetBrakeTorque" or "SetSteerAngle". Please take a look if you want to
    // modify the way the simulated vehicle is being controlled.
    vehicle_movement_component_->SetDriveTorque(wheel_torque(0), 0); // Torque applied to the wheel, expressed in [N.m]. The applied driveTorque persists until the next call to setDriveTorque.
    vehicle_movement_component_->SetDriveTorque(wheel_torque(1), 1); // Torque applied to the wheel, expressed in [N.m]. The applied driveTorque persists until the next call to setDriveTorque.
    vehicle_movement_component_->SetDriveTorque(wheel_torque(2), 2); // Torque applied to the wheel, expressed in [N.m]. The applied driveTorque persists until the next call to setDriveTorque.
    vehicle_movement_component_->SetDriveTorque(wheel_torque(3), 3); // Torque applied to the wheel, expressed in [N.m]. The applied driveTorque persists until the next call to setDriveTorque.

    // Reset duty cycle value:
    duty_cycle_.setZero();
}

// clamp a vector between two values.
Eigen::Vector4f AOpenBotPawn::clamp(Eigen::Vector4f v, Eigen::Vector4f v_min, Eigen::Vector4f v_max)
{
    Eigen::Vector4f v_clamped;
    v_clamped = v;
    for (unsigned int i = 0; i < v.size(); i++) {
        if (v(i) > v_max(i)) {
            v_clamped(i) = v_max(i);
        }
        if (v(i) < v_min(i)) {
            v_clamped(i) = v_min(i);
        }
    }
    return v_clamped;
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
