#include "OpenBotPawn.h"

PRAGMA_DISABLE_DEPRECATION_WARNINGS

#include <iostream>

#include <Camera/CameraComponent.h>
#include <TireConfig.h>
#include <VehicleWheel.h>

#include "OpenBotWheel.h"

FName AOpenBotPawn::VehicleMovementComponentName(TEXT("SimpleWheeledVehicleMovement"));
FName AOpenBotPawn::VehicleMeshComponentName(TEXT("VehicleMesh'"));

AOpenBotPawn::AOpenBotPawn(const FObjectInitializer& ObjectInitializer) : APawn(ObjectInitializer)
{
    // To create components, you can use CreateDefaultSubobject<Type>("InternalName").
    Mesh = CreateDefaultSubobject<USkeletalMeshComponent>(VehicleMeshComponentName);

    // Setup skeletal mesh
    static ConstructorHelpers::FObjectFinder<USkeletalMesh> CarMesh(TEXT("/Agents/OpenBot/OpenBot.OpenBot"));
    Mesh->SetSkeletalMesh(CarMesh.Object);

    // Setup animation
    static ConstructorHelpers::FClassFinder<UAnimInstance> finderAnim(TEXT("/Agents/OpenBot/OpenBot_Animation.OpenBot_Animation_C"));
    ASSERT(finderAnim.Succeeded());

    Mesh->SetAnimClass(finderAnim.Class);

    Mesh->SetCollisionProfileName(UCollisionProfile::Vehicle_ProfileName);
    Mesh->BodyInstance.bSimulatePhysics = true;
    Mesh->BodyInstance.bNotifyRigidBodyCollision = true;
    Mesh->BodyInstance.bUseCCD = true;
    Mesh->bBlendPhysics = true;
    Mesh->SetGenerateOverlapEvents(true);
    Mesh->SetCanEverAffectNavigation(false);
    // example for adding user-defined collision callback
    // Mesh->OnComponentHit.AddDynamic(this,&ASimpleVehiclePawn::OnComponentCollision);
    RootComponent = Mesh;
    // TODO read from config or fixed position?
    // Create camera component
    FVector CameraPos(5.0f, 5.0f, 5.0f);
    FRotator CameraOri(0.0f, 0.0f, 0.0f);

    Camera = CreateDefaultSubobject<UCameraComponent>(TEXT("OpenBotSmartPhone"));
    Camera->SetRelativeLocationAndRotation(CameraPos, CameraOri);
    Camera->SetupAttachment(Mesh);
    Camera->bUsePawnControlRotation = false;
    Camera->FieldOfView = 90.f;

    VehicleMovement = CreateDefaultSubobject<UWheeledVehicleMovementComponent, USimpleWheeledVehicleMovementComponent>(VehicleMovementComponentName);
    VehicleMovement->SetIsReplicated(true); // Enable replication by default
    VehicleMovement->UpdatedComponent = Mesh;

    // Setup wheels:
    VehicleMovement->WheelSetups.SetNum(4);

    // TODO dynamic tire?
    UClass* wheelClasss = UOpenBotWheel::StaticClass();

    // Wheels to create:
    VehicleMovement->WheelSetups[0].WheelClass = wheelClasss;
    VehicleMovement->WheelSetups[0].BoneName = FName("FL");
    VehicleMovement->WheelSetups[0].AdditionalOffset = FVector(0.f, 0.f, 0.f); // If BoneName is specified, offset the wheel from the bone's location.
                                                                                 // Otherwise this offsets the wheel from the vehicle's origin.

    VehicleMovement->WheelSetups[1].WheelClass = wheelClasss;
    VehicleMovement->WheelSetups[1].BoneName = FName("FR");
    VehicleMovement->WheelSetups[1].AdditionalOffset = FVector(0.f, 0.f, 0.f); // If BoneName is specified, offset the wheel from the bone's location.
                                                                                // Otherwise this offsets the wheel from the vehicle's origin.

    VehicleMovement->WheelSetups[2].WheelClass = wheelClasss;
    VehicleMovement->WheelSetups[2].BoneName = FName("RL");
    VehicleMovement->WheelSetups[2].AdditionalOffset = FVector(0.f, 0.f, 0.f); // If BoneName is specified, offset the wheel from the bone's location.
                                                                                 // Otherwise this offsets the wheel from the vehicle's origin.

    VehicleMovement->WheelSetups[3].WheelClass = wheelClasss;
    VehicleMovement->WheelSetups[3].BoneName = FName("RR");
    VehicleMovement->WheelSetups[3].AdditionalOffset = FVector(0.f, 0.f, 0.f); // If BoneName is specified, offset the wheel from the bone's location.
    // Otherwise this offsets the wheel from the vehicle's origin.

    wheelVelocity_.setZero();
    motorVelocity_.setZero();
    counterElectromotiveForce_.setZero();
    motorTorque_.setZero();
    wheelTorque_.setZero();
    dutyCycle_.setZero();
    motorWindingCurrent_.setZero();
    actionVec_.setZero();

    gearRatio_ = Config::getValue<float>({"ROBOT_SIM", "GEAR_RATIO"});
    motorVelocityConstant_ = Config::getValue<float>({"ROBOT_SIM", "MOTOR_VELOCITY_CONSTANT"});
    motorTorqueConstant_ = 1 / motorVelocityConstant_;
    controlDeadZone_ = Config::getValue<float>({"ROBOT_SIM", "CONTROL_DEAD_ZONE"});
    motorTorqueMax_ = Config::getValue<float>({"ROBOT_SIM", "MOTOR_TORQUE_MAX"});
    electricalResistance_ = Config::getValue<float>({"ROBOT_SIM", "ELECTRICAL_RESISTANCE"});
    electricalInductance_ = Config::getValue<float>({"ROBOT_SIM", "ELECTRICAL_INDUCTANCE"});
    actionScale_ = Config::getValue<float>({"ROBOT_SIM", "ACTION_SCALE"});
    batteryVoltage_ = Config::getValue<float>({"ROBOT_SIM", "BATTERY_VOLTAGE"});

    // Vehicle dynamics:
    VehicleMovement->DragCoefficient = Config::getValue<float>({"ROBOT_SIM", "DRAG_COEFFICIENT"}); // DragCoefficient of the vehicle chassis.
    VehicleMovement->ChassisWidth = Config::getValue<float>({"ROBOT_SIM", "CHASSIS_WIDTH"});       // Chassis width used for drag force computation in [cm]
    VehicleMovement->ChassisHeight = Config::getValue<float>({"ROBOT_SIM", "CHASSIS_HEIGHT"});     // Chassis height used for drag force computation in [cm]
    VehicleMovement->MaxEngineRPM = Config::getValue<float>({"ROBOT_SIM", "MOTOR_MAX_RPM"});       // Max RPM for engine

    VehicleMovement->DragCoefficient = 1.0f;  // DragCoefficient of the vehicle chassis.
    VehicleMovement->ChassisWidth = 15.0f;    // Chassis width used for drag force computation in [cm]
    VehicleMovement->ChassisHeight = 15.0f;   // Chassis height used for drag force computation in [cm]
    VehicleMovement->MaxEngineRPM = 20000.0f; // Max RPM for engine

    vehiclePawn_ = static_cast<USimpleWheeledVehicleMovementComponent*>(GetVehicleMovementComponent());\
}

AOpenBotPawn::~AOpenBotPawn()
{
    // Set this pawn to call Tick() every frame.  You can turn this off to
    // improve performance if you don't need it.
    PrimaryActorTick.bCanEverTick = true;
}

void AOpenBotPawn::SetupPlayerInputComponent(class UInputComponent* PlayerInputComponent)
{
    Super::SetupPlayerInputComponent(PlayerInputComponent);

    // set up gameplay key bindings in RobotSimVehicleGameMode
    ASSERT(PlayerInputComponent);

    PlayerInputComponent->BindAxis("MoveForward", this, &AOpenBotPawn::MoveForward);
    PlayerInputComponent->BindAxis("MoveRight", this, &AOpenBotPawn::MoveRight);
}

// This command is meant to be bound to keyboard input. It will be executed at
// each press or unpress event.
void AOpenBotPawn::MoveForward(float Forward)
{
    // Forward describes the percentage of input voltage to be applied to the
    // motor by the H-bridge controller: 1.0 = 100%, -1.0 = reverse 100%.

    dutyCycle_(0) += Forward; // in [%]
    dutyCycle_(1) += Forward; // in [%]
    dutyCycle_(2) += Forward; // in [%]
    dutyCycle_(3) += Forward; // in [%]
}

// This command is meant to be bound to keyboard input. It will be executed at
// each press or unpress event.
void AOpenBotPawn::MoveRight(float Right)
{
    // Right describes the percentage of input voltage to be applied to the
    // motor by the H-bridge controller: 1.0 = 100%, -1.0 = reverse 100%.

    dutyCycle_(0) += Right; // in [%]
    dutyCycle_(1) -= Right; // in [%]
    dutyCycle_(2) += Right; // in [%]
    dutyCycle_(3) -= Right; // in [%]
}

// This command is meant to be used by the python client interface.
void AOpenBotPawn::MoveLeftRight(float leftCtrl, float rightCtrl)
{
    // leftCtrl and rightCtrl describe the percentage of input voltage to be
    // applied to the left and right motors by the H-bridge controller: 1 =
    // 100%, -1 = reverse 100%.

    dutyCycle_(0) += leftCtrl;  // in [%]
    dutyCycle_(1) += rightCtrl; // in [%]
    dutyCycle_(2) += leftCtrl;  // in [%]
    dutyCycle_(3) += rightCtrl; // in [%]
}

// Provides feedback on the action executed by the robot. This action can either
// be defined through the python client or by keyboard/game controller input.
Eigen::Vector2f AOpenBotPawn::GetControlState()
{
    return actionVec_;
}

void AOpenBotPawn::ComputeMotorTorques(float DeltaTime)
{
    // std::cout << "Vehicle velocity: " << this->GetVelocity().Size()*0.036<< "km/h" << std::endl; // GetVelocity() gives results in cm/s
    // First make sure the duty cycle is not getting above 100%. This is done simillarly on the real OpenBot: (c.f.
    // https://github.com/isl-org/OpenBot/blob/master/android/app/src/main/java/org/openbot/vehicle/Control.java)
    dutyCycle_ = Clamp(dutyCycle_, -Eigen::Vector4f::Ones(), Eigen::Vector4f::Ones());

    // Acquire the ground truth motor and wheel velocity for motor counter-electromotive force computation purposes
    // (or alternatively friction computation purposes):
    wheelVelocity_(0) = vehiclePawn_->PVehicle->mWheelsDynData.getWheelRotationSpeed(0); // Expressed in [RPM]
    wheelVelocity_(1) = vehiclePawn_->PVehicle->mWheelsDynData.getWheelRotationSpeed(1); // Expressed in [RPM]
    wheelVelocity_(2) = vehiclePawn_->PVehicle->mWheelsDynData.getWheelRotationSpeed(2); // Expressed in [RPM]
    wheelVelocity_(3) = vehiclePawn_->PVehicle->mWheelsDynData.getWheelRotationSpeed(3); // Expressed in [RPM]

    // vehiclePawn_->PVehicleDrive->setToRestState();
    // vehiclePawn_->PVehicleDrive->mDriveDynData.setToRestState();
    // vehiclePawn_->PVehicle->mWheelsDynData.setToRestState();

    motorVelocity_ = gearRatio_ * RPMToRadSec(wheelVelocity_); // Expressed in [rad/s]

    // Compute the counter electromotive force using the motor torque constant:
    counterElectromotiveForce_ = motorTorqueConstant_ * motorVelocity_; // Expressed in [V]

    // The current allowed to circulate in the motor is the result of the
    // difference between the applied voltage and the counter electromotive force:
    motorWindingCurrent_ = ((batteryVoltage_ * dutyCycle_) - counterElectromotiveForce_) / electricalResistance_; // Expressed in [A]
    // motorWindingCurrent_ = motorWindingCurrent_ *
    // (1-(electricalResistance_/electricalInductance_)*DeltaTime) +
    // ((batteryVoltage_*dutyCycle_)-counterElectromotiveForce_)*DeltaTime/electricalInductance_;
    // // If deltaTime is "small enouth" (which is definitely not the case here)

    // The torque is then obtained using the torque coefficient of the motor:
    motorTorque_ = motorTorqueConstant_ * motorWindingCurrent_;

    // Motor torque is saturated to match the motor limits:
    motorTorque_ = Clamp(motorTorque_, Eigen::Vector4f::Constant(-motorTorqueMax_), Eigen::Vector4f::Constant(motorTorqueMax_));

    // The torque applied to the robot wheels is finally computed acounting for the gear ratio:
    wheelTorque_ = gearRatio_ * motorTorque_;

    // Control dead zone at near-zero velocity:
    // Note: this is a simplified but reliable way to deal with the friction
    // behavior observed on the real vehicle in the low-velocities/low-duty-cycle dommain.
    for (size_t i = 0; i < dutyCycle_.size(); i++)
    {
        if (std::abs(motorVelocity_(i)) < 1e-5 and std::abs(dutyCycle_(i)) <= controlDeadZone_ / actionScale_) // If the motor is "nearly" stopped
        {
            wheelTorque_(i) = 0.f;
        }
    }

    // Apply the drive torque in [N.m] to the vehicle wheels.
    // Note that the "SetDriveTorque" command can be found in the code of the unreal engine at the following location:
    // UnrealEngine-4.26.2-release/Engine/Plugins/Runtime/PhysXVehicles/Source/PhysXVehicles/Public/SimpleWheeledVehicleMovementComponent.h.
    // This file also contains a bunch of useful functions such as "SetBrakeTorque" or "SetSteerAngle". Please take a look if you want to
    // modify the way the simulated vehicle is being controlled.
    vehiclePawn_->SetDriveTorque(wheelTorque_(0), 0); // Torque applied to the wheel, expressed in [N.m]. The applied driveTorque persists until the next call to setDriveTorque.
    vehiclePawn_->SetDriveTorque(wheelTorque_(1), 1); // Torque applied to the wheel, expressed in [N.m]. The applied driveTorque persists until the next call to setDriveTorque.
    vehiclePawn_->SetDriveTorque(wheelTorque_(2), 2); // Torque applied to the wheel, expressed in [N.m]. The applied driveTorque persists until the next call to setDriveTorque.
    vehiclePawn_->SetDriveTorque(wheelTorque_(3), 3); // Torque applied to the wheel, expressed in [N.m]. The applied driveTorque persists until the next call to setDriveTorque.

    // Fill the observed action vector to be used for RL purposes:
    actionVec_(0) = (dutyCycle_(0) + dutyCycle_(2)) / 2; // leftCtrl
    actionVec_(1) = (dutyCycle_(1) + dutyCycle_(3)) / 2; // rightCtrl

    // Reset duty cycle value:
    dutyCycle_.setZero();
}

// Called every simulator update
void AOpenBotPawn::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);
    ComputeMotorTorques(DeltaTime);
    count_++;
}

void AOpenBotPawn::NotifyHit(class UPrimitiveComponent* HitComponent,
                                   class AActor* OtherActor,
                                   class UPrimitiveComponent* otherComp,
                                   bool bSelfMoved,
                                   FVector hitLocation,
                                   FVector hitNormal,
                                   FVector normalImpulse,
                                   const FHitResult& hit)
{
    FString hitComponent = HitComponent->GetName();
    FString otherComponent = otherComp->GetName();

    std::cout << "    COLLISION    " << std::string(TCHAR_TO_UTF8(*hitComponent)) << "  ---  " << std::string(TCHAR_TO_UTF8(*otherComponent)) << std::endl;
}

void AOpenBotPawn::OnComponentCollision(UPrimitiveComponent* HitComponent,
                                              AActor* OtherActor,
                                              UPrimitiveComponent* OtherComp,
                                              FVector NormalImpulse,
                                              const FHitResult& Hit)
{}

// Called when the game starts or when spawned
void AOpenBotPawn::BeginPlay()
{
    Super::BeginPlay();
}

void AOpenBotPawn::SetWheelsFrictionScale(TArray<float>& WheelsFrictionScale)
{

}

void AOpenBotPawn::TeleportToLocation(FVector position,
                                            FQuat orientation,
                                            bool teleport)
{
//    FVector translation = (position * URobotBlueprintLib::GetWorldToMetersScale(this)) - this->GetActorLocation();
//    FRotator rotation = (orientation * this->GetActorQuat().Inverse()).Rotator();
//    RobotBase::TeleportToLocation(position * URobotBlueprintLib::GetWorldToMetersScale(this), orientation, teleport);
}

PRAGMA_ENABLE_DEPRECATION_WARNINGS
