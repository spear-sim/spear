
PRAGMA_DISABLE_DEPRECATION_WARNINGS

#include "SimpleVehiclePawn.h"
//#include "Camera/CameraComponent.h"
//#include "GameFramework/SpringArmComponent.h"
#include "SimpleWheel.h"
#include "TireConfig.h"
#include "VehicleWheel.h"
#include <iostream>

FName ASimpleVehiclePawn::VehicleMovementComponentName(TEXT("SimpleWheeledVehicleMovement"));
FName ASimpleVehiclePawn::VehicleMeshComponentName(TEXT("VehicleMesh'"));

ASimpleVehiclePawn::ASimpleVehiclePawn(const FObjectInitializer& ObjectInitializer) : APawn(ObjectInitializer)
{
    // To create components, you can use CreateDefaultSubobject<Type>("InternalName").
    Mesh = CreateDefaultSubobject<USkeletalMeshComponent>(VehicleMeshComponentName);

    // Setup skeletal mesh
    static ConstructorHelpers::FObjectFinder<USkeletalMesh> CarMesh(TEXT("/RobotSim/SimpleVehicle/freight/freight.freight"));
    Mesh->SetSkeletalMesh(CarMesh.Object);

    // Setup animation
    static ConstructorHelpers::FClassFinder<UAnimInstance> finderAnim(TEXT("/RobotSim/SimpleVehicle/freight/freight_Animation.freight_Animation_C"));

    if (finderAnim.Succeeded())
    {
        Mesh->SetAnimClass(finderAnim.Class);
        UE_LOG(LogTemp, Warning, TEXT("finderAnim success"));
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("finderAnim failed"));
    }

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

    VehicleMovement = CreateDefaultSubobject<UWheeledVehicleMovementComponent, USimpleWheeledVehicleMovementComponent>(VehicleMovementComponentName);
    VehicleMovement->SetIsReplicated(true); // Enable replication by default
    VehicleMovement->UpdatedComponent = Mesh;

    // Setup wheels:
    VehicleMovement->WheelSetups.SetNum(4);

    // TODO dynamic tire?
    UClass* wheelClasss = USimpleWheel::StaticClass();

    // Wheels to create:
    VehicleMovement->WheelSetups[0].WheelClass = wheelClasss;
    VehicleMovement->WheelSetups[0].BoneName = FName("FL");
    VehicleMovement->WheelSetups[0].AdditionalOffset = FVector(0.f, -1.2f, 0.f); // If BoneName is specified, offset the wheel from the bone's location.
                                                                                 // Otherwise this offsets the wheel from the vehicle's origin.

    VehicleMovement->WheelSetups[1].WheelClass = wheelClasss;
    VehicleMovement->WheelSetups[1].BoneName = FName("FR");
    VehicleMovement->WheelSetups[1].AdditionalOffset = FVector(0.f, 1.2f, 0.f); // If BoneName is specified, offset the wheel from the bone's location.
                                                                                // Otherwise this offsets the wheel from the vehicle's origin.

    VehicleMovement->WheelSetups[2].WheelClass = wheelClasss;
    VehicleMovement->WheelSetups[2].BoneName = FName("RL");
    VehicleMovement->WheelSetups[2].AdditionalOffset = FVector(0.f, -1.2f, 0.f); // If BoneName is specified, offset the wheel from the bone's location.
                                                                                 // Otherwise this offsets the wheel from the vehicle's origin.

    VehicleMovement->WheelSetups[3].WheelClass = wheelClasss;
    VehicleMovement->WheelSetups[3].BoneName = FName("RR");
    VehicleMovement->WheelSetups[3].AdditionalOffset = FVector(0.f, 1.2f, 0.f); // If BoneName is specified, offset the wheel from the bone's location.
    // Otherwise this offsets the wheel from the vehicle's origin.

    wheelVelocity_.setZero();
    motorVelocity_.setZero();
    counterElectromotiveForce_.setZero();
    motorTorque_.setZero();
    wheelTorque_.setZero();
    dutyCycle_.setZero();
    motorWindingCurrent_.setZero();
    actionVec_.setZero();
    targetLocation_ = FVector2D::ZeroVector;

    // gearRatio_ = Config::getValue<float>({"ROBOT_SIM", "GEAR_RATIO"});
    // motorVelocityConstant_ = Config::getValue<float>({"ROBOT_SIM", "MOTOR_VELOCITY_CONSTANT"});
    // motorTorqueConstant_ = 1 / motorVelocityConstant_;
    // controlDeadZone_ = Config::getValue<float>({"ROBOT_SIM", "CONTROL_DEAD_ZONE"});
    // motorTorqueMax_ = Config::getValue<float>({"ROBOT_SIM", "MOTOR_TORQUE_MAX"});
    // electricalResistance_ = Config::getValue<float>({"ROBOT_SIM", "ELECTRICAL_RESISTANCE"});
    // electricalInductance_ = Config::getValue<float>({"ROBOT_SIM", "ELECTRICAL_INDUCTANCE"});
    // actionScale_ = Config::getValue<float>({"ROBOT_SIM", "ACTION_SCALE"});
    // batteryVoltage_ = Config::getValue<float>({"ROBOT_SIM", "BATTERY_VOLTAGE"});

    // Vehicle dynamics:
    // VehicleMovement->DragCoefficient = Config::getValue<float>({"ROBOT_SIM", "DRAG_COEFFICIENT"}); // DragCoefficient of the vehicle chassis.
    // VehicleMovement->ChassisWidth = Config::getValue<float>({"ROBOT_SIM", "CHASSIS_WIDTH"});       // Chassis width used for drag force computation in [cm]
    // VehicleMovement->ChassisHeight = Config::getValue<float>({"ROBOT_SIM", "CHASSIS_HEIGHT"});     // Chassis height used for drag force computation in [cm]
    // VehicleMovement->MaxEngineRPM = Config::getValue<float>({"ROBOT_SIM", "MOTOR_MAX_RPM"});       // Max RPM for engine

    // VehicleMovement->DragCoefficient = 1.0f; // DragCoefficient of the vehicle chassis.
    // VehicleMovement->ChassisWidth = 15.0f;       // Chassis width used for drag force computation in [cm]
    // VehicleMovement->ChassisHeight = 15.0f;     // Chassis height used for drag force computation in [cm]
    // VehicleMovement->MaxEngineRPM = 20000.0f;       // Max RPM for engine

    vehiclePawn_ = static_cast<USimpleWheeledVehicleMovementComponent*>(GetVehicleMovementComponent());

    // for (int i = 0; i < 4; i++)
    // {

    //     std::cout << "##########################################################################" << std::endl;
    //     PxVehicleTireData PTireData = vehiclePawn_->mWheelsSimData.getTireData(i);
    //     std::cout << "##########################################################################" << std::endl;
    //     std::cout << "PTireData.mLatStiffX = " << PTireData.mLatStiffX << std::endl;
    //     std::cout << "PTireData.mLatStiffY = " << PTireData.mLatStiffY << std::endl;
    //     std::cout << "PTireData.mLongitudinalStiffnessPerUnitGravity = " << PTireData.mLongitudinalStiffnessPerUnitGravity << std::endl;
    //     std::cout << "##########################################################################" << std::endl;
    //     PTireData.mLatStiffX = 20.0;
    //     PTireData.mLatStiffY = 20.0;
    //     PTireData.mLongitudinalStiffnessPerUnitGravity = 1000.0;
    //     std::cout << "##########################################################################" << std::endl;
    //     vehiclePawn_->mWheelsSimData.setTireData(i, PTireData);
    //     std::cout << "##########################################################################" << std::endl;
    //     // VehicleMovement->Wheels[i]->TireConfig->SetFrictionScale(10.0);
    //     std::cout << "##########################################################################" << std::endl;
    // }
}

ASimpleVehiclePawn::~ASimpleVehiclePawn()
{
    // Set this pawn to call Tick() every frame.  You can turn this off to
    // improve performance if you don't need it.
    PrimaryActorTick.bCanEverTick = true;
}

void ASimpleVehiclePawn::SetupPlayerInputComponent(class UInputComponent* PlayerInputComponent)
{
    Super::SetupPlayerInputComponent(PlayerInputComponent);

    // set up gameplay key bindings in RobotSimVehicleGameMode
    check(PlayerInputComponent);

    PlayerInputComponent->BindAxis("MoveForward", this, &ASimpleVehiclePawn::MoveForward);
    PlayerInputComponent->BindAxis("MoveRight", this, &ASimpleVehiclePawn::MoveRight);
}

void ASimpleVehiclePawn::SetupInputBindings()
{
    UE_LOG(LogTemp, Warning, TEXT("ASimpleVehiclePawn::SetupInputBindings start"));

    this->EnableInput(this->GetWorld()->GetFirstPlayerController());

    UE_LOG(LogTemp, Warning, TEXT("ASimpleVehiclePawn::SetupInputBindings end"));

    // Keyboard control in RobotSimGameMode
    APlayerController* controller = this->GetWorld()->GetFirstPlayerController();
    controller->InputComponent->BindAxis("MoveForward", this, &ASimpleVehiclePawn::MoveForward);
    controller->InputComponent->BindAxis("MoveRight", this, &ASimpleVehiclePawn::MoveRight);
}

// This command is meant to be bound to keyboard input. It will be executed at
// each press or unpress event.
void ASimpleVehiclePawn::MoveForward(float Forward)
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
void ASimpleVehiclePawn::MoveRight(float Right)
{
    // Right describes the percentage of input voltage to be applied to the
    // motor by the H-bridge controller: 1.0 = 100%, -1.0 = reverse 100%.

    dutyCycle_(0) += Right; // in [%]
    dutyCycle_(1) -= Right; // in [%]
    dutyCycle_(2) += Right; // in [%]
    dutyCycle_(3) -= Right; // in [%]
}

// This command is meant to be used by the python client interface.
void ASimpleVehiclePawn::MoveLeftRight(float leftCtrl, float rightCtrl)
{
    // leftCtrl and rightCtrl describe the percentage of input voltage to be
    // applied to the left and right motors by the H-bridge controller: 1 =
    // 100%, -1 = reverse 100%.

    dutyCycle_(0) += leftCtrl;  // in [%]
    dutyCycle_(1) += rightCtrl; // in [%]
    dutyCycle_(2) += leftCtrl;  // in [%]
    dutyCycle_(3) += rightCtrl; // in [%]
}

// This command is meant to be used by the python client interface.
bool ASimpleVehiclePawn::MoveTo(const FVector2D& target)
{
    targetLocation_ = target;
    useAutopilot_ = true;
    return targetLocationReached_;
}

// Provides feedback on the action executed by the robot. This action can either
// be defined through the python client or by keyboard/game controller input.
Eigen::Vector2f ASimpleVehiclePawn::GetControlState()
{
    return actionVec_;
}

void ASimpleVehiclePawn::ComputeMotorTorques(float DeltaTime)
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

    // std::cout << " ----------------------------------------------- " <<
    // std::endl; std::cout << "actionVec_ = " << actionVec_.transpose() <<
    // std::endl; std::cout << "dutyCycle_ = " << dutyCycle_.transpose() <<
    // std::endl; std::cout << "motorVelocity_ = " << motorVelocity_.transpose()
    // << std::endl; std::cout << "wheelVelocity_ = " <<
    // wheelVelocity_.transpose() << std::endl; std::cout << "appliedVoltage = "
    // << (batteryVoltage_*dutyCycle_).transpose() << std::endl; std::cout <<
    // "counterElectromotiveForce_ = " << counterElectromotiveForce_.transpose()
    // << std::endl; std::cout << "motorWindingCurrent_ = " <<
    // motorWindingCurrent_.transpose() << std::endl; std::cout << "motorTorque_
    // = " << motorTorque_.transpose() << std::endl; std::cout << "wheelTorque_
    // = " << wheelTorque_.transpose() << std::endl;

    // Reset duty cycle value:
    dutyCycle_.setZero();
}

void ASimpleVehiclePawn::SetRobotParameters(const RobotSim::RobotSimSettings::VehicleSetting& settings)
{
    gearRatio_ = settings.actuationSetting.gearRatio;
    motorVelocityConstant_ = settings.actuationSetting.motorVelocityConstant;
    motorTorqueConstant_ = 1 / motorVelocityConstant_;
    controlDeadZone_ = settings.actuationSetting.controlDeadZone;
    motorTorqueMax_ = settings.actuationSetting.motorTorqueMax;
    electricalResistance_ = settings.actuationSetting.electricalResistance;
    electricalInductance_ = settings.actuationSetting.electricalInductance;
    actionScale_ = settings.actuationSetting.actionScale;
    batteryVoltage_ = settings.actuationSetting.batteryVoltage;
}

// Called every simulator update
void ASimpleVehiclePawn::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    if (useAutopilot_)
    {
        TrackWayPoint(DeltaTime);
    }

    ComputeMotorTorques(DeltaTime);

    // Display the tick number to the simulation screen:
    // URobotBlueprintLib::LogMessage(FString("Tick - "),
    //                                FString::SanitizeFloat(this->count_),
    //                                LogDebugLevel::Informational, 30);
    count_++;
}

void ASimpleVehiclePawn::NotifyHit(class UPrimitiveComponent* HitComponent,
                                   class AActor* OtherActor,
                                   class UPrimitiveComponent* otherComp,
                                   bool bSelfMoved,
                                   FVector hitLocation,
                                   FVector hitNormal,
                                   FVector normalImpulse,
                                   const FHitResult& hit)
{
    // URobotBlueprintLib::LogMessage(FString("NotifyHit: ") + OtherActor->GetName(), " location: " + hitLocation.ToString() + " normal: " + normalImpulse.ToString(), LogDebugLevel::Informational, 30);

    FString hitComponent = HitComponent->GetName();
    FString otherComponent = otherComp->GetName();

    std::cout << "    COLLISION    " << std::string(TCHAR_TO_UTF8(*hitComponent)) << "  ---  " << std::string(TCHAR_TO_UTF8(*otherComponent)) << std::endl;
    this->pawnEvents_.getCollisionSignal().emit(HitComponent, OtherActor, otherComp, bSelfMoved, hitLocation, hitNormal, normalImpulse, hit);
}

void ASimpleVehiclePawn::OnComponentCollision(UPrimitiveComponent* HitComponent,
                                              AActor* OtherActor,
                                              UPrimitiveComponent* OtherComp,
                                              FVector NormalImpulse,
                                              const FHitResult& Hit)
{
    URobotBlueprintLib::LogMessage(FString("OnComponentCollision: ") + OtherActor->GetName(), " location: " + Hit.Location.ToString() + " normal: " + Hit.Normal.ToString(), LogDebugLevel::Informational, 30);
}

void ASimpleVehiclePawn::TrackWayPoint(float DeltaTime)
{
    const FVector currentLocation = this->GetActorLocation();     // Relative to global coordinate system
    const FRotator currentOrientation = this->GetActorRotation(); // Relative to global coordinate system
    const FVector2D relativePositionToTarget(targetLocation_.X - currentLocation.X, targetLocation_.Y - currentLocation.Y);
    float forwardCtrl = 0.0f;
    float rightCtrl = 0.0f;

    targetLocationReached_ = false;

    // If the waypoint is reached:
    if ((relativePositionToTarget.Size() * 0.01) < Config::getValue<float>({"ROBOT_SIM", "ACCEPTANCE_RADIUS"}))
    {
        targetLocationReached_ = true;
    }

    if (targetLocationReached_ == false)
    {
        // Compute Euclidean distance to target:
        float dist = relativePositionToTarget.Size();

        // Compute robot forward axis (global coordinate system)
        FVector forwardAxis = FVector(1.f, 0.f, 0.f); // Front axis is the X axis.
        FVector forwardAxisRotated = currentOrientation.RotateVector(forwardAxis);

        // Compute yaw in [rad]:
        float deltaYaw = std::atan2f(forwardAxisRotated.Y, forwardAxisRotated.X) - std::atan2f(relativePositionToTarget.Y, relativePositionToTarget.X);

        // std::cout << "std::atan2f(forwardAxisRotated.Y, forwardAxisRotated.X) " << std::atan2f(forwardAxisRotated.Y, forwardAxisRotated.X) << std::endl;
        // std::cout << "std::atan2f(relativePositionToTarget.Y, relativePositionToTarget.X) " << std::atan2f(relativePositionToTarget.Y, relativePositionToTarget.X) << std::endl;

        // Fit to range [-pi, pi]:
        if (deltaYaw > PI)
        {
            deltaYaw -= 2 * PI;
        }
        else
        {
            if (deltaYaw <= -PI)
            {
                deltaYaw += 2 * PI;
            }
        }
        FVector2D vel2D(this->GetVelocity().X, this->GetVelocity().Y);
        float linVel = vel2D.Size() * 0.036; // In [m/s]
        float yawVel = (FMath::DegreesToRadians(currentOrientation.Yaw) - yaw_) / DeltaTime;
        yaw_ = FMath::DegreesToRadians(currentOrientation.Yaw);

        rightCtrl = -Config::getValue<float>({"ROBOT_SIM", "PROPORTIONAL_GAIN_HEADING"}) * deltaYaw + Config::getValue<float>({"ROBOT_SIM", "DERIVATIVE_GAIN_HEADING"}) * yawVel;

        if (std::abs(deltaYaw) < Config::getValue<float>({"ROBOT_SIM", "FORWARD_MIN_ANGLE"}))
        {
            forwardCtrl = Config::getValue<float>({"ROBOT_SIM", "PROPORTIONAL_GAIN_DIST"}) * relativePositionToTarget.Size() * 0.01 - Config::getValue<float>({"ROBOT_SIM", "DERIVATIVE_GAIN_DIST"}) * linVel;
            forwardCtrl = Clamp(forwardCtrl, -Config::getValue<float>({"ROBOT_SIM", "CONTROL_SATURATION"}), Config::getValue<float>({"ROBOT_SIM", "CONTROL_SATURATION"}));
            forwardCtrl *= std::cosf(deltaYaw); // Full throttle if the vehicle face the objective. Otherwise give more priority to the yaw command.
        }
        else
        {
            forwardCtrl = 0.0f;
        }
        // std::cout << "dist " << relativePositionToTarget.Size() * 0.01 << std::endl;
        // std::cout << "deltaYaw " << deltaYaw << std::endl;
        // std::cout << "rightCtrl " << rightCtrl << std::endl;
        // std::cout << "forwardCtrl " << forwardCtrl << std::endl;
    }
    MoveRight(Clamp(rightCtrl, -Config::getValue<float>({"ROBOT_SIM", "CONTROL_SATURATION"}), Config::getValue<float>({"ROBOT_SIM", "CONTROL_SATURATION"})));
    MoveForward(Clamp(forwardCtrl, -Config::getValue<float>({"ROBOT_SIM", "CONTROL_SATURATION"}), Config::getValue<float>({"ROBOT_SIM", "CONTROL_SATURATION"})));
}

// Called when the game starts or when spawned
void ASimpleVehiclePawn::BeginPlay()
{
    Super::BeginPlay();

//     float FrictionScale = 3.5f;
//     std::cout << "####################################### " <<  __LINE__  << " #######################################" << std::endl;
//     USimpleWheeledVehicleMovementComponent* Vehicle = Cast<USimpleWheeledVehicleMovementComponent>(GetVehicleMovementComponent());
//     ASSERT(Vehicle != nullptr);
// std::cout << "####################################### " <<  __LINE__  << " #######################################" << std::endl;
//     // Setup Tire Configs with default value. This is needed to avoid getting
//     // friction values of previously created TireConfigs for the same vehicle
//     // blueprint.
//     std::cout << "####################################### " <<  __LINE__  << " #######################################" << std::endl;
//     TArray<float> OriginalFrictions;
//     std::cout << "####################################### " <<  __LINE__  << " #######################################" << std::endl;
//     OriginalFrictions.Init(FrictionScale, Vehicle->Wheels.Num());
//     std::cout << "####################################### " <<  __LINE__  << " #######################################" << std::endl;
//     SetWheelsFrictionScale(OriginalFrictions);
// std::cout << "####################################### " <<  __LINE__  << " #######################################" << std::endl;
//     // Check if it overlaps with a Friction trigger, if so, update the friction scale.
    // TArray<AActor*> OverlapActors;
    // GetOverlappingActors(OverlapActors, AFrictionTrigger::StaticClass());
    // for (const auto& Actor : OverlapActors)
    // {
    //     AFrictionTrigger* FrictionTrigger = Cast<AFrictionTrigger>(Actor);
    //     if (FrictionTrigger)
    //     {
    //         FrictionScale = FrictionTrigger->Friction;
    //     }
    // }

    // Set the friction scale to Wheel CDO and update wheel setups
//     TArray<FWheelSetup> NewWheelSetups = Vehicle->WheelSetups;
// std::cout << "####################################### " <<  __LINE__  << " #######################################" << std::endl;
//     for (const auto& WheelSetup : NewWheelSetups)
//     {
//         std::cout << "####################################### " <<  __LINE__  << " #######################################" << std::endl;
//         UVehicleWheel* Wheel = WheelSetup.WheelClass.GetDefaultObject();
//         std::cout << "####################################### " <<  __LINE__  << " #######################################" << std::endl;
//         ASSERT(Wheel != nullptr);
//     }
// std::cout << "####################################### " <<  __LINE__  << " #######################################" << std::endl;
//     Vehicle->WheelSetups = NewWheelSetups;
//     std::cout << "####################################### " <<  __LINE__  << " #######################################" << std::endl;
}

void ASimpleVehiclePawn::SetWheelsFrictionScale(TArray<float>& WheelsFrictionScale)
{
    // std::cout << "####################################### " <<  __LINE__  << " #######################################" << std::endl;
    // USimpleWheeledVehicleMovementComponent* Vehicle = Cast<USimpleWheeledVehicleMovementComponent>(GetVehicleMovementComponent());
    // std::cout << "####################################### " <<  __LINE__  << " #######################################" << std::endl;
    // ASSERT(Vehicle != nullptr);
    // std::cout << "####################################### " <<  __LINE__  << " #######################################" << std::endl;
    // ASSERT(Vehicle->Wheels.Num() == WheelsFrictionScale.Num());
    // std::cout << "####################################### " <<  __LINE__  << " #######################################" << std::endl;

    // for (int i = 0; i < Vehicle->Wheels.Num(); ++i)
    // {
    //     std::cout << "####################################### " <<  __LINE__  << " #######################################" << std::endl;
    //     Vehicle->Wheels[i]->TireConfig->SetFrictionScale(WheelsFrictionScale[i]);
    //     std::cout << "####################################### " <<  __LINE__  << " #######################################" << std::endl;
    // }
}

USceneComponent* ASimpleVehiclePawn::GetComponent(FString componentName)
{
    ASSERT(false);
}

void ASimpleVehiclePawn::GetComponentReferenceTransform(FString componentName,
                                                        FVector& translation,
                                                        FRotator& rotation)
{
    ASSERT(false);
}

void ASimpleVehiclePawn::TeleportToLocation(FVector position,
                                            FQuat orientation,
                                            bool teleport)
{
    FVector translation = (position * URobotBlueprintLib::GetWorldToMetersScale(this)) - this->GetActorLocation();
    FRotator rotation = (orientation * this->GetActorQuat().Inverse()).Rotator();
    RobotBase::TeleportToLocation(position * URobotBlueprintLib::GetWorldToMetersScale(this), orientation, teleport);
}

PawnEvents* ASimpleVehiclePawn::GetPawnEvents()
{
    return &(this->pawnEvents_);
}

PRAGMA_ENABLE_DEPRECATION_WARNINGS
