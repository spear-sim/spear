#include "OpenBotPawn.h"

PRAGMA_DISABLE_DEPRECATION_WARNINGS

#include <iostream>

#include <Camera/CameraComponent.h>
#include <Components/SceneCaptureComponent2D.h>
#include <Engine/TextureRenderTarget2D.h>
#include <TireConfig.h>
#include <VehicleWheel.h>

#include "OpenBotWheel.h"

FName AOpenBotPawn::VehicleMovementComponentName(TEXT("SimpleWheeledVehicleMovement"));
FName AOpenBotPawn::VehicleMeshComponentName(TEXT("VehicleMesh'"));

AOpenBotPawn::AOpenBotPawn(const FObjectInitializer& object_initializer) : APawn(object_initializer)
{
    // To create components, you can use CreateDefaultSubobject<Type>("InternalName").
    mesh_ = CreateDefaultSubobject<USkeletalMeshComponent>(VehicleMeshComponentName);

    // Setup skeletal mesh
    static ConstructorHelpers::FObjectFinder<USkeletalMesh> openbot_mesh_finder(TEXT("/OpenBot/v0/Meshes/OpenBot_Mesh.OpenBot_Mesh"));
    ASSERT(openbot_mesh_finder.Succeeded());
    mesh_->SetSkeletalMesh(openbot_mesh_finder.Object);

    // Setup animation
    static ConstructorHelpers::FClassFinder<UAnimInstance> openbot_animation_finder(TEXT("/OpenBot/v0/Meshes/OpenBot_Animation.OpenBot_Animation_C"));
    ASSERT(openbot_animation_finder.Succeeded());
    mesh_->SetAnimClass(openbot_animation_finder.Class);

    mesh_->SetCollisionProfileName(UCollisionProfile::Vehicle_ProfileName);
    mesh_->BodyInstance.bSimulatePhysics = true;
    mesh_->BodyInstance.bNotifyRigidBodyCollision = true;
    mesh_->BodyInstance.bUseCCD = true;
    mesh_->bBlendPhysics = true;
    mesh_->SetGenerateOverlapEvents(true);
    mesh_->SetCanEverAffectNavigation(false);
    // example for adding user-defined collision callback
    // Mesh->OnComponentHit.AddDynamic(this,&ASimpleVehiclePawn::OnComponentCollision);
    RootComponent = mesh_;

    vehicle_movement_ = CreateDefaultSubobject<USimpleWheeledVehicleMovementComponent>(VehicleMovementComponentName);
    vehicle_movement_->SetIsReplicated(true); // Enable replication by default
    vehicle_movement_->UpdatedComponent = mesh_;

    // Setup wheels:
    vehicle_movement_->WheelSetups.SetNum(4);

    // TODO dynamic tire?
    UClass* wheel_class = UOpenBotWheel::StaticClass();

    // Wheels to create:
    vehicle_movement_->WheelSetups[0].WheelClass = wheel_class;
    vehicle_movement_->WheelSetups[0].BoneName = FName("FL");
    vehicle_movement_->WheelSetups[0].AdditionalOffset = FVector(0.f, 0.f, 0.f); // If BoneName is specified, offset the wheel from the bone's location.
                                                                                 // Otherwise this offsets the wheel from the vehicle's origin.

    vehicle_movement_->WheelSetups[1].WheelClass = wheel_class;
    vehicle_movement_->WheelSetups[1].BoneName = FName("FR");
    vehicle_movement_->WheelSetups[1].AdditionalOffset = FVector(0.f, 0.f, 0.f); // If BoneName is specified, offset the wheel from the bone's location.
                                                                                // Otherwise this offsets the wheel from the vehicle's origin.

    vehicle_movement_->WheelSetups[2].WheelClass = wheel_class;
    vehicle_movement_->WheelSetups[2].BoneName = FName("RL");
    vehicle_movement_->WheelSetups[2].AdditionalOffset = FVector(0.f, 0.f, 0.f); // If BoneName is specified, offset the wheel from the bone's location.
                                                                                 // Otherwise this offsets the wheel from the vehicle's origin.

    vehicle_movement_->WheelSetups[3].WheelClass = wheel_class;
    vehicle_movement_->WheelSetups[3].BoneName = FName("RR");
    vehicle_movement_->WheelSetups[3].AdditionalOffset = FVector(0.f, 0.f, 0.f); // If BoneName is specified, offset the wheel from the bone's location.
    // Otherwise this offsets the wheel from the vehicle's origin.

    wheelVelocity_.setZero();
    motorVelocity_.setZero();
    counterElectromotiveForce_.setZero();
    motorTorque_.setZero();
    wheelTorque_.setZero();
    dutyCycle_.setZero();
    motorWindingCurrent_.setZero();
    actionVec_.setZero();

    gearRatio_ = Config::getValue<float>({"OPENBOT", "GEAR_RATIO"});
    motorVelocityConstant_ = Config::getValue<float>({"OPENBOT", "MOTOR_VELOCITY_CONSTANT"});
    motorTorqueConstant_ = 1 / motorVelocityConstant_;
    controlDeadZone_ = Config::getValue<float>({"OPENBOT", "CONTROL_DEAD_ZONE"});
    motorTorqueMax_ = Config::getValue<float>({"OPENBOT", "MOTOR_TORQUE_MAX"});
    electricalResistance_ = Config::getValue<float>({"OPENBOT", "ELECTRICAL_RESISTANCE"});
    electricalInductance_ = Config::getValue<float>({"OPENBOT", "ELECTRICAL_INDUCTANCE"});
    actionScale_ = Config::getValue<float>({"OPENBOT", "ACTION_SCALE"});
    batteryVoltage_ = Config::getValue<float>({"OPENBOT", "BATTERY_VOLTAGE"});

    // Vehicle dynamics:
    vehicle_movement_->DragCoefficient = Config::getValue<float>({"OPENBOT", "DRAG_COEFFICIENT"}); // DragCoefficient of the vehicle chassis.
    vehicle_movement_->ChassisWidth = Config::getValue<float>({"OPENBOT", "CHASSIS_WIDTH"});       // Chassis width used for drag force computation in [cm]
    vehicle_movement_->ChassisHeight = Config::getValue<float>({"OPENBOT", "CHASSIS_HEIGHT"});     // Chassis height used for drag force computation in [cm]
    vehicle_movement_->MaxEngineRPM = Config::getValue<float>({"OPENBOT", "MOTOR_MAX_RPM"});       // Max RPM for engine

    vehicle_movement_->DragCoefficient = 1.0f;  // DragCoefficient of the vehicle chassis.
    vehicle_movement_->ChassisWidth = 15.0f;    // Chassis width used for drag force computation in [cm]
    vehicle_movement_->ChassisHeight = 15.0f;   // Chassis height used for drag force computation in [cm]
    vehicle_movement_->MaxEngineRPM = 20000.0f; // Max RPM for engine

    // TODO read from config or fixed position?
    // Create camera component
    FVector CameraPos(5.0f, 5.0f, 5.0f);
    FRotator CameraOri(0.0f, 0.0f, 0.0f);

    camera_ = CreateDefaultSubobject<UCameraComponent>(TEXT("OpenBotSmartPhoneCamera"));
    camera_->SetRelativeLocationAndRotation(CameraPos, CameraOri);
    camera_->SetupAttachment(mesh_);
    camera_->bUsePawnControlRotation = false;
    camera_->FieldOfView = 90.f;

    //CaptureComponent = this->GetWorld()->SpawnActor<ASceneCapture2D>(FVector::ZeroVector, FRotator::ZeroRotator);

    scene_capture_ = CreateDefaultSubobject<USceneCaptureComponent2D>(TEXT("OpenBotSmartPhoneCapture"));
    scene_capture_->SetRelativeLocationAndRotation(CameraPos, CameraOri);
    scene_capture_->SetupAttachment(mesh_);

    // Create RenderTargets
    UTextureRenderTarget2D* render_target = NewObject<UTextureRenderTarget2D>();

    //renderTarget2D->TargetGamma = GEngine->GetDisplayGamma();           // 1.2f; for Vulkan | GEngine->GetDisplayGamma(); for DX11/12
    render_target->InitAutoFormat(32,32);                         // Setup the RenderTarget capture format: some random format, got crashing otherwise frameWidht = 2048 and frameHeight = 2048.
    render_target->InitCustomFormat(32,32, PF_B8G8R8A8, true);    // PF_B8G8R8A8 disables HDR which will boost storing to disk due to less image information
    render_target->RenderTargetFormat = ETextureRenderTargetFormat::RTF_RGBA8;
    render_target->bGPUSharedFlag = true; // demand buffer on GPU

    // Set Camera Properties
    scene_capture_->bAlwaysPersistRenderingState = 1;
    scene_capture_->bCaptureEveryFrame = 0;
    scene_capture_->FOVAngle = 90.0f;               // Standard FOV
    scene_capture_->TextureTarget = render_target; // Assign RenderTarget
    scene_capture_->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
    scene_capture_->ShowFlags.SetTemporalAA(false);
    scene_capture_->ShowFlags.SetAntiAliasing(true);
    // Lookup more showflags in the UE4 documentation..

    // Set post processing parameters:
    FPostProcessSettings prost_process_setting;
    prost_process_setting.MotionBlurAmount = 10.f; // Strength of motion blur, 0:off, should be renamed to intensity
    prost_process_setting.MotionBlurMax = 10.f;    // Max distortion caused by motion blur, in percent of the screen width, 0:off
    scene_capture_->PostProcessSettings = prost_process_setting;
    scene_capture_->PostProcessBlendWeight = 1.f; // Range (0.0, 1.0) where 0 indicates no effect, 1 indicates full effect.

}

AOpenBotPawn::~AOpenBotPawn()
{
    // Set this pawn to call Tick() every frame.  You can turn this off to
    // improve performance if you don't need it.
    PrimaryActorTick.bCanEverTick = true;
}

void AOpenBotPawn::SetupPlayerInputComponent(class UInputComponent* input_component)
{
    Super::SetupPlayerInputComponent(input_component);

    // set up gameplay key bindings in RobotSimVehicleGameMode
    ASSERT(input_component);

    input_component->BindAxis("MoveForward", this, &AOpenBotPawn::moveForward);
    input_component->BindAxis("MoveRight", this, &AOpenBotPawn::moveRight);
}

// This command is meant to be bound to keyboard input. It will be executed at
// each press or unpress event.
void AOpenBotPawn::moveForward(float forward)
{
    // Forward describes the percentage of input voltage to be applied to the
    // motor by the H-bridge controller: 1.0 = 100%, -1.0 = reverse 100%.

    dutyCycle_(0) += forward; // in [%]
    dutyCycle_(1) += forward; // in [%]
    dutyCycle_(2) += forward; // in [%]
    dutyCycle_(3) += forward; // in [%]
}

// This command is meant to be bound to keyboard input. It will be executed at
// each press or unpress event.
void AOpenBotPawn::moveRight(float right)
{
    // Right describes the percentage of input voltage to be applied to the
    // motor by the H-bridge controller: 1.0 = 100%, -1.0 = reverse 100%.

    dutyCycle_(0) += right; // in [%]
    dutyCycle_(1) -= right; // in [%]
    dutyCycle_(2) += right; // in [%]
    dutyCycle_(3) -= right; // in [%]
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

void AOpenBotPawn::setDriveTorques(float DeltaTime)
{
    // std::cout << "Vehicle velocity: " << this->GetVelocity().Size()*0.036<< "km/h" << std::endl; // GetVelocity() gives results in cm/s
    // First make sure the duty cycle is not getting above 100%. This is done simillarly on the real OpenBot: (c.f.
    // https://github.com/isl-org/OpenBot/blob/master/android/app/src/main/java/org/openbot/vehicle/Control.java)
    dutyCycle_ = Clamp(dutyCycle_, -Eigen::Vector4f::Ones(), Eigen::Vector4f::Ones());

    // Acquire the ground truth motor and wheel velocity for motor counter-electromotive force computation purposes
    // (or alternatively friction computation purposes):
    wheelVelocity_(0) = vehicle_movement_->PVehicle->mWheelsDynData.getWheelRotationSpeed(0); // Expressed in [RPM]
    wheelVelocity_(1) = vehicle_movement_->PVehicle->mWheelsDynData.getWheelRotationSpeed(1); // Expressed in [RPM]
    wheelVelocity_(2) = vehicle_movement_->PVehicle->mWheelsDynData.getWheelRotationSpeed(2); // Expressed in [RPM]
    wheelVelocity_(3) = vehicle_movement_->PVehicle->mWheelsDynData.getWheelRotationSpeed(3); // Expressed in [RPM]

    // vehicle_movement_->PVehicleDrive->setToRestState();
    // vehicle_movement_->PVehicleDrive->mDriveDynData.setToRestState();
    // vehicle_movement_->PVehicle->mWheelsDynData.setToRestState();

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
    vehicle_movement_->SetDriveTorque(wheelTorque_(0), 0); // Torque applied to the wheel, expressed in [N.m]. The applied driveTorque persists until the next call to setDriveTorque.
    vehicle_movement_->SetDriveTorque(wheelTorque_(1), 1); // Torque applied to the wheel, expressed in [N.m]. The applied driveTorque persists until the next call to setDriveTorque.
    vehicle_movement_->SetDriveTorque(wheelTorque_(2), 2); // Torque applied to the wheel, expressed in [N.m]. The applied driveTorque persists until the next call to setDriveTorque.
    vehicle_movement_->SetDriveTorque(wheelTorque_(3), 3); // Torque applied to the wheel, expressed in [N.m]. The applied driveTorque persists until the next call to setDriveTorque.

    // Fill the observed action vector to be used for RL purposes:
    actionVec_(0) = (dutyCycle_(0) + dutyCycle_(2)) / 2; // leftCtrl
    actionVec_(1) = (dutyCycle_(1) + dutyCycle_(3)) / 2; // rightCtrl

    // Reset duty cycle value:
    dutyCycle_.setZero();
}

// Called every simulator update
void AOpenBotPawn::Tick(float delta_time)
{
    Super::Tick(delta_time);
    setDriveTorques(delta_time);
}

void AOpenBotPawn::NotifyHit(class UPrimitiveComponent* hit_component,
                                   class AActor* other_actor,
                                   class UPrimitiveComponent* other_component,
                                   bool bself_moved,
                                   FVector hit_location,
                                   FVector hit_normal,
                                   FVector normal_impulse,
                                   const FHitResult& hit)
{
    FString hit_component_name = hit_component->GetName();
    FString other_component_name = other_component->GetName();

    std::cout << "    COLLISION    " << std::string(TCHAR_TO_UTF8(*hit_component_name)) << "  ---  " << std::string(TCHAR_TO_UTF8(*other_component_name)) << std::endl;
}


    //@brief Clamp a vector between two values.
Eigen::Vector4f AOpenBotPawn::Clamp(Eigen::Vector4f v, Eigen::Vector4f vMin, Eigen::Vector4f vMax)
{
    Eigen::Vector4f v_clamped;
    v_clamped = v;
    for (unsigned int i = 0; i < v.size(); i++)
    {
        if (v(i) > vMax(i))
        {
            v_clamped(i) = vMax(i);
        }
        if (v(i) < vMin(i))
        {
            v_clamped(i) = vMin(i);
        }
    }
    return v_clamped;
}

//Rev per minute to rad/s
Eigen::VectorXf AOpenBotPawn::RPMToRadSec(Eigen::VectorXf RPM)
{
    return RPM * PI / 30.f;
}

//rad/s to rev per minute
Eigen::VectorXf AOpenBotPawn::RadSecToRPM(Eigen::VectorXf Omega)
{
    return Omega * 30.f / PI;
}

PRAGMA_ENABLE_DEPRECATION_WARNINGS
