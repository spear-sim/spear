#include "Motor.h"
#include "Engine/Engine.h"

void Motor::Attach(AUrdfLink* baseLink,
                   AUrdfLink* actuationLink,
                   UrdfJointSpecification* jointSpecification,
                   UPhysicsConstraintComponent* constraintComponent)
{
    ControlledMotionComponent::Attach(baseLink, actuationLink,
                                      jointSpecification, constraintComponent);

    if (jointSpecification->Type != CONTINUOUS_TYPE)
    {
        throw std::runtime_error(
            "Attempting to initialize a motor on joint '" +
            std::string(TCHAR_TO_UTF8(*jointSpecification->Name)) +
            "', which is not of type continuous.");
    }

    this->rotationAxis_ = jointSpecification->Axis;
    this->maxVelocity_ = jointSpecification->Limit->Velocity;
    this->controlSignalSetPoint_ = 0.0f;
    this->radius_ = 0.0;
    this->targetConstTorque_ = FVector::ZeroVector;
    this->bUseTorqueControl = false;

    float stiffness = constraintComponent_->ConstraintInstance.ProfileInstance
                          .AngularDrive.TwistDrive.Stiffness;
    float damping = constraintComponent_->ConstraintInstance.ProfileInstance
                        .AngularDrive.TwistDrive.Damping;
    float maxForce = constraintComponent_->ConstraintInstance.ProfileInstance
                         .AngularDrive.TwistDrive.MaxForce;

    // default stiffness and damping instead of URDF value to optimize
    // movement performance
    float val = 1000000;
    this->SetDrive(val, val / 10);
}

void Motor::SetMotionSpeed(const float& liner_speed)
{
    //转速转换
    if (fabs(this->radius_) < FLT_MIN)
    {
        throw std::runtime_error("the motor radius value is not initialized, "
                                 "please set a reasonable number");
    }
    float actual_angular_velocity = liner_speed / this->radius_;
    // 除以2* PI 时因为 SetAngularVelocityTarget的参数 为转的圈数
    float target_velocity = actual_angular_velocity / (2 * PI);
    float ConvertToSignalPoint = target_velocity / this->maxVelocity_;

    this->controlSignalSetPoint_ = ConvertToSignalPoint;
    //必须要将bUseTorqueControl，   flag设置为false；
    this->bUseTorqueControl = false;
}

float Motor::GetMotionSpeed() const
{
    FVector targetAngularVelocity =
        this->actuationLink_->GetPhysicsAngularVelocityInRadians();
    FVector baseAngularVelocity =
        this->baseLink_->GetPhysicsAngularVelocityInRadians();
    // relative angular velocity
    FVector worldDiffAngularVelocity =
        targetAngularVelocity - baseAngularVelocity;
    // angular velocity in base_link frame
    FVector baseFrameAngularVelocity =
        this->baseLink_->GetTransform().InverseTransformVector(
            worldDiffAngularVelocity);
    // velocity vector magnitude in axis direction
    float val = FVector::DotProduct(this->jointSpecification_->Axis,
                                    baseFrameAngularVelocity);

    return val;
}

void Motor::SetControl(TMap<FString, float> controlSignals)
{
    if (!controlSignals.Contains(TEXT("Value")))
    {
        throw std::runtime_error(
            "Motor '" + std::string(TCHAR_TO_UTF8(*(this->GetName()))) +
            "' is missing the required 'Value' parameter in SetControl().");
    }

    float value = controlSignals[TEXT("Value")];

    if (value < -1 || value > 1)
        throw std::runtime_error(
            "Motor control signal has value '" + std::to_string(value) +
            "', which is outside the expected range of -1 to 1.");

    this->controlSignalSetPoint_ = value;
}

void Motor::SetWheelRadius(const float wheel_radius)
{
    this->radius_ = wheel_radius;
}

void Motor::ControlAbsoluteJointPose(TMap<FString, float> controlSignals)
{
    this->bUseTargetControl = true;
    if (!controlSignals.Contains(TEXT("Value")))
    {
        throw std::runtime_error(
            "Motor '" + std::string(TCHAR_TO_UTF8(*(this->GetName()))) +
            "' is missing the required 'Value' parameter in SetControl().");
    }

    float delta = controlSignals[TEXT("Value")];
    if (delta < 0 || delta > 360)
    {
        delta = delta + 360;
    }
    if (delta > 360)
    {
        delta = delta - 360;
    }
    float rotateDegrees = delta;
    FRotator setRotator = FRotator(0, 0, rotateDegrees);
    this->actuationLink_->GetRootMesh()->WakeAllRigidBodies();
    this->baseLink_->GetRootMesh()->WakeAllRigidBodies();
    this->constraintComponent_->SetAngularOrientationTarget(setRotator);
}

void Motor::ControlRelativeJointPose(TMap<FString, float> controlSignals)
{
    this->bUseTargetControl = true;
    if (!controlSignals.Contains(TEXT("Value")))
    {
        throw std::runtime_error(
            "Motor '" + std::string(TCHAR_TO_UTF8(*(this->GetName()))) +
            "' is missing the required 'Value' parameter in SetControl().");
    }

    float delta = controlSignals[TEXT("Value")];
    if (delta < 0 || delta > 1)
    {
        throw std::runtime_error("Motor control signal has value is outside "
                                 "the expected range of 0 to 1.");
    }
    float rotateDegrees = delta * this->rotationRange;
    FRotator setRotator = FRotator(0, 0, rotateDegrees);
    this->actuationLink_->GetRootMesh()->WakeAllRigidBodies();
    this->baseLink_->GetRootMesh()->WakeAllRigidBodies();
    this->constraintComponent_->SetAngularOrientationTarget(setRotator);
}

void Motor::ComputeForces(float delta)
{
    if (bUseTargetControl)
    {
    }
    else if (!bUseTorqueControl)
    {
        FVector targetAngularVelocity =
            FVector(this->maxVelocity_, 0, 0) * this->controlSignalSetPoint_;
        this->baseLink_->GetRootMesh()->WakeAllRigidBodies();
        this->actuationLink_->GetRootMesh()->WakeAllRigidBodies();
        this->constraintComponent_->SetAngularVelocityTarget(
            targetAngularVelocity);
    }
    else
    {
        this->ConvertTorqueToVelocity();
    }

    ////打印出速度值。
    // FVector  currentVeloctiy;
    // FPhysicsInterface::GetDriveAngularVelocity(constraintComponent_->ConstraintInstance.ConstraintHandle,
    // currentVeloctiy); currentVeloctiy *= this->radius_;
    // GEngine->AddOnScreenDebugMessage(-1, 8, FColor::Green,
    // FString::Printf(TEXT("urdfbot speed: %s "),
    // *currentVeloctiy.ToString()));
}

TMap<FString, FString> Motor::GetState()
{
    TMap<FString, FString> state;
    state.Add(TEXT("SetPoint"),
              FString::SanitizeFloat(this->controlSignalSetPoint_));

    float currentSpeed = this->GetAngularSpeed();
    state.Add(TEXT("ActualPoint"), FString::SanitizeFloat(currentSpeed));

    float ActualSpeed = currentSpeed * this->radius_;
    state.Add(TEXT("ActualSpeed"), FString::SanitizeFloat(ActualSpeed));

    ////debug urdfbot speed
    // GEngine->AddOnScreenDebugMessage(1, 1, FColor::Red,
    // FString::Printf(TEXT("urdfbot speed: %s m/s"),
    // *FString::SanitizeFloat(ActualSpeed)));
    return state;
}

ControlledMotionComponent::MotionComponentType Motor::GetMotionType()
{
    return MotionComponentType::MOTOR;
}

void Motor::EnableDrive(bool isDriveEnabled)
{
    this->constraintComponent_->ConstraintInstance.SetAngularPositionDrive(
        false, isDriveEnabled);
    this->constraintComponent_->ConstraintInstance.SetAngularVelocityDrive(
        false, isDriveEnabled);
}

void Motor::SetDrive(float stiffness, float damping)
{
    float maxForce = constraintComponent_->ConstraintInstance.ProfileInstance
                         .AngularDrive.TwistDrive.MaxForce;
    constraintComponent_->SetAngularDriveParams(stiffness, damping,
                                                stiffness * 100);
    this->constraintComponent_->ConstraintInstance.SetAngularPositionDrive(
        false, true);
    this->constraintComponent_->ConstraintInstance.SetAngularVelocityDrive(
        false, true);
}

void Motor::GetDrive(float* stiffness, float* damping)
{
    *stiffness = this->constraintComponent_->ConstraintInstance.ProfileInstance
                     .AngularDrive.TwistDrive.Stiffness;
    *damping = this->constraintComponent_->ConstraintInstance.ProfileInstance
                   .AngularDrive.TwistDrive.Damping;
}

void Motor::SetDriveTarget(float target)
{
    // target in degree
    this->bUseTargetControl = true;

    this->actuationLink_->GetRootMesh()->WakeAllRigidBodies();
    this->baseLink_->GetRootMesh()->WakeAllRigidBodies();

    //   TMap<FString, float> map;
    // map.Add(TEXT("Value"), target);
    // this->ControlAbsoluteJointPose(map);
    target = target * 180 / PI;
    if (target < 0 or target > 360)
    {
        target = target - 360 * floor(target / 360);
    }
    FRotator setRotator = FRotator(0, 0, target);
    this->constraintComponent_->SetAngularOrientationTarget(setRotator);
}

void Motor::SetDriveTargetVelocity(float target)
{
    this->bUseTargetControl = true;
    FVector setVector = FVector(target, 0, 0);
    this->actuationLink_->GetRootMesh()->WakeAllRigidBodies();
    this->baseLink_->GetRootMesh()->WakeAllRigidBodies();
    this->constraintComponent_->SetAngularVelocityTarget(setVector);
}

float Motor::GetDriveTarget()
{
    FRotator currentTarget =
        this->GetConstraintComponent()
            ->ConstraintInstance.ProfileInstance.AngularDrive.OrientationTarget;
    float q = currentTarget.Roll;
    return q * PI / 180;
}

void Motor::SetConstTorque(const FVector& torque)
{
    this->targetConstTorque_ = torque;
    this->bUseTorqueControl = true;
}

FVector Motor::GetInstantTorque() const
{
    return FVector::ZeroVector;
}

void Motor::ConvertTorqueToVelocity()
{
    float currentSpeed = this->GetAngularSpeed();
    // 除以2* PI 时因为 SetAngularVelocityTarget的参数 为转的圈数
    currentSpeed /= (2 * PI);
    // GEngine->AddOnScreenDebugMessage(1, 1, FColor::Red,
    // FString::Printf(TEXT("urdfbot speed: %s "),
    // *FString::SanitizeFloat(ActualSpeed)));

    /*
       进行实时力矩到targetVelocity的转换,采用公式如下：
       1. force = damping * (targetVelocity  - currentVelocity)
       1. motor初始化的时候已经将 stiffness 设置为 0.
       2. targetVelocity = torque/damping + CurrentVelocity
    */
    float damping = constraintComponent_->ConstraintInstance.ProfileInstance
                        .AngularDrive.TwistDrive.Damping;
    FVector targetVelocity =
        this->targetConstTorque_ / damping + FVector(currentSpeed, 0, 0);
    this->baseLink_->GetRootMesh()->WakeAllRigidBodies();
    this->actuationLink_->GetRootMesh()->WakeAllRigidBodies();
    this->constraintComponent_->SetAngularVelocityTarget(targetVelocity);
}

float Motor::GetAngularSpeed() const
{
    float CurrentSpeed = 0.0f;
    if (this->baseLink_ && this->actuationLink_)
    {
        //根据baselink 和  actualLink来计算 实际运行速度
        FVector targetAngularVelocity =
            this->actuationLink_->GetPhysicsAngularVelocityInRadians();
        FVector baseAngularVelocity =
            this->baseLink_->GetPhysicsAngularVelocityInRadians();
        FVector worldDiffAngularVelocity =
            targetAngularVelocity - baseAngularVelocity;
        FRotator componentRotation =
            this->constraintComponent_->GetComponentRotation();
        FVector localDiffAngularVelocity =
            componentRotation.RotateVector(worldDiffAngularVelocity);
        CurrentSpeed = localDiffAngularVelocity.Size();
    }

    return CurrentSpeed;
}
