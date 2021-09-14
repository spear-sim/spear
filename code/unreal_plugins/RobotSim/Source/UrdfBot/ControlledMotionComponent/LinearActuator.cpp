#include "LinearActuator.h"

void LinearActuator::Attach(AUrdfLink* baseLink,
                            AUrdfLink* actuationLink,
                            UrdfJointSpecification* jointSpecification,
                            UPhysicsConstraintComponent* constraintComponent)
{
    this->mindegree = jointSpecification->Limit->Lower;
    this->maxdegree = jointSpecification->Limit->Upper;
    ControlledMotionComponent::Attach(baseLink, actuationLink,
                                      jointSpecification, constraintComponent);

    if (jointSpecification->Type != PRISMATIC_TYPE)
    {
        throw std::runtime_error(
            "Attempting to attach linear actuator to joint '" +
            std::string(TCHAR_TO_UTF8(*jointSpecification->Name)) +
            "', which is not of type prismatic.");
    }

    if (jointSpecification->Limit->Upper <= jointSpecification->Limit->Lower)
    {
        std::string jointName =
            std::string(TCHAR_TO_UTF8(*jointSpecification->Name));
        std::string lower = std::to_string(jointSpecification->Limit->Lower);
        std::string upper = std::to_string(jointSpecification->Limit->Upper);

        throw std::runtime_error("Joint '" + jointName + "' has a lower of '" +
                                 lower + "', which is >= the upper of '" +
                                 upper + "'.");
    }

    this->range_ =
        (jointSpecification->Limit->Upper - jointSpecification->Limit->Lower) *
        this->worldScale;
    this->invRange_ = 1.0f / this->range_;
    this->maxActuationRate_ = jointSpecification->Limit->Velocity;

    // Compute min and max transform points
    FVector parentLocation = this->baseLink_->GetActorLocation();
    FRotator parentRotation = this->baseLink_->GetActorRotation();

    //获取joint 中心位置， 即为linearActual的中间状态。
    FVector jointCenterLocation = constraintComponent->GetComponentLocation();
    FVector actuationAxis =
        parentRotation.RotateVector(jointSpecification->Axis);

    if (!actuationAxis.Normalize())
    {
        throw std::runtime_error(
            "Cannot normalize joint axis in Linear Actuator.");
    }

    // lower < 0 ,  upper > 0 ;  必须要讲actualPosition 从 0~1  映射成 -0.5 ~
    // 0.5
    FVector actuatorOne =
        jointCenterLocation + (0.5 * actuationAxis * this->range_);
    FVector actuatorZero =
        jointCenterLocation - (0.5 * actuationAxis * this->range_);

    // record zero position
    this->baseToActuatorZero_ = actuatorZero - parentLocation;
    this->baseToActuatorOne_ = actuatorOne - parentLocation;
    this->lower_ = jointSpecification->Limit->Lower;

    this->isAttached_ = true;
    this->runOneTick_ = false;
    float a = this->constraintComponent_->ConstraintInstance.ProfileInstance
                  .LinearDrive.XDrive.Stiffness;
    float b = this->constraintComponent_->ConstraintInstance.ProfileInstance
                  .LinearDrive.XDrive.Damping;
    // default stiffness and damping instead of URDF value to optimize movement
    // performance
    URobotBlueprintLib::LogMessage(FString("prismatic") + this->name_,
                                   FString::SanitizeFloat(a) + " " +
                                       FString::SanitizeFloat(b),
                                   LogDebugLevel::Failure, 30);
    if (this->name_ == "l_gripper_finger_joint" or
        this->name_ == "r_gripper_finger_joint")
    {
        this->SetDrive(60000, 5);
    }
    else
    {
        float val = 1000000;
        this->SetDrive(val, val / 10);
    }
}

void LinearActuator::SetControl(TMap<FString, float> controlSignals)
{
    if (!controlSignals.Contains(TEXT("Value")))
    {
        throw std::runtime_error(
            "LinearActuator '" +
            std::string(TCHAR_TO_UTF8(*(this->GetName()))) +
            "' is missing the required 'Value' parameter in SetControl().");
    }

    float value = controlSignals[TEXT("Value")];

    if (value < 0 || value > 1)
        throw std::runtime_error(
            "LinearActuator control signal has value '" +
            std::to_string(value) +
            "', which is outside the expected range of 0-1.");

    this->controlSignalSetPoint_ = value;
}

void LinearActuator::ControlAbsoluteJointPose(
    TMap<FString, float> controlSignals)
{
    FVector basePosition = this->baseLink_->GetActorLocation();
    FVector actuatorPosition = this->actuationLink_->GetActorLocation();
    FVector actuatorZero = basePosition + this->baseToActuatorZero_;
    FVector actuatorOne = basePosition + this->baseToActuatorOne_;

    if (!controlSignals.Contains(TEXT("Value")))
    {
        throw std::runtime_error(
            "Servo '" + std::string(TCHAR_TO_UTF8(*(this->GetName()))) +
            "' is missing the required 'Value' parameter in SetControl().");
    }

    float delta = controlSignals[TEXT("Value")];
    if (delta < 0 || delta > 1)
    {
        throw std::runtime_error("Servo control signal has value is outside "
                                 "the expected range of 0-1.");
    }

    FVector zeroToOne = (actuatorOne - actuatorZero);
    float error = delta - this->actualActuatorPosition_;
    float maxActuationAmount = this->maxActuationRate_ * delta;

    error = std::max(std::min(error, maxActuationAmount),
                     -1.0f * maxActuationAmount);
    this->actualActuatorPosition_ += error;

    FVector setVec = zeroToOne * (this->actualActuatorPosition_ - 0.5f);
    // FVector setVec = zeroToOne * this->actualActuatorPosition_;
    FVector ss =
        this->jointSpecification_->Axis.Rotation().RotateVector(setVec);
    this->actuationLink_->GetRootMesh()->WakeAllRigidBodies();
    this->baseLink_->GetRootMesh()->WakeAllRigidBodies();
    this->constraintComponent_->SetLinearPositionTarget(ss);
}

void LinearActuator::ControlRelativeJointPose(
    TMap<FString, float> controlSignals)
{
    FVector basePosition = this->baseLink_->GetActorLocation();
    FVector actuatorPosition = this->actuationLink_->GetActorLocation();
    FVector actuatorZero = basePosition + this->baseToActuatorZero_;
    FVector actuatorOne = basePosition + this->baseToActuatorOne_;

    if (!controlSignals.Contains(TEXT("Value")))
    {
        throw std::runtime_error(
            "Servo '" + std::string(TCHAR_TO_UTF8(*(this->GetName()))) +
            "' is missing the required 'Value' parameter in SetControl().");
    }

    float delta = controlSignals[TEXT("Value")];
    if (delta < 0 || delta > 1)
    {
        throw std::runtime_error("Servo control signal has value is outside "
                                 "the expected range of 0-1.");
    }

    FVector zeroToOne = (actuatorOne - actuatorZero);
    float error = delta - this->actualActuatorPosition_;
    float maxActuationAmount = this->maxActuationRate_ * delta;

    error = std::max(std::min(error, maxActuationAmount),
                     -1.0f * maxActuationAmount);
    this->actualActuatorPosition_ += error;

    FVector setVec = zeroToOne * (this->actualActuatorPosition_ - 0.5f);
    // FVector setVec = zeroToOne * this->actualActuatorPosition_;
    FVector ss =
        this->jointSpecification_->Axis.Rotation().RotateVector(setVec);
    this->actuationLink_->GetRootMesh()->WakeAllRigidBodies();
    this->baseLink_->GetRootMesh()->WakeAllRigidBodies();
    this->constraintComponent_->SetLinearPositionTarget(ss);
}

void LinearActuator::SetMotionSpeed(const float& liner_speed)
{
    this->controlSignalSetPoint_ = liner_speed;
    //必须要将bUseTorqueControl  flag设置为false,
    //以设置用速度控制。（而不是torque）
    this->bUseTorqueControl = false;
}

float LinearActuator::GetMotionSpeed() const
{
    return this->actuationLink_->GetPhysicsLinearVelocity().Size();
}

void LinearActuator::ComputeForces(float delta)
{
    if (!this->isAttached_)
        return;

    // Get the min and max points in world coordinates
    FVector basePosition = this->baseLink_->GetActorLocation();
    FVector actuatorPosition = this->actuationLink_->GetActorLocation();
    FVector actuatorZero = basePosition + this->baseToActuatorZero_;
    FVector actuatorOne = basePosition + this->baseToActuatorOne_;

    // For the first tick, initialize the clock and initial position
    if (!this->runOneTick_)
    {
        float actualDistance =
            FVector::Distance(actuatorPosition, actuatorZero);
        UE_LOG(LogTemp, Warning, TEXT("actualDistance:%f"), actualDistance);
        float noopControlSignal = actualDistance * this->invRange_;
        UE_LOG(LogTemp, Warning, TEXT("noopControlSignal:%f"),
               noopControlSignal);
        if (noopControlSignal < 0.0f)
        {
            throw std::runtime_error(
                "The initial state of the actuator could not be resolved. The "
                "actuator is compressed shorter than the minimal allowable "
                "distance.");
        }
        else if (noopControlSignal > 1.0f)
        {
            noopControlSignal = 1.0f;
            // throw std::runtime_error("The initial state of the actuator could
            // not be resolved. The actuator is extended longer than the maximal
            // allowable distance.");
        }

        //// Start with no-op. User will set control signal in future frames.
        // this->controlSignalSetPoint_ = noopControlSignal;
        // this->actualActuatorPosition_ = noopControlSignal;
        this->controlSignalSetPoint_ = 0.0;
        this->actualActuatorPosition_ = 0.0;
        // reset to origin
        this->SetDriveTarget(0);
        this->runOneTick_ = true;
        return;
    }
    if (bUseTargetControl)
    {
    }
    else if (!bUseTorqueControl)
    {
        // FVector actuatorOneHalf = (actuatorZero + actuatorOne) * 0.5f;
        // float setPointDistance = this->controlSignalSetPoint_ * this->range_;
        FVector zeroToOne = (actuatorOne - actuatorZero);
        float error =
            this->controlSignalSetPoint_ - this->actualActuatorPosition_;
        float maxActuationAmount = this->maxActuationRate_ * delta;

        error = std::max(std::min(error, maxActuationAmount),
                         -1.0f * maxActuationAmount);
        this->actualActuatorPosition_ += error;

        FVector setVec = zeroToOne * (this->actualActuatorPosition_ - 0.5f);
        // FVector setVec = zeroToOne * this->actualActuatorPosition_;
        FVector ss =
            this->jointSpecification_->Axis.Rotation().RotateVector(setVec);
        this->actuationLink_->GetRootMesh()->WakeAllRigidBodies();
        this->baseLink_->GetRootMesh()->WakeAllRigidBodies();
        this->constraintComponent_->SetLinearPositionTarget(ss);
    }
    else
    {
        this->ConvertTorqueToVelocity();
    }
}

TMap<FString, FString> LinearActuator::GetState()
{
    TMap<FString, FString> state;
    state.Add(TEXT("SetPoint"),
              FString::SanitizeFloat(this->controlSignalSetPoint_));

    float actuatorPoint = this->GetlinearDistance() * this->invRange_;
    state.Add(
        TEXT("ActualPoint"),
        FString::SanitizeFloat(actuatorPoint)); // convert back to 0-1 range

    FVector baseLinkSpeed = this->baseLink_->GetPhysicsLinearVelocity();
    FVector actuationLinkSpeed =
        this->actuationLink_->GetPhysicsLinearVelocity();
    FVector ActualSpeed = actuationLinkSpeed - baseLinkSpeed;
    state.Add(TEXT("ActualSpeed"), FString::SanitizeFloat(ActualSpeed.Size()));

    return state;
}

ControlledMotionComponent::MotionComponentType LinearActuator::GetMotionType()
{
    return ControlledMotionComponent::MotionComponentType::LINERACTUATOR;
}

void LinearActuator::SetConstTorque(const FVector& torque)
{
    this->targetConstTorque_ = torque;
    this->bUseTorqueControl = true;
}

void LinearActuator::SetDrive(float stiffness, float damping)
{
    this->bUseTargetControl = true;
    constraintComponent_->SetLinearDriveParams(stiffness, damping,
                                               stiffness * 100);
    this->constraintComponent_->ConstraintInstance.SetLinearPositionDrive(
        true, false, false);
    this->constraintComponent_->ConstraintInstance.SetLinearVelocityDrive(
        true, false, false);
}

void LinearActuator::GetDrive(float* stiffness, float* damping)
{
    *stiffness = this->constraintComponent_->ConstraintInstance.ProfileInstance
                     .LinearDrive.XDrive.Stiffness;
    *damping = this->constraintComponent_->ConstraintInstance.ProfileInstance
                   .LinearDrive.XDrive.Damping;
}

void LinearActuator::SetDriveTarget(float target)
{
    this->bUseTargetControl = true;
    if (target < this->mindegree)
    {
        target = this->mindegree;
    }
    if (target > this->maxdegree)
    {
        target = this->maxdegree;
    }

    this->actuationLink_->GetRootMesh()->WakeAllRigidBodies();
    this->baseLink_->GetRootMesh()->WakeAllRigidBodies();

    float actual =
        0.5 * this->range_ - (target - this->mindegree) * this->worldScale;

    FVector setVector = FVector(actual, 0, 0);
    this->constraintComponent_->SetLinearPositionTarget(setVector);
}

float LinearActuator::GetDriveTarget()
{
    UPhysicsConstraintComponent* constraintComponent =
        this->GetConstraintComponent();
    FVector currentTarget = constraintComponent->ConstraintInstance
                                .ProfileInstance.LinearDrive.PositionTarget;
    return (0.5 * this->range_ - currentTarget.X) / this->worldScale +
           this->mindegree;
}

FVector LinearActuator::GetInstantTorque() const
{
    return FVector::ZeroVector;
}

void LinearActuator::ConvertTorqueToVelocity() const
{
    /*
       进行实时力矩到targetOrientation的转换,采用公式如下：
       1. force(torque) = stiffness * (targetOrientation  - currentOrientation)
       1. motor初始化的时候已经将 damping 设置为 0.
       2. targetOrientation = force(torque) / stiffness + CurrentOrientation;
    */
    float CurrentLinear = this->GetlinearDistance();
    float stiffness = constraintComponent_->ConstraintInstance.ProfileInstance
                          .AngularDrive.TwistDrive.Stiffness;
    float targetPostition =
        this->targetConstTorque_.X / stiffness + CurrentLinear;

    FVector zeroToOne = this->baseToActuatorOne_ - this->baseToActuatorZero_;
    FVector setVec = zeroToOne * (targetPostition - 0.5f);
    FVector ss =
        this->jointSpecification_->Axis.Rotation().RotateVector(setVec);
    this->actuationLink_->GetRootMesh()->WakeAllRigidBodies();
    this->baseLink_->GetRootMesh()->WakeAllRigidBodies();
    this->constraintComponent_->SetLinearPositionTarget(ss);
}

float LinearActuator::GetlinearDistance() const
{
    // Actually query the position rather than use cached values, as cached
    // values may be incorrect.
    FVector basePosition = this->baseLink_->GetActorLocation();
    FVector actuatorPosition = this->actuationLink_->GetActorLocation();
    FVector actuatorZero = basePosition + this->baseToActuatorZero_;
    float actualDistance = FVector::Distance(actuatorPosition, actuatorZero);
    float actuatorPoint = actualDistance - this->lower_;

    return actuatorPoint;
}
