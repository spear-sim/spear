#include "Servo.h"

void Servo::Attach(AUrdfLink* baseLink, AUrdfLink* actuationLink, UrdfJointSpecification *jointSpecification, UPhysicsConstraintComponent* constraintComponent)
{
    ControlledMotionComponent::Attach(baseLink, actuationLink, jointSpecification, constraintComponent);

    if (jointSpecification->Type != REVOLUTE_TYPE)
    {
        throw std::runtime_error("Trying to initialize a servo on joint '" + std::string(TCHAR_TO_UTF8(*jointSpecification->Name)) + "', which is not of type revolute.");
    }

    if (jointSpecification->Limit->Upper <= jointSpecification->Limit->Lower)
    {
        std::string jointName = std::string(TCHAR_TO_UTF8(*jointSpecification->Name));
        std::string lower = std::to_string(jointSpecification->Limit->Lower);
        std::string upper = std::to_string(jointSpecification->Limit->Upper);

        throw std::runtime_error("Joint '" + jointName + "' has a lower of '" + lower + "', which is >= the upper of '" + upper + "'.");
    }

    if (jointSpecification->Limit->Lower > 0)
    {
        throw std::runtime_error("Joint '" + std::string(TCHAR_TO_UTF8(*jointSpecification->Name)) + "' has a lower of '" + std::to_string(jointSpecification->Limit->Lower) + "', which is > 0.");
    }

    if (jointSpecification->Limit->Upper < 0)
    {
        throw std::runtime_error("Joint '" + std::string(TCHAR_TO_UTF8(*jointSpecification->Name)) + "' has an upper of '" + std::to_string(jointSpecification->Limit->Upper) + "', which is < 0.");
    }

    this->rotationRange_ = FMath::RadiansToDegrees(jointSpecification->Limit->Upper - jointSpecification->Limit->Lower);
	this->mindegree = FMath::RadiansToDegrees(jointSpecification->Limit->Lower);
	this->maxdegree = FMath::RadiansToDegrees(jointSpecification->Limit->Upper);
    this->invRotationRange_ = 1.0f / this->rotationRange_;
    this->maxRotationRateSpeed_ = jointSpecification->Limit->Velocity;
    this->lower_ = FMath::RadiansToDegrees(jointSpecification->Limit->Lower);

    this->isAttached_ = true;
    this->runOneTick_ = false;
	this->bUseTorqueControl = false; // 默认不使用setTorque控制

	//default stiffness and damping instead of URDF value to optimize movement performance
	float val = 1000000;
	this->SetDrive(val, val / 10);
}

void Servo::SetMotionSpeed(const float& liner_speed)
{
	this->controlSignalSetPoint_ = liner_speed;

	//必须要将bUseTorqueControl  flag设置为false, 以设置用速度控制。（而不是torque） 
	this->bUseTorqueControl = false;
}

float Servo::GetMotionSpeed() const
{
	return this->constraintComponent_->GetComponentVelocity().Size();
}

void Servo::SetControl(TMap<FString, float> controlSignals)
{
	this->bUseTargetControl = false;
    if (!controlSignals.Contains(TEXT("Value")))
    {
        throw std::runtime_error("Servo '" + std::string(TCHAR_TO_UTF8(*(this->GetName()))) + "' is missing the required 'Value' parameter in SetControl().");
    }

    float value = controlSignals[TEXT("Value")];

    if (value < 0 || value > 1)
        throw std::runtime_error("Servo control signal has value '" + std::to_string(value) + "', which is outside the expected range of 0-1.");

    this->controlSignalSetPoint_ = value;
}

void Servo::ControlAbsoluteJointPose(TMap<FString, float> controlSignals)
{
	this->bUseTargetControl = true;

    if (!controlSignals.Contains(TEXT("Value")))
    {
        throw std::runtime_error("Servo '" + std::string(TCHAR_TO_UTF8(*(this->GetName()))) + "' is missing the required 'Value' parameter in SetControl().");
    }

    float delta = controlSignals[TEXT("Value")];
    if(delta < this->mindegree ){
		delta = this->mindegree;
	}
	if (delta > this->maxdegree)
	{
	    delta = this->maxdegree;
	}
	float rotateDegrees = delta;
	FRotator setRotator = FRotator(0, 0, rotateDegrees);
	this->actuationLink_->GetRootMesh()->WakeAllRigidBodies();
	this->baseLink_->GetRootMesh()->WakeAllRigidBodies();
	this->constraintComponent_->SetAngularOrientationTarget(setRotator);
}

void Servo::ControlRelativeJointPose(TMap<FString, float> controlSignals)
{
	this->bUseTargetControl = true;
    if (!controlSignals.Contains(TEXT("Value")))
    {
        throw std::runtime_error("Servo '" + std::string(TCHAR_TO_UTF8(*(this->GetName()))) + "' is missing the required 'Value' parameter in SetControl().");
    }

    float delta = controlSignals[TEXT("Value")];
	if(delta < 0 ){
		delta = 0;
	}
	if (delta > 1)
	{
	    delta = 1;
	}
	float error = (delta - this->actualServoPosition_);
	float maxRotationAmount = this->maxRotationRateSpeed_ * delta;

	//对revolue joint的运动速度进行限制。 
	error = std::max(std::min(error, maxRotationAmount), -1.0f * maxRotationAmount);
	this->actualServoPosition_ += error;

	//lower < 0 ,  upper > 0 ;  必须要讲actualPosition 从 0~1  映射成 -0.5 ~ 0.5 
	float rotateDegrees = (this->actualServoPosition_ * this->rotationRange_) - (0.5 * this->rotationRange_);
	FRotator setRotator = FRotator(0, 0, rotateDegrees);
	this->actuationLink_->GetRootMesh()->WakeAllRigidBodies();
	this->baseLink_->GetRootMesh()->WakeAllRigidBodies();
	this->constraintComponent_->SetAngularOrientationTarget(setRotator);
}

void Servo::ComputeForces(float delta)
{
    if (!this->isAttached_)
        return;

    if (!this->runOneTick_)
    {
		//减去this->lower_， 就是为了初始化讲servo置于中间状态。 
        float noopControlSignal = (0.0f - this->lower_) * this->invRotationRange_;

        if (noopControlSignal < 0.0f)
        {
            throw std::runtime_error("The initial state of the actuator could not be resolved. The actuator is compressed shorter than the minimal allowable distance.");
        }
        else if (noopControlSignal > 1.0f)
        {
            throw std::runtime_error("The initial state of the actuator could not be resolved. The actuator is extended longer than the maximal allowable distance.");
        }

        // Start with no-op. User will set control signal in future frames.
        this->controlSignalSetPoint_ = noopControlSignal;
        this->actualServoPosition_ = noopControlSignal;
        this->runOneTick_ = true;

        return;
    }
    if (bUseTargetControl)
    {

    }
	else if (!bUseTorqueControl)
	{
		float error = (this->controlSignalSetPoint_ - this->actualServoPosition_);
		float maxRotationAmount = this->maxRotationRateSpeed_ * delta;

		//对revolue joint的运动速度进行限制。 
		error = std::max(std::min(error, maxRotationAmount), -1.0f * maxRotationAmount);
		this->actualServoPosition_ += error;

		//lower < 0 ,  upper > 0 ;  必须要讲actualPosition 从 0~1  映射成 -0.5 ~ 0.5 
		float rotateDegrees = (this->actualServoPosition_ * this->rotationRange_) - (0.5 * this->rotationRange_);
		FRotator setRotator = FRotator(0, 0, rotateDegrees);
		this->actuationLink_->GetRootMesh()->WakeAllRigidBodies();
		this->baseLink_->GetRootMesh()->WakeAllRigidBodies();
		this->constraintComponent_->SetAngularOrientationTarget(setRotator);
	}
	else
	{
		this->ConvertTorqueToVelocity();
	}

}

TMap<FString, FString> Servo::GetState()
{
	TMap<FString, FString> state;
	state.Add(TEXT("SetPoint"), FString::SanitizeFloat(this->controlSignalSetPoint_));
 
	float  resultAngle = this->GetServoRadian() * PI;
	state.Add(TEXT("ActualPoint"), FString::SanitizeFloat(resultAngle));

	FVector baseLinkSpeed = this->baseLink_->GetVelocity();
	FVector actuationLinkSpeed = this->actuationLink_->GetVelocity();
	FVector ActualSpeed = actuationLinkSpeed - baseLinkSpeed;
	state.Add(TEXT("ActualSpeed"), FString::SanitizeFloat(ActualSpeed.Size()));

	return state;
}

ControlledMotionComponent::MotionComponentType Servo::GetMotionType()
{
	return ControlledMotionComponent::MotionComponentType::SERVO;
}

void Servo::SetDrive(float stiffness, float damping)
{
	float maxForce = constraintComponent_->ConstraintInstance.ProfileInstance.AngularDrive.TwistDrive.MaxForce;
	constraintComponent_->SetAngularDriveParams(stiffness, damping, stiffness * 100);
	this->constraintComponent_->ConstraintInstance.SetAngularPositionDrive(false, true);
	this->constraintComponent_->ConstraintInstance.SetAngularVelocityDrive(false, true);
}

void Servo::GetDrive(float* stiffness, float* damping)
{
	*stiffness = this->constraintComponent_->ConstraintInstance.ProfileInstance.AngularDrive.TwistDrive.Stiffness;
	*damping = this->constraintComponent_->ConstraintInstance.ProfileInstance.AngularDrive.TwistDrive.Damping;
}

void Servo::SetDriveTarget(float target)
{
	//TODO update ControlAbsoluteJointPose input type?
	this->bUseTargetControl = true;

	//TMap<FString, float> map;
	//map.Add(TEXT("Value"),target);
	//this->ControlAbsoluteJointPose(map);

	this->actuationLink_->GetRootMesh()->WakeAllRigidBodies();
	this->baseLink_->GetRootMesh()->WakeAllRigidBodies();

	float targetDegree = target * 180 / PI;
	if (targetDegree < this->mindegree)
	{
		targetDegree = this->mindegree;
	}
	else if (targetDegree > this->maxdegree)
	{
		targetDegree = this->maxdegree;
	}
	if (this->mindegree + this->maxdegree == 0)
	{
		FRotator setRotator = FRotator(0, 0, target * 180 / PI);
		this->constraintComponent_->SetAngularOrientationTarget(setRotator);
	}
	else
	{
		// adjust asymetric joint
		float actual = (target * 180 / PI - this->lower_) - this->rotationRange_ * 0.5;
		FRotator setRotator = FRotator(0, 0, actual);
		this->constraintComponent_->SetAngularOrientationTarget(setRotator);
	}
}

float Servo::GetDriveTarget()
{
	if (this->mindegree + this->maxdegree == 0)
	{
		FRotator currentTarget = this->GetConstraintComponent()->ConstraintInstance.ProfileInstance.AngularDrive.OrientationTarget;
		float q = currentTarget.Roll;
		return q * PI / 180;
	}
	else
	{
		FRotator currentTarget = this->GetConstraintComponent()->ConstraintInstance.ProfileInstance.AngularDrive.OrientationTarget;
		float q = currentTarget.Roll;
		return (q + this->rotationRange_ * 0.5 + this->lower_) * PI / 180;
	}
}

void Servo::SetConstTorque(const FVector& torque)
{
	this->targetConstTorque_ = torque;
	this->bUseTorqueControl = true;
}

FVector Servo::GetInstantTorque() const
{
	return FVector::ZeroVector;
}

void Servo::ConvertTorqueToVelocity() const
{
	/*
	   进行实时力矩到targetOrientation的转换,采用公式如下：
	   1. force(torque) = stiffness * (targetOrientation  - currentOrientation)
	   1. motor初始化的时候已经将 damping 设置为 0.
	   2. targetOrientation = force(torque) / stiffness + CurrentOrientation;
	*/
	float CurrentDegrees = this->GetServoRadian() * PI ;
	float stiffness = constraintComponent_->ConstraintInstance.ProfileInstance.AngularDrive.TwistDrive.Stiffness;
	float  targetOritation = this->targetConstTorque_.X / stiffness + CurrentDegrees;


	FRotator setRotator = FRotator(0, 0, targetOritation);
	this->actuationLink_->GetRootMesh()->WakeAllRigidBodies();
	this->baseLink_->GetRootMesh()->WakeAllRigidBodies();
	this->constraintComponent_->SetAngularOrientationTarget(setRotator);
}

float Servo::GetServoRadian() const
{
	/*
	ActualPoint 需要获取实时的数值， 不能依赖于 actualServoPosition_的数据
	*/
	float  jointAngle = 0.0;
	FQuat   baselinkQuat = this->baseLink_->GetActorQuat();
	FQuat   actualLinkQuat = this->actuationLink_->GetActorQuat();
	FVector  baseLinkVec = baselinkQuat.RotateVector(FVector(1.0, 0.0, 0.0));
	FVector  actualLinkVec = actualLinkQuat.RotateVector(FVector(1.0, 0.0, 0.0));
	float  result = FVector::DotProduct(baseLinkVec, actualLinkVec);
	if (result > 0)
	{
		jointAngle = acos(result);
	}
	else
	{
		jointAngle = acos(result) + PI;
	}
	return jointAngle;
}

