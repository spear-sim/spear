#pragma once

#include "CoreMinimal.h"

#include "Runtime/Engine/Classes/PhysicsEngine/PhysicsConstraintComponent.h"

#include "common_utils/Utils.hpp"
#include "ControlledMotionComponent.h"
#include "UrdfBot/UrdfLink.h"
#include "UrdfBot/UrdfParser/UrdfJointSpecification.h"

class Servo : public ControlledMotionComponent
{
public:
    virtual ~Servo() override {};

    virtual void Attach(AUrdfLink* baseLink, AUrdfLink* actuationLink, UrdfJointSpecification *jointSpecification, UPhysicsConstraintComponent* constraintComponent) override;
	//Todo list 
	//此函数并非直接设置运动组件速度值， 是通过转化关系来间接设置速度。
	virtual void SetMotionSpeed(const float& liner_speed) override;
	virtual float GetMotionSpeed() const override;
    virtual void SetControl(TMap<FString, float> controlSignals) override;
    virtual void ComputeForces(float delta) override;
    virtual void ControlAbsoluteJointPose(TMap<FString, float> controlSignals) override;
    virtual void ControlRelativeJointPose(TMap<FString, float> controlSignals) override;
    virtual TMap<FString, FString> GetState() override;
	virtual ControlledMotionComponent::MotionComponentType  GetMotionType() override;
	virtual void SetConstTorque(const FVector& torque) override;
    virtual void SetDrive(float stiffness, float damping) override;
	virtual void GetDrive(float* stiffness, float* damping) override;
    virtual void SetDriveTarget(float target) override;
	virtual float GetDriveTarget() override;
	virtual FVector GetInstantTorque()  const override;

private :
	void ConvertTorqueToVelocity() const;
	float GetServoRadian() const;


private:
	FVector targetConstTorque_;

    float controlSignalSetPoint_ = 0.0f;
    float actualServoPosition_ = 0.0f;

    float maxRotationRateSpeed_ = 0.0f;
    float rotationRange_ = 0.0f;
    float deadZone_ = 0.05f;
    float invRotationRange_ = 1.0f;
    float mindegree = 0.0f;
    float maxdegree = 0.0f;
    float lower_ = 0.0f;
    bool  runOneTick_ = false;
	bool  bUseTorqueControl; //defalut is using velocity, which is false.
    bool  bUseTargetControl = true;   //whether use target control directly and override control signal control

};