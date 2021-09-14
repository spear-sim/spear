#pragma once

#include "CoreMinimal.h"
#include "Runtime/Engine/Classes/PhysicsEngine/PhysicsConstraintComponent.h"

#include "common_utils/Utils.hpp"

#include "./ControlledMotionComponent.h"
#include "./UrdfBot/UrdfLink.h"
#include "./UrdfBot/UrdfParser/UrdfJointSpecification.h"


class Motor : public ControlledMotionComponent
{
public:
    virtual ~Motor() override {};

    virtual void Attach(AUrdfLink* baseLink, AUrdfLink* actuationLink, UrdfJointSpecification *jointSpecification, UPhysicsConstraintComponent* constraintComponent) override;
	/*
		此函数由baselink和actuallink的速度插值来获取motor的瞬时运动速度。
	*/
	virtual float GetMotionSpeed() const override;
	//Todo list 
	//此函数并非直接设置运动组件速度值， 是通过转化关系来间接设置速度。
	virtual void SetMotionSpeed(const float& liner_speed) override;
    virtual void SetControl(TMap<FString, float> controlSignals) override;
	virtual void SetWheelRadius(const float wheel_radius) override;
    virtual void ComputeForces(float delta) override;
	virtual void ControlAbsoluteJointPose(TMap<FString, float> controlSignals) override;
	virtual void ControlRelativeJointPose(TMap<FString, float> controlSignals) override;
    virtual TMap<FString, FString> GetState() override;
	virtual ControlledMotionComponent::MotionComponentType GetMotionType() override;
	virtual void SetConstTorque(const FVector& torque) override;
	void EnableDrive(bool isDriveEnabled);
    virtual void SetDrive(float stiffness, float damping) override;
	virtual void GetDrive(float* stiffness, float* damping) override;
    virtual void SetDriveTarget(float target) override;
	virtual float GetDriveTarget() override;
	// target velocity control, currently only valid for continuous joint. TODO implement this to all joint type?
	virtual void SetDriveTargetVelocity(float target);
	virtual FVector GetInstantTorque() const override;

private:
	void ConvertTorqueToVelocity(); 
	/*
		return value:  radain 
	*/
	float  GetAngularSpeed()  const;

private:
	FVector rotationAxis_;
	FVector targetConstTorque_;
    float   controlSignalSetPoint_ = 0;
    float   actualMotorSpeed_ = 0;
	float   actualMotorPosition = 0.0f;
	float   radius_;
    float   maxVelocity_ = 0.0f;
    float   deadZone_ = 0.05f;
	float   rotationRange = 360;
	bool	bUseTorqueControl; //defalut is using velocity, which is false.
    bool    bUseTargetControl = true;   //whether use target control directly and override control signal control
};