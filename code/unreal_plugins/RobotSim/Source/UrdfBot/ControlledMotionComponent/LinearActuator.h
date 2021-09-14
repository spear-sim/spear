#pragma once

#include "CoreMinimal.h"

#include "Runtime/Engine/Classes/PhysicsEngine/PhysicsConstraintComponent.h"

#include "ControlledMotionComponent.h"
#include "UrdfBot/UrdfLink.h"
#include "UrdfBot/UrdfParser/UrdfJointSpecification.h"

class LinearActuator : public ControlledMotionComponent
{
    public:
        virtual ~LinearActuator() override {};

        virtual void Attach(AUrdfLink* baseLink, AUrdfLink* actuationLink, UrdfJointSpecification *jointSpecification, UPhysicsConstraintComponent* constraintComponent) override;
        virtual void SetControl(TMap<FString, float> controlSignals) override;
        virtual void ControlRelativeJointPose(TMap<FString, float> controlSignals) override;
        virtual void ControlAbsoluteJointPose(TMap<FString, float> controlSignals) override;
		virtual void SetMotionSpeed(const float& liner_speed)  override;
		virtual float GetMotionSpeed() const override;
        virtual void ComputeForces(float delta) override;
        virtual TMap<FString, FString> GetState() override;
		virtual ControlledMotionComponent::MotionComponentType  GetMotionType()  override;
		virtual void SetConstTorque(const FVector& torque) override;
        virtual void SetDrive(float stiffness, float damping) override;
		virtual void GetDrive(float* stiffness, float* damping) override;
        virtual void SetDriveTarget(float target) override;
		virtual float GetDriveTarget() override;
		virtual FVector GetInstantTorque() const override;


private:
	void ConvertTorqueToVelocity() const;
	float GetlinearDistance() const ;

    private:
		FVector targetConstTorque_;
		FVector baseToActuatorZero_;
		FVector baseToActuatorOne_;

        float maxActuationRate_ = 1.0f;
        float deadZone_ = 0.05f;
        float controlSignalSetPoint_ = 0;
        float actualActuatorPosition_ = 0;
        float invRange_ = 1;
        float range_ = 0;
        float lower_ = 0;
        float mindegree = 0;
        float maxdegree = 0;
        bool runOneTick_ = false;
		bool  bUseTorqueControl; //defalut is using velocity contorl .
	    bool bUseTargetControl = true; //true-using drive_target control
};