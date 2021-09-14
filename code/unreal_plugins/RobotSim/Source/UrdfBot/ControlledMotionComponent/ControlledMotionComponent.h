#pragma once

#include "CoreMinimal.h"

#include "Runtime/Engine/Classes/PhysicsEngine/PhysicsConstraintComponent.h"

#include "RobotBlueprintLib.h"
//#include "UrdfBot/UrdfLink.h"
#include "UrdfBot/UrdfParser/UrdfJointSpecification.h"


class AUrdfLink;
class ControlledMotionComponent
{
    public:
		enum class MotionComponentType
		{
			//运动节点组件的类型
			MOTOR = 0,
			SERVO,
			LINERACTUATOR
		};

        virtual ~ControlledMotionComponent() {};

        virtual FString GetName()
        {
            return this->name_;
        }

        // Inserts a UrdfForceSpecification into the two objects
		virtual void Attach(AUrdfLink* baseLink, AUrdfLink* actuationLink, UrdfJointSpecification *jointSpecification, UPhysicsConstraintComponent* constraintComponent);
		virtual void SetMotionSpeed(const float& liner_speed) = 0;
		virtual float GetMotionSpeed() const { return 0.0; };
		virtual void SetWheelRadius(const float wheel_radius) {};
        virtual void SetControl(TMap<FString, float> controlSignals) = 0;
        virtual void ControlAbsoluteJointPose(TMap<FString, float> controlSignals) = 0;
        virtual void ControlRelativeJointPose(TMap<FString, float> controlSignals) = 0;
		virtual UPhysicsConstraintComponent* GetConstraintComponent();
        virtual TMap<FString, FString> GetState() = 0;
		virtual MotionComponentType   GetMotionType() = 0;
		virtual void SetConstTorque(const FVector&  torque) = 0;
		virtual void SetDrive(float stiffness, float damping) = 0;
		virtual void GetDrive(float* stiffness, float* damping) = 0;
		virtual void SetDriveTarget(float target) = 0;
		virtual float GetDriveTarget() = 0;
		AUrdfLink* GetParentLink();
		AUrdfLink* GetActuatorLink();
		
        // Should be called on each tick
        virtual void ComputeForces(float delta) = 0;
		virtual FVector GetInstantTorque() const = 0;


    protected:
        bool GetFloatFromConfiguration(FString configName, TMap<FString, FString> configuration, float& out, bool throwIfNotExist = false, bool throwIfNonNumeric = false);

        FString name_;
        float worldScale = 100.0f;
        bool isAttached_ = false;
        AUrdfLink* baseLink_;
        AUrdfLink* actuationLink_;
        UrdfJointSpecification *jointSpecification_;
        UPhysicsConstraintComponent* constraintComponent_;
};