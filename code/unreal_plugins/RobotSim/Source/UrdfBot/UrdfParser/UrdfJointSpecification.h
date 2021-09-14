#pragma once

#include "CoreMinimal.h"
#include "UrdfOrigin.h"
#include "UrdfLinkSpecification.h"

enum UrdfJointType
{
    // Some of these (such as FIXED) are already macros
    REVOLUTE_TYPE = 0,
    CONTINUOUS_TYPE,
    PRISMATIC_TYPE,
    FIXED_TYPE,
    FLOATING_TYPE,
    PLANAR_TYPE
};

class UrdfJointSpecification;

class UrdfJointCalibrationSpecification
{
    public:
        float Rising = std::numeric_limits<float>::quiet_NaN();
        float Falling = std::numeric_limits<float>::quiet_NaN();
};

class UrdfJointDynamicsSpecification
{
    public:
        float Damping = 0;
        float Friction = 0;
};

class UrdfJointLimitSpecification
{
    public:
        float Lower = 0;
        float Upper = 0;
        float Effort;
        float Velocity;
};

class UrdfJointMimicSpecification
{
    public: 
        FString JointName;
        UrdfJointSpecification* Joint;
        float Multiplier = 1;
        float Offset = 0;
};

class UrdfJointSafetyControllerSpecification
{
    public:
        float SoftLowerLimit = 0;
        float SoftUpperLimit = 0;
        float KPosition = 0;
        float KVelocity;
};

class UrdfJointSpecification
{
    public:
        FString Name;
        UrdfJointType Type;
        UrdfOrigin Origin;

        FString ParentLinkName;
        UrdfLinkSpecification* ParentLinkSpecification;
        FString ChildLinkName;
        UrdfLinkSpecification* ChildLinkSpecification;

        FRotator RollPitchYaw;
        FVector Axis;
        UrdfJointCalibrationSpecification* Calibration = nullptr;
        UrdfJointDynamicsSpecification Dynamics;
        UrdfJointLimitSpecification* Limit = nullptr;
        UrdfJointMimicSpecification* Mimic = nullptr;
        UrdfJointSafetyControllerSpecification* SafetyController = nullptr;

        static UrdfJointType ParseJointType(FString type)
        {
            if (type.Equals(TEXT("revolute")))
                return REVOLUTE_TYPE;
            if (type.Equals(TEXT("continuous")))
                return CONTINUOUS_TYPE;
            if (type.Equals(TEXT("prismatic")))
                return PRISMATIC_TYPE;
            if (type.Equals(TEXT("fixed")))
                return FIXED_TYPE;
            if (type.Equals(TEXT("floating")))
                return FLOATING_TYPE;
            if (type.Equals(TEXT("planar")))
                return PLANAR_TYPE;

            throw std::runtime_error("Unable to parse joint type of " + std::string(TCHAR_TO_UTF8(*type)) + ".");
        }
};
