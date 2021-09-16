#pragma once
#include "CoreMinimal.h"

#include "UrdfLinkSpecification.h"
#include "UrdfBot/RobotApi.h"

enum UrdfForceSpecificationType
{
    FORCE_ANGULAR = 0,
    FORCE_LINEAR,
    FORCE_ACTUATOR
};

class UrdfForceSpecification
{
public:
    FString Name;
    float Magnitude = 0;
    FString LinkName;
    FVector Axis;
    virtual UrdfForceSpecificationType GetForceSpecificationType() = 0;
    virtual ~UrdfForceSpecification(){};
};
class UrdfAngularForceSpecification : public UrdfForceSpecification
{
public:
    UrdfAngularForceSpecification(){};

    UrdfAngularForceSpecification(const RobotApi::AddAngularForce& alf)
    {
        this->LinkName = FString(alf.link_name.c_str());
        this->Axis = FVector(alf.axis[0], alf.axis[1], alf.axis[2]);
        this->Name = FString(alf.force_name.c_str());
    }

    UrdfForceSpecificationType GetForceSpecificationType()
    {
        return FORCE_ANGULAR;
    }
};

class UrdfLinearForceSpecification : public UrdfForceSpecification
{
public:
    FVector ApplicationPoint;

    UrdfLinearForceSpecification(){};

    UrdfLinearForceSpecification(const RobotApi::AddLinearForce& alf)
    {
        this->LinkName = FString(alf.link_name.c_str());
        this->ApplicationPoint = FVector(alf.axis[0], alf.axis[1], alf.axis[2]);
        this->Axis = FVector(alf.axis[0], alf.axis[1], alf.axis[2]);
        this->Name = FString(alf.force_name.c_str());
    }

    UrdfForceSpecificationType GetForceSpecificationType()
    {
        return FORCE_LINEAR;
    }
};
