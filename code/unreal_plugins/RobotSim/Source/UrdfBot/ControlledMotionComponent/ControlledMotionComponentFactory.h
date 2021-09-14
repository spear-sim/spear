#pragma once

#include "common_utils/Settings.hpp"
#include "ControlledMotionComponent.h"
#include "LinearActuator.h"
#include "Servo.h"
#include "Motor.h"

class ControlledMotionComponentFactory
{
public:
    static ControlledMotionComponent* CreateControlledMotionComponent(
        AUrdfLink* baseLink,
        AUrdfLink* actuatedLink,
        UrdfJointSpecification* jointSpecification,
        UPhysicsConstraintComponent* constraintComponent)
    {
        ControlledMotionComponent* motionComponent = nullptr;
        if (jointSpecification->Type == PRISMATIC_TYPE)
        {
            motionComponent = new LinearActuator();
        }
        else if (jointSpecification->Type == REVOLUTE_TYPE)
        {
            motionComponent = new Servo();
        }
        else if (jointSpecification->Type == CONTINUOUS_TYPE)
        {
            motionComponent = new Motor();
        }
        else
        {
            throw std::runtime_error(
                "Unsupported joint type for joint '" +
                std::string(TCHAR_TO_UTF8(*jointSpecification->Name)) +
                "'. Cannot create controlled motion component.");
        }

        motionComponent->Attach(baseLink, actuatedLink, jointSpecification,
                                constraintComponent);

        return motionComponent;
    }
};
