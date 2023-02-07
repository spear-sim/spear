//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "UrdfBot/UrdfJointComponent.h"

#include <iostream>

#include <PhysicsEngine/PhysicsConstraintComponent.h>

#include "CoreUtils/Assert.h"
#include "UrdfBot/UrdfLinkComponent.h"
#include "UrdfBot/UrdfParser.h"

void UUrdfJointComponent::initializeComponent(UrdfJointDesc* joint_desc, UUrdfLinkComponent* parent_link_component, UUrdfLinkComponent* child_link_component)
{
    ASSERT(joint_desc);
    ASSERT(parent_link_component);
    ASSERT(child_link_component);

    joint_type_ = joint_desc->type_;

    ConstraintInstance.ProfileInstance.ConeLimit.Swing1Motion = EAngularConstraintMotion::ACM_Locked;
    ConstraintInstance.ProfileInstance.ConeLimit.Swing2Motion = EAngularConstraintMotion::ACM_Locked;
    ConstraintInstance.ProfileInstance.TwistLimit.TwistMotion = EAngularConstraintMotion::ACM_Locked;
    ConstraintInstance.ProfileInstance.AngularDrive.AngularDriveMode = EAngularDriveMode::TwistAndSwing;
    ConstraintInstance.ProfileInstance.ConeLimit.bSoftConstraint = true;
    ConstraintInstance.ProfileInstance.TwistLimit.bSoftConstraint = false;
    ConstraintInstance.UpdateAngularLimit();

    float m_to_cm = 100.0f;

    SetRelativeLocation(joint_desc->origin_.GetLocation() * m_to_cm);
    SetRelativeRotation(FRotationMatrix::MakeFromX(joint_desc->origin_.GetRotation().Rotator().RotateVector(joint_desc->axis_)).Rotator());

    switch (joint_desc->type_) {
    case UrdfJointType::Revolute:
        {
            float twist_limit_angle = FMath::RadiansToDegrees(joint_desc->upper_ - joint_desc->lower_) * 0.5f;
            float spring = FMath::RadiansToDegrees(joint_desc->damping_) * m_to_cm * 10.0f;
            float damping = FMath::RadiansToDegrees(joint_desc->damping_) * m_to_cm;
            float force_limit = FMath::RadiansToDegrees(joint_desc->effort_) * m_to_cm * 10.0f;

            // use twist degree for single angular freedom with best PhysX optimization
            ConstraintInstance.SetAngularTwistLimit(EAngularConstraintMotion::ACM_Limited, twist_limit_angle);
            ConstraintInstance.SetOrientationDriveTwistAndSwing(true, false);
            ConstraintInstance.SetAngularVelocityDriveTwistAndSwing(true, false);
            ConstraintInstance.SetAngularDriveParams(spring, damping, force_limit);
            break;
        }
    case UrdfJointType::Continuous:
        {
            float spring = FMath::RadiansToDegrees(joint_desc->damping_) * m_to_cm * 10.0f;
            float damping = FMath::RadiansToDegrees(joint_desc->damping_) * m_to_cm;
            float force_limit = FMath::RadiansToDegrees(joint_desc->effort_) * m_to_cm * 10.0f;

            // use twist degree for single angular freedom with best PhysX optimization
            ConstraintInstance.SetAngularTwistLimit(EAngularConstraintMotion::ACM_Free, 0.0f);
            ConstraintInstance.SetOrientationDriveTwistAndSwing(true, false);
            ConstraintInstance.SetAngularVelocityDriveTwistAndSwing(true, false);
            ConstraintInstance.SetAngularDriveParams(spring, damping, force_limit);
            break;
        }
    case UrdfJointType::Prismatic:
        {
            float linear_limit_size = (joint_desc->upper_ - joint_desc->lower_) * m_to_cm;
            float spring = joint_desc->damping_ * m_to_cm * m_to_cm * 10.0f;
            float damping = joint_desc->damping_ * m_to_cm * m_to_cm;
            float force_limit = joint_desc->effort_ * m_to_cm * m_to_cm;

            ConstraintInstance.SetLinearXLimit(ELinearConstraintMotion::LCM_Limited, linear_limit_size);
            ConstraintInstance.SetLinearDriveParams(spring, damping, force_limit);
            ConstraintInstance.SetLinearPositionDrive(true, false, false);
            break;
        }
    case UrdfJointType::Fixed:
        break;
    case UrdfJointType::Floating:
        {
            ConstraintInstance.SetLinearXLimit(ELinearConstraintMotion::LCM_Free, 0.0f);
            ConstraintInstance.SetLinearYLimit(ELinearConstraintMotion::LCM_Free, 0.0f);
            ConstraintInstance.SetLinearZLimit(ELinearConstraintMotion::LCM_Free, 0.0f);
            ConstraintInstance.SetAngularTwistLimit(EAngularConstraintMotion::ACM_Free, 0.0f);
            ConstraintInstance.SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Free, 0.0f);
            ConstraintInstance.SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Free, 0.0f);
            break;
        }
    case UrdfJointType::Planar:
        {
            float linear_limit_size = (joint_desc->upper_ - joint_desc->lower_) * m_to_cm;

            ConstraintInstance.SetLinearXLimit(ELinearConstraintMotion::LCM_Limited, linear_limit_size);
            ConstraintInstance.SetLinearYLimit(ELinearConstraintMotion::LCM_Limited, linear_limit_size);
            break;
        }
    default:
        ASSERT(false);
        break;
    }

    SetDisableCollision(true);
    SetConstrainedComponents(parent_link_component, NAME_None, child_link_component, NAME_None);
}
