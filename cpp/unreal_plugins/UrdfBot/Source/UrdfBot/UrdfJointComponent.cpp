//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "UrdfJointComponent.h"

#include <iostream>

#include <PhysicsEngine/PhysicsConstraintComponent.h>

#include "CoreUtils/Assert.h"
#include "UrdfLinkComponent.h"
#include "UrdfParser.h"

void UUrdfJointComponent::init(const UrdfJointDesc& joint_desc_)
{
    this->joint_type_ = joint_desc_.type_;

    ConstraintInstance.SetDisableCollision(true);

    ConstraintInstance.ProfileInstance.ConeLimit.Swing1Motion = EAngularConstraintMotion::ACM_Locked;
    ConstraintInstance.ProfileInstance.ConeLimit.Swing2Motion = EAngularConstraintMotion::ACM_Locked;
    ConstraintInstance.ProfileInstance.TwistLimit.TwistMotion = EAngularConstraintMotion::ACM_Locked;
    ConstraintInstance.UpdateAngularLimit();

    ConstraintInstance.ProfileInstance.AngularDrive.AngularDriveMode = EAngularDriveMode::TwistAndSwing;
    ConstraintInstance.ProfileInstance.ConeLimit.bSoftConstraint = true;
    ConstraintInstance.ProfileInstance.TwistLimit.bSoftConstraint = false;

    float range = 0.0f;
    switch (joint_desc_.type_) {
    case UrdfJointType::Revolute:
        // use twist degree for single angular freedom with best PhysX optimization
        range = FMath::RadiansToDegrees(joint_desc_.upper_ - joint_desc_.lower_) * 0.5f;
        ConstraintInstance.SetAngularTwistLimit(EAngularConstraintMotion::ACM_Limited, range);
        ConstraintInstance.SetOrientationDriveTwistAndSwing(true, false);
        ConstraintInstance.SetAngularVelocityDriveTwistAndSwing(true, false);
        ConstraintInstance.SetAngularDriveParams(1e6, 1e4, 1e6); // TODO determine drive strength with urdf value
        break;
    case UrdfJointType::Continuous:
        // use twist degree for single angular freedom with best PhysX optimization
        ConstraintInstance.SetAngularTwistLimit(EAngularConstraintMotion::ACM_Free, 0.0f);
        ConstraintInstance.SetOrientationDriveTwistAndSwing(true, false);
        ConstraintInstance.SetAngularVelocityDriveTwistAndSwing(true, false);
        ConstraintInstance.SetAngularDriveParams(1e6, 1e4, 1e6); // TODO determine drive strength with urdf value
        break;
    case UrdfJointType::Prismatic:
        range = (joint_desc_.upper_ - joint_desc_.lower_) * unit_conversion_m_to_cm * 0.5f;
        ConstraintInstance.SetLinearXLimit(ELinearConstraintMotion::LCM_Limited, range);
        ConstraintInstance.SetLinearDriveParams(1e6, 1e4, 1e6); // TODO determine drive strength with urdf value
        ConstraintInstance.SetLinearPositionDrive(true, false, false);
        break;
    case UrdfJointType::Fixed:
        break;
    case UrdfJointType::Floating:
        ConstraintInstance.SetLinearXLimit(ELinearConstraintMotion::LCM_Free, 0.0f);
        ConstraintInstance.SetLinearYLimit(ELinearConstraintMotion::LCM_Free, 0.0f);
        ConstraintInstance.SetLinearZLimit(ELinearConstraintMotion::LCM_Free, 0.0f);
        ConstraintInstance.SetAngularTwistLimit(EAngularConstraintMotion::ACM_Free, 0.0f);
        ConstraintInstance.SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Free, 0.0f);
        ConstraintInstance.SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Free, 0.0f);
        break;
    case UrdfJointType::Planar:
        range = (joint_desc_.upper_ - joint_desc_.lower_) * unit_conversion_m_to_cm * 0.5f;
        ConstraintInstance.SetLinearXLimit(ELinearConstraintMotion::LCM_Limited, range);
        ConstraintInstance.SetLinearYLimit(ELinearConstraintMotion::LCM_Limited, range);
        break;
    default:
        ASSERT(false);
        break;
    }

    this->SetDisableCollision(true);
    this->SetConstrainedComponents(parent_link_, NAME_None, child_link_, NAME_None);
}
