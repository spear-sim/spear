//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "UrdfJointComponent.h"

#include <iostream>

#include <PhysicsEngine/PhysicsConstraintComponent.h>

#include "CoreUtils/Assert.h"
#include "UrdfLinkComponent.h"
#include "UrdfParser.h"

void UUrdfJointComponent::initialize(UrdfJointDesc* joint_desc, UUrdfLinkComponent* parent_link, UUrdfLinkComponent* child_link)
{
    this->joint_type_ = joint_desc->type_;

    ConstraintInstance.SetDisableCollision(true);

    ConstraintInstance.ProfileInstance.ConeLimit.Swing1Motion = EAngularConstraintMotion::ACM_Locked;
    ConstraintInstance.ProfileInstance.ConeLimit.Swing2Motion = EAngularConstraintMotion::ACM_Locked;
    ConstraintInstance.ProfileInstance.TwistLimit.TwistMotion = EAngularConstraintMotion::ACM_Locked;
    ConstraintInstance.UpdateAngularLimit();

    ConstraintInstance.ProfileInstance.AngularDrive.AngularDriveMode = EAngularDriveMode::TwistAndSwing;
    ConstraintInstance.ProfileInstance.ConeLimit.bSoftConstraint = true;
    ConstraintInstance.ProfileInstance.TwistLimit.bSoftConstraint = false;
    
    const float M_TO_CM = 100.0f;

    float range = (joint_desc->upper_ - joint_desc->lower_) * 0.5f;
    float spring = FMath::DegreesToRadians(1e6);
    float damping = FMath::DegreesToRadians(1e5);
    float force_limit = FMath::DegreesToRadians(1e6);

    switch (joint_desc->type_) {
    case UrdfJointType::Revolute:
        // use twist degree for single angular freedom with best PhysX optimization
        ConstraintInstance.SetAngularTwistLimit(EAngularConstraintMotion::ACM_Limited, FMath::RadiansToDegrees(range));
        ConstraintInstance.SetOrientationDriveTwistAndSwing(true, false);
        ConstraintInstance.SetAngularVelocityDriveTwistAndSwing(true, false);
        ConstraintInstance.SetAngularDriveParams(FMath::RadiansToDegrees(spring), FMath::RadiansToDegrees(damping), FMath::RadiansToDegrees(force_limit));
        break;
    case UrdfJointType::Continuous:
        // use twist degree for single angular freedom with best PhysX optimization
        ConstraintInstance.SetAngularTwistLimit(EAngularConstraintMotion::ACM_Free, 0.0f);
        ConstraintInstance.SetOrientationDriveTwistAndSwing(true, false);
        ConstraintInstance.SetAngularVelocityDriveTwistAndSwing(true, false);
        ConstraintInstance.SetAngularDriveParams(FMath::RadiansToDegrees(spring), FMath::RadiansToDegrees(damping), FMath::RadiansToDegrees(force_limit));
        break;
    case UrdfJointType::Prismatic:
        ConstraintInstance.SetLinearXLimit(ELinearConstraintMotion::LCM_Limited, range * M_TO_CM);
        ConstraintInstance.SetLinearDriveParams(spring, damping, force_limit);
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
        ConstraintInstance.SetLinearXLimit(ELinearConstraintMotion::LCM_Limited, range * M_TO_CM);
        ConstraintInstance.SetLinearYLimit(ELinearConstraintMotion::LCM_Limited, range * M_TO_CM);
        break;
    default:
        ASSERT(false);
        break;
    }

    this->SetDisableCollision(true);
    this->SetConstrainedComponents(parent_link, NAME_None, child_link, NAME_None);
}
