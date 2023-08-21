//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "UrdfRobot/UrdfJointComponent.h"

#include <string>
#include <vector>

#include <PhysicsEngine/PhysicsConstraintComponent.h>
#include <Math/Vector.h>

#include "CoreUtils/Assert.h"
#include "CoreUtils/Log.h"
#include "CoreUtils/Unreal.h"
#include "UrdfRobot/UrdfLinkComponent.h"
#include "UrdfRobot/UrdfParser.h"

UUrdfJointComponent::UUrdfJointComponent()
{
    SP_LOG_CURRENT_FUNCTION();
}

UUrdfJointComponent::~UUrdfJointComponent()
{
    SP_LOG_CURRENT_FUNCTION();
}

void UUrdfJointComponent::initialize(const UrdfJointDesc* const joint_desc, UUrdfLinkComponent* parent_link_component, UUrdfLinkComponent* child_link_component)
{
    SP_ASSERT(joint_desc);
    SP_ASSERT(parent_link_component);
    SP_ASSERT(child_link_component);

    double m_to_cm = 100.0;

    joint_type_ = joint_desc->type_;
    parent_link_component_ = parent_link_component;
    child_link_component_ = child_link_component;

    FVector location = FVector({joint_desc->xyz_.at(0), joint_desc->xyz_.at(1), joint_desc->xyz_.at(2)}) * m_to_cm; // m to cm
    FRotator rotation = FMath::RadiansToDegrees(FRotator({joint_desc->rpy_.at(1), joint_desc->rpy_.at(2), joint_desc->rpy_.at(0)})); // rpy to pyr, rad to deg

    SetRelativeLocation(location);
    SetRelativeRotation(rotation);

    //    // component 1, bone name 1, component 2, bone name 2
    //    SetConstrainedComponents(parent_link_component_, NAME_None, child_link_component_, NAME_None);

    //ConstraintInstance.ProfileInstance.ConeLimit.Swing1Motion = EAngularConstraintMotion::ACM_Locked;
    //ConstraintInstance.ProfileInstance.ConeLimit.Swing2Motion = EAngularConstraintMotion::ACM_Locked;
    //ConstraintInstance.ProfileInstance.TwistLimit.TwistMotion = EAngularConstraintMotion::ACM_Locked;
    //ConstraintInstance.ProfileInstance.AngularDrive.AngularDriveMode = EAngularDriveMode::TwistAndSwing;
    //ConstraintInstance.ProfileInstance.ConeLimit.bSoftConstraint = true;
    //ConstraintInstance.ProfileInstance.TwistLimit.bSoftConstraint = false;
    //ConstraintInstance.UpdateAngularLimit();

    //float m_to_cm = 100.0f;

    //FTransform link_offset_ = joint_desc->parent_link_desc_->visual_descs_[0].origin_;
    //SetRelativeLocation((joint_desc->origin_.GetLocation() - link_offset_.GetLocation()) * m_to_cm);
    //SetRelativeRotation(FRotationMatrix::MakeFromX(joint_desc->origin_.GetRotation().Rotator().RotateVector(joint_desc->axis_)).Rotator());

    //switch (joint_desc->type_) {
    //    case UrdfJointType::Revolute: {
    //        float twist_limit_angle = FMath::RadiansToDegrees(joint_desc->upper_ - joint_desc->lower_) * 0.5f;
    //        float spring = FMath::RadiansToDegrees(joint_desc->spring_) * m_to_cm;
    //        float damping = FMath::RadiansToDegrees(joint_desc->damping_) * m_to_cm;
    //        float force_limit = FMath::RadiansToDegrees(joint_desc->effort_) * m_to_cm;

    //        // use twist degree for single angular freedom with best PhysX optimization
    //        ConstraintInstance.SetAngularTwistLimit(EAngularConstraintMotion::ACM_Limited, twist_limit_angle);
    //        ConstraintInstance.SetOrientationDriveTwistAndSwing(true, false);
    //        ConstraintInstance.SetAngularVelocityDriveTwistAndSwing(true, false);
    //        ConstraintInstance.SetAngularDriveParams(spring, damping, force_limit);
    //        break;
    //    }
    //    case UrdfJointType::Continuous: {
    //        float spring = FMath::RadiansToDegrees(joint_desc->spring_) * m_to_cm;
    //        float damping = FMath::RadiansToDegrees(joint_desc->damping_) * m_to_cm;
    //        float force_limit = FMath::RadiansToDegrees(joint_desc->effort_) * m_to_cm;

    //        // use twist degree for single angular freedom with best PhysX optimization
    //        ConstraintInstance.SetAngularTwistLimit(EAngularConstraintMotion::ACM_Free, 0.0f);
    //        ConstraintInstance.SetOrientationDriveTwistAndSwing(true, false);
    //        ConstraintInstance.SetAngularVelocityDriveTwistAndSwing(true, false);
    //        ConstraintInstance.SetAngularDriveParams(spring, damping, force_limit);
    //        break;
    //    }
    //    case UrdfJointType::Prismatic:
    //        float linear_limit_size = (joint_desc->upper_ - joint_desc->lower_) * m_to_cm;
    //        float spring = joint_desc->spring_ * m_to_cm * m_to_cm;
    //        float damping = joint_desc->damping_ * m_to_cm * m_to_cm;
    //        float force_limit = joint_desc->effort_ * m_to_cm * m_to_cm;

    //        ConstraintInstance.SetLinearXLimit(ELinearConstraintMotion::LCM_Limited, linear_limit_size);
    //        ConstraintInstance.SetLinearDriveParams(spring, damping, force_limit);
    //        ConstraintInstance.SetLinearPositionDrive(true, false, false);
    //        break;

    //    case UrdfJointType::Fixed:
    //        break;

    //    case UrdfJointType::Floating:
    //        SP_ASSERT(false);
    //        break;

    //    case UrdfJointType::Planar:
    //        SP_ASSERT(false);
    //        break;

    //    default:
    //        SP_ASSERT(false);
    //        break;
    //}

    //SetDisableCollision(true);
}

void UUrdfJointComponent::applyAction(const std::string& action_component_name, const std::vector<double>& action_component_data)
{
    switch (joint_type_) {
        case UrdfJointType::Continuous:
        case UrdfJointType::Revolute:
            if (action_component_name == "set_angular_orientation_target") {
                SetAngularOrientationTarget({action_component_data.at(0), action_component_data.at(1), action_component_data.at(2)});

            } else if (action_component_name == "add_to_angular_orientation_target") {
                SetAngularOrientationTarget(
                    ConstraintInstance.ProfileInstance.AngularDrive.OrientationTarget.Add(
                        action_component_data.at(0), action_component_data.at(1), action_component_data.at(2)));

            } else if (action_component_name == "set_angular_velocity_target") {
                SetAngularVelocityTarget({action_component_data.at(0), action_component_data.at(1), action_component_data.at(2)});

            } else if (action_component_name == "add_to_angular_velocity_target") {
                SetAngularVelocityTarget({
                    action_component_data.at(0) + ConstraintInstance.ProfileInstance.AngularDrive.AngularVelocityTarget.X,
                    action_component_data.at(1) + ConstraintInstance.ProfileInstance.AngularDrive.AngularVelocityTarget.Y,
                    action_component_data.at(2) + ConstraintInstance.ProfileInstance.AngularDrive.AngularVelocityTarget.Z});

            } else if (action_component_name == "add_torque_in_radians") {
                FVector torque = GetComponentTransform().GetRotation().RotateVector({action_component_data.at(0), action_component_data.at(1), action_component_data.at(2)});
                child_link_component_->AddTorqueInRadians(torque);
                parent_link_component_->AddTorqueInRadians(-torque);

            } else {
                SP_ASSERT(false);
            }
            break;

        case UrdfJointType::Prismatic:
            if (action_component_name == "set_linear_position_target") {
                SetLinearPositionTarget({action_component_data.at(0), action_component_data.at(1), action_component_data.at(2)});

            } else if (action_component_name == "add_to_linear_position_target") {
                SetLinearPositionTarget({
                    action_component_data.at(0) + ConstraintInstance.ProfileInstance.LinearDrive.PositionTarget.X,
                    action_component_data.at(1) + ConstraintInstance.ProfileInstance.LinearDrive.PositionTarget.Y,
                    action_component_data.at(2) + ConstraintInstance.ProfileInstance.LinearDrive.PositionTarget.Z});

            } else if (action_component_name == "set_linear_velocity_target") {
                SetLinearVelocityTarget({action_component_data.at(0), action_component_data.at(1), action_component_data.at(2)});

            } else if (action_component_name == "add_to_linear_velocity_target") {
                SetLinearVelocityTarget({
                    action_component_data.at(0) + ConstraintInstance.ProfileInstance.LinearDrive.VelocityTarget.X,
                    action_component_data.at(1) + ConstraintInstance.ProfileInstance.LinearDrive.VelocityTarget.Y,
                    action_component_data.at(2) + ConstraintInstance.ProfileInstance.LinearDrive.VelocityTarget.Z});

            } else if (action_component_name == "add_force") {
                FVector force = GetComponentTransform().GetRotation().RotateVector({action_component_data.at(0), action_component_data.at(1), action_component_data.at(2)});
                child_link_component_->AddForce(force);
                parent_link_component_->AddForce(-force);

            } else {
                SP_ASSERT(false);
            }
            break;

        default:
            SP_ASSERT(false);
            break;
    }
}
