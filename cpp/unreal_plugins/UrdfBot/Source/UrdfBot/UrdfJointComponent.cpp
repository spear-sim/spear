//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "UrdfBot/UrdfJointComponent.h"

#include <iostream>

#include <PhysicsEngine/PhysicsConstraintComponent.h>

#include "CoreUtils/Assert.h"
#include "UrdfBot/UrdfLinkComponent.h"
#include "UrdfBot/UrdfParser.h"

UUrdfJointComponent::UUrdfJointComponent()
{
    std::cout << "[SPEAR | UrdfJointComponent.cpp] UUrdfJointComponent::UUrdfJointComponent" << std::endl;
}

UUrdfJointComponent::~UUrdfJointComponent()
{
    std::cout << "[SPEAR | UrdfJointComponent.cpp] UUrdfJointComponent::~UUrdfJointComponent" << std::endl;
}

void UUrdfJointComponent::BeginPlay()
{
    Super::BeginPlay();

    // SetConstrainedComponents(...) in constructor functions properly yet leads to warning message:
    //     Warning: Constraint in '/Script/UrdfBot.Default__UrdfBotPawn:AUrdfBotPawn::urdf_robot_component_.UrdfJointComponent_0'
    //     attempting to create a joint between objects that are both static.  No joint created.
    SetConstrainedComponents(parent_link_component_, NAME_None, child_link_component_, NAME_None);
}

void UUrdfJointComponent::initializeComponent(UrdfJointDesc* joint_desc, UUrdfLinkComponent* parent_link_component, UUrdfLinkComponent* child_link_component)
{
    ASSERT(joint_desc);
    ASSERT(parent_link_component);
    ASSERT(child_link_component);

    joint_type_ = joint_desc->type_;
    control_type_ = joint_desc->control_type_;

    parent_link_component_ = parent_link_component;
    child_link_component_ = child_link_component;

    ConstraintInstance.ProfileInstance.ConeLimit.Swing1Motion = EAngularConstraintMotion::ACM_Locked;
    ConstraintInstance.ProfileInstance.ConeLimit.Swing2Motion = EAngularConstraintMotion::ACM_Locked;
    ConstraintInstance.ProfileInstance.TwistLimit.TwistMotion = EAngularConstraintMotion::ACM_Locked;
    ConstraintInstance.ProfileInstance.AngularDrive.AngularDriveMode = EAngularDriveMode::TwistAndSwing;
    ConstraintInstance.ProfileInstance.ConeLimit.bSoftConstraint = true;
    ConstraintInstance.ProfileInstance.TwistLimit.bSoftConstraint = false;
    ConstraintInstance.UpdateAngularLimit();

    float m_to_cm = 100.0f;

    FTransform link_offset_ = joint_desc->parent_link_desc_->visual_descs_[0].origin_;
    SetRelativeLocation((joint_desc->origin_.GetLocation() - link_offset_.GetLocation()) * m_to_cm);
    SetRelativeRotation(FRotationMatrix::MakeFromX(joint_desc->origin_.GetRotation().Rotator().RotateVector(joint_desc->axis_)).Rotator());

    switch (joint_desc->type_) {
        case UrdfJointType::Revolute: {
            float twist_limit_angle = FMath::RadiansToDegrees(joint_desc->upper_ - joint_desc->lower_) * 0.5f;
            float spring = FMath::RadiansToDegrees(joint_desc->spring_) * m_to_cm;
            float damping = FMath::RadiansToDegrees(joint_desc->damping_) * m_to_cm;
            float force_limit = FMath::RadiansToDegrees(joint_desc->effort_) * m_to_cm;

            // use twist degree for single angular freedom with best PhysX optimization
            ConstraintInstance.SetAngularTwistLimit(EAngularConstraintMotion::ACM_Limited, twist_limit_angle);
            ConstraintInstance.SetOrientationDriveTwistAndSwing(true, false);
            ConstraintInstance.SetAngularVelocityDriveTwistAndSwing(true, false);
            ConstraintInstance.SetAngularDriveParams(spring, damping, force_limit);
            break;
        }
        case UrdfJointType::Continuous: {
            float spring = FMath::RadiansToDegrees(joint_desc->spring_) * m_to_cm;
            float damping = FMath::RadiansToDegrees(joint_desc->damping_) * m_to_cm;
            float force_limit = FMath::RadiansToDegrees(joint_desc->effort_) * m_to_cm;

            // use twist degree for single angular freedom with best PhysX optimization
            ConstraintInstance.SetAngularTwistLimit(EAngularConstraintMotion::ACM_Free, 0.0f);
            ConstraintInstance.SetOrientationDriveTwistAndSwing(true, false);
            ConstraintInstance.SetAngularVelocityDriveTwistAndSwing(true, false);
            ConstraintInstance.SetAngularDriveParams(spring, damping, force_limit);
            break;
        }
        case UrdfJointType::Prismatic: {
            float linear_limit_size = (joint_desc->upper_ - joint_desc->lower_) * m_to_cm;
            float spring = joint_desc->spring_ * m_to_cm * m_to_cm;
            float damping = joint_desc->damping_ * m_to_cm * m_to_cm;
            float force_limit = joint_desc->effort_ * m_to_cm * m_to_cm;

            ConstraintInstance.SetLinearXLimit(ELinearConstraintMotion::LCM_Limited, linear_limit_size);
            ConstraintInstance.SetLinearDriveParams(spring, damping, force_limit);
            ConstraintInstance.SetLinearPositionDrive(true, false, false);
            break;
        }
        case UrdfJointType::Fixed: {
            break;
        }
        case UrdfJointType::Floating: {
            ConstraintInstance.SetLinearXLimit(ELinearConstraintMotion::LCM_Free, 0.0f);
            ConstraintInstance.SetLinearYLimit(ELinearConstraintMotion::LCM_Free, 0.0f);
            ConstraintInstance.SetLinearZLimit(ELinearConstraintMotion::LCM_Free, 0.0f);
            ConstraintInstance.SetAngularTwistLimit(EAngularConstraintMotion::ACM_Free, 0.0f);
            ConstraintInstance.SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Free, 0.0f);
            ConstraintInstance.SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Free, 0.0f);
            break;
        }
        case UrdfJointType::Planar: {
            float linear_limit_size = (joint_desc->upper_ - joint_desc->lower_) * m_to_cm;

            ConstraintInstance.SetLinearXLimit(ELinearConstraintMotion::LCM_Limited, linear_limit_size);
            ConstraintInstance.SetLinearYLimit(ELinearConstraintMotion::LCM_Limited, linear_limit_size);
            break;
        }
        default: {
            ASSERT(false);
            break;
        }
    }

    SetDisableCollision(true);
}

void UUrdfJointComponent::addAction(float action)
{
    float m_to_cm = 100.0f;

    switch (control_type_) {
        case UrdfJointControlType::Position:
            switch (joint_type_) {
                case UrdfJointType::Continuous:
                case UrdfJointType::Revolute:
                    SetAngularOrientationTarget(
                        FRotator(0, 0, ConstraintInstance.ProfileInstance.AngularDrive.OrientationTarget.Roll + FMath::RadiansToDegrees(action)));
                    break;
                case UrdfJointType::Prismatic:
                    SetLinearPositionTarget(FVector(ConstraintInstance.ProfileInstance.LinearDrive.PositionTarget.X + m_to_cm * action, 0, 0));
                    break;
                default:
                    ASSERT(false);
                    break;
            }
            break;
        case UrdfJointControlType::Velocity:
            switch (joint_type_) {
                case UrdfJointType::Continuous:
                case UrdfJointType::Revolute:
                    SetAngularVelocityTarget(
                        FVector(ConstraintInstance.ProfileInstance.AngularDrive.AngularVelocityTarget.X + FMath::RadiansToDegrees(action), 0, 0));
                    break;
                case UrdfJointType::Prismatic:
                    SetLinearVelocityTarget(FVector(ConstraintInstance.ProfileInstance.LinearDrive.VelocityTarget.X + m_to_cm * action, 0, 0));
                    break;
                default:
                    ASSERT(false);
                    break;
            }
            break;
        case UrdfJointControlType::Torque:
            switch (joint_type_) {
                case UrdfJointType::Continuous:
                case UrdfJointType::Revolute: {
                    // action in unit [N * m], force in unit [N*cm]
                    FVector torque = action * m_to_cm * m_to_cm * GetComponentTransform().GetRotation().RotateVector(FVector::XAxisVector);
                    child_link_component_->AddTorqueInRadians(torque);
                    parent_link_component_->AddTorqueInRadians(-torque);
                    break;
                }
                case UrdfJointType::Prismatic: {
                    // action in unit [N], force in unit [N*cm/m]
                    FVector force = action * m_to_cm * GetComponentTransform().GetRotation().RotateVector(FVector::XAxisVector);
                    child_link_component_->AddForce(force);
                    parent_link_component_->AddForce(-force);
                    break;
                }
                default: {
                    ASSERT(false);
                    break;
                }
            }
            break;
        default:
            ASSERT(false);
            break;
    }
}

void UUrdfJointComponent::applyAction(float action)
{
    float m_to_cm = 100.0f;

    switch (control_type_) {
        case UrdfJointControlType::Position:
            switch (joint_type_) {
                case UrdfJointType::Continuous:
                case UrdfJointType::Revolute:
                    SetAngularOrientationTarget(FRotator(0, 0, FMath::RadiansToDegrees(action)));
                    break;
                case UrdfJointType::Prismatic:
                    SetLinearPositionTarget(FVector(m_to_cm * action, 0, 0));
                    break;
                default:
                    ASSERT(false);
                    break;
            }
            break;
        case UrdfJointControlType::Velocity:
            switch (joint_type_) {
                case UrdfJointType::Continuous:
                case UrdfJointType::Revolute:
                    SetAngularVelocityTarget(FVector(FMath::RadiansToDegrees(action), 0, 0));
                    break;
                case UrdfJointType::Prismatic:
                    SetLinearVelocityTarget(FVector(m_to_cm * action, 0, 0));
                    break;
                default:
                    ASSERT(false);
                    break;
            }
            break;
        case UrdfJointControlType::Torque:
            switch (joint_type_) {
                case UrdfJointType::Continuous:
                case UrdfJointType::Revolute: {
                    // action in unit [N * m], force in unit [N*cm]
                    FVector torque = action * m_to_cm * m_to_cm * GetComponentTransform().GetRotation().RotateVector(FVector::XAxisVector);
                    child_link_component_->AddTorqueInRadians(torque);
                    parent_link_component_->AddTorqueInRadians(-torque);
                    break;
                }
                case UrdfJointType::Prismatic: {
                    // action in unit [N], force in unit [N*cm/m]
                    FVector force = action * m_to_cm * GetComponentTransform().GetRotation().RotateVector(FVector::XAxisVector);
                    child_link_component_->AddForce(force);
                    parent_link_component_->AddForce(-force);
                    break;
                }
                default: {
                    ASSERT(false);
                    break;
                }
            }
            break;
        default:
            ASSERT(false);
            break;
    }
}
