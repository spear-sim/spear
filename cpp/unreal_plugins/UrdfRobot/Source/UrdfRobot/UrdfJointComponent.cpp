//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "UrdfRobot/UrdfJointComponent.h"

#include <string>
#include <vector>

#include <PhysicalMaterials/PhysicalMaterial.h>
#include <PhysicsEngine/PhysicsConstraintComponent.h>
#include <Math/Vector.h>

#include "CoreUtils/Assert.h"
#include "CoreUtils/Log.h"
#include "CoreUtils/PlayerInputComponent.h"
#include "CoreUtils/Unreal.h"
#include "UrdfRobot/UrdfLinkComponent.h"
#include "UrdfRobot/UrdfParser.h"

// TODO (MR): extend to support all possible action types
const std::map<std::string, std::pair<std::string, std::vector<double>>> PLAYER_INPUT_ACTIONS = {
    {"One", {"add_torque_in_radians", { 1000000.0, 0.0, 0.0}}},
    {"Two", {"add_torque_in_radians", {-1000000.0, 0.0, 0.0}}},
};

UUrdfJointComponent::UUrdfJointComponent()
{
    SP_LOG_CURRENT_FUNCTION();

    // UPlayerInputComponent
    player_input_component_ = CreateDefaultSubobject<UPlayerInputComponent>(Unreal::toFName("player_input_component"));
    SP_ASSERT(player_input_component_);
}

UUrdfJointComponent::~UUrdfJointComponent()
{
    SP_LOG_CURRENT_FUNCTION();

    JointType = EJointType::Invalid;
    JointControlType = EJointControlType::Invalid;
    EnableKeyboardControl = false;
    //LinearTranslationOffset = FVector::ZeroVector; // TODO (MR): support linear translation offsets

    parent_static_mesh_component_ = nullptr;
    child_static_mesh_component_ = nullptr;

    SP_ASSERT(player_input_component_);
    player_input_component_ = nullptr;
}

void UUrdfJointComponent::BeginPlay()
{
    UActorComponent::BeginPlay();

    parent_static_mesh_component_ = dynamic_cast<UStaticMeshComponent*>(GetComponentInternal(EConstraintFrame::Frame1));
    child_static_mesh_component_ = dynamic_cast<UStaticMeshComponent*>(GetComponentInternal(EConstraintFrame::Frame2));

    const std::map<std::string, std::pair<std::string, std::vector<double>>> player_input_actions = PLAYER_INPUT_ACTIONS;
    player_input_component_->setPlayerInputActions(player_input_actions);
    player_input_component_->addAxisMappingsAndBindAxes();
    player_input_component_->apply_action_func_ = [this, player_input_actions](const PlayerInputActionDesc& player_input_action_desc, float axis_value) -> void {
        if (EnableKeyboardControl) {
            std::pair<std::string, std::vector<double>> action_component = player_input_actions.at(player_input_action_desc.key_);
            bool assert_if_action_is_inconsistent = !WITH_EDITOR; // if we're in the editor, then don't assert if the action is inconsistent with the joint definition
            applyAction(action_component.first, action_component.second, assert_if_action_is_inconsistent);
        }
    };

    // TODO (MR): generalize this code, which currently applies a hard-coded linear translation offset

    //if (Unreal::toStdString(GetName()) == "joint.joint_1") {
    //    FTransform A1Transform = GetBodyTransform(EConstraintFrame::Frame1);
    //    A1Transform.RemoveScaling();

    //    FTransform A2Transform = GetBodyTransform(EConstraintFrame::Frame2);
    //    A2Transform.RemoveScaling();

    //    // World ref frame
    //    const FVector WPos = GetComponentLocation();
    //    const FVector Pos1 = A1Transform.InverseTransformPosition(WPos) - FVector(0.0, 0.0, 0.0);
    //    const FVector Pos2 = A2Transform.InverseTransformPosition(WPos) - FVector(0.0, 25.0, 0.0);

    //    ConstraintInstance.SetRefPosition(EConstraintFrame::Frame1, Pos1);
    //    ConstraintInstance.SetRefPosition(EConstraintFrame::Frame2, Pos2);
    //}
}

void UUrdfJointComponent::initialize(const UrdfJointDesc* joint_desc, UUrdfLinkComponent* parent_link_component, UUrdfLinkComponent* child_link_component)
{
    SP_ASSERT(joint_desc);
    SP_ASSERT(parent_link_component);
    SP_ASSERT(child_link_component);

    double m_to_cm = 100.0;

    JointType = static_cast<EJointType>(joint_desc->type_);
    JointControlType = static_cast<EJointControlType>(joint_desc->control_type_);
    parent_static_mesh_component_ = parent_link_component;
    child_static_mesh_component_ = child_link_component;

    FVector location = FVector({joint_desc->xyz_.at(0), joint_desc->xyz_.at(1), joint_desc->xyz_.at(2)}) * m_to_cm;                  // m to cm
    FRotator rotation = FMath::RadiansToDegrees(FRotator({joint_desc->rpy_.at(1), joint_desc->rpy_.at(2), joint_desc->rpy_.at(0)})); // rpy to pyr, rad to deg
    FVector axis = FVector({joint_desc->axis_.at(0), joint_desc->axis_.at(1), joint_desc->axis_.at(2)});
    FMatrix rotation_matrix = FRotationMatrix::MakeFromX(rotation.RotateVector(axis));

    SetRelativeLocation(location);
    SetRelativeRotation(rotation_matrix.Rotator());
    SetDisableCollision(true);

    // component 1, bone name 1, component 2, bone name 2
    SetConstrainedComponents(parent_static_mesh_component_, NAME_None, child_static_mesh_component_, NAME_None);

    // Set limits
    ConstraintInstance.SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Locked, 0.0f);
    ConstraintInstance.SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Locked, 0.0f);

    float twist_limit_angle = FMath::RadiansToDegrees(joint_desc->upper_ - joint_desc->lower_) * 0.5f; // rad to deg
    float linear_limit_size = (joint_desc->upper_ - joint_desc->lower_) * m_to_cm;                     // m to cm
    switch (joint_desc->type_) {
        case UrdfJointType::Revolute:
            SP_ASSERT(joint_desc->lower_ == -joint_desc->upper_); // TODO (MR): support asymmetric limits
            ConstraintInstance.SetAngularTwistLimit(EAngularConstraintMotion::ACM_Limited, twist_limit_angle);
            break;
        case UrdfJointType::Continuous:
            break;
        case UrdfJointType::Prismatic:
            SP_ASSERT(joint_desc->lower_ == -joint_desc->upper_); // TODO (MR): support asymmetric limits
            ConstraintInstance.SetAngularTwistLimit(EAngularConstraintMotion::ACM_Locked, 0.0f);
            ConstraintInstance.SetLinearXLimit(ELinearConstraintMotion::LCM_Limited, linear_limit_size);
            break;
        case UrdfJointType::Fixed:
            ConstraintInstance.SetAngularTwistLimit(EAngularConstraintMotion::ACM_Locked, 0.0f);
            break;
        default:
            SP_ASSERT(false); // TODO (MR): support planar joints
            break;
    }

    // Enable position and velocity drives
    bool enable_twist_drive = true;
    bool enable_swing_drive = false;
    bool enable_x_drive     = true;
    bool enable_y_drive     = false;
    bool enable_z_drive     = false;
    switch (joint_desc->control_type_) {
        case UrdfJointControlType::Position:
            switch (joint_desc->type_) {
                case UrdfJointType::Revolute:
                case UrdfJointType::Continuous:
                    ConstraintInstance.SetOrientationDriveTwistAndSwing(enable_twist_drive, enable_swing_drive);
                case UrdfJointType::Prismatic:
                    ConstraintInstance.SetLinearPositionDrive(enable_x_drive, enable_y_drive, enable_z_drive);
                    break;
                case UrdfJointType::Fixed:
                    break;
                default:
                    SP_ASSERT(false); // TODO (MR): support planar joints
                    break;
            }
            break;
        case UrdfJointControlType::Velocity:
            switch (joint_desc->type_) {
                case UrdfJointType::Revolute:
                case UrdfJointType::Continuous:
                    ConstraintInstance.SetAngularVelocityDriveTwistAndSwing(enable_twist_drive, enable_swing_drive);
                    break;
                case UrdfJointType::Prismatic:
                    ConstraintInstance.SetLinearVelocityDrive(enable_x_drive, enable_y_drive, enable_z_drive);
                    break;
                case UrdfJointType::Fixed:
                    break;
                default:
                    SP_ASSERT(false); // TODO (MR): support planar joints
                    break;
            }
            break;
        case UrdfJointControlType::PositionAndVelocity:
            switch (joint_desc->type_) {
                case UrdfJointType::Revolute:
                case UrdfJointType::Continuous:
                    ConstraintInstance.SetOrientationDriveTwistAndSwing(enable_twist_drive, enable_swing_drive);
                    ConstraintInstance.SetAngularVelocityDriveTwistAndSwing(enable_twist_drive, enable_swing_drive);
                    break;
                case UrdfJointType::Prismatic:
                    ConstraintInstance.SetLinearPositionDrive(enable_x_drive, enable_y_drive, enable_z_drive);
                    ConstraintInstance.SetLinearVelocityDrive(enable_x_drive, enable_y_drive, enable_z_drive);
                    break;
                case UrdfJointType::Fixed:
                    break;
                default:
                    SP_ASSERT(false); // TODO (MR): support planar joints
                    break;
            }
            break;
        case UrdfJointControlType::Torque:
            break;
        default:
            SP_ASSERT(false);
    }

    // Set drive params
    float spring      = 0.0f;
    float damping     = 0.0f;
    float force_limit = 0.0f;
    switch (joint_desc->control_type_) {
        case UrdfJointControlType::Position:
        case UrdfJointControlType::Velocity:
        case UrdfJointControlType::PositionAndVelocity:
            switch (joint_desc->type_) {
                case UrdfJointType::Revolute:
                case UrdfJointType::Continuous:
                    spring      = FMath::RadiansToDegrees(joint_desc->spring_)  * m_to_cm; // m to cm, rad to deg
                    damping     = FMath::RadiansToDegrees(joint_desc->damping_) * m_to_cm; // m to cm, rad to deg
                    force_limit = FMath::RadiansToDegrees(joint_desc->effort_)  * m_to_cm; // m to cm, rad to deg
                    ConstraintInstance.SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
                    ConstraintInstance.SetAngularDriveParams(spring, damping, force_limit);
                    break;
                case UrdfJointType::Prismatic:
                    spring      = joint_desc->spring_  * m_to_cm * m_to_cm; // m to cm applied twice
                    damping     = joint_desc->damping_ * m_to_cm * m_to_cm; // m to cm applied twice
                    force_limit = joint_desc->effort_  * m_to_cm * m_to_cm; // m to cm applied twice
                    ConstraintInstance.SetLinearDriveParams(spring, damping, force_limit);
                    break;
                case UrdfJointType::Fixed:
                    break;
                default:
                    SP_ASSERT(false); // TODO (MR): support planar joints
                    break;
            }
            break;
        case UrdfJointControlType::Torque:
            break;
        default:
            SP_ASSERT(false);
    }
}

void UUrdfJointComponent::applyAction(
    const std::string& action_component_name,
    const std::vector<double>& action_component_data,
    bool assert_if_action_is_inconsistent)
{
    switch (JointControlType) {
        case EJointControlType::Position:
            switch (JointType) {
                case EJointType::Continuous:
                case EJointType::Revolute:
                    if (action_component_name == "set_angular_orientation_target") {
                        SetAngularOrientationTarget({action_component_data.at(0), action_component_data.at(1), action_component_data.at(2)});
                    } else if (action_component_name == "add_to_angular_orientation_target") {
                        SetAngularOrientationTarget(
                            ConstraintInstance.ProfileInstance.AngularDrive.OrientationTarget.Add(
                                action_component_data.at(0), action_component_data.at(1), action_component_data.at(2)));
                    } else if (assert_if_action_is_inconsistent) {
                        SP_ASSERT(false);
                    }
                    break;

                case EJointType::Prismatic:
                    if (action_component_name == "set_linear_position_target") {
                        SetLinearPositionTarget({ action_component_data.at(0), action_component_data.at(1), action_component_data.at(2) });
                    } else if (action_component_name == "add_to_linear_position_target") {
                        SetLinearPositionTarget({
                            action_component_data.at(0) + ConstraintInstance.ProfileInstance.LinearDrive.PositionTarget.X,
                            action_component_data.at(1) + ConstraintInstance.ProfileInstance.LinearDrive.PositionTarget.Y,
                            action_component_data.at(2) + ConstraintInstance.ProfileInstance.LinearDrive.PositionTarget.Z});
                    } else if (assert_if_action_is_inconsistent) {
                        SP_ASSERT(false);
                    }
                    break;

                case EJointType::Fixed:
                    break;

                default:
                    SP_ASSERT(false); // TODO (MR): support planar joints
                    break;
            }
            break;

        case EJointControlType::Velocity:
            switch (JointType) {
                case EJointType::Continuous:
                case EJointType::Revolute:
                    if (action_component_name == "set_angular_velocity_target") {
                        SetAngularVelocityTarget({action_component_data.at(0), action_component_data.at(1), action_component_data.at(2)});
                    } else if (action_component_name == "add_to_angular_velocity_target") {
                        SetAngularVelocityTarget({
                            action_component_data.at(0) + ConstraintInstance.ProfileInstance.AngularDrive.AngularVelocityTarget.X,
                            action_component_data.at(1) + ConstraintInstance.ProfileInstance.AngularDrive.AngularVelocityTarget.Y,
                            action_component_data.at(2) + ConstraintInstance.ProfileInstance.AngularDrive.AngularVelocityTarget.Z});
                    } else if (assert_if_action_is_inconsistent) {
                        SP_ASSERT(false);
                    }
                    break;

                case EJointType::Prismatic:
                    if (action_component_name == "set_linear_velocity_target") {
                        SetLinearVelocityTarget({action_component_data.at(0), action_component_data.at(1), action_component_data.at(2)});
                    } else if (action_component_name == "add_to_linear_velocity_target") {
                        SetLinearVelocityTarget({
                            action_component_data.at(0) + ConstraintInstance.ProfileInstance.LinearDrive.VelocityTarget.X,
                            action_component_data.at(1) + ConstraintInstance.ProfileInstance.LinearDrive.VelocityTarget.Y,
                            action_component_data.at(2) + ConstraintInstance.ProfileInstance.LinearDrive.VelocityTarget.Z});
                    } else if (assert_if_action_is_inconsistent) {
                        SP_ASSERT(false);
                    }
                    break;

                case EJointType::Fixed:
                    break;

                default:
                    SP_ASSERT(false); // TODO (MR): support planar joints
                    break;
            }
            break;

        case EJointControlType::PositionAndVelocity:
            switch (JointType) {
            case EJointType::Continuous:
            case EJointType::Revolute:
                if (action_component_name == "set_angular_orientation_target") {
                    SetAngularOrientationTarget({ action_component_data.at(0), action_component_data.at(1), action_component_data.at(2) });
                } else if (action_component_name == "add_to_angular_orientation_target") {
                    SetAngularOrientationTarget(
                        ConstraintInstance.ProfileInstance.AngularDrive.OrientationTarget.Add(
                            action_component_data.at(0), action_component_data.at(1), action_component_data.at(2)));
                } else if (action_component_name == "set_angular_velocity_target") {
                    SetAngularVelocityTarget({ action_component_data.at(0), action_component_data.at(1), action_component_data.at(2) });
                } else if (action_component_name == "add_to_angular_velocity_target") {
                    SetAngularVelocityTarget({
                        action_component_data.at(0) + ConstraintInstance.ProfileInstance.AngularDrive.AngularVelocityTarget.X,
                        action_component_data.at(1) + ConstraintInstance.ProfileInstance.AngularDrive.AngularVelocityTarget.Y,
                        action_component_data.at(2) + ConstraintInstance.ProfileInstance.AngularDrive.AngularVelocityTarget.Z});
                } else if (assert_if_action_is_inconsistent) {
                    SP_ASSERT(false);
                }
                break;

            case EJointType::Prismatic:
                if (action_component_name == "set_linear_position_target") {
                    SetLinearPositionTarget({ action_component_data.at(0), action_component_data.at(1), action_component_data.at(2) });
                } else if (action_component_name == "add_to_linear_position_target") {
                    SetLinearPositionTarget({
                        action_component_data.at(0) + ConstraintInstance.ProfileInstance.LinearDrive.PositionTarget.X,
                        action_component_data.at(1) + ConstraintInstance.ProfileInstance.LinearDrive.PositionTarget.Y,
                        action_component_data.at(2) + ConstraintInstance.ProfileInstance.LinearDrive.PositionTarget.Z});
                } else if (action_component_name == "set_linear_velocity_target") {
                    SetLinearVelocityTarget({ action_component_data.at(0), action_component_data.at(1), action_component_data.at(2) });
                } else if (action_component_name == "add_to_linear_velocity_target") {
                    SetLinearVelocityTarget({
                        action_component_data.at(0) + ConstraintInstance.ProfileInstance.LinearDrive.VelocityTarget.X,
                        action_component_data.at(1) + ConstraintInstance.ProfileInstance.LinearDrive.VelocityTarget.Y,
                        action_component_data.at(2) + ConstraintInstance.ProfileInstance.LinearDrive.VelocityTarget.Z});
                } else if (assert_if_action_is_inconsistent) {
                    SP_ASSERT(false);
                }
                break;

            case EJointType::Fixed:
                break;

            default:
                SP_ASSERT(false); // TODO (MR): support planar joints
                break;
            }
            break;

        case EJointControlType::Torque:
            switch (JointType) {
                case EJointType::Continuous:
                case EJointType::Revolute:
                    if (action_component_name == "add_torque_in_radians") {
                        FVector torque = GetComponentTransform().GetRotation().RotateVector({action_component_data.at(0), action_component_data.at(1), action_component_data.at(2)});
                        if (child_static_mesh_component_ && child_static_mesh_component_->BodyInstance.ShouldInstanceSimulatingPhysics()) {
                            child_static_mesh_component_->AddTorqueInRadians(torque);
                        }
                        if (parent_static_mesh_component_ && parent_static_mesh_component_->BodyInstance.ShouldInstanceSimulatingPhysics()) {
                            parent_static_mesh_component_->AddTorqueInRadians(-torque);
                        }
                    } else if (assert_if_action_is_inconsistent) {
                        SP_ASSERT(false);
                    }
                    break;

                case EJointType::Prismatic:
                    if (action_component_name == "add_force") {
                        FVector force = GetComponentTransform().GetRotation().RotateVector({action_component_data.at(0), action_component_data.at(1), action_component_data.at(2)});
                        if (child_static_mesh_component_ && child_static_mesh_component_->BodyInstance.ShouldInstanceSimulatingPhysics()) {
                            child_static_mesh_component_->AddForce(force);
                        }
                        if (parent_static_mesh_component_ && parent_static_mesh_component_->BodyInstance.ShouldInstanceSimulatingPhysics()) {
                            parent_static_mesh_component_->AddForce(-force);
                        }
                    } else if (assert_if_action_is_inconsistent) {
                        SP_ASSERT(false);
                    }
                    break;

                case EJointType::Fixed:
                    break;

                default:
                    SP_ASSERT(false); // TODO (MR): support planar joints
                    break;
            }
            break;

        default:
            SP_ASSERT(false);
            break;
    }
}
