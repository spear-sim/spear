//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "UrdfRobot/UrdfJointComponent.h"

#include <map>
#include <string>
#include <utility>
#include <vector>

#include <PhysicalMaterials/PhysicalMaterial.h>
#include <PhysicsEngine/PhysicsConstraintComponent.h>
#include <Math/Matrix.h>
#include <Math/RotationMatrix.h>
#include <Math/Rotator.h>
#include <Math/Vector.h>
#include <UObject/Object.h> // CreateDefaultSubobject

#include "CoreUtils/ArrayDesc.h"
#include "CoreUtils/Assert.h"
#include "CoreUtils/InputActionComponent.h"
#include "CoreUtils/Log.h"
#include "CoreUtils/Std.h"
#include "CoreUtils/Unreal.h"
#include "UrdfRobot/UrdfLinkComponent.h"
#include "UrdfRobot/UrdfParser.h"

// useful for debugging fetch.urdf
const std::map<std::string, std::pair<std::string, std::vector<double>>> INPUT_ACTIONS = {
    {"One",        {"add_torque_in_radians",             { 1000.0,  0.0,  0.0}}},
    {"Two",        {"add_torque_in_radians",             {-1000.0,  0.0,  0.0}}},
    {"Three",      {"add_force",                         {   50.0,  0.0,  0.0}}},
    {"Four",       {"add_force",                         {  -50.0,  0.0,  0.0}}},
    {"Five",       {"add_to_angular_velocity_target",    {    0.1,  0.0,  0.0}}},
    {"Six",        {"add_to_angular_velocity_target",    {   -0.1,  0.0,  0.0}}},
    {"Seven",      {"add_to_linear_velocity_target",     {    0.1,  0.0,  0.0}}},
    {"Eight",      {"add_to_linear_velocity_target",     {   -0.1,  0.0,  0.0}}},
    {"Nine",       {"add_to_angular_orientation_target", {    0.0,  0.0,  2.0}}},
    {"Zero",       {"add_to_angular_orientation_target", {    0.0,  0.0, -2.0}}},
    {"Underscore", {"add_to_linear_position_target",     {    2.0,  0.0,  0.0}}},
    {"Equals",     {"add_to_linear_position_target",     {   -2.0,  0.0,  0.0}}}
};

UUrdfJointComponent::UUrdfJointComponent()
{
    SP_LOG_CURRENT_FUNCTION();

    // UInputActionComponent
    input_action_component_ = CreateDefaultSubobject<UInputActionComponent>(Unreal::toFName("input_action_component"));
    SP_ASSERT(input_action_component_);
    // Need to explicitly set this up so that the component hierarchy is well-defined.
    input_action_component_->SetupAttachment(this);
}

UUrdfJointComponent::~UUrdfJointComponent()
{
    SP_LOG_CURRENT_FUNCTION();

    JointType = EJointType::Invalid;
    JointControlType = EJointControlType::NotActuated;
    EnableKeyboardControl = false;
    //LinearTranslationOffset = FVector::ZeroVector; // TODO (MR): support linear translation offsets

    SP_ASSERT(input_action_component_);
    input_action_component_ = nullptr;
}

void UUrdfJointComponent::BeginPlay()
{
    UPhysicsConstraintComponent::BeginPlay();

    const std::map<std::string, std::pair<std::string, std::vector<double>>> input_actions = INPUT_ACTIONS;
    input_action_component_->bindInputActions(input_actions);
    input_action_component_->apply_input_action_func_ = [this, input_actions](const std::string& key) -> void {
        if (EnableKeyboardControl) {
            applyActionComponent(input_actions.at(key));
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
    JointInterfaceType = static_cast<EJointInterfaceType>(joint_desc->interface_type_);
    ParentStaticMeshComponent = parent_link_component;
    ChildStaticMeshComponent = child_link_component;

    FVector location = FVector({ joint_desc->xyz_.at(0), joint_desc->xyz_.at(1), joint_desc->xyz_.at(2) }) * m_to_cm;                  // m to cm
    FRotator rotation = FMath::RadiansToDegrees(FRotator({ joint_desc->rpy_.at(1), joint_desc->rpy_.at(2), joint_desc->rpy_.at(0) })); // rpy to pyr, rad to deg
    FVector axis = FVector({ joint_desc->axis_.at(0), joint_desc->axis_.at(1), joint_desc->axis_.at(2) });
    FMatrix rotation_matrix = FRotationMatrix::MakeFromX(rotation.RotateVector(axis));

    SetRelativeLocation(location);
    SetRelativeRotation(rotation_matrix.Rotator());
    SetDisableCollision(true);

    // The convention in Unreal is that component 1 is the child and component 2 is the parent, which is relevant when setting the "Parent Dominates" flag.
    SetConstrainedComponents(ChildStaticMeshComponent, NAME_None, ParentStaticMeshComponent, NAME_None);

    // Optionally enable the "parent dominates" flag to reduce jittering.
    if (joint_desc->parent_dominates_) {
        ConstraintInstance.EnableParentDominates();
    }

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
    bool enable_x_drive = true;
    bool enable_y_drive = false;
    bool enable_z_drive = false;
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
    }

    // Set drive params
    float spring = 0.0f;
    float damping = 0.0f;
    float force_limit = 0.0f;
    switch (joint_desc->control_type_) {
    case UrdfJointControlType::Position:
    case UrdfJointControlType::Velocity:
    case UrdfJointControlType::PositionAndVelocity:
        switch (joint_desc->type_) {
        case UrdfJointType::Revolute:
        case UrdfJointType::Continuous:
            spring = FMath::RadiansToDegrees(joint_desc->spring_) * m_to_cm; // m to cm, rad to deg
            damping = FMath::RadiansToDegrees(joint_desc->damping_) * m_to_cm; // m to cm, rad to deg
            force_limit = FMath::RadiansToDegrees(joint_desc->effort_) * m_to_cm; // m to cm, rad to deg
            ConstraintInstance.SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
            ConstraintInstance.SetAngularDriveParams(spring, damping, force_limit);
            break;
        case UrdfJointType::Prismatic:
            spring = joint_desc->spring_ * m_to_cm * m_to_cm; // m to cm applied twice
            damping = joint_desc->damping_ * m_to_cm * m_to_cm; // m to cm applied twice
            force_limit = joint_desc->effort_ * m_to_cm * m_to_cm; // m to cm applied twice
            ConstraintInstance.SetLinearDriveParams(spring, damping, force_limit);
            break;
        case UrdfJointType::Fixed:
            break;
        default:
            SP_ASSERT(false); // TODO (MR): support planar joints
            break;
        }
        break;
    }
}

std::map<std::string, ArrayDesc> UUrdfJointComponent::getActionSpace() const
{
    std::map<std::string, ArrayDesc> action_space;

    ArrayDesc array_desc;
    array_desc.low_ = std::numeric_limits<double>::lowest();
    array_desc.high_ = std::numeric_limits<double>::max();
    array_desc.shape_ = {3};
    array_desc.datatype_ = DataType::Float64;

    std::string separator = ".";
    std::string action_name = Unreal::toStdString(GetName()) + separator;

    switch (JointControlType) {
        case EJointControlType::Position:
        case EJointControlType::PositionAndVelocity:
            switch (JointType) {
                case EJointType::Continuous:
                case EJointType::Revolute:
                    switch(JointInterfaceType) {
                        case EJointInterfaceType::Set:
                            action_name += "set_angular_orientation_target";
                            break;

                        case EJointInterfaceType::AddTo:
                            action_name += "add_to_angular_orientation_target";
                            break;

                        case EJointInterfaceType::NoInterface:
                            break;

                        default:
                            SP_ASSERT(false);
                            break;
                    }
                    action_space[action_name] = std::move(array_desc);
                    break;

                case EJointType::Prismatic:
                    switch (JointInterfaceType) {
                        case EJointInterfaceType::Set:
                            action_name += "set_linear_position_target";
                            break;

                        case EJointInterfaceType::AddTo:
                            action_name += "add_to_linear_position_target";
                            break;

                        case EJointInterfaceType::NoInterface:
                            break;

                        default:
                            SP_ASSERT(false);
                            break;
                    }
                    action_space[action_name] = std::move(array_desc);
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
                    switch (JointInterfaceType) {
                        case EJointInterfaceType::Set:
                            action_name += "set_angular_velocity_target";
                            break;

                        case EJointInterfaceType::AddTo:
                            action_name += "add_to_angular_velocity_target";
                            break;

                        case EJointInterfaceType::NoInterface:
                            break;

                        default:
                            SP_ASSERT(false);
                            break;
                    }
                    action_space[action_name] = std::move(array_desc);
                    break;

                case EJointType::Prismatic:
                    switch (JointInterfaceType) {
                        case EJointInterfaceType::Set:
                            action_name += "set_linear_velocity_target";
                            break;

                        case EJointInterfaceType::AddTo:
                            action_name += "add_to_linear_velocity_target";
                            break;

                        case EJointInterfaceType::NoInterface:
                            break;

                        default:
                            SP_ASSERT(false);
                            break;
                    }
                    action_space[action_name] = std::move(array_desc);
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
                    action_name += "add_torque_in_radians";
                    action_space[action_name] = std::move(array_desc);
                    break;

                case EJointType::Prismatic:
                    action_name += "add_force";
                    action_space[action_name] = std::move(array_desc);
                    break;

                case EJointType::Fixed:
                    break;

                default:
                    SP_ASSERT(false); // TODO (MR): support planar joints
                    break;
            }
            break;
    }

    return action_space;
}

std::map<std::string, ArrayDesc> UUrdfJointComponent::getObservationSpace() const
{
    std::map<std::string, ArrayDesc> observation_space;

    return observation_space;
}

void UUrdfJointComponent::applyActionComponent(const std::pair<std::string, std::vector<double>>& action_component)
{
    std::string action_component_name = action_component.first;
    std::vector<double> action_component_data = action_component.second;

    std::vector<std::string> tokens = Std::tokenize(action_component_name, ".");
    std::string& action_name = tokens.at(1);

    switch (JointControlType) {
        case EJointControlType::NotActuated:
            SP_ASSERT(WITH_EDITOR); // defined in an auto-generated header
            break;

        // PositionAndVelocity joints are usually used to implement reasonable "position-based" control without excessive jittering.
        // We assume in this function that the only reasonable control actions for PositionAndVelocity joints are the same as for Position joints.
        case EJointControlType::Position:
        case EJointControlType::PositionAndVelocity:
            switch (JointType) {
                case EJointType::Continuous:
                case EJointType::Revolute:
                    if (action_name == "set_angular_orientation_target") {
                        SetAngularOrientationTarget({ action_component_data.at(0), action_component_data.at(1), action_component_data.at(2) });
                    } else if (action_name == "add_to_angular_orientation_target") {
                        SetAngularOrientationTarget(
                            ConstraintInstance.ProfileInstance.AngularDrive.OrientationTarget.Add(
                                action_component_data.at(0), action_component_data.at(1), action_component_data.at(2)));
                    } else {
                        SP_ASSERT(WITH_EDITOR); // defined in an auto-generated header
                    }
                    break;

                case EJointType::Prismatic:
                    if (action_name == "set_linear_position_target") {
                        SetLinearPositionTarget({ action_component_data.at(0), action_component_data.at(1), action_component_data.at(2) });
                    } else if (action_name == "add_to_linear_position_target") {
                        SetLinearPositionTarget({
                            action_component_data.at(0) + ConstraintInstance.ProfileInstance.LinearDrive.PositionTarget.X,
                            action_component_data.at(1) + ConstraintInstance.ProfileInstance.LinearDrive.PositionTarget.Y,
                            action_component_data.at(2) + ConstraintInstance.ProfileInstance.LinearDrive.PositionTarget.Z });
                    } else {
                        SP_ASSERT(WITH_EDITOR); // defined in an auto-generated header
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
                    if (action_name == "set_angular_velocity_target") {
                        SetAngularVelocityTarget({ action_component_data.at(0), action_component_data.at(1), action_component_data.at(2) });
                    } else if (action_name == "add_to_angular_velocity_target") {
                        SetAngularVelocityTarget({
                            action_component_data.at(0) + ConstraintInstance.ProfileInstance.AngularDrive.AngularVelocityTarget.X,
                            action_component_data.at(1) + ConstraintInstance.ProfileInstance.AngularDrive.AngularVelocityTarget.Y,
                            action_component_data.at(2) + ConstraintInstance.ProfileInstance.AngularDrive.AngularVelocityTarget.Z });
                    } else {
                        SP_ASSERT(WITH_EDITOR); // defined in an auto-generated header
                    }
                    break;

                case EJointType::Prismatic:
                    if (action_name == "set_linear_velocity_target") {
                        SetLinearVelocityTarget({ action_component_data.at(0), action_component_data.at(1), action_component_data.at(2) });
                    } else if (action_name == "add_to_linear_velocity_target") {
                        SetLinearVelocityTarget({
                            action_component_data.at(0) + ConstraintInstance.ProfileInstance.LinearDrive.VelocityTarget.X,
                            action_component_data.at(1) + ConstraintInstance.ProfileInstance.LinearDrive.VelocityTarget.Y,
                            action_component_data.at(2) + ConstraintInstance.ProfileInstance.LinearDrive.VelocityTarget.Z });
                    } else {
                        SP_ASSERT(WITH_EDITOR); // defined in an auto-generated header
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
                    if (action_name == "add_torque_in_radians") {
                        FVector torque = GetComponentTransform().GetRotation().RotateVector({ action_component_data.at(0), action_component_data.at(1), action_component_data.at(2) });
                        if (ChildStaticMeshComponent && ChildStaticMeshComponent->BodyInstance.ShouldInstanceSimulatingPhysics()) {
                            ChildStaticMeshComponent->AddTorqueInRadians(torque);
                        }
                        if (ParentStaticMeshComponent && ParentStaticMeshComponent->BodyInstance.ShouldInstanceSimulatingPhysics()) {
                            ParentStaticMeshComponent->AddTorqueInRadians(-torque);
                        }
                    } else {
                        SP_ASSERT(WITH_EDITOR); // defined in an auto-generated header
                    }
                    break;

                case EJointType::Prismatic:
                    if (action_name == "add_force") {
                        FVector force = GetComponentTransform().GetRotation().RotateVector({ action_component_data.at(0), action_component_data.at(1), action_component_data.at(2) });
                        if (ChildStaticMeshComponent && ChildStaticMeshComponent->BodyInstance.ShouldInstanceSimulatingPhysics()) {
                            ChildStaticMeshComponent->AddForce(force);
                        }
                        if (ParentStaticMeshComponent && ParentStaticMeshComponent->BodyInstance.ShouldInstanceSimulatingPhysics()) {
                            ParentStaticMeshComponent->AddForce(-force);
                        }
                    } else {
                        SP_ASSERT(WITH_EDITOR); // defined in an auto-generated header
                    }
                    break;

                case EJointType::Fixed:
                    break;

                default:
                    SP_ASSERT(false); // TODO (MR): support planar joints
                    break;
            }
            break;
        }
}

std::map<std::string, std::vector<uint8_t>> UUrdfJointComponent::getObservation() const
{
    std::map<std::string, std::vector<uint8_t>> observation;

    return observation;
}
