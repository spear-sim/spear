//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "UrdfBot/UrdfRobotComponent.h"

#include <iostream>

#include "CoreUtils/Config.h"
#include "CoreUtils/Unreal.h"
#include "UrdfBot/UrdfJointComponent.h"
#include "UrdfBot/UrdfLinkComponent.h"
#include "UrdfBot/UrdfParser.h"

UUrdfRobotComponent::UUrdfRobotComponent()
{
    std::cout << "[SPEAR | UrdfRobotComponent.cpp] UUrdfRobotComponent::UUrdfRobotComponent" << std::endl;
}

UUrdfRobotComponent::~UUrdfRobotComponent()
{
    std::cout << "[SPEAR | UrdfRobotComponent.cpp] UUrdfRobotComponent::~UUrdfRobotComponent" << std::endl;
}

void UUrdfRobotComponent::createChildComponents(UrdfRobotDesc* robot_desc)
{
    ASSERT(robot_desc);

    UrdfLinkDesc* root_link_desc = robot_desc->root_link_desc_;
    ASSERT(root_link_desc);

    root_link_component_ = NewObject<UUrdfLinkComponent>(this);
    root_link_component_->initializeComponent(root_link_desc);
    root_link_component_->SetupAttachment(this);
    link_components_[root_link_desc->name_] = root_link_component_;

    createChildComponents(root_link_desc, root_link_component_, FTransform());
}

void UUrdfRobotComponent::createChildComponents(UrdfLinkDesc* parent_link_desc, UUrdfLinkComponent* parent_link_component, FTransform parent_frame_transform)
{
    ASSERT(parent_link_desc);
    ASSERT(parent_link_component);
    for (auto& child_link_desc : parent_link_desc->child_link_descs_) {
        UrdfJointDesc* child_joint_desc = child_link_desc->parent_joint_desc_;
        ASSERT(child_link_desc);
        ASSERT(child_joint_desc);

        FTransform child_joint_transform = getChildTransfromInWorld(parent_frame_transform, child_joint_desc->origin_);

        UUrdfLinkComponent* child_link_component = NewObject<UUrdfLinkComponent>(this);
        ASSERT(child_link_component);
        child_link_component->initializeComponent(child_link_desc);
        FTransform child_link_transform = getChildTransfromInWorld(child_joint_transform, child_link_desc->visual_descs_[0].origin_);
        child_link_component->SetRelativeLocation(rightHandToLeftHand(child_link_transform.GetLocation()));
        child_link_component->SetRelativeRotation(rightHandToLeftHand(child_link_transform.GetRotation()));
        child_link_component->SetupAttachment(this);
        link_components_[child_link_desc->name_] = child_link_component;

        UUrdfJointComponent* child_joint_component = NewObject<UUrdfJointComponent>(this);
        ASSERT(child_joint_component);
        child_joint_component->initializeComponent(child_joint_desc, parent_link_component, child_link_component);
        child_joint_component->SetRelativeLocation(rightHandToLeftHand(child_joint_transform.GetLocation()));
        child_joint_component->SetRelativeRotation(FRotationMatrix::MakeFromX(rightHandToLeftHand(child_joint_transform.GetRotation()).RotateVector(child_joint_desc->axis_)).Rotator());
        child_joint_component->SetupAttachment(this);
        joint_components_[child_joint_desc->name_] = child_joint_component;

        createChildComponents(child_link_desc, child_link_component, child_joint_transform);
    }
}

void UUrdfRobotComponent::applyAction(std::map<std::string, float> actions)
{
    for (auto& action : actions) {
        UUrdfJointComponent* joint_component = joint_components_.at(action.first);
        ASSERT(joint_component);
        joint_component->applyAction(action.second);
    }
}

void UUrdfRobotComponent::addAction(std::map<std::string, float> actions)
{
    for (auto& action : actions) {
        UUrdfJointComponent* joint_component = joint_components_.at(action.first);
        ASSERT(joint_component);
        joint_component->addAction(action.second);
    }
}

FTransform UUrdfRobotComponent::getChildTransfromInWorld(const FTransform& parent_transform_in_world, const FTransform& child_transform_in_parent)
{
    // both input in right hand coordinate and unit of [cm] or [degree]
    Eigen::Affine3f child_in_world = transformToAffine(parent_transform_in_world) * transformToAffine(child_transform_in_parent);
    Eigen::Vector3f location = child_in_world.translation();
    Eigen::Vector3f rotation = FMath::RadiansToDegrees(child_in_world.rotation().eulerAngles(0, 1, 2));
    return FTransform(FRotator(rotation.y(), rotation.z(), rotation.x()), FVector(location.x(), location.y(), location.z()));
}

FVector UUrdfRobotComponent::rightHandToLeftHand(FVector input)
{
    return FVector(input.X, -input.Y, input.Z);
}

FQuat UUrdfRobotComponent::rightHandToLeftHand(FQuat input)
{
    FVector euler = input.Rotator().Euler();
    return FRotator(-euler.Y, -euler.Z, euler.X).Quaternion();
}

Eigen::Affine3f UUrdfRobotComponent::transformToAffine(const FTransform& transform)
{
    FVector location = transform.GetLocation();
    FVector euler = FMath::DegreesToRadians(transform.GetRotation().Euler());

    Eigen::Affine3f affine;
    affine = Eigen::Translation<float, 3>(location.X, location.Y, location.Z);
    affine.rotate(Eigen::AngleAxis<float>(euler.X, Eigen::Vector3f::UnitX()));
    affine.rotate(Eigen::AngleAxis<float>(euler.Y, Eigen::Vector3f::UnitY()));
    affine.rotate(Eigen::AngleAxis<float>(euler.Z, Eigen::Vector3f::UnitZ()));
    return affine;
}
