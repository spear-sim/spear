//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "UrdfRobotComponent.h"

#include "CoreUtils/Config.h"
#include "UrdfJointComponent.h"
#include "UrdfParser.h"

UUrdfRobotComponent::UUrdfRobotComponent(const FObjectInitializer& object_initializer): USceneComponent(object_initializer)
{
    UrdfRobotDesc robot_desc = UrdfParser::parse(Config::get<std::string>("URDFBOT.URDFBOT_PAWN.URDF_FILE_PATH"));

    // init links
    for (auto& link_desc_pair : robot_desc.link_descs_) {
        UUrdfLinkComponent* link_component = CreateDefaultSubobject<UUrdfLinkComponent>(FName(link_desc_pair.first.c_str()));
        link_component->init(link_desc_pair.second);
        link_components_[link_desc_pair.first] = link_component;
    }

    // set root link
    root_link_component_ = link_components_[robot_desc.root_link_desc_->name_];

    // recursively build joint and setup relative coordiante transformation
    for (auto& child_joint_desc : robot_desc.root_link_desc_->child_joint_descs_) {
        attachChildLinks(child_joint_desc, robot_desc, root_link_component_, link_components_[child_joint_desc->child_]);
    }

    // init joint after link initialization
    for (auto& kvp : this->joint_components_) {
        const UrdfJointDesc& joint_desc = robot_desc.joint_descs_[kvp.first];
        kvp.second->init(joint_desc);

        kvp.second->SetupAttachment(kvp.second->parent_link_);
        kvp.second->child_link_->SetupAttachment(kvp.second->parent_link_);
    }
}

void UUrdfRobotComponent::attachChildLinks(UrdfJointDesc* joint_desc, const UrdfRobotDesc& robot_desc, UUrdfLinkComponent* parent_link, UUrdfLinkComponent* child_link)
{
    const UrdfLinkDesc* parent_link_desc = &(robot_desc.link_descs_.at(joint_desc->parent_));
    const UrdfLinkDesc* child_link_desc = &(robot_desc.link_descs_.at(joint_desc->child_));

    UUrdfJointComponent* joint = CreateDefaultSubobject<UUrdfJointComponent>(FName(joint_desc->name_.c_str()));
    ASSERT(joint);

    joint_components_[joint_desc->name_] = joint;
    joint->parent_link_ = parent_link;
    joint->child_link_ = child_link;

    // set joint transform
    FVector joint_location = joint_desc->origin_.GetLocation() * unit_conversion_m_to_cm;
    FRotator joint_rotation = joint_desc->origin_.GetRotation().Rotator();
    FVector joint_axis_in_parent_frame = joint_rotation.RotateVector(joint_desc->axis_);
    // alight joint x-axis with motion axis.
    FRotator aligned_joint_frame_rotation = FRotationMatrix::MakeFromX(joint_axis_in_parent_frame).Rotator();
    joint->SetRelativeTransform(FTransform(aligned_joint_frame_rotation, joint_location, FVector::OneVector));

    // set child link transform
    FVector child_link_location = (joint_desc->origin_.GetLocation() + child_link_desc->visual_descs_[0].origin_.GetLocation()) * unit_conversion_m_to_cm;
    FRotator child_link_rotation = joint_desc->origin_.GetRotation().Rotator() + child_link_desc->visual_descs_[0].origin_.GetRotation().Rotator();
    child_link->SetWorldLocationAndRotation(child_link_location, child_link_rotation);

    // attach all of the child node's children
    for (auto& next_joint_desc : child_link_desc->child_joint_descs_) {
        const UrdfLinkDesc& next_child_link_desc = robot_desc.link_descs_.at(next_joint_desc->child_);
        UUrdfLinkComponent* next_child_link = this->link_components_[next_child_link_desc.name_];
        ASSERT(next_child_link);

        this->attachChildLinks(next_joint_desc, robot_desc, child_link, next_child_link);
    }
}
