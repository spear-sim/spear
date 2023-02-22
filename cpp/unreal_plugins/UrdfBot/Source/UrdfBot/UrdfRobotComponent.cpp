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
    root_link_component_->initializeComponent(root_link_desc,nullptr);
    // root_link_component_->AttachToComponent(this,FAttachmentTransformRules::SnapToTargetNotIncludingScale);
    link_components_[root_link_desc->name_] = root_link_component_;

    createChildComponents(root_link_desc, root_link_component_);
}

void UUrdfRobotComponent::createChildComponents(UrdfLinkDesc* parent_link_desc, UUrdfLinkComponent* parent_link_component)
{
    ASSERT(parent_link_desc);
    ASSERT(parent_link_component);

    for (auto& child_link_desc : parent_link_desc->child_link_descs_) {
        ASSERT(child_link_desc);

        UUrdfLinkComponent* child_link_component = NewObject<UUrdfLinkComponent>(this);
        ASSERT(child_link_component);
        child_link_component->initializeComponent(child_link_desc,parent_link_component);
         child_link_component->SetupAttachment(parent_link_component);
        //child_link_component->AttachToComponent(parent_link_component, FAttachmentTransformRules::SnapToTargetNotIncludingScale);
        link_components_[child_link_desc->name_] = child_link_component;

        UrdfJointDesc* child_joint_desc = child_link_desc->parent_joint_desc_;
        ASSERT(child_joint_desc);

        UUrdfJointComponent* child_joint_component = NewObject<UUrdfJointComponent>(this);
        ASSERT(child_joint_component);
        child_joint_component->initializeComponent(child_joint_desc, parent_link_component, child_link_component);
         child_joint_component->SetupAttachment(parent_link_component);
        // child_joint_component->AttachToComponent(parent_link_component, FAttachmentTransformRules::SnapToTargetNotIncludingScale);
        joint_components_[child_joint_desc->name_] = child_joint_component;

        createChildComponents(child_link_desc, child_link_component);
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
