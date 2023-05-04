//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "UrdfBot/UrdfRobotComponent.h"

#include <iostream>

#include "CoreUtils/ArrayDesc.h"
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
    link_components_["link." + root_link_desc->name_] = root_link_component_;

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
        child_link_component->initializeComponent(child_link_desc);
        child_link_component->SetupAttachment(parent_link_component);
        link_components_["link." + child_link_desc->name_] = child_link_component;

        UrdfJointDesc* child_joint_desc = child_link_desc->parent_joint_desc_;
        ASSERT(child_joint_desc);

        UUrdfJointComponent* child_joint_component = NewObject<UUrdfJointComponent>(this);
        ASSERT(child_joint_component);
        child_joint_component->initializeComponent(child_joint_desc, parent_link_component, child_link_component);
        child_joint_component->SetupAttachment(parent_link_component);
        joint_components_["joint." + child_joint_desc->name_] = child_joint_component;

        createChildComponents(child_link_desc, child_link_component);
    }
}

std::map<std::string, ArrayDesc> UUrdfRobotComponent::getActionSpace(const std::vector<std::string>& action_components) const
{
    std::map<std::string, ArrayDesc> action_space;

    if (Std::contains(action_components, "control_joints")) {
        for (auto& joint_component : joint_components_) {
            if (joint_component.second->control_type_ != UrdfJointControlType::Invalid) {
                ArrayDesc array_desc;
                array_desc.low_ = std::numeric_limits<float>::lowest();
                array_desc.high_ = std::numeric_limits<float>::max();
                array_desc.shape_ = {1};
                array_desc.datatype_ = DataType::Float32;
                action_space[joint_component.first] = std::move(array_desc);
            }
        }
    }

    return action_space;
}

std::map<std::string, ArrayDesc> UUrdfRobotComponent::getObservationSpace(const std::vector<std::string>& observation_components) const
{
    std::map<std::string, ArrayDesc> observation_space;

    if (Std::contains(observation_components, "link_state")) {
        for (auto& link_component : link_components_) {
            ArrayDesc array_desc;
            array_desc.low_ = std::numeric_limits<float>::lowest();
            array_desc.high_ = std::numeric_limits<float>::max();
            array_desc.shape_ = {6};
            array_desc.datatype_ = DataType::Float32;
            observation_space[link_component.first] = std::move(array_desc);
        }
    }

    return observation_space;
}

void UUrdfRobotComponent::applyAction(const std::map<std::string, std::vector<uint8_t>>& actions)
{
    std::map<std::string, float> joint_actions;

    for (auto& action : actions) {
        if (Std::containsKey(joint_components_, action.first)) {
            std::vector<float> action_data = Std::reinterpret_as<float>(action.second);
            joint_actions[action.first] = action_data.at(0);
        }
    }

    applyAction(joint_actions);
}

std::map<std::string, std::vector<uint8_t>> UUrdfRobotComponent::getObservation(const std::vector<std::string>& observation_components) const
{
    std::map<std::string, std::vector<uint8_t>> observation;

    if (Std::contains(observation_components, "link_state")) {
        for (auto& link_component : link_components_) {
            FVector position = link_component.second->GetRelativeLocation();
            FRotator rotation = link_component.second->GetRelativeRotation();
            observation[link_component.first] =
                Std::reinterpret_as<uint8_t>(std::vector<float>{position.X, position.Y, position.Z, rotation.Roll, rotation.Yaw, rotation.Pitch});
        }
    }

    return observation;
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
