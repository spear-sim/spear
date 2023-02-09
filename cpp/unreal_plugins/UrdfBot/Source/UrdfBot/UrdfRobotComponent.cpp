//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "UrdfBot/UrdfRobotComponent.h"

#include <DrawDebugHelpers.h>

#include "CoreUtils/Config.h"
#include "CoreUtils/Unreal.h"
#include "UrdfBot/UrdfJointComponent.h"
#include "UrdfBot/UrdfParser.h"

void UUrdfRobotComponent::createChildComponents(UrdfRobotDesc* robot_desc)
{
    ASSERT(robot_desc);

    UrdfLinkDesc* root_link_desc = robot_desc->root_link_desc_;
    ASSERT(root_link_desc);

    root_link_component_ = NewObject<UUrdfLinkComponent>(this, FName(robot_desc->name_.c_str()));
    root_link_component_->initializeComponent(root_link_desc);
    root_link_component_->SetupAttachment(this);
    link_components_[root_link_desc->name_] = root_link_component_;

    createChildComponents(root_link_desc, root_link_component_);
}

void UUrdfRobotComponent::createChildComponents(UrdfLinkDesc* parent_link_desc, UUrdfLinkComponent* parent_link_component)
{
    ASSERT(parent_link_desc);
    ASSERT(parent_link_component);

    for (auto& child_link_desc : parent_link_desc->child_link_descs_) {
        ASSERT(child_link_desc);

        UUrdfLinkComponent* child_link_component = NewObject<UUrdfLinkComponent>(this, FName(child_link_desc->name_.c_str()));
        ASSERT(child_link_component);
        child_link_component->initializeComponent(child_link_desc);
        child_link_component->SetupAttachment(parent_link_component);
        link_components_[child_link_desc->name_] = child_link_component;

        UrdfJointDesc* child_joint_desc = child_link_desc->parent_joint_desc_;
        ASSERT(child_joint_desc);

        UUrdfJointComponent* child_joint_component = NewObject<UUrdfJointComponent>(this, FName(child_joint_desc->name_.c_str()));
        ASSERT(child_joint_component);
        child_joint_component->initializeComponent(child_joint_desc, parent_link_component, child_link_component);
        child_joint_component->SetupAttachment(parent_link_component);
        joint_components_[child_joint_desc->name_] = child_joint_component;

        createChildComponents(child_link_desc, child_link_component);
    }
}

void UUrdfRobotComponent::applyKeyInputs()
{
    UInputComponent* InputComponent = GetOwner()->InputComponent;

    for (auto& input_axis_binding : input_axis_bindings_) {
        float axis_value = InputComponent->GetAxisValue(input_axis_binding.axis_name_);

        switch (input_axis_binding.type_) {
        case AxisBindingType::Set:
            if (axis_value > 0) {
                for (uint32 i = 0; i < input_axis_binding.component_names_.size(); i++) {
                    UUrdfJointComponent* joint_component = joint_components_.at(input_axis_binding.component_names_[i]);
                    joint_component->applyAction(input_axis_binding.values_[i]);
                }
            }
            break;
        case AxisBindingType::Add:
            for (uint32 i = 0; i < input_axis_binding.component_names_.size(); i++) {
                UUrdfJointComponent* joint_component = joint_components_.at(input_axis_binding.component_names_[i]);
                joint_component->applyAction(joint_component->getAction() + axis_value * input_axis_binding.values_[i]);
            }
            break;
        default:
            break;
        }
    }
}
