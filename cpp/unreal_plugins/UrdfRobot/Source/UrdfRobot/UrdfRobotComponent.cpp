//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "UrdfRobot/UrdfRobotComponent.h"

#include <map>
#include <string>
//#include <vector>

#include <Components/SceneComponent.h>

//#include "CoreUtils/ArrayDesc.h"
#include "CoreUtils/Assert.h"
#include "CoreUtils/Log.h"
#include "CoreUtils/Std.h"
#include "CoreUtils/Unreal.h"
#include "UrdfRobot/UrdfJointComponent.h"
#include "UrdfRobot/UrdfLinkComponent.h"
#include "UrdfRobot/UrdfParser.h"

UUrdfRobotComponent::UUrdfRobotComponent()
{
    SP_LOG_CURRENT_FUNCTION();
}

UUrdfRobotComponent::~UUrdfRobotComponent()
{
    SP_LOG_CURRENT_FUNCTION();
}

//std::map<std::string, ArrayDesc> UUrdfRobotComponent::getActionSpace(const std::vector<std::string>& action_components) const
//{
//    std::map<std::string, ArrayDesc> action_space;
//
//    if (Std::contains(action_components, "control_joints")) {
//        for (auto& joint_component : joint_components_) {
//            if (joint_component.second->control_type_ != UrdfJointControlType::Invalid) {
//                ArrayDesc array_desc;
//                array_desc.low_ = std::numeric_limits<float>::lowest();
//                array_desc.high_ = std::numeric_limits<float>::max();
//                array_desc.shape_ = {1};
//                array_desc.datatype_ = DataType::Float32;
//                action_space[joint_component.first] = std::move(array_desc);
//            }
//        }
//    }
//
//    return action_space;
//}

//std::map<std::string, ArrayDesc> UUrdfRobotComponent::getObservationSpace(const std::vector<std::string>& observation_components) const
//{
//    std::map<std::string, ArrayDesc> observation_space;
//
//    if (Std::contains(observation_components, "link_state")) {
//        for (auto& link_component : link_components_) {
//            ArrayDesc array_desc;
//            array_desc.low_ = std::numeric_limits<float>::lowest();
//            array_desc.high_ = std::numeric_limits<float>::max();
//            array_desc.shape_ = {6};
//            array_desc.datatype_ = DataType::Float32;
//            observation_space[link_component.first] = std::move(array_desc);
//        }
//    }
//
//    return observation_space;
//}

//void UUrdfRobotComponent::applyAction(const std::map<std::string, std::vector<uint8_t>>& actions)
//{
//    std::map<std::string, float> joint_actions;
//
//    for (auto& action : actions) {
//        if (Std::containsKey(joint_components_, action.first)) {
//            std::vector<float> action_data = Std::reinterpret_as<float>(action.second);
//            joint_actions[action.first] = action_data.at(0);
//        }
//    }
//
//    applyAction(joint_actions);
//}

//std::map<std::string, std::vector<uint8_t>> UUrdfRobotComponent::getObservation(const std::vector<std::string>& observation_components) const
//{
//    std::map<std::string, std::vector<uint8_t>> observation;
//
//    if (Std::contains(observation_components, "link_state")) {
//        for (auto& link_component : link_components_) {
//            FVector position = link_component.second->GetRelativeLocation();
//            FRotator rotation = link_component.second->GetRelativeRotation();
//            observation[link_component.first] =
//                Std::reinterpret_as<uint8_t>(std::vector<float>{position.X, position.Y, position.Z, rotation.Roll, rotation.Yaw, rotation.Pitch});
//        }
//    }
//
//    return observation;
//}

void UUrdfRobotComponent::initialize(const UrdfRobotDesc* const robot_desc)
{
    SP_ASSERT(robot_desc);

    UrdfLinkDesc* root_link_desc = robot_desc->root_link_desc_;
    SP_ASSERT(root_link_desc);

    auto root_link_component = NewObject<UUrdfLinkComponent>(this, Unreal::toFName("link." + root_link_desc->name_));
    SP_ASSERT(root_link_component);
    root_link_component->initialize(root_link_desc);
    root_link_component->SetupAttachment(this);
    root_link_component->RegisterComponent();

    link_components_["link." + root_link_desc->name_] = root_link_component;
    link_components_editor_.Add(root_link_component);

    initialize(root_link_desc, root_link_component);
}

void UUrdfRobotComponent::initialize(const UrdfLinkDesc* const parent_link_desc, UUrdfLinkComponent* parent_link_component)
{
    SP_ASSERT(parent_link_desc);
    SP_ASSERT(parent_link_component);
    SP_ASSERT(parent_link_desc->child_link_descs_.size() == parent_link_desc->child_joint_descs_.size());

    int num_children = parent_link_desc->child_link_descs_.size();

    for (int i = 0; i < num_children; i++) {
        UrdfLinkDesc* child_link_desc = parent_link_desc->child_link_descs_.at(i);
        UrdfJointDesc* child_joint_desc = parent_link_desc->child_joint_descs_.at(i);
        SP_ASSERT(child_link_desc);
        SP_ASSERT(child_joint_desc);

        auto child_link_component = NewObject<UUrdfLinkComponent>(this, Unreal::toFName("link." + child_link_desc->name_));
        SP_ASSERT(child_link_component);
        child_link_component->initialize(child_link_desc);
        child_link_component->SetupAttachment(parent_link_component);
        child_link_component->RegisterComponent();

        link_components_["link." + child_link_desc->name_] = child_link_component;
        link_components_editor_.Add(child_link_component);

        // We create joints here, rather than inside UUrdfLinkComponent::initialize(...), because it only makes sense to create
        // a joint once the parent link and child link have already been created. But the UUrdfLinkComponent::initialize(...)
        // method is designed to be called as soon as a parent link has been created, and before a child link has been created.
        // There are several alternative approaches that would also work here, but this approach is the cleanest as measured by
        // the simplicity of our URDF parsing code, the simplicity of our UrdfRobotDesc data structure, and the simplicity of
        // our recursive code for creating the Unreal component hierarchy.
        auto child_joint_component = NewObject<UUrdfJointComponent>(this, Unreal::toFName("joint." + child_joint_desc->name_));
        SP_ASSERT(child_joint_component);
        child_joint_component->initialize(child_joint_desc, parent_link_component, child_link_component);
        child_joint_component->SetupAttachment(parent_link_component);
        child_joint_component->RegisterComponent();

        joint_components_["joint." + child_joint_desc->name_] = child_joint_component;
        joint_components_editor_.Add(child_joint_component);

        initialize(child_link_desc, child_link_component);
    }
}

void UUrdfRobotComponent::applyAction(const std::map<std::string, std::vector<double>>& action)
{
    for (auto& action_component : action) {
        std::vector<std::string> tokens = Std::tokenize(action_component.first, ".");
        SP_ASSERT(tokens.size() == 3);
        SP_ASSERT(tokens.at(0) == "joint");
        UUrdfJointComponent* joint_component = joint_components_.at("joint." + tokens.at(1));
        SP_ASSERT(joint_component);
        joint_component->applyAction(tokens.at(2), action_component.second);
    }
}
