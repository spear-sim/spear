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
#include "CoreUtils/Config.h"
#include "CoreUtils/Log.h"
#include "CoreUtils/Std.h"
#include "CoreUtils/Unreal.h"
#include "UrdfRobot/UrdfJointComponent.h"
#include "UrdfRobot/UrdfLinkComponent.h"
#include "UrdfRobot/UrdfParser.h"

// useful for debugging pendulum.urdf
const std::map<std::string, std::map<std::string, std::vector<double>>> DEFAULT_PLAYER_INPUT_ACTIONS = {
    {"One",   {{"joint_0.add_torque_in_radians",          { 1000000.0, 0.0, 0.0}}}},
    {"Two",   {{"joint_0.add_torque_in_radians",          {-1000000.0, 0.0, 0.0}}}},
    {"Three", {{"joint_1.add_to_angular_velocity_target", { 0.1,       0.0, 0.0}}}},
    {"Four",  {{"joint_1.add_to_angular_velocity_target", {-0.1,       0.0, 0.0}}}}
};

UUrdfRobotComponent::UUrdfRobotComponent()
{
    SP_LOG_CURRENT_FUNCTION();

    // UPlayerInputComponent
    player_input_component_ = CreateDefaultSubobject<UPlayerInputComponent>(Unreal::toFName("player_input_component"));
    SP_ASSERT(player_input_component_);
}

UUrdfRobotComponent::~UUrdfRobotComponent()
{
    SP_LOG_CURRENT_FUNCTION();

    // Objects created with CreateDefaultSubobject, DuplicateObject, LoadObject, NewObject don't need to be cleaned up explicitly.

    LinkComponents.Empty();
    JointComponents.Empty();

    link_components_.clear();
    joint_components_.clear();

    SP_ASSERT(player_input_component_);
    player_input_component_ = nullptr;
}

void UUrdfRobotComponent::BeginPlay()
{
    UActorComponent::BeginPlay();

    // Cache components in maps so we can refer to them by name. We need to do this in BeginPlay because after pressing play
    // in the editor, the Unreal Engine creates a new replica object for each object in the World Outliner. For any object
    // that we spawned in the editor, we need to re-compute any local state that isn't visible to the Unreal reflection system.
    // We can't re-compute this local state in the constructor because LinkComponents and JointComponents have not been updated
    // by the the Unreal reflection system yet. So we need to do it in BeginPlay.

    for (auto link_component : LinkComponents) {
        link_components_[Unreal::toStdString(link_component->GetName())] = link_component;
    }

    for (auto joint_component : JointComponents) {
        joint_components_[Unreal::toStdString(joint_component->GetName())] = joint_component;
    }

    // Get player input actions from the config system if it is initialized, otherwise use hard-coded keyboard actions, which
    // can be useful for debugging.
    std::map<std::string, std::map<std::string, std::vector<double>>> player_input_actions;
    if (Config::s_initialized_) {
        player_input_actions =
            Config::get<std::map<std::string, std::map<std::string, std::vector<double>>>>("URDF_ROBOT.URDF_ROBOT_COMPONENT.PLAYER_INPUT_ACTIONS");
    } else {
        player_input_actions =
            DEFAULT_PLAYER_INPUT_ACTIONS;
    }

    player_input_component_->setPlayerInputActions(player_input_actions);
    player_input_component_->addAxisMappingsAndBindAxes();
    player_input_component_->apply_action_func_ = [this, player_input_actions](const PlayerInputActionDesc& player_input_action_desc, float axis_value) -> void {
        if (EnableKeyboardControl) {
            // only assert if we're not in the editor
            bool assert_if_action_is_inconsistent_with_joint = !WITH_EDITOR;
            bool assert_if_joint_not_found                   = !WITH_EDITOR;
            applyAction(player_input_actions.at(player_input_action_desc.key_), assert_if_joint_not_found, assert_if_action_is_inconsistent_with_joint);
        }
    };
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

void UUrdfRobotComponent::initialize(const UrdfRobotDesc* robot_desc)
{
    SP_ASSERT(robot_desc);

    UrdfLinkDesc* root_link_desc = robot_desc->root_link_desc_;
    SP_ASSERT(root_link_desc);

    SP_ASSERT(!Std::containsSubstring(root_link_desc->name_, "."));
    auto root_link_component = NewObject<UUrdfLinkComponent>(this, Unreal::toFName(root_link_desc->name_));
    SP_ASSERT(root_link_component);
    root_link_component->initialize(root_link_desc);
    root_link_component->SetupAttachment(this);
    root_link_component->RegisterComponent();
    LinkComponents.Add(root_link_component);

    initialize(root_link_desc, root_link_component);
}

void UUrdfRobotComponent::initialize(const UrdfLinkDesc* parent_link_desc, UUrdfLinkComponent* parent_link_component)
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

        SP_ASSERT(!Std::containsSubstring(child_link_desc->name_, "."));
        auto child_link_component = NewObject<UUrdfLinkComponent>(this, Unreal::toFName(child_link_desc->name_));
        SP_ASSERT(child_link_component);
        child_link_component->initialize(child_link_desc);
        child_link_component->SetupAttachment(parent_link_component);
        child_link_component->RegisterComponent();
        LinkComponents.Add(child_link_component);

        // We create joints here, rather than inside UUrdfLinkComponent::initialize(...), because it only makes sense to create
        // a joint once the parent link and child link have already been created. But the UUrdfLinkComponent::initialize(...)
        // method is designed to be called as soon as a parent link has been created, and before a child link has been created.
        // There are several alternative approaches that would also work here, but we think this approach is the cleanest as
        // measured by the simplicity of our URDF parsing code, the simplicity of our UrdfRobotDesc data structure, and the
        // simplicity of our recursive code for creating the Unreal component hierarchy.

        SP_ASSERT(!Std::containsSubstring(child_joint_desc->name_, "."));
        auto child_joint_component = NewObject<UUrdfJointComponent>(this, Unreal::toFName(child_joint_desc->name_));
        SP_ASSERT(child_joint_component);
        child_joint_component->initialize(child_joint_desc, parent_link_component, child_link_component);
        child_joint_component->SetupAttachment(parent_link_component);
        child_joint_component->RegisterComponent();
        JointComponents.Add(child_joint_component);

        initialize(child_link_desc, child_link_component);
    }
}

void UUrdfRobotComponent::applyAction(
    const std::map<std::string,
    std::vector<double>>& action,
    bool assert_if_joint_not_found,
    bool assert_if_action_is_inconsistent_with_joint)
{
    for (auto& action_component : action) {
        std::vector<std::string> tokens = Std::tokenize(action_component.first, ".");
        SP_ASSERT(tokens.size() == 2);
        bool found = Std::containsKey(joint_components_, tokens.at(0));
        SP_ASSERT(found || !assert_if_joint_not_found);
        if (found) {
            UUrdfJointComponent* joint_component = joint_components_.at(tokens.at(0));
            SP_ASSERT(joint_component);
            joint_component->applyAction(tokens.at(1), action_component.second, assert_if_action_is_inconsistent_with_joint);
        }
    }
}
