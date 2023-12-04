//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "UrdfRobot/UrdfRobotComponent.h"

#include <map>
#include <string>
#include <utility>
#include <vector>

#include <Containers/Array.h>       // TArray
#include <UObject/Object.h>         // CreateDefaultSubobject
#include <UObject/UObjectGlobals.h> // NewObject

#include "CoreUtils/ArrayDesc.h"
#include "CoreUtils/Assert.h"
#include "CoreUtils/Config.h"
#include "CoreUtils/InputActionComponent.h"
#include "CoreUtils/Log.h"
#include "CoreUtils/Std.h"
#include "CoreUtils/Unreal.h"
#include "UrdfRobot/UrdfJointComponent.h"
#include "UrdfRobot/UrdfLinkComponent.h"
#include "UrdfRobot/UrdfParser.h"

// useful for debugging horizontal_pendulum.urdf
const std::map<std::string, std::map<std::string, std::vector<double>>> DEFAULT_INPUT_ACTIONS = {
    {"One",   {{"joint_0.add_torque_in_radians",          { 1000000.0, 0.0, 0.0}}}},
    {"Two",   {{"joint_0.add_torque_in_radians",          {-1000000.0, 0.0, 0.0}}}},
    {"Three", {{"joint_1.add_to_angular_velocity_target", { 0.1,       0.0, 0.0}}}},
    {"Four",  {{"joint_1.add_to_angular_velocity_target", {-0.1,       0.0, 0.0}}}}
};

UUrdfRobotComponent::UUrdfRobotComponent()
{
    SP_LOG_CURRENT_FUNCTION();

    // In the current setup, the UrdfRobotComponent and the UrdfBotPawn (UrdfRobotComponent is the RootComponent of this Pawn) 
    // do not move along with the movement of child LinkComponents. However, we want the UrdfBotPawn's pose to update/follow 
    // the pose of child LinkComponents. Hence, we enable this component to tick and in every tick we update it's pose to 
    // follow the root link's pose.
    PrimaryComponentTick.bCanEverTick = true;
    PrimaryComponentTick.TickGroup = ETickingGroup::TG_PostPhysics;

    // UInputActionComponent
    input_action_component_ = CreateDefaultSubobject<UInputActionComponent>(Unreal::toFName("input_action_component"));
    SP_ASSERT(input_action_component_);
    // Need to explicitly set this up so that the component hierarchy is well-defined.
    input_action_component_->SetupAttachment(this);
}

UUrdfRobotComponent::~UUrdfRobotComponent()
{
    SP_LOG_CURRENT_FUNCTION();

    // Objects created with CreateDefaultSubobject, DuplicateObject, LoadObject, NewObject don't need to be cleaned up explicitly.

    LinkComponents.Empty();
    JointComponents.Empty();

    link_components_.clear();
    joint_components_.clear();

    SP_ASSERT(input_action_component_);
    input_action_component_ = nullptr;
}

void UUrdfRobotComponent::BeginPlay()
{
    USceneComponent::BeginPlay();

    // Get player input actions from the config system if it is initialized, otherwise use hard-coded keyboard actions, which
    // can be useful for debugging.
    std::map<std::string, std::map<std::string, std::vector<double>>> input_actions;
    if (Config::s_initialized_) {
        input_actions = Config::get<std::map<std::string, std::map<std::string, std::vector<double>>>>("URDF_ROBOT.URDF_ROBOT_COMPONENT.INPUT_ACTIONS");
    } else {
        input_actions = DEFAULT_INPUT_ACTIONS;
    }

    input_action_component_->bindInputActions(input_actions);
    input_action_component_->apply_input_action_func_ = [this, input_actions](const std::string& key) -> void {
        if (EnableKeyboardControl) {
            applyAction(input_actions.at(key));
        }
    };
}

void UUrdfRobotComponent::TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    USceneComponent::TickComponent(DeltaTime, TickType, ThisTickFunction);

    if (!ticked_once_) {
        SP_LOG_CURRENT_FUNCTION();
        initializeDeferred();
        ticked_once_ = true;
    }

    // Update this component's pose to match root link component
    SP_ASSERT(root_link_component_);
    bool sweep = false;
    FHitResult* hit_result = nullptr;
    SetWorldLocationAndRotation(root_link_component_->GetComponentLocation(), root_link_component_->GetComponentRotation(), sweep, hit_result, ETeleportType::None);
}

void UUrdfRobotComponent::setActionComponents(const std::vector<std::string>& action_components)
{
    action_components_ = action_components;
}

void UUrdfRobotComponent::setObservationComponents(const std::vector<std::string>& observation_components)
{
    observation_components_ = observation_components;
}

std::map<std::string, ArrayDesc> UUrdfRobotComponent::getActionSpace() const
{
    std::map<std::string, ArrayDesc> action_space;

    if (Std::contains(action_components_, "control_joints")) {
        for (auto joint_component : JointComponents) {
            action_space.merge(joint_component->getActionSpace());
        }
    }

    return action_space;
}

std::map<std::string, ArrayDesc> UUrdfRobotComponent::getObservationSpace() const
{
    std::map<std::string, ArrayDesc> observation_space;

    if (Std::contains(observation_components_, "link_configurations")) {
        for (auto link_component : LinkComponents) {
            observation_space.merge(link_component->getObservationSpace());
        }
    }

    if (Std::contains(observation_components_, "joint_configurations")) {
        for (auto joint_component : JointComponents) {
            observation_space.merge(joint_component->getObservationSpace());
        }
    }

    return observation_space;
}

void UUrdfRobotComponent::applyAction(const std::map<std::string, std::vector<uint8_t>>& actions)
{
    if (Std::contains(action_components_, "control_joints")) {
        std::map <std::string, std::vector<double>> actions_reinterpreted;
        for (auto& action_component : actions) {
            std::vector<double> action_component_data_reinterpreted = Std::reinterpretAs<double>(action_component.second);
            actions_reinterpreted[action_component.first] = action_component_data_reinterpreted;
        }
        applyAction(actions_reinterpreted);
    }
}

std::map<std::string, std::vector<uint8_t>> UUrdfRobotComponent::getObservation() const
{
    std::map<std::string, std::vector<uint8_t>> observation;

    if (Std::contains(observation_components_, "link_configurations")) {
        for (auto link_component : LinkComponents) {
            observation.merge(link_component->getObservation());
        }
    }

    if (Std::contains(observation_components_, "joint_configurations")) {
        for (auto joint_component : JointComponents) {
            observation.merge(joint_component->getObservation());
        }
    }

    return observation;
}

void UUrdfRobotComponent::reset()
{
    for (auto link_component : LinkComponents) {
        link_component->reset();
    }
}

void UUrdfRobotComponent::initialize(const UrdfRobotDesc* robot_desc)
{
    bool promote_to_children = true;
    for (auto link_component : LinkComponents) {
        link_component->DestroyComponent(promote_to_children);
    }

    for (auto joint_component : JointComponents) {
        joint_component->DestroyComponent(promote_to_children);
    }

    LinkComponents.Empty();
    JointComponents.Empty();

    SP_ASSERT(link_components_.empty());
    SP_ASSERT(joint_components_.empty());

    SP_ASSERT(robot_desc);

    UrdfLinkDesc* root_link_desc = robot_desc->root_link_desc_;
    SP_ASSERT(root_link_desc);

    SP_ASSERT(!Std::containsSubstring(root_link_desc->name_, "."));
    root_link_component_ = NewObject<UUrdfLinkComponent>(this, Unreal::toFName(root_link_desc->name_));
    SP_ASSERT(root_link_component_);
    root_link_component_->SetupAttachment(this);
    root_link_component_->RegisterComponent();
    // It is very important to setup attachments, and register components before we create more child components.
    // If this order is reversed and we end up creating more child components before registering the parent component,
    // the physics simulation will be unstable.
    root_link_component_->initialize(root_link_desc);
    LinkComponents.Add(root_link_component_);

    initialize(root_link_desc, root_link_component_);
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
        child_link_component->SetupAttachment(parent_link_component);
        child_link_component->RegisterComponent();
        // It is very important to setup attachments, and register components before we create more child components.
        // If this order is reversed and we end up creating more child components before registering the parent component,
        // the physics simulation will be unstable.
        child_link_component->initialize(child_link_desc);
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
        child_joint_component->SetupAttachment(parent_link_component);
        child_joint_component->RegisterComponent();
        // It is very important to setup attachments, and register components before we create more child components.
        // If this order is reversed and we end up creating more child components before registering the parent component,
        // the physics simulation will be unstable.
        child_joint_component->initialize(child_joint_desc, parent_link_component, child_link_component);
        JointComponents.Add(child_joint_component);

        initialize(child_link_desc, child_link_component);
    }
}

void UUrdfRobotComponent::initializeDeferred()
{
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

    // We cache root link component's reference to a local variable root_link_component_ in initalize().
    // In Editor mode, this root_link_component_ gets broken for the same reason mentioned above, hence
    // we need to cache it again here.
    if (!root_link_component_ && LinkComponents.Num()) {
        root_link_component_ = LinkComponents[0];
    }
    SP_ASSERT(root_link_component_);

    for (auto joint_component : JointComponents) {
        joint_component->initializeDeferred();
    }
}

void UUrdfRobotComponent::applyAction(const std::map<std::string, std::vector<double>>& action)
{
    // Since this method is private, we assume that there is no need to check action_components_, either because we're
    // being called directly due to keyboard input, or because we've already checked it in the public applyAction method.

    SP_ASSERT(action.size());

    for (auto& action_component : action) {
        std::vector<std::string> tokens = Std::tokenize(action_component.first, ".");
        SP_ASSERT(tokens.size() == 2 || WITH_EDITOR); // defined in an auto-generated header
        if (tokens.size() != 2) {
            SP_LOG("ERROR: Can't parse action component name:", action_component.first);
            continue;
        }

        bool found = Std::containsKey(joint_components_, tokens.at(0));
        SP_ASSERT(found || WITH_EDITOR); // defined in an auto-generated header
        if (!found) {
            SP_LOG("ERROR: Can't find joint: ", tokens.at(0));
            continue;
        }

        UUrdfJointComponent* joint_component = joint_components_.at(tokens.at(0));
        SP_ASSERT(joint_component);
        joint_component->applyActionComponent(action_component);
    }
}
