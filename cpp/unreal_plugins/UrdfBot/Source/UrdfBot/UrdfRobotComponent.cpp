//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "UrdfBot/UrdfRobotComponent.h"

#include "CoreUtils/Config.h"
#include "CoreUtils/Unreal.h"
#include "UrdfBot/UrdfJointComponent.h"
#include "UrdfBot/UrdfParser.h"

void UUrdfRobotComponent::initializeComponent(UrdfRobotDesc* robot_desc)
{
    ASSERT(robot_desc);
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
        link_components_[child_link_desc->name_] = child_link_component;

        UrdfJointDesc* child_joint_desc = child_link_desc->parent_joint_desc_;
        ASSERT(child_joint_desc);

        UUrdfJointComponent* child_joint_component = NewObject<UUrdfJointComponent>(this);
        ASSERT(child_joint_component);
        child_joint_component->initializeComponent(child_joint_desc, parent_link_component, child_link_component);
        child_joint_component->SetupAttachment(parent_link_component);
        joint_components_[child_joint_desc->name_] = child_joint_component;

        createChildComponents(child_link_desc, child_link_component);
    }
}

void UUrdfRobotComponent::test(AActor* actor)
{
    for (auto& kvp : this->link_components_) {
        actor->AddInstanceComponent(kvp.second);
    }

    for (auto& kvp : this->joint_components_) {
        actor->AddInstanceComponent(kvp.second);
    }
}

void UUrdfRobotComponent::action(int signal)
{
    std::vector<float> actions;
    actions.resize(7);
    switch (signal % 7) {
    case 0:
        // default pose
        // X=18.148 Y=-15.953 Z=17.380 P=83.363029 Y=-132.573883 R=137.650787
        actions = { 1.1707963267948966, 1.4707963267948965, -0.4, 1.6707963267948966, 0.0, 1.5707963267948966, 0.0 };
        break;
    case 1:
        // vertical
        // X=8.856 Y=49.415 Z=45.806 P=-88.179077 Y=-40.3714500 R=64.837486
        actions = { 0.94121, -0.64134, 1.55186, 1.65672, -0.93218, 1.53416, 2.14474 };
        break;
    case 2:
        // diagonal15
        // X=64.881 Y=-17.830 Z=37.338 P=-74.079781 Y=108.486938 R=164.004868
        actions = { -0.95587, -0.34778, 1.46388, 1.47821, -0.93813, 1.4587, 1.9939 };
        break;
    case 3:
        // diagonal30
        // X=63.951 Y=-21.104 Z=36.286 P=-59.728626 Y=90.069901 R=179.718170
        actions = { -1.06595, -0.22184, 1.53448, 1.46076, -0.84995, 1.36904, 1.90996 };
        break;
    case 4:
        // diagonal45
        // X=63.658 Y=-24.072 Z=33.813 P=-45.393436 Y=89.949089 R=178.276947
        actions = { -1.11479, -0.0685, 1.5696, 1.37304, -0.74273, 1.3983, 1.79618 };
        break;
    case 5:
        // horizontal
        // X=47.447 Y=-23.175 Z=43.536 P=-1.568553 Y=90.588928 R=174.634186
        actions = { -1.43016, 0.20965, 1.86816, 1.77576, -0.27289, 1.31715, 2.01226 };
        break;
    case 6:
        // intial pose
        actions = { 0, 0, 0, 0, 0, 0, 0 };
        break;
    default:
        ASSERT(false);
    }

    // apply actions to joints
    std::vector<std::string> arm_joint_names_ = Config::get<std::vector<std::string>>("URDFBOT.URDFBOT_PAWN.ARM_JOINTS");
    for (size_t i = 0; i < arm_joint_names_.size(); i++) {
        std::string joint_name = arm_joint_names_.at(i);
        UUrdfJointComponent* joint_component = joint_components_[joint_name];
        ASSERT(joint_component);

        FRotator rotation = FRotator(0, 0, actions[i] * 180 / PI);
        joint_component->SetAngularOrientationTarget(rotation);
    }
}

void UUrdfRobotComponent::actionJoint(std::string joint_name, float action)
{
    UUrdfJointComponent* joint_component = joint_components_[joint_name];

    FRotator exist_target = joint_component->ConstraintInstance.ProfileInstance.AngularDrive.OrientationTarget;
    FRotator rotation = FRotator(0, 0, exist_target.Roll + action);
    joint_component->SetAngularOrientationTarget(rotation);
    UE_LOG(LogTemp, Log, TEXT("actionJoint %f "), exist_target.Roll + action);
}
