//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "UrdfRobotComponent.h"

#include "CoreUtils/Config.h"
#include "CoreUtils/Unreal.h"
#include "UrdfJointComponent.h"
#include "UrdfParser.h"

void UUrdfRobotComponent::initialize(UrdfRobotDesc* robot_desc)
{
    // initialize root link
    UrdfLinkDesc* root_link_desc = robot_desc->root_link_desc_;
    root_link_component_ = NewObject<UUrdfLinkComponent>(this, Unreal::toFName("UUrdfRobotComponent::link_component_" + root_link_desc->name_));
    root_link_component_->initialize(root_link_desc);
    link_components_[root_link_desc->name_] = root_link_component_;

    // recursively build joint
    ASSERT(root_link_desc->child_link_descs_.size() == root_link_desc->child_joint_descs_.size());
    unsigned int num_child_links = root_link_desc->child_link_descs_.size();

    for (unsigned int i = 0; i < num_child_links; i++) {
        constructChildLinks(root_link_desc->child_link_descs_.at(i), root_link_desc->child_joint_descs_.at(i), root_link_component_);
    }
}

void UUrdfRobotComponent::constructChildLinks(UrdfLinkDesc* link_desc, UrdfJointDesc* joint_desc, UUrdfLinkComponent* parent_link)
{
    UUrdfLinkComponent* child_link = NewObject<UUrdfLinkComponent>(this, Unreal::toFName("UUrdfRobotComponent::link_component_" + link_desc->name_));
    ASSERT(child_link);
    child_link->initialize(link_desc);
    link_components_[link_desc->name_] = child_link;
    
    const float M_TO_CM = 100.0f;

    // set child link transform
    FVector child_link_location = (joint_desc->origin_.GetLocation() + link_desc->visual_descs_[0].origin_.GetLocation()) * M_TO_CM;
    FRotator child_link_rotation = joint_desc->origin_.GetRotation().Rotator() + link_desc->visual_descs_[0].origin_.GetRotation().Rotator();
    child_link->SetRelativeLocationAndRotation(child_link_location, child_link_rotation);
    child_link->SetupAttachment(parent_link);

    UUrdfJointComponent* joint = NewObject<UUrdfJointComponent>(this, Unreal::toFName("UUrdfRobotComponent::joint_component_" + joint_desc->name_));
    ASSERT(joint);
    joint_components_[joint_desc->name_] = joint;

    parent_link->child_link_components_.push_back(child_link);
    parent_link->child_joint_components_.push_back(joint);

    // set joint transform
    FVector joint_location = joint_desc->origin_.GetLocation() * M_TO_CM;
    FRotator joint_rotation = joint_desc->origin_.GetRotation().Rotator();
    FVector joint_axis_in_parent_frame = joint_rotation.RotateVector(joint_desc->axis_);
    // alight joint x-axis with motion axis.
    FRotator aligned_joint_frame_rotation = FRotationMatrix::MakeFromX(joint_axis_in_parent_frame).Rotator();
    joint->SetRelativeLocationAndRotation(joint_location, aligned_joint_frame_rotation);
    joint->SetupAttachment(parent_link);

    joint->initialize(joint_desc, parent_link, child_link);

    // construct child links recursively
    ASSERT(link_desc->child_link_descs_.size() == link_desc->child_joint_descs_.size());

    unsigned int num_child_links = link_desc->child_link_descs_.size();

    for (unsigned int i = 0; i < num_child_links; i++) {
        UrdfLinkDesc* child_link_desc = link_desc->child_link_descs_.at(i);
        ASSERT(child_link_desc);
        UrdfJointDesc* child_joint_desc = link_desc->child_joint_descs_.at(i);
        ASSERT(child_joint_desc);

        constructChildLinks(child_link_desc, child_joint_desc, child_link);
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
