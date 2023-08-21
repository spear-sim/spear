//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <map>
#include <string>
#include <vector>

#include <CoreMinimal.h>
#include <Components/SceneComponent.h>

#include "UrdfRobotComponent.generated.h"

class UUrdfJointComponent;
class UUrdfLinkComponent;
//struct ArrayDesc;
struct UrdfLinkDesc;
struct UrdfRobotDesc;

UCLASS()
class URDFROBOT_API UUrdfRobotComponent : public USceneComponent
{
    GENERATED_BODY()
public:
    UUrdfRobotComponent();
    ~UUrdfRobotComponent();

    //// used by UrdfRobotAgent
    //std::map<std::string, ArrayDesc> getActionSpace(const std::vector<std::string>& action_components) const;
    //std::map<std::string, ArrayDesc> getObservationSpace(const std::vector<std::string>& observation_components) const;
    //void applyAction(const std::map<std::string, std::vector<uint8_t>>& action);
    //std::map<std::string, std::vector<uint8_t>> getObservation(const std::vector<std::string>& observation_components) const;

    // This function recursively creates and configures the component hierarchy for an entire URDF robot.
    void initialize(const UrdfRobotDesc* const robot_desc);

    // Applies a collection of named action components to the underlying joints. The expected format of
    // the input action is as follows.
    //     {
    //          {"joint.joint_a.add_to_angular_orientation_target": [1.23, 4.56, 7.89]},
    //          {"joint.joint_b.set_angular_orientation_target":    [0.1,  0.2,  0.3]},
    //          {"joint.joint_c.add_torque":                        [1.0,  2.0,  3.0]},
    //          ...
    //     }
    void applyAction(const std::map<std::string, std::vector<double>>& action);

    // use UPROPERTY to enable inspecting and editing in the Unreal Editor
    UPROPERTY(EditAnywhere, DisplayName = "Link Components")
    TArray<UUrdfLinkComponent*> link_components_editor_;
    UPROPERTY(EditAnywhere, DisplayName = "Joint Components")
    TArray<UUrdfJointComponent*> joint_components_editor_;

    std::map<std::string, UUrdfLinkComponent*> link_components_;
    std::map<std::string, UUrdfJointComponent*> joint_components_;

private:
    void initialize(const UrdfLinkDesc* const parent_link_desc, UUrdfLinkComponent* parent_link_component);
};
