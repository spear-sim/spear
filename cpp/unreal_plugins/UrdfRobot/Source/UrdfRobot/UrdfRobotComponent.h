//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <map>
#include <string>
#include <vector>

#include <CoreMinimal.h>
#include <Components/SceneComponent.h>

#include "CoreUtils/PlayerInputComponent.h"

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

    // UActorComponent interface
    void BeginPlay() override;

    UPROPERTY(EditAnywhere, Category = "SPEAR", DisplayName = "Enable Keyboard Control")
    bool EnableKeyboardControl = false;

    // We make these TArrays VisibleAnywhere rather than EditAnywhere to avoid counter-intuitive editor behavior.
    // More specifically, depending on how a user removes an entry from these lists (the leftmost dropdown, the
    // rightmost Insert/Delete/Duplicate dropdown, or the reset button), the editor will either: (1) delete the
    // underlying component and remove it from the list; or (2) remove the component from the list but the
    // component will remain active (e.g., if it is a joint, it will remain in the scene constraining the motion
    // of objects). In the latter case, there will be no way to edit the component. We prefer to set these TArrays
    // to be VisibleAnywhere to avoid all such confusion.
    UPROPERTY(VisibleAnywhere, Category = "SPEAR", DisplayName = "Link Components")
    TArray<UUrdfLinkComponent*> LinkComponents;
    UPROPERTY(VisibleAnywhere, Category = "SPEAR", DisplayName = "Joint Components")
    TArray<UUrdfJointComponent*> JointComponents;

    // This function recursively creates and configures the component hierarchy for an entire URDF robot.
    void initialize(const UrdfRobotDesc* robot_desc);

    //// Used by UrdfRobotAgent.
    //std::map<std::string, ArrayDesc> getActionSpace() const;
    //std::map<std::string, ArrayDesc> getObservationSpace() const;
    //void applyAction(const std::map<std::string, std::vector<uint8_t>>& action);
    //std::map<std::string, std::vector<uint8_t>> getObservation() const;

private:
    void initialize(const UrdfLinkDesc* parent_link_desc, UUrdfLinkComponent* parent_link_component);

    void applyAction(
        const std::map<std::string,
        std::vector<double>>& action,
        bool assert_if_joint_not_found = true,
        bool assert_if_action_is_inconsistent = true);

    std::map<std::string, UUrdfLinkComponent*> link_components_;
    std::map<std::string, UUrdfJointComponent*> joint_components_;

    UPlayerInputComponent* player_input_component_ = nullptr;
};
