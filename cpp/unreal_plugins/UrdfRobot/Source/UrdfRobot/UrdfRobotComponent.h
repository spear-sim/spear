//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // uint8_t

#include <map>
#include <string>
#include <vector>

#include <Components/SceneComponent.h>
#include <Containers/Array.h>
#include <UObject/ObjectMacros.h> // GENERATED_BODY, UCLASS, UPROPERTY

#include "SpCore/ArrayDesc.h"

#include "UrdfRobotComponent.generated.h"

class UUrdfJointComponent;
class UUrdfLinkComponent;
class UUserInputComponent;
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
    void TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

    UPROPERTY(VisibleAnywhere, Category="SPEAR", DisplayName="Root LinkComponent")
    UUrdfLinkComponent* RootLinkComponent;

    // We make these TArrays VisibleAnywhere rather than EditAnywhere to avoid counter-intuitive editor behavior.
    // More specifically, depending on how a user removes an entry from these lists (the leftmost dropdown, the
    // rightmost Insert/Delete/Duplicate dropdown, or the reset button), the editor will either: (1) delete the
    // underlying component and remove it from the list; or (2) remove the component from the list but the
    // component will remain active (e.g., if it is a joint, it will remain in the scene constraining the motion
    // of objects). In the latter case, there will be no way to edit the component. We prefer to set these TArrays
    // to be VisibleAnywhere to avoid all such confusion.
    UPROPERTY(VisibleAnywhere, Category="SPEAR", DisplayName="Link Components")
    TArray<UUrdfLinkComponent*> LinkComponents;
    UPROPERTY(VisibleAnywhere, Category="SPEAR", DisplayName="Joint Components")
    TArray<UUrdfJointComponent*> JointComponents;

    // This function recursively creates and configures the component hierarchy for an entire URDF robot.
    void initialize(const UrdfRobotDesc* robot_desc);

    // Used by UrdfRobotAgent. Note that UrdfRobotAgent must call setActionComponents(...) and setObservationComponents(...) before using the rest of this interface.
    void setActionComponents(const std::vector<std::string>& action_components);
    void setObservationComponents(const std::vector<std::string>& observation_components);
    std::map<std::string, ArrayDesc> getActionSpace() const;
    std::map<std::string, ArrayDesc> getObservationSpace() const;
    void applyAction(const std::map<std::string, std::vector<uint8_t>>& action);
    std::map<std::string, std::vector<uint8_t>> getObservation() const;
    void reset();

private:
    void initialize(const UrdfLinkDesc* parent_link_desc, UUrdfLinkComponent* parent_link_component);
    void initializeDeferred();

    void applyAction(const std::map<std::string, std::vector<double>>& action);

    std::vector<std::string> action_components_;
    std::vector<std::string> observation_components_;

    std::map<std::string, UUrdfLinkComponent*> link_components_;
    std::map<std::string, UUrdfJointComponent*> joint_components_;

    UUserInputComponent* user_input_component_ = nullptr;
    bool request_initialize_deferred_ = false;
};
