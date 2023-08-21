//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <string>
#include <vector>

#include <CoreMinimal.h>
#include <PhysicsEngine/PhysicsConstraintComponent.h>

#include "UrdfJointComponent.generated.h"

class UUrdfLinkComponent;
enum class UrdfJointType;
struct UrdfJointDesc;

UCLASS()
class URDFROBOT_API UUrdfJointComponent : public UPhysicsConstraintComponent
{
    GENERATED_BODY()
public:
    UUrdfJointComponent();
    ~UUrdfJointComponent();

    // This function configures all of the joint's properties, including configuring the joint to operate on the input parent link and child link.
    void initialize(const UrdfJointDesc* const joint_desc, UUrdfLinkComponent* parent_link, UUrdfLinkComponent* child_link);

    // Apply a named action, e.g., "add_torque", to the joint using a given vector of input data.
    void applyAction(const std::string& action_component_name, const std::vector<double>& action_component_data);

    UrdfJointType joint_type_;
    UUrdfLinkComponent* parent_link_component_;
    UUrdfLinkComponent* child_link_component_;
};
