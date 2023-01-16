//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <CoreMinimal.h>
#include <PhysicsEngine/PhysicsConstraintComponent.h>

#include "UrdfJointComponent.generated.h"

class UUrdfLinkComponent;
struct UrdfJointDesc;
enum class UrdfJointType;

// unreal representation for urdf joint 
UCLASS()
class UUrdfJointComponent : public UPhysicsConstraintComponent
{
    GENERATED_BODY()

public:
    // Create constraint after move to proper transformation
    void init(const UrdfJointDesc& joint_desc_);
    // Move the joint to proper location for non-symmetric joint
    //FVector moveChildLinkForLimitedXAxisMotion(const UrdfJointSpecification& jointSpecification);

    UUrdfLinkComponent* parent_link_ = nullptr;
    UUrdfLinkComponent* child_link_ = nullptr;

private:
    UrdfJointType joint_type_;
};
