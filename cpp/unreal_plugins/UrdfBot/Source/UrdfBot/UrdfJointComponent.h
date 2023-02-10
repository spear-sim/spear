//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <CoreMinimal.h>
#include <PhysicsEngine/PhysicsConstraintComponent.h>

#include "UrdfJointComponent.generated.h"

class UUrdfLinkComponent;
enum class UrdfJointType;
struct UrdfJointDesc;

UCLASS()
class UUrdfJointComponent : public UPhysicsConstraintComponent
{
    GENERATED_BODY()
public:
    void initializeComponent(UrdfJointDesc* joint_desc, UUrdfLinkComponent* parent_link, UUrdfLinkComponent* child_link);

private:
    UrdfJointType joint_type_;
};
