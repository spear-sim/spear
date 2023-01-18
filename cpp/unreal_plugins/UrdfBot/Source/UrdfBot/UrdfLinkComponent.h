//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <vector>

#include <CoreMinimal.h>
#include <Components/StaticMeshComponent.h>

#include "UrdfLinkComponent.generated.h"

struct UrdfLinkDesc;

class UUrdfJointComponent;

// unreal representation for urdf link
UCLASS()
class UUrdfLinkComponent : public UStaticMeshComponent
{
    GENERATED_BODY()
public:
    // set up link based on link description.
    void initialize(UrdfLinkDesc* link_desc);

    std::vector<UUrdfLinkComponent*> child_link_components_; // defined in header
    std::vector<UUrdfLinkComponent*> child_joint_components_; // defined in header
};
