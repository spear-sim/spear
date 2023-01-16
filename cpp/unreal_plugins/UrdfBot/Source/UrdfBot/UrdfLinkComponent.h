//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <CoreMinimal.h>
#include <Components/StaticMeshComponent.h>

#include "UrdfLinkComponent.generated.h"

struct UrdfLinkDesc;

// unreal representation for urdf link
UCLASS()
class UUrdfLinkComponent : public UStaticMeshComponent
{
    GENERATED_BODY()

public:
    // set up link based on link description.
    void init(UrdfLinkDesc& link_desc);
};
