//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <vector>

#include <CoreMinimal.h>
#include <Components/StaticMeshComponent.h>

#include "UrdfLinkComponent.generated.h"

class UUrdfJointComponent;
struct UrdfLinkDesc;

UCLASS()
class UUrdfLinkComponent : public UStaticMeshComponent
{
    GENERATED_BODY()
public:
    UUrdfLinkComponent();
    ~UUrdfLinkComponent();

    void initializeComponent(UrdfLinkDesc* link_desc);
};
