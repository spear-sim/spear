//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <map>
#include <string>

#include <CoreMinimal.h>
#include <Components/StaticMeshComponent.h>

#include "UrdfLinkComponent.generated.h"

struct UrdfLinkDesc;

UCLASS()
class URDFROBOT_API UUrdfLinkComponent : public UStaticMeshComponent
{
    GENERATED_BODY()
public:
    UUrdfLinkComponent();
    ~UUrdfLinkComponent();

    // This function non-recursively configures all of a link's properties, and creates all of the link's child components
    // except child links and child joints. We choose not to create child links here, because it is desirable to separate the
    // recursive and non-recursive aspects of initializing the component hierarchy, and this function is designed to handle
    // the non-recursive aspects. To that end, we also choose not to create child joints here, because it only makes sense to
    // create a child joint once the parent and child links have already been created. Since this function is designed to be
    // called as soon as a parent link has been created, but before the child link has been created, it does not make sense
    // to create child joints here. Recursively creating the hierarchy of child links and child joints must be handled in
    // higher-level code.
    void initialize(const UrdfLinkDesc* const link_desc);

    // use UPROPERTY to enable inspecting and editing in the Unreal Editor
    UPROPERTY(EditAnywhere, DisplayName = "Static Mesh Components")
    TArray<UStaticMeshComponent*> static_mesh_components_editor_;

    std::map<std::string, UStaticMeshComponent*> static_mesh_components_;
};
