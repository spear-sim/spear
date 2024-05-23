//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // uint8_t

#include <map>
#include <string>
#include <vector>

#include <Components/StaticMeshComponent.h>
#include <Containers/Array.h>
#include <UObject/ObjectMacros.h> // GENERATED_BODY, UCLASS, UPROPERTY

#include "SpCore/ArrayDesc.h"

#include "UrdfLinkComponent.generated.h"

struct UrdfLinkDesc;

UCLASS()
class URDFROBOT_API UUrdfLinkComponent : public UStaticMeshComponent
{
    GENERATED_BODY()
public:
    UUrdfLinkComponent();
    ~UUrdfLinkComponent();

    // We make this TArray VisibleAnywhere rather than EditAnywhere to avoid counter-intuitive editor behavior.
    // More specifically, depending on how a user removes an entry from these lists (the leftmost dropdown, the
    // rightmost Insert/Delete/Duplicate dropdown, or the reset button), the editor will either: (1) delete the
    // underlying component and remove it from the list; or (2) remove the component from the list but the
    // component will remain active (e.g., if it is a joint, it will remain in the scene constraining the motion
    // of objects). In the latter case, there will be no way to edit the component. We prefer to set this TArray
    // to be VisibleAnywhere to avoid all such confusion.
    UPROPERTY(VisibleAnywhere, Category="SPEAR", DisplayName="Static Mesh Components")
    TArray<UStaticMeshComponent*> StaticMeshComponents;

    // This function non-recursively configures all of a link's properties, and creates all of the link's child
    // components except child links and child joints. We choose not to create child links here, because it is
    // desirable to separate the recursive and non-recursive aspects of initializing the component hierarchy,
    // and this function is designed to handle the non-recursive aspects. To that end, we also choose not to
    // create child joints here, because it only makes sense to create a child joint once the parent and child
    // links have already been created. Since this function is designed to be called as soon as a parent link
    // has been created, but before the child link has been created, it does not make sense to create child
    // joints here. Recursively creating the hierarchy of child links and child joints must be handled in
    // higher-level code.
    void initialize(const UrdfLinkDesc* link_desc);

    // Used by UrdfRobotComponent.
    std::map<std::string, ArrayDesc> getObservationSpace() const;
    std::map<std::string, std::vector<uint8_t>> getObservation() const;
    void reset();
};
