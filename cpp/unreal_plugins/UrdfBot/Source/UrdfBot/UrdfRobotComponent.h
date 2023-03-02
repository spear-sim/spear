//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Eigen/Dense>
#include <map>
#include <string>
#include <vector>

#include <CoreMinimal.h>
#include <Components/SceneComponent.h>

#include "UrdfRobotComponent.generated.h"

class UUrdfJointComponent;
class UUrdfLinkComponent;
struct UrdfLinkDesc;
struct UrdfRobotDesc;

UCLASS()
class UUrdfRobotComponent : public USceneComponent
{
    GENERATED_BODY()
public:
    UUrdfRobotComponent();
    ~UUrdfRobotComponent();

    void createChildComponents(UrdfRobotDesc* robot_desc);

    void applyAction(std::map<std::string, float> action);
    void addAction(std::map<std::string, float> action);

    UUrdfLinkComponent* root_link_component_ = nullptr;
    std::map<std::string, UUrdfLinkComponent*> link_components_;
    std::map<std::string, UUrdfJointComponent*> joint_components_;

private:
    void createChildComponents(UrdfLinkDesc* parent_link_desc, UUrdfLinkComponent* parent_link,FTransform parent_tf);

    static FTransform getChildTransfromInWorld(const FTransform& parent_transform_in_world, const FTransform& child_transform_in_parent);
    static FVector rightHandToLeftHand(FVector input);
    static FQuat rightHandToLeftHand(FQuat input);
    static Eigen::Affine3f transformToAffine(const FTransform& tf);
};
