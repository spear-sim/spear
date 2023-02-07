//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <map>
#include <string>

#include <CoreMinimal.h>
#include <Components/SceneComponent.h>

#include "UrdfBot/UrdfLinkComponent.h"

#include "UrdfRobotComponent.generated.h"

class UUrdfJointComponent;

struct UrdfJointDesc;
struct UrdfLinkDesc;
struct UrdfRobotDesc;

// unreal representation for urdf robot
UCLASS()
class UUrdfRobotComponent : public USceneComponent
{
    GENERATED_BODY()
public:
    void initializeComponent(UrdfRobotDesc* robot_desc);
    void createChildComponents(UrdfRobotDesc* robot_desc);

    UUrdfLinkComponent* root_link_component_;
    std::map<std::string, UUrdfLinkComponent*> link_components_;
    std::map<std::string, UUrdfJointComponent*> joint_components_;

private:
    void createChildComponents(UrdfLinkDesc* parent_link_desc, UUrdfLinkComponent* parent_link);
};
