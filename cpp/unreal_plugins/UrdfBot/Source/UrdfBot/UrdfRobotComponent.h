//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <map>
#include <string>
#include <vector>

#include <CoreMinimal.h>
#include <Components/SceneComponent.h>

#include "UrdfBot/UrdfLinkComponent.h"

#include "UrdfRobotComponent.generated.h"

class UUrdfJointComponent;
class UrdfSimpleControl;

struct UrdfJointDesc;
struct UrdfLinkDesc;
struct UrdfRobotDesc;

enum class AxisBindingType
{
    Invalid,
    Set,
    Add,
};

struct InputAxisBinding
{
    FName axis_name_;

    std::string key_;
    AxisBindingType type_ = AxisBindingType::Invalid;
    std::vector<std::string> component_names_;
    std::vector<float> values_;
};

// unreal representation for urdf robot
UCLASS()
class UUrdfRobotComponent : public USceneComponent
{
    GENERATED_BODY()
public:
    void createChildComponents(UrdfRobotDesc* robot_desc);

    void applyKeyInputs();

    UUrdfLinkComponent* root_link_component_;
    std::map<std::string, UUrdfLinkComponent*> link_components_;
    std::map<std::string, UUrdfJointComponent*> joint_components_;

    std::vector<InputAxisBinding> input_axis_bindings_;

private:
    void createChildComponents(UrdfLinkDesc* parent_link_desc, UUrdfLinkComponent* parent_link);
};
