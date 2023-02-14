//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <map>
#include <string>
#include <vector>

#include <CoreMinimal.h>
#include <GameFramework/Pawn.h>

#include "UrdfBotPawn.generated.h"

class UCameraComponent;
class UUrdfRobotComponent;

struct KeyboardAction
{
    std::string axis_;
    std::map<std::string, float> apply_action_;
    std::map<std::string, float> add_action_;
};

UCLASS()
class URDFBOT_API AUrdfBotPawn : public APawn
{
    GENERATED_BODY()
public:
    AUrdfBotPawn(const FObjectInitializer& object_initializer);

    // This initialize() method must be called before an AUrdfBotPawn instance is used. This style of deferred initialization
    // is required because if we attempt to call urdf_robot_component_->createChildComponents(...) from inside this constructor
    // during cooking, we get the following error:
    //     Error: FBodyInstance::GetSimplePhysicalMaterial : GEngine not initialized! Cannot call this during
    //     native CDO construction, wrap with if(!HasAnyFlags(RF_ClassDefaultObject)) or move out of constructor,
    //     material parameters will not be correct.
    void initialize();

    // APawn interface
    void SetupPlayerInputComponent(UInputComponent* input_component) override;
    void Tick(float delta_time) override;

    UUrdfRobotComponent* urdf_robot_component_ = nullptr;
    UCameraComponent* camera_component_ = nullptr;

private: 
    std::vector<KeyboardAction> keyboard_actions_;
};
