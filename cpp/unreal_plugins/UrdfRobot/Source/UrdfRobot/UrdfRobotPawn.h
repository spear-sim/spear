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
class URDFROBOT_API AUrdfRobotPawn : public APawn
{
    GENERATED_BODY()
public:
    AUrdfRobotPawn(const FObjectInitializer& object_initializer);
    ~AUrdfRobotPawn();

    // APawn interface
    void SetupPlayerInputComponent(UInputComponent* input_component) override;
    void Tick(float delta_time) override;

    UUrdfRobotComponent* urdf_robot_component_ = nullptr;
    UCameraComponent* camera_component_ = nullptr;

private:
    std::vector<KeyboardAction> keyboard_actions_;
};
