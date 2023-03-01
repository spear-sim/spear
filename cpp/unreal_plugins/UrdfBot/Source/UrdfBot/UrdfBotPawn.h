//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <map>
#include <string>
#include <vector>

#include <CoreMinimal.h>
#include <GameFramework/Pawn.h>

#include <UrdfBot/UrdfMujocoControl.h>

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
    ~AUrdfBotPawn();

    // APawn interface
    void BeginPlay() override;
    void SetupPlayerInputComponent(UInputComponent* input_component) override;
    void Tick(float delta_time) override;

    UUrdfRobotComponent* urdf_robot_component_ = nullptr;
    UCameraComponent* camera_component_ = nullptr;

    // debug
    void gravityCompensation();
    void taskSpaceControl();

    void testKey();
    void testKey2();
    UFUNCTION(BlueprintCallable)
    static void resetConfig();

private:
    std::vector<KeyboardAction> keyboard_actions_;
    
    UrdfMujocoControl* mujoco_control_;
    std::vector<std::string> joint_names_;
    UStaticMeshComponent* eef_target_ ;
    
    // debug
    int flag = 0;
};
