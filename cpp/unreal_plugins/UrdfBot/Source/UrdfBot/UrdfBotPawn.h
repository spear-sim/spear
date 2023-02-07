//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <CoreMinimal.h>
#include <GameFramework/Pawn.h>

#include "UrdfBotPawn.generated.h"

class UCameraComponent;
class UUrdfRobotComponent;

UCLASS()
class URDFBOT_API AUrdfBotPawn : public APawn
{
    GENERATED_BODY()
public:
    AUrdfBotPawn(const FObjectInitializer& object_initializer);

    // APawn interface
    void SetupPlayerInputComponent(UInputComponent* input_component) override;
    void Tick(float delta_time) override;

    // debug only
    void test();
    void test1();
    void test2();
    int signal = 0;

    UUrdfRobotComponent* robot_component_ = nullptr;
    UCameraComponent* camera_component_ = nullptr;
};
