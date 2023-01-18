//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <CoreMinimal.h>
#include <GameFramework/Pawn.h>

#include "UrdfBotPawn.generated.h"

class UCameraComponent;

class UUrdfRobotComponent;

// UrdfBot agent
UCLASS()
class URDFBOT_API AUrdfBotPawn : public APawn
{
    GENERATED_BODY()
public:
    AUrdfBotPawn(const FObjectInitializer& object_initializer);

    void Tick(float delta_time) override;
    void SetupPlayerInputComponent(UInputComponent* input_component) override;

    // debug only
    void test();
    int signal = 0;
    void test1();
    void test2();

private:
    UUrdfRobotComponent* robot_component_ = nullptr;
    // Camera component that will be our viewpoint
    UCameraComponent* camera_component_ = nullptr;
};
