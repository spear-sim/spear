//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <map>
#include <string>
#include <vector>

#include <CoreMinimal.h>
#include <GameFramework/SpectatorPawn.h>

#include "SpearSimSpectatorPawn.generated.h"

class UInputComponent;

class AUrdfRobotPawn;

UCLASS()
class ASpearSimSpectatorPawn : public ASpectatorPawn
{
    GENERATED_BODY()
public:
    ASpearSimSpectatorPawn();
    ~ASpearSimSpectatorPawn();

    void SetupPlayerInputComponent(UInputComponent* input_component) override;
    void Tick(float delta_time) override;
};
