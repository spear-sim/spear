//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <GameFramework/SpectatorPawn.h>
#include <UObject/ObjectMacros.h> // GENERATED_BODY, UCLASS

#include "SpearSimSpectatorPawn.generated.h"

class FObjectInitializer;
class UInputComponent;

UCLASS()
class ASpearSimSpectatorPawn : public ASpectatorPawn
{
    GENERATED_BODY()
public:
    ASpearSimSpectatorPawn(const FObjectInitializer& object_initializer);
    ~ASpearSimSpectatorPawn();

    void SetupPlayerInputComponent(UInputComponent* input_component) override;

private:
    void exit(float value);
};
