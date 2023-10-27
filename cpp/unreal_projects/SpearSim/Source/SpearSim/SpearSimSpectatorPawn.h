//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <GameFramework/SpectatorPawn.h>
#include <UObject/ObjectMacros.h> // GENERATED_BODY, UCLASS

#include "SpearSimSpectatorPawn.generated.h"

class UInputActionComponent;

UCLASS()
class ASpearSimSpectatorPawn : public ASpectatorPawn
{
    GENERATED_BODY()
public:
    ASpearSimSpectatorPawn();
    ~ASpearSimSpectatorPawn();

    // ASpectatorPawn interface
    void BeginPlay() override;

private:
    UInputActionComponent* input_action_component_ = nullptr;
};
