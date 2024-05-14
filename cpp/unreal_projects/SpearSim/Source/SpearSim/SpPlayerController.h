//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Engine/EngineTypes.h>   // EEndPlayReason
#include <GameFramework/PlayerController.h>
#include <UObject/ObjectMacros.h> // GENERATED_BODY, UCLASS

#include "SpPlayerController.generated.h"

class UUserInputComponent;

UCLASS()
class ASpPlayerController : public APlayerController
{
    GENERATED_BODY()
public:
    ASpPlayerController();
    ~ASpPlayerController();

    // APlayerController interface
    void BeginPlay() override;
    void EndPlay(const EEndPlayReason::Type end_play_reason) override;

private:
    UPROPERTY(EditAnywhere, Category="SPEAR", DisplayName="User Input Component")
    UUserInputComponent* UserInputComponent = nullptr;
};
