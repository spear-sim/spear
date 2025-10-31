//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Engine/EngineTypes.h>   // EEndPlayReason
#include <GameFramework/PlayerController.h>
#include <UObject/ObjectMacros.h> // GENERATED_BODY, UCLASS, UPROPERTY

#include "SpPlayerController.generated.h"

class USpUserInputComponent;

UCLASS()
class ASpPlayerController : public APlayerController
{
    GENERATED_BODY()
public:
    ASpPlayerController();
    ~ASpPlayerController() override;

    // APlayerController interface
    void BeginPlay() override;
    void EndPlay(const EEndPlayReason::Type end_play_reason) override;

private:
    UPROPERTY(VisibleAnywhere, Category="SPEAR")
    USpUserInputComponent* SpUserInputComponent = nullptr;
};
