//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <GameFramework/PlayerController.h>
#include <UObject/ObjectMacros.h> // GENERATED_BODY, UCLASS

#include "SpPlayerController.generated.h"

UCLASS()
class ASpPlayerController : public APlayerController
{
    GENERATED_BODY()
public:
    ASpPlayerController();
    ~ASpPlayerController();
};
