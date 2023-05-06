//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright Epic Games, Inc. All Rights Reserved.
//

#pragma once

#include <Containers/UnrealString.h>
#include <CoreMinimal.h>
#include <GameFramework/GameModeBase.h>

#include "SpearSimGameMode.generated.h"

UCLASS()
class ASpearSimGameMode : public AGameModeBase
{
    GENERATED_BODY()
public:
    ASpearSimGameMode(const FObjectInitializer& object_initializer);
    ~ASpearSimGameMode();

    UFUNCTION(Exec)
    void SpearSimAddOnScreenDebugMessage(float display_time, FString debug_message);
};
