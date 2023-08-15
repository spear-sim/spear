//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <CoreMinimal.h> // TCHAR
#include <Engine/GameEngine.h>

#include "SpearSimGameEngine.generated.h"

class FOutputDevice;
class UWorld;

// The purpose of this class is to access Unreal console commands when the game is running.

UCLASS()
class USpearSimGameEngine : public UGameEngine
{
    GENERATED_BODY()
public:
    USpearSimGameEngine();
    ~USpearSimGameEngine();

    bool Exec(UWorld* world, const TCHAR* cmd, FOutputDevice& output_device) override;
};
