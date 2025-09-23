//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Engine/GameEngine.h>
#include <HAL/Platform.h> // TCHAR

#include "SpGameEngine.generated.h"

class FOutputDevice;
class UWorld;

// The purpose of this class is to access Unreal console commands when the game is running.

UCLASS()
class USpGameEngine : public UGameEngine
{
    GENERATED_BODY()
public:
    USpGameEngine();
    ~USpGameEngine() override;

    bool Exec(UWorld* world, const TCHAR* cmd, FOutputDevice& output_device) override;
};
