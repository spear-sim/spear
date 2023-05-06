//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright Epic Games, Inc. All Rights Reserved.
//

#pragma once

#include <CoreMinimal.h>
#include <Engine/GameEngine.h>

#include "SpearSimGameEngine.generated.h"

class FOutputDevice;
class UWorld;

// This class is intended to mimic the functionality of SpearSimEditorUnrealEdEngine. SpearSimEditorUnrealEdEngine
// inherits from UnrealEdEngine and is active when the editor is running, whereas this class inherits from UGameEngine
// and is active when the game is running in standalone mode.

UCLASS()
class USpearSimGameEngine : public UGameEngine
{
    GENERATED_BODY()
public:
    USpearSimGameEngine();
    ~USpearSimGameEngine();

    bool Exec(UWorld* world, const TCHAR* cmd, FOutputDevice& output_device) override;
};
