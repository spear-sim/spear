//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright Epic Games, Inc. All Rights Reserved.
//

#pragma once

#include <Containers/UnrealString.h>
#include <CoreMinimal.h>
#include <GameFramework/GameModeBase.h>

#include "SpearSimGameMode.generated.h"

// This class has two distinct responsibilities. First, it is responsible for setting AGameModeBase::DefaultPawnClass to ASpearSimSpectatorPawn.
// We need to customize AGameModeBase::DefaultPawnClass so we can use have custom pawn behavior when the user runs the SpearSim executable with
// no additional command-line arguments. There is no other way to set AGameModeBase::DefaultPawnClass programmatically at runtime. Second,
// AGameModeBase is one of a few classes in the Unreal class hierarchy that can have UFUNCTION(Exec) methods for responding to Unreal console
// commands. So we use this class as a central entry point for Unreal console commands. Note that UFUNCTION(Exec) methods will only execute when
// the game is running, either in standalone mode or in play-in-editor mode. To respond to console commands when only the editor is running, we
// need to use SpearSimEditorUnrealEdEngine::Exec(...).

UCLASS()
class ASpearSimGameMode : public AGameModeBase
{
    GENERATED_BODY()
public:
    ASpearSimGameMode(const FObjectInitializer& object_initializer);
    ~ASpearSimGameMode();

    // Call this function by typing the following into the Unreal console: spearAddOnScreenDebugMessage 10.0 Hello World
    UFUNCTION(Exec)
    void spearAddOnScreenDebugMessage(float display_time, FString debug_message);
};
