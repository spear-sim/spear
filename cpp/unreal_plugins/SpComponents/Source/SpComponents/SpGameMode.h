//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Containers/UnrealString.h> // FString
#include <GameFramework/GameModeBase.h>
#include <UObject/ObjectMacros.h>    // GENERATED_BODY, UCLASS, UFUNCTION

#include "SpGameMode.generated.h"

class APlayerController;

// This class has two distinct responsibilities. First, it is responsible for setting AGameModeBase::DefaultPawnClass
// and AGameModeBase::PlayerControllerClass. We need to set these classes in a game mode so we can implement
// custom spectator pawn behavior in the SpearSim executable. After defining our game mode, it can be
// specified to Unreal in DefaultEngine.ini, or set very early in the application's life cycle (see SpServices/GameMapSettingsService.h),
// or specified when loading a new map. Second, AGameModeBase is one of a few classes in the Unreal class
// hierarchy that can have UFUNCTION(Exec) methods for responding to Unreal console commands. So we use this
// class as a central entry point for Unreal console commands. Note that UFUNCTION(Exec) methods will only
// execute when the game is running, either in standalone mode or in play-in-editor mode. To respond to
// console commands when only the editor is running, we need to use SpUnrealEdEngine::Exec(...).

UCLASS()
class ASpGameMode : public AGameModeBase
{
    GENERATED_BODY()
public:
    ASpGameMode();
    ~ASpGameMode();

    // AGameModeBase interface
    void PostLogin(APlayerController* new_player) override;

private:
    // Call this function by typing the following into the Unreal console: SpAddOnScreenDebugMessage 10.0 Hello World
    UFUNCTION(Exec)
    void SpAddOnScreenDebugMessage(float display_time, FString message);
};
