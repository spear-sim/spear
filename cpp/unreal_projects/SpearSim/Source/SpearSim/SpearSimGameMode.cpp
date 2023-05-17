//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright Epic Games, Inc. All Rights Reserved.
//

#include "SpearSim/SpearSimGameMode.h"

#include <CoreMinimal.h>
#include <Containers/UnrealString.h>
#include <Engine/Engine.h>
#include <Math/Color.h>

#include "CoreUtils/Log.h"
#include "CoreUtils/Unreal.h"
#include "SpearSim/SpearSimSpectatorPawn.h"

class FObjectInitializer;

ASpearSimGameMode::ASpearSimGameMode(const FObjectInitializer& object_initializer) : AGameModeBase(object_initializer)
{
    SP_LOG_CURRENT_FUNCTION();

    DefaultPawnClass = ASpearSimSpectatorPawn::StaticClass();
}

ASpearSimGameMode::~ASpearSimGameMode()
{
    SP_LOG_CURRENT_FUNCTION();
}

void ASpearSimGameMode::spearDebugMessage(float display_time, FString message)
{
    // Note that GEngine->AddOnScreenDebugMessage(...) is only available when the game is running, either in standalone mode or
    // in play-in-editor mode. But in pracice this is not an issue, because UFUNTION(Exec) methods only execute when the game
    // is running anyway.
    uint64 key              = -1;
    FColor color            = FColor::Yellow;
    std::string message_str = SP_LOG_GET_PREFIX() + Unreal::toStdString(message);
    GEngine->AddOnScreenDebugMessage(key, display_time, color, *Unreal::toFString(message_str));
}
