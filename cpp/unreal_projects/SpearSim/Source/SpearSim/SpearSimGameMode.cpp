//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright Epic Games, Inc. All Rights Reserved.
//

#include "SpearSim/SpearSimGameMode.h"

#include <iostream>

#include <Containers/UnrealString.h>
#include <Engine/Engine.h>

#include "CoreUtils/Unreal.h"
#include "SpearSim/SpearSimSpectatorPawn.h"

ASpearSimGameMode::ASpearSimGameMode(const FObjectInitializer& object_initializer) : AGameModeBase(object_initializer)
{
    std::cout << "[SPEAR | SpearSimGameMode.cpp] ASpearSimGameMode::ASpearSimGameMode" << std::endl;

    DefaultPawnClass = ASpearSimSpectatorPawn::StaticClass();
}

ASpearSimGameMode::~ASpearSimGameMode()
{
    std::cout << "[SPEAR | SpearSimGameMode.cpp] ASpearSimGameMode::~ASpearSimGameMode" << std::endl;
}

void ASpearSimGameMode::spearAddOnScreenDebugMessage(float display_time, FString message)
{
    // Note that GEngine->AddOnScreenDebugMessage(...) is only available when the game is running, either in standalone mode or
    // in play-in-editor mode. But in pracice this is not an issue, because UFUNTION(Exec) methods only execute when the game
    // is running anyway.
    uint64 key = -1;
    GEngine->AddOnScreenDebugMessage(key, display_time, FColor::Yellow, *Unreal::toFString("[SPEAR | SpearSimGameMode.cpp] " + Unreal::toStdString(message)));
}
