//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright Epic Games, Inc. All Rights Reserved.
//

#include "SpearSim/SpearSimGameMode.h"

#include <iostream>

#include "SpearSim/SpearSimSpectatorPawn.h"


#include "OpenBot/OpenBotPawn.h"

ASpearSimGameMode::ASpearSimGameMode(const FObjectInitializer& object_initializer) : AGameModeBase(object_initializer)
{
    std::cout << "[SPEAR | SpearSimGameMode.cpp] ASpearSimGameMode::ASpearSimGameMode" << std::endl;

    //DefaultPawnClass = ASpearSimSpectatorPawn::StaticClass();
    //DefaultPawnClass = AOpenBotPawn::StaticClass();
}

ASpearSimGameMode::~ASpearSimGameMode()
{
    std::cout << "[SPEAR | SpearSimGameMode.cpp] ASpearSimGameMode::~ASpearSimGameMode" << std::endl;
}
