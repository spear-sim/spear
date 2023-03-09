//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright Epic Games, Inc. All Rights Reserved.
//

#include "SpearSimGameMode.h"

#include <GameFramework/SpectatorPawn.h>

ASpearSimGameMode::ASpearSimGameMode(const FObjectInitializer& ObjectInitializer)
    : Super(ObjectInitializer)
{
    DefaultPawnClass = ASpectatorPawn::StaticClass();
}
