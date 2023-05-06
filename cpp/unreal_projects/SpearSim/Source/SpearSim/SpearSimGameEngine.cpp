//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright Epic Games, Inc. All Rights Reserved.
//

#include "SpearSim/SpearSimGameEngine.h"

#include <iostream>

#include <CoreMinimal.h>
#include <Engine/GameEngine.h>

#include "CoreUtils/Unreal.h"

USpearSimGameEngine::USpearSimGameEngine()
{
    std::cout << "[SPEAR | SpearSimGameEngine.cpp] USpearSimGameEngine::USpearSimGameEngine" << std::endl;
}

USpearSimGameEngine::~USpearSimGameEngine()
{
    std::cout << "[SPEAR | SpearSimGameEngine.cpp] USpearSimGameEngine::~USpearSimGameEngine" << std::endl;
}

bool USpearSimGameEngine::Exec(UWorld* world, const TCHAR* cmd, FOutputDevice& output_device)
{
    std::cout << "[SPEAR | SpearSimGameEngine.cpp] " << Unreal::toStdString(cmd) << std::endl;

    return UGameEngine::Exec(world, cmd, output_device);
}
