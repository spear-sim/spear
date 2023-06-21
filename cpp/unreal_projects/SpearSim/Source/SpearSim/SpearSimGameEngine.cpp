//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright Epic Games, Inc. All Rights Reserved.
//

#include "SpearSim/SpearSimGameEngine.h"

#include <string>

#include <Engine/GameEngine.h>

#include "CoreUtils/Log.h"
#include "CoreUtils/Unreal.h"

USpearSimGameEngine::USpearSimGameEngine()
{
    SP_LOG_CURRENT_FUNCTION();
}

USpearSimGameEngine::~USpearSimGameEngine()
{
    SP_LOG_CURRENT_FUNCTION();
}

bool USpearSimGameEngine::Exec(UWorld* world, const TCHAR* cmd, FOutputDevice& output_device)
{
    std::string cmd_str = Unreal::toStdString(cmd);
    SP_LOG(cmd_str);

    return UGameEngine::Exec(world, cmd, output_device);
}
