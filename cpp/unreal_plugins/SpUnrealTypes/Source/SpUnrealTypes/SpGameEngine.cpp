//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpUnrealTypes/SpGameEngine.h"

#include <string>

#include <Engine/GameEngine.h>
#include <HAL/Platform.h> // TCHAR

#include "SpCore/Log.h"
#include "SpCore/Unreal.h"

class FOutputDevice;
class UWorld;

USpGameEngine::USpGameEngine()
{
    SP_LOG_CURRENT_FUNCTION();
}

USpGameEngine::~USpGameEngine()
{
    SP_LOG_CURRENT_FUNCTION();
}

bool USpGameEngine::Exec(UWorld* world, const TCHAR* cmd, FOutputDevice& output_device)
{
    std::string cmd_str = Unreal::toStdString(cmd);
    SP_LOG(cmd_str);

    return UGameEngine::Exec(world, cmd, output_device);
}
