//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpUnrealTypesEditor/SpUnrealEdEngine.h"

#include <string>

#include <Editor/UnrealEdEngine.h>
#include <HAL/Platform.h> // TCHAR

#include "SpCore/Log.h"
#include "SpCore/Unreal.h"

class FOutputDevice;
class UWorld;

USpUnrealEdEngine::USpUnrealEdEngine()
{
    SP_LOG_CURRENT_FUNCTION();
}

USpUnrealEdEngine::~USpUnrealEdEngine()
{
    SP_LOG_CURRENT_FUNCTION();
}

bool USpUnrealEdEngine::Exec(UWorld* world, const TCHAR* cmd, FOutputDevice& output_device)
{
    std::string cmd_str = Unreal::toStdString(cmd);
    SP_LOG(cmd_str);

    return UUnrealEdEngine::Exec(world, cmd, output_device);
}
