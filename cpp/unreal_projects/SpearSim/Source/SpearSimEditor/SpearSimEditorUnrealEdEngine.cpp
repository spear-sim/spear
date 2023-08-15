//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpearSimEditor/SpearSimEditorUnrealEdEngine.h"

#include <string>

#include <CoreMinimal.h> // TCHAR
#include <Editor/UnrealEdEngine.h>

#include "CoreUtils/Log.h"
#include "CoreUtils/Unreal.h"

USpearSimEditorUnrealEdEngine::USpearSimEditorUnrealEdEngine()
{
    SP_LOG_CURRENT_FUNCTION();
}

USpearSimEditorUnrealEdEngine::~USpearSimEditorUnrealEdEngine()
{
    SP_LOG_CURRENT_FUNCTION();
}

bool USpearSimEditorUnrealEdEngine::Exec(UWorld* world, const TCHAR* cmd, FOutputDevice& output_device)
{
    std::string cmd_str = Unreal::toStdString(cmd);
    SP_LOG(cmd_str);

    return UUnrealEdEngine::Exec(world, cmd, output_device);
}
