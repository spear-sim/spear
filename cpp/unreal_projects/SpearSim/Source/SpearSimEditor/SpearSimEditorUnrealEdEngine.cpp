//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright Epic Games, Inc. All Rights Reserved.
//

#include "SpearSimEditor/SpearSimEditorUnrealEdEngine.h"

#include <iostream>

#include <CoreMinimal.h>
#include <Editor/UnrealEdEngine.h>

#include "CoreUtils/Unreal.h"

DEFINE_LOG_CATEGORY(LogSpearSimEditor);

USpearSimEditorUnrealEdEngine::USpearSimEditorUnrealEdEngine()
{
    std::cout << "[SPEAR | SpearSimEditorUnrealEdEngine.cpp] USpearSimGameEngine::USpearSimGameEngine" << std::endl;
    UE_LOG(LogSpearSimEditor, Log, TEXT("[SPEAR | SpearSimEditorUnrealEdEngine.cpp] USpearSimGameEngine::USpearSimGameEngine"));
}

USpearSimEditorUnrealEdEngine::~USpearSimEditorUnrealEdEngine()
{
    std::cout << "[SPEAR | SpearSimEditorUnrealEdEngine.cpp] USpearSimGameEngine::~USpearSimGameEngine" << std::endl;
    UE_LOG(LogSpearSimEditor, Log, TEXT("[SPEAR | SpearSimEditorUnrealEdEngine.cpp] USpearSimGameEngine::USpearSimGameEngine"));
}

bool USpearSimEditorUnrealEdEngine::Exec(UWorld* world, const TCHAR* cmd, FOutputDevice& output_device)
{
    std::cout << "[SPEAR | SpearSimEditorUnrealEdEngine.cpp] " << Unreal::toStdString(cmd) << std::endl;
    UE_LOG(LogSpearSimEditor, Log, TEXT("[SPEAR | SpearSimEditorUnrealEdEngine.cpp] %s"), cmd);

    return UUnrealEdEngine::Exec(world, cmd, output_device);
}
