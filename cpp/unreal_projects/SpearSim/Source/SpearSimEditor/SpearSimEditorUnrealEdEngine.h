//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright Epic Games, Inc. All Rights Reserved.
//

#pragma once

#include <CoreMinimal.h>
#include <Editor/UnrealEdEngine.h>

#include "SpearSimEditorUnrealEdEngine.generated.h"

class FOutputDevice;
class UWorld;

DECLARE_LOG_CATEGORY_EXTERN(LogSpearSimEditor, Log, All);

UCLASS()
class USpearSimEditorUnrealEdEngine : public UUnrealEdEngine
{
    GENERATED_BODY()
public:
    USpearSimEditorUnrealEdEngine();
    ~USpearSimEditorUnrealEdEngine();

    bool Exec(UWorld* world, const TCHAR* cmd, FOutputDevice& output_device);
};
