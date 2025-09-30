//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Engine/LevelStreaming.h>
#include <UObject/Object.h> // UObject

#include "SpCore/Assert.h"

#include "SpLevelStreaming.generated.h"

UCLASS()
class USpLevelStreaming : public UObject
{
    GENERATED_BODY()
public: 
    UFUNCTION()
    static bool ShouldBeVisible(ULevelStreaming* LevelStreaming) { SP_ASSERT(LevelStreaming); return LevelStreaming->ShouldBeVisible(); }
};
