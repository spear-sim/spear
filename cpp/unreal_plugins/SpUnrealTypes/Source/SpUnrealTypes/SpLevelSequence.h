//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Kismet/BlueprintFunctionLibrary.h>
#include <LevelSequence.h>

#include "SpCore/Assert.h"

#include "SpLevelSequence.generated.h"

class UBlueprint;

UCLASS()
class USpLevelSequence : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()
public:
    #if WITH_EDITOR
        UFUNCTION(BlueprintCallable, Category="SPEAR")
        static UBlueprint* GetDirectorBlueprint(ULevelSequence* LevelSequence)
        {
            SP_ASSERT(LevelSequence);
            return LevelSequence->GetDirectorBlueprint();
        }
    #endif
};
