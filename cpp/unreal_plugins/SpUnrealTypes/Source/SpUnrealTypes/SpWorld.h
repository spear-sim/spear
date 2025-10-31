//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Containers/Array.h>
#include <Engine/World.h>
#include <Kismet/BlueprintFunctionLibrary.h>

#include "SpCore/Assert.h"

#include "SpWorld.generated.h"

class ULevelStreaming;

UCLASS()
class USpWorld : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()
public: 
    UFUNCTION()
    static TArray<ULevelStreaming*> GetStreamingLevels(UWorld* World)
    {
        SP_ASSERT(World);
        return World->GetStreamingLevels();
    }
};
