//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Containers/Array.h>
#include <Engine/World.h>
#include <Kismet/BlueprintFunctionLibrary.h>
#include <UObject/ObjectMacros.h> // GENERATED_BODY, UCLASS, UFUNCTION

#include "SpCore/Assert.h"
#include "SpCore/Unreal.h"

#include "SpUnrealTypes/SpRHIFeatureLevel.h"

#include "SpWorld.generated.h"

class ULevelStreaming;

UCLASS()
class USpWorld : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()
public:
    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static ESpRHIFeatureLevel GetFeatureLevel(UWorld* World)
    {
        SP_ASSERT(World);
        return Unreal::getEnumValueAs<ESpRHIFeatureLevel>(World->GetFeatureLevel());
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static TArray<ULevelStreaming*> GetStreamingLevels(UWorld* World)
    {
        SP_ASSERT(World);
        return World->GetStreamingLevels();
    }
};
