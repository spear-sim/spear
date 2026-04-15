//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <DistanceFieldAtlas.h> // FDistanceFieldAsyncQueue, GDistanceFieldAsyncQueue
#include <HAL/Platform.h>       // int32
#include <Kismet/BlueprintFunctionLibrary.h>

#include "SpCore/Assert.h"

#include "SpDistanceFieldAsyncQueue.generated.h"

UCLASS()
class USpDistanceFieldAsyncQueue : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()
public:

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static int32 GetNumOutstandingTasks()
    {
        SP_ASSERT(GDistanceFieldAsyncQueue);
        return GDistanceFieldAsyncQueue->GetNumOutstandingTasks();
    }
};
