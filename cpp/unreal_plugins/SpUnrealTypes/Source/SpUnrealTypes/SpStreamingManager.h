//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <ContentStreaming.h> // IStreamingManager
#include <HAL/Platform.h>     // int32
#include <Kismet/BlueprintFunctionLibrary.h>

#include "SpCore/Assert.h"

#include "SpStreamingManager.generated.h"

UCLASS()
class USpStreamingManager : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()
public:

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static bool IsStreamingEnabled()
    {
        return IStreamingManager::Get().IsStreamingEnabled();
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static int32 GetNumWantingResources()
    {
        return IStreamingManager::Get().GetNumWantingResources();
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static void UpdateResourceStreaming(float DeltaTime, bool bProcessEverything = false)
    {
        IStreamingManager::Get().UpdateResourceStreaming(DeltaTime, bProcessEverything);
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static int32 BlockTillAllRequestsFinished(float TimeLimit = 0.0f, bool bLogResults = false)
    {
        return IStreamingManager::Get().BlockTillAllRequestsFinished(TimeLimit, bLogResults);
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static int32 StreamAllResources(float TimeLimit = 0.0f)
    {
        return IStreamingManager::Get().StreamAllResources(TimeLimit);
    }
};
