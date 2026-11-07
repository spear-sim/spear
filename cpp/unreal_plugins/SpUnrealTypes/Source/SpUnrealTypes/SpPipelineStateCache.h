//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <HAL/Platform.h>         // int32, int64
#include <Kismet/BlueprintFunctionLibrary.h>
#include <PipelineStateCache.h>   // PipelineStateCache
#include <RHICommandList.h>       // FRHICommandListImmediate
#include <RenderingThread.h>      // ENQUEUE_RENDER_COMMAND, FlushRenderingCommands
#include <UObject/ObjectMacros.h> // GENERATED_BODY, UCLASS, UFUNCTION

#include "SpPipelineStateCache.generated.h"

UCLASS()
class USpPipelineStateCache : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()
public:

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static void WaitForAllTasks()
    {
        ENQUEUE_RENDER_COMMAND(SpWaitForAllTasks)(
            [](FRHICommandListImmediate& rhi_command_list) {
                PipelineStateCache::WaitForAllTasks();
            });
        FlushRenderingCommands();
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static int32 GetNumActivePipelinePrecompileTasks()
    {
        FlushRenderingCommands(); // force rendering thread to be fully up-to-date before querying GetNumActivePipelinePrecompileTasks()
        return PipelineStateCache::GetNumActivePipelinePrecompileTasks();
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static bool IsPrecaching()
    {
        FlushRenderingCommands(); // force rendering thread to be fully up-to-date before querying IsPrecaching()
        return PipelineStateCache::IsPrecaching();
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static int64 NumActivePrecacheRequests() // uint32 not supported for BlueprintCallable
    {
        FlushRenderingCommands(); // force rendering thread to be fully up-to-date before querying NumActivePrecacheRequests()
        return PipelineStateCache::NumActivePrecacheRequests();
    }
};
