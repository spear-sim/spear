//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <ContentStreaming.h>                        // IStreamingManager
#include <Engine/StreamableRenderAsset.h>            // UStreamableRenderAsset
#include <HAL/Platform.h>                            // int32
#include <Kismet/BlueprintFunctionLibrary.h>
#include <Streaming/StreamableRenderResourceState.h> // FStreamableRenderResourceState
#include <UObject/ObjectMacros.h>                    // GENERATED_BODY, UCLASS, UFUNCTION
#include <UObject/UObjectIterator.h>                 // TObjectIterator

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

    // IStreamingManager doesn't provide a non-blocking way to get the number of pending streaming requests. The
    // number of pending requests is only returned by BlockTillAllRequestsFinished(...) and StreamAllResources(...),
    // which both block. Of all the streaming managers, only FRenderAssetStreamingManager reports a nonzero count of
    // pending requests from BlockTillAllRequestsFinished(...), and it computes this count by counting streamable render
    // assets whose requested LOD count differs from their resident LOD count. So we compute the same count here without
    // blocking. FRenderAssetStreamingManager calls UStreamableRenderAsset::TickStreaming(...) on each asset before
    // reading its state, but we can't call TickStreaming(...) because it isn't exported from the Engine module. However,
    // FRenderAssetStreamingManager calls TickStreaming(...) on each asset every frame anyway, so the state we read here
    // is at most one frame stale.
    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static int32 GetNumPendingRenderAssetRequests()
    {
        int32 num_pending_requests = 0;
        for (TObjectIterator<UStreamableRenderAsset> itr; itr; ++itr) {
            UStreamableRenderAsset* streamable_render_asset = *itr;
            SP_ASSERT(streamable_render_asset);
            if (streamable_render_asset->IsStreamable()) {
                const FStreamableRenderResourceState& streamable_render_resource_state = streamable_render_asset->GetStreamableResourceState();
                if (streamable_render_resource_state.IsValid() && streamable_render_resource_state.NumRequestedLODs != streamable_render_resource_state.NumResidentLODs) {
                    num_pending_requests++;
                }
            }
        }
        return num_pending_requests;
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
