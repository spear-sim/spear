//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Containers/Array.h>        // TArray
#include <Containers/UnrealString.h> // FString
#include <HAL/Platform.h>            // uint64
#include <Kismet/BlueprintFunctionLibrary.h>
#include <RHI.h>                     // FRHIResourceStats, IsRHIDeviceAMD, IsRHIDeviceApple, IsRHIDeviceIntel, IsRHIDeviceNVIDIA, RHIGetTrackedResourceStats
#include <RHIFwd.h>                  // RHI_ENABLE_RESOURCE_INFO
#include <Templates/SharedPointer.h> // TSharedPtr
#include <UObject/ObjectMacros.h>    // GENERATED_BODY, UCLASS, UFUNCTION, UPROPERTY, USTRUCT

#include "SpCore/Assert.h"

#include "SpRHI.generated.h"

//
// This struct mirrors Unreal's FRHIResourceStats struct, see Engine/Source/Runtime/RHI/Public/RHI.h (UE 5.8). The
// Name and OwnerName fields are FName in Unreal, but we transcribe them as FString.
//

USTRUCT(BlueprintType)
struct FSpRHIResourceStats
{
    GENERATED_BODY()

    UPROPERTY()
    FString Name;

    UPROPERTY()
    FString OwnerName;

    UPROPERTY()
    FString Type;

    UPROPERTY()
    FString Flags;

    UPROPERTY()
    uint64 SizeInBytes = 0;

    UPROPERTY()
    bool bResident = false;

    UPROPERTY()
    bool bMarkedForDelete = false;

    UPROPERTY()
    bool bTransient = false;

    UPROPERTY()
    bool bStreaming = false;

    UPROPERTY()
    bool bRenderTarget = false;

    UPROPERTY()
    bool bDepthStencil = false;

    UPROPERTY()
    bool bUnorderedAccessView = false;

    UPROPERTY()
    bool bRayTracingAccelerationStructure = false;

    UPROPERTY()
    bool bHasFlags = false;
};

//
// Functions in this file wrap the free functions in Engine/Source/Runtime/RHI/Public/RHI.h.
//

UCLASS()
class USpRHI : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()
public:

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static bool IsRHIDeviceNVIDIA()
    {
        return ::IsRHIDeviceNVIDIA();
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static bool IsRHIDeviceAMD()
    {
        return ::IsRHIDeviceAMD();
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static bool IsRHIDeviceIntel()
    {
        return ::IsRHIDeviceIntel();
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static bool IsRHIDeviceApple()
    {
        return ::IsRHIDeviceApple();
    }

    // Mimics ::RHIGetTrackedResourceStats(...), see Engine/Source/Runtime/RHI/Public/RHI.h (UE 5.8). Takes a fresh
    // snapshot of all currently tracked RHI resources. Not BlueprintCallable because uint64 is not a Blueprint type.
    UFUNCTION()
    static TArray<FSpRHIResourceStats> RHIGetTrackedResourceStats()
    {
        TArray<FSpRHIResourceStats> sp_stats_array;
        #if RHI_ENABLE_RESOURCE_INFO
            TArray<TSharedPtr<FRHIResourceStats>> stats_array;
            ::RHIGetTrackedResourceStats(stats_array);
            for (int i = 0; i < stats_array.Num(); i++) {
                TSharedPtr<FRHIResourceStats> stats = stats_array[i];
                SP_ASSERT(stats.IsValid());
                FSpRHIResourceStats sp_stats;
                sp_stats.Name                             = stats->Name.ToString();
                sp_stats.OwnerName                        = stats->OwnerName.ToString();
                sp_stats.Type                             = stats->Type;
                sp_stats.Flags                            = stats->Flags;
                sp_stats.SizeInBytes                      = stats->SizeInBytes;
                sp_stats.bResident                        = stats->bResident;
                sp_stats.bMarkedForDelete                 = stats->bMarkedForDelete;
                sp_stats.bTransient                       = stats->bTransient;
                sp_stats.bStreaming                       = stats->bStreaming;
                sp_stats.bRenderTarget                    = stats->bRenderTarget;
                sp_stats.bDepthStencil                    = stats->bDepthStencil;
                sp_stats.bUnorderedAccessView             = stats->bUnorderedAccessView;
                sp_stats.bRayTracingAccelerationStructure = stats->bRayTracingAccelerationStructure;
                sp_stats.bHasFlags                        = stats->bHasFlags;
                sp_stats_array.Add(sp_stats);
            }
        #endif
        return sp_stats_array;
    }
};
