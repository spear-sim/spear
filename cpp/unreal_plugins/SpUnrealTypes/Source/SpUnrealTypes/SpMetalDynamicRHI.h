//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <HAL/Platform.h>                    // uint64
#include <Kismet/BlueprintFunctionLibrary.h> // UBlueprintFunctionLibrary
#include <UObject/ObjectMacros.h>            // GENERATED_BODY, UCLASS, UFUNCTION, UPROPERTY, USTRUCT

#include "SpCore/Assert.h" // SP_ASSERT

#include "SpMetalDynamicRHI.generated.h"

//
// This class is intended to mimic the low-level Metal device queries we need, following the same pattern as our
// other backends (see SpD3D12DynamicRHI.h, SpD3D11DynamicRHI.h, SpVulkanDynamicRHI.h). It transcribes native handles
// as uint64 values and native structs as faithful USTRUCT instances so they can participate in UFUNCTION signatures.
//
// TODO: UE 5.5 does not expose a public IMetalDynamicRHI accessor analogous to GetID3D12DynamicRHI(),
// GetID3D11DynamicRHI(), and GetIVulkanDynamicRHI(), so the functions below are not yet implemented. They will need
// an Objective-C++/metal-cpp implementation that queries id<MTLDevice> (recommendedMaxWorkingSetSize,
// currentAllocatedSize, hasUnifiedMemory) directly.
//

// Faithful transcription of the id<MTLDevice> memory properties we care about.
USTRUCT()
struct FSpMTLDeviceMemoryInfo
{
    GENERATED_BODY()

    UPROPERTY()
    uint64 RecommendedMaxWorkingSetSize = 0;

    UPROPERTY()
    uint64 CurrentAllocatedSize = 0;

    UPROPERTY()
    bool HasUnifiedMemory = false;
};

UCLASS()
class USpMetalDynamicRHIInterface : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()
public:

    // Returns the id<MTLDevice>, reinterpreted as a uint64 handle.
    UFUNCTION(Category="SPEAR") // uint64 is not supported for BlueprintCallable
    static uint64 RHIGetDevice()
    {
        SP_ASSERT(false); // TODO
        return 0;
    }

    UFUNCTION(Category="SPEAR") // uint64 is not supported for BlueprintCallable
    static FSpMTLDeviceMemoryInfo GetDeviceMemoryInfo(uint64 MTLDevice)
    {
        SP_ASSERT(false); // TODO
        return FSpMTLDeviceMemoryInfo();
    }
};
