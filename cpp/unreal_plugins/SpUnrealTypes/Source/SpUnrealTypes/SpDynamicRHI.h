//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <string>
#include <vector>

#include <boost/predef.h> // BOOST_OS_LINUX, BOOST_OS_WINDOWS

#include <Containers/Array.h>        // TArray
#include <Containers/UnrealString.h> // FString
#include <DynamicRHI.h>              // GDynamicRHI, RHIGetInterfaceType
#include <HAL/IConsoleManager.h>     // IConsoleManager, IConsoleVariable
#include <HAL/Platform.h>            // int32, uint32, uint64
#include <Kismet/BlueprintFunctionLibrary.h>
#include <RHIDefinitions.h>          // ERHIInterfaceType
#include <RHIStats.h>                // FD3DMemoryStats
#include <Templates/RefCounting.h>   // TRefCountPtr
#include <UObject/ObjectMacros.h>    // GENERATED_BODY, UCLASS, UFUNCTION, UPROPERTY, USTRUCT

#include "SpCore/Assert.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"

#if BOOST_OS_WINDOWS
    #include <ID3D11DynamicRHI.h> // ID3D11DynamicRHI, GetID3D11DynamicRHI
    #include <ID3D12DynamicRHI.h> // ID3D12DynamicRHI, GetID3D12DynamicRHI, FD3D12MinimalAdapterDesc
    #include <DXGIUtilities.h>    // UE::DXGIUtilities::GetD3DMemoryStats, FD3DMemoryStats
#endif

#if BOOST_OS_WINDOWS || BOOST_OS_LINUX
    #include <ranges> // std::views::transform

    #include <IVulkanDynamicRHI.h> // GetIVulkanDynamicRHI, IVulkanDynamicRHI, VkExtensionProperties, VkPhysicalDevice,
                                   // VkPhysicalDeviceMemoryBudgetPropertiesEXT, VkPhysicalDeviceMemoryProperties2,
                                   // VK_EXT_MEMORY_BUDGET_EXTENSION_NAME, VK_MEMORY_HEAP_DEVICE_LOCAL_BIT
#endif

#include "SpDynamicRHI.generated.h"

//
// This struct is intended to be identical to Unreal's FRHIMemoryStats struct, see
// Engine/Source/Runtime/RHI/Public/RHIStats.h (UE 5.8).
//

USTRUCT(BlueprintType)
struct FSpRHIMemoryStats
{
    GENERATED_BODY()

    // Budget assigned by the OS/driver. Total memory the app should use.
    UPROPERTY()
    uint64 BudgetLocal = 0; // GPU VRAM budget

    UPROPERTY()
    uint64 BudgetSystem = 0; // System/host memory budget

    // Currently used memory.
    UPROPERTY()
    uint64 UsedLocal = 0; // GPU VRAM used

    UPROPERTY()
    uint64 UsedSystem = 0; // System/host memory used

    // Over-budget memory.
    UPROPERTY()
    uint64 DemotedLocal = 0;

    UPROPERTY()
    uint64 DemotedSystem = 0;

    // Available memory within budget.
    UPROPERTY()
    uint64 AvailableLocal = 0;

    UPROPERTY()
    uint64 AvailableSystem = 0;
};

//
// Functions in this file wrap the global functions in Engine/Source/Runtime/RHI/Public/DynamicRHI.h.
//

UCLASS()
class USpDynamicRHI : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()
public:

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static FString GetName()
    {
        SP_ASSERT(GDynamicRHI);
        return GDynamicRHI->GetName();
    }

    UFUNCTION()
    static FSpRHIMemoryStats RHIGetMemoryStats() // uint64 not supported for BlueprintCallable
    {
        FSpRHIMemoryStats stats;

        FRHIMemoryStats rhi_memory_stats;
        ::RHIGetMemoryStats(rhi_memory_stats);

        stats.BudgetLocal     = rhi_memory_stats.BudgetLocal;
        stats.BudgetSystem    = rhi_memory_stats.BudgetSystem;
        stats.UsedLocal       = rhi_memory_stats.UsedLocal;
        stats.UsedSystem      = rhi_memory_stats.UsedSystem;
        stats.DemotedLocal    = rhi_memory_stats.DemotedLocal;
        stats.DemotedSystem   = rhi_memory_stats.DemotedSystem;
        stats.AvailableLocal  = rhi_memory_stats.AvailableLocal;
        stats.AvailableSystem = rhi_memory_stats.AvailableSystem;
        return stats;
    }
};
