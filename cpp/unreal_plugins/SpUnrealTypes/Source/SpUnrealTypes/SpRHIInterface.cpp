//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpUnrealTypes/SpRHIInterface.h"

#include <boost/predef.h> // BOOST_OS_LINUX, BOOST_OS_MACOS, BOOST_OS_WINDOWS

#include <Containers/Array.h>        // TArray
#include <DynamicRHI.h>              // GDynamicRHI
#include <HAL/Platform.h>            // uint64
#include <RHI.h>                     // FRHIResourceStats, RHIGetTrackedResourceStats
#include <RHIDefinitions.h>          // ERHIInterfaceType
#include <RHIFwd.h>                  // RHI_ENABLE_RESOURCE_INFO
#include <Templates/SharedPointer.h> // TSharedPtr
#include "SpCore/Assert.h" // SP_ASSERT
#include "SpUnrealTypes/SpD3D11DynamicRHI.h"  // USpD3D11DynamicRHIInterface
#include "SpUnrealTypes/SpD3D12DynamicRHI.h"  // USpD3D12DynamicRHIInterface
#include "SpUnrealTypes/SpDXGI.h"             // ESpDXGIMemorySegmentGroup, FSpDXGIQueryVideoMemoryInfo
#include "SpUnrealTypes/SpMetalDynamicRHI.h"  // USpMetalDynamicRHIInterface
#include "SpUnrealTypes/SpVulkanDynamicRHI.h" // FSpVkMemoryHeap, FSpVkPhysicalDeviceMemoryProperties2, USpVulkanDynamicRHIInterface

#if BOOST_OS_WINDOWS || BOOST_OS_LINUX
    #include <IVulkanDynamicRHI.h> // VK_MEMORY_HEAP_DEVICE_LOCAL_BIT
#endif

FSpRHIMemoryStats USpRHIInterface::GetMemoryStats()
{
    FSpRHIMemoryStats stats;

    SP_ASSERT(GDynamicRHI);
    ERHIInterfaceType interface_type = GDynamicRHI->GetInterfaceType();

    if (interface_type == ERHIInterfaceType::D3D12) {
        #if BOOST_OS_WINDOWS
            uint64 d3d12_device = USpD3D12DynamicRHIInterface::RHIGetDevice(0);
            FSpDXGIQueryVideoMemoryInfo local_info = USpD3D12DynamicRHIInterface::QueryVideoMemoryInfo(d3d12_device, ESpDXGIMemorySegmentGroup::Local);
            FSpDXGIQueryVideoMemoryInfo non_local_info = USpD3D12DynamicRHIInterface::QueryVideoMemoryInfo(d3d12_device, ESpDXGIMemorySegmentGroup::NonLocal);
            stats.BudgetLocal = local_info.Budget;
            stats.UsedLocal = local_info.CurrentUsage;
            stats.BudgetSystem = non_local_info.Budget;
            stats.UsedSystem = non_local_info.CurrentUsage;
        #endif
    } else if (interface_type == ERHIInterfaceType::D3D11) {
        #if BOOST_OS_WINDOWS
            uint64 dxgi_adapter = USpD3D11DynamicRHIInterface::RHIGetAdapter();
            FSpDXGIQueryVideoMemoryInfo local_info = USpD3D11DynamicRHIInterface::QueryVideoMemoryInfo(dxgi_adapter, ESpDXGIMemorySegmentGroup::Local);
            FSpDXGIQueryVideoMemoryInfo non_local_info = USpD3D11DynamicRHIInterface::QueryVideoMemoryInfo(dxgi_adapter, ESpDXGIMemorySegmentGroup::NonLocal);
            stats.BudgetLocal = local_info.Budget;
            stats.UsedLocal = local_info.CurrentUsage;
            stats.BudgetSystem = non_local_info.Budget;
            stats.UsedSystem = non_local_info.CurrentUsage;
        #endif
    } else if (interface_type == ERHIInterfaceType::Vulkan) {
        #if BOOST_OS_WINDOWS || BOOST_OS_LINUX
            uint64 vk_physical_device = USpVulkanDynamicRHIInterface::RHIGetVkPhysicalDevice();
            FSpVkPhysicalDeviceMemoryProperties2 memory_properties = USpVulkanDynamicRHIInterface::VkGetPhysicalDeviceMemoryProperties2(vk_physical_device);
            for (int i = 0; i < memory_properties.MemoryProperties.MemoryHeaps.Num(); i++) {
                const FSpVkMemoryHeap& memory_heap = memory_properties.MemoryProperties.MemoryHeaps[i];
                if (memory_heap.Flags & VK_MEMORY_HEAP_DEVICE_LOCAL_BIT) {
                    stats.BudgetLocal += memory_properties.MemoryBudget.HeapBudget[i];
                    stats.UsedLocal += memory_properties.MemoryBudget.HeapUsage[i];
                } else {
                    stats.BudgetSystem += memory_properties.MemoryBudget.HeapBudget[i];
                    stats.UsedSystem += memory_properties.MemoryBudget.HeapUsage[i];
                }
            }
        #endif
    } else if (interface_type == ERHIInterfaceType::Metal) {
        #if BOOST_OS_MACOS
            uint64 mtl_device = USpMetalDynamicRHIInterface::RHIGetDevice();
            stats.BudgetLocal = USpMetalDynamicRHIInterface::RecommendedMaxWorkingSetSize(mtl_device);
            stats.UsedLocal = USpMetalDynamicRHIInterface::CurrentAllocatedSize(mtl_device);
        #endif
    }

    stats.AvailableLocal = (stats.UsedLocal < stats.BudgetLocal) ? (stats.BudgetLocal - stats.UsedLocal) : 0;
    stats.DemotedLocal = (stats.UsedLocal > stats.BudgetLocal) ? (stats.UsedLocal - stats.BudgetLocal) : 0;
    stats.AvailableSystem = (stats.UsedSystem < stats.BudgetSystem) ? (stats.BudgetSystem - stats.UsedSystem) : 0;
    stats.DemotedSystem = (stats.UsedSystem > stats.BudgetSystem) ? (stats.UsedSystem - stats.BudgetSystem) : 0;

    return stats;
}

TArray<FSpRHIResourceStats> USpRHIInterface::GetResourceStats()
{
    TArray<FSpRHIResourceStats> result_stats;
    #if RHI_ENABLE_RESOURCE_INFO
        TArray<TSharedPtr<FRHIResourceStats>> orig_stats;
        RHIGetTrackedResourceStats(orig_stats);
        for (int i = 0; i < orig_stats.Num(); i++) {
            TSharedPtr<FRHIResourceStats> orig = orig_stats[i];
            SP_ASSERT(orig.IsValid());
            FSpRHIResourceStats stats;
            stats.Name = orig->Name.ToString();
            stats.OwnerName = orig->OwnerName.ToString();
            stats.Type = orig->Type;
            stats.Flags = orig->Flags;
            stats.SizeInBytes = orig->SizeInBytes;
            stats.bResident = orig->bResident;
            stats.bMarkedForDelete = orig->bMarkedForDelete;
            stats.bTransient = orig->bTransient;
            stats.bStreaming = orig->bStreaming;
            stats.bRenderTarget = orig->bRenderTarget;
            stats.bDepthStencil = orig->bDepthStencil;
            stats.bUnorderedAccessView = orig->bUnorderedAccessView;
            stats.bRayTracingAccelerationStructure = orig->bRayTracingAccelerationStructure;
            stats.bHasFlags = orig->bHasFlags;
            result_stats.Add(stats);
        }
    #endif
    return result_stats;
}
