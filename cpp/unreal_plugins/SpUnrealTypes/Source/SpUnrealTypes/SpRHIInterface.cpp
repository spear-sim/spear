//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpUnrealTypes/SpRHIInterface.h"

#include <string.h> // strcmp

#include <DynamicRHI.h>              // GDynamicRHI
#include <HAL/Platform.h>            // uint32, uint64
#if PLATFORM_WINDOWS
    #include <ID3D12DynamicRHI.h>        // GetID3D12DynamicRHI, ID3D12DynamicRHI
#endif
#if PLATFORM_WINDOWS || PLATFORM_LINUX
    #include <IVulkanDynamicRHI.h>       // GetIVulkanDynamicRHI, IVulkanDynamicRHI
#endif
#include <RHI.h>                     // FRHIResourceStats, RHIGetTrackedResourceStats
#include <RHIDefinitions.h>          // ERHIInterfaceType
#include <RHIFwd.h>                  // RHI_ENABLE_RESOURCE_INFO
#if PLATFORM_WINDOWS
    #include <Templates/RefCounting.h>   // TRefCountPtr
#endif
#include <Templates/SharedPointer.h> // TSharedPtr

#include "SpCore/Assert.h"

namespace
{
    struct FSpRHIMemoryStats
    {
        bool is_valid = false;
        uint64 budget_bytes = 0;
        uint64 used_bytes = 0;
        uint64 physical_bytes = 0;
        uint64 system_budget_bytes = 0;
        uint64 system_used_bytes = 0;
    };

    #if PLATFORM_WINDOWS
    bool getD3D12MemoryStats(FSpRHIMemoryStats& out_stats)
    {
        // We only ever query GPU node 0 (the primary adapter). Unlike the engine's own
        // FD3D12Adapter::CollectMemoryStats(...), we deliberately don't special-case explicit multi-GPU AFR setups,
        // since SPEAR doesn't use them.
        ID3D12Device* device = GetID3D12DynamicRHI()->RHIGetDevice(0);
        if (!device) {
            return false;
        }

        LUID adapter_luid = device->GetAdapterLuid();

        TRefCountPtr<IDXGIFactory4> dxgi_factory;
        if (FAILED(CreateDXGIFactory2(0, IID_PPV_ARGS(dxgi_factory.GetInitReference())))) {
            return false;
        }

        TRefCountPtr<IDXGIAdapter> dxgi_adapter;
        if (FAILED(dxgi_factory->EnumAdapterByLuid(adapter_luid, IID_PPV_ARGS(dxgi_adapter.GetInitReference())))) {
            return false;
        }

        DXGI_ADAPTER_DESC adapter_desc;
        if (FAILED(dxgi_adapter->GetDesc(&adapter_desc))) {
            return false;
        }

        TRefCountPtr<IDXGIAdapter3> dxgi_adapter_3;
        if (FAILED(dxgi_adapter->QueryInterface(IID_PPV_ARGS(dxgi_adapter_3.GetInitReference())))) {
            return false;
        }

        DXGI_QUERY_VIDEO_MEMORY_INFO local_memory_info;
        if (FAILED(dxgi_adapter_3->QueryVideoMemoryInfo(0, DXGI_MEMORY_SEGMENT_GROUP_LOCAL, &local_memory_info))) {
            return false;
        }

        DXGI_QUERY_VIDEO_MEMORY_INFO non_local_memory_info;
        if (FAILED(dxgi_adapter_3->QueryVideoMemoryInfo(0, DXGI_MEMORY_SEGMENT_GROUP_NON_LOCAL, &non_local_memory_info))) {
            return false;
        }

        out_stats.budget_bytes = local_memory_info.Budget;
        out_stats.used_bytes = local_memory_info.CurrentUsage;
        out_stats.physical_bytes = adapter_desc.DedicatedVideoMemory;
        out_stats.system_budget_bytes = non_local_memory_info.Budget;
        out_stats.system_used_bytes = non_local_memory_info.CurrentUsage;
        return true;
    }
    #endif

    #if PLATFORM_WINDOWS || PLATFORM_LINUX
    bool getVulkanMemoryStats(FSpRHIMemoryStats& out_stats)
    {
        IVulkanDynamicRHI* vulkan_rhi = GetIVulkanDynamicRHI();
        VkPhysicalDevice physical_device = vulkan_rhi->RHIGetVkPhysicalDevice();

        // VK_EXT_memory_budget's fields are only meaningful if the physical device actually supports it, and
        // chaining the extension struct in when it's unsupported triggers a validation error, so we check first.
        bool supports_memory_budget = false;
        for (const VkExtensionProperties& extension : vulkan_rhi->RHIGetAllDeviceExtensions(physical_device)) {
            if (strcmp(extension.extensionName, VK_EXT_MEMORY_BUDGET_EXTENSION_NAME) == 0) {
                supports_memory_budget = true;
                break;
            }
        }
        if (!supports_memory_budget) {
            return false;
        }

        PFN_vkGetPhysicalDeviceMemoryProperties2 get_memory_properties_2 =
            reinterpret_cast<PFN_vkGetPhysicalDeviceMemoryProperties2>(vulkan_rhi->RHIGetVkInstanceProcAddr("vkGetPhysicalDeviceMemoryProperties2"));
        if (!get_memory_properties_2) {
            get_memory_properties_2 =
                reinterpret_cast<PFN_vkGetPhysicalDeviceMemoryProperties2>(vulkan_rhi->RHIGetVkInstanceProcAddr("vkGetPhysicalDeviceMemoryProperties2KHR"));
        }
        if (!get_memory_properties_2) {
            return false;
        }

        VkPhysicalDeviceMemoryBudgetPropertiesEXT memory_budget = {};
        memory_budget.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_MEMORY_BUDGET_PROPERTIES_EXT;

        VkPhysicalDeviceMemoryProperties2 memory_properties_2 = {};
        memory_properties_2.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_MEMORY_PROPERTIES_2;
        memory_properties_2.pNext = &memory_budget;

        get_memory_properties_2(physical_device, &memory_properties_2);

        for (uint32 heap = 0; heap < memory_properties_2.memoryProperties.memoryHeapCount; ++heap) {
            const VkMemoryHeap& memory_heap = memory_properties_2.memoryProperties.memoryHeaps[heap];
            if (memory_heap.flags & VK_MEMORY_HEAP_DEVICE_LOCAL_BIT) {
                out_stats.budget_bytes += memory_budget.heapBudget[heap];
                out_stats.used_bytes += memory_budget.heapUsage[heap];
                out_stats.physical_bytes += memory_heap.size;
            } else {
                out_stats.system_budget_bytes += memory_budget.heapBudget[heap];
                out_stats.system_used_bytes += memory_budget.heapUsage[heap];
            }
        }

        return true;
    }
    #endif

    FSpRHIMemoryStats getRHIMemoryStats()
    {
        FSpRHIMemoryStats stats;

        if (!GDynamicRHI) {
            return stats;
        }

        ERHIInterfaceType interface_type = GDynamicRHI->GetInterfaceType();

        if (interface_type == ERHIInterfaceType::D3D12) {
            #if PLATFORM_WINDOWS
                stats.is_valid = getD3D12MemoryStats(stats);
            #endif
        } else if (interface_type == ERHIInterfaceType::Vulkan) {
            #if PLATFORM_WINDOWS || PLATFORM_LINUX
                stats.is_valid = getVulkanMemoryStats(stats);
            #endif
        }

        return stats;
    }
}

uint64 USpRHIInterface::GetTotalVideoMemoryBytes()
{
    return getRHIMemoryStats().budget_bytes;
}

uint64 USpRHIInterface::GetUsedVideoMemoryBytes()
{
    return getRHIMemoryStats().used_bytes;
}

uint64 USpRHIInterface::GetTotalPhysicalVideoMemoryBytes()
{
    return getRHIMemoryStats().physical_bytes;
}

uint64 USpRHIInterface::GetTotalSystemMemoryBytes()
{
    return getRHIMemoryStats().system_budget_bytes;
}

uint64 USpRHIInterface::GetUsedSystemMemoryBytes()
{
    return getRHIMemoryStats().system_used_bytes;
}

TArray<FSpRHIResourceStats> USpRHIInterface::GetResourceStats()
{
    TArray<FSpRHIResourceStats> result_stats;
    #if RHI_ENABLE_RESOURCE_INFO
        TArray<TSharedPtr<FRHIResourceStats>> orig_stats;
        RHIGetTrackedResourceStats(orig_stats);
        result_stats.SetNum(orig_stats.Num());
        for (int32 i = 0; i < orig_stats.Num(); i++) {
            TSharedPtr<FRHIResourceStats> orig = orig_stats[i];
            if (!orig.IsValid() || orig->SizeInBytes == 0) {
                continue;
            }
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
