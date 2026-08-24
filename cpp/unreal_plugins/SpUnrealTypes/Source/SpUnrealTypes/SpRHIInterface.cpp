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

#if BOOST_OS_WINDOWS
    // ID3D11DynamicRHI.h and ID3D12DynamicRHI.h pull in the DXGI types below via D3D11ThirdParty.h / D3D12ThirdParty.h,
    // which wrap the raw Windows SDK headers in AllowMicrosoftPlatformTypes.h / HideMicrosoftPlatformTypes.h. We must
    // not include the raw dxgi headers directly.
    #include <ID3D11DynamicRHI.h>      // GetID3D11DynamicRHI, ID3D11DynamicRHI, IDXGIAdapter, IDXGIAdapter3, DXGI_MEMORY_SEGMENT_GROUP, DXGI_QUERY_VIDEO_MEMORY_INFO
    #include <ID3D12DynamicRHI.h>      // CreateDXGIFactory2, GetID3D12DynamicRHI, ID3D12Device, ID3D12DynamicRHI, IDXGIAdapter, IDXGIAdapter3, IDXGIFactory4, DXGI_MEMORY_SEGMENT_GROUP, DXGI_QUERY_VIDEO_MEMORY_INFO
    #include <Templates/RefCounting.h> // TRefCountPtr
#endif

#if BOOST_OS_WINDOWS || BOOST_OS_LINUX
    #include <Misc/CString.h> // FCStringAnsi

    // On Windows the D3D headers above transitively include windows.h, which defines "interface" and "small" as
    // macros. These leak into the rest of the translation unit and collide with identifiers in the Vulkan headers,
    // so we undefine them before including IVulkanDynamicRHI.h.
    #undef interface
    #undef small
    #include <IVulkanDynamicRHI.h> // GetIVulkanDynamicRHI, IVulkanDynamicRHI, VkPhysicalDevice, VkPhysicalDeviceMemoryBudgetPropertiesEXT, VkPhysicalDeviceMemoryProperties2, VK_EXT_MEMORY_BUDGET_EXTENSION_NAME, VK_MEMORY_HEAP_DEVICE_LOCAL_BIT
#endif

#if BOOST_OS_MACOS
    #include <RHICommandList.h> // RHIGetNativeDevice
    THIRD_PARTY_INCLUDES_START
        #include <Metal/Metal.h>  // id<MTLDevice>
        #include <MetalInclude.h> // MTL::Device
    THIRD_PARTY_INCLUDES_END
#endif

#if BOOST_OS_WINDOWS
// Returns the IDXGIAdapter3 backing the D3D12 device at GPU node 0. The D3D12 RHI exposes an ID3D12Device but not the
// DXGI adapter, so we resolve it ourselves via the device's adapter LUID.
static TRefCountPtr<IDXGIAdapter3> getD3D12DXGIAdapter()
{
    ID3D12DynamicRHI* d3d12_dynamic_rhi = GetID3D12DynamicRHI();
    SP_ASSERT(d3d12_dynamic_rhi);
    ID3D12Device* d3d12_device = d3d12_dynamic_rhi->RHIGetDevice(0);
    SP_ASSERT(d3d12_device);

    LUID adapter_luid = d3d12_device->GetAdapterLuid();

    HRESULT status;

    TRefCountPtr<IDXGIFactory4> dxgi_factory;
    status = CreateDXGIFactory2(0, IID_PPV_ARGS(dxgi_factory.GetInitReference()));
    SP_ASSERT(!FAILED(status));

    TRefCountPtr<IDXGIAdapter> dxgi_adapter;
    status = dxgi_factory->EnumAdapterByLuid(adapter_luid, IID_PPV_ARGS(dxgi_adapter.GetInitReference()));
    SP_ASSERT(!FAILED(status));

    TRefCountPtr<IDXGIAdapter3> dxgi_adapter_3;
    status = dxgi_adapter->QueryInterface(IID_PPV_ARGS(dxgi_adapter_3.GetInitReference()));
    SP_ASSERT(!FAILED(status));

    return dxgi_adapter_3;
}

// Adds the budget and usage reported by IDXGIAdapter3::QueryVideoMemoryInfo for the given DXGI memory segment group
// (local == GPU VRAM, non-local == system/host memory) into stats.
static void addDXGIVideoMemoryInfo(FSpRHIMemoryStats& stats, IDXGIAdapter3* dxgi_adapter, DXGI_MEMORY_SEGMENT_GROUP memory_segment_group)
{
    SP_ASSERT(dxgi_adapter);

    DXGI_QUERY_VIDEO_MEMORY_INFO dxgi_query_video_memory_info;
    HRESULT status = dxgi_adapter->QueryVideoMemoryInfo(0, memory_segment_group, &dxgi_query_video_memory_info);
    SP_ASSERT(!FAILED(status));

    if (memory_segment_group == DXGI_MEMORY_SEGMENT_GROUP_LOCAL) {
        stats.BudgetLocal = dxgi_query_video_memory_info.Budget;
        stats.UsedLocal = dxgi_query_video_memory_info.CurrentUsage;
    } else {
        stats.BudgetSystem = dxgi_query_video_memory_info.Budget;
        stats.UsedSystem = dxgi_query_video_memory_info.CurrentUsage;
    }
}
#endif

FSpRHIMemoryStats USpRHIInterface::GetMemoryStats()
{
    FSpRHIMemoryStats stats;

    SP_ASSERT(GDynamicRHI);
    ERHIInterfaceType interface_type = GDynamicRHI->GetInterfaceType();

    if (interface_type == ERHIInterfaceType::D3D12) {
        #if BOOST_OS_WINDOWS
            TRefCountPtr<IDXGIAdapter3> dxgi_adapter = getD3D12DXGIAdapter();
            addDXGIVideoMemoryInfo(stats, dxgi_adapter, DXGI_MEMORY_SEGMENT_GROUP_LOCAL);
            addDXGIVideoMemoryInfo(stats, dxgi_adapter, DXGI_MEMORY_SEGMENT_GROUP_NON_LOCAL);
        #endif
    } else if (interface_type == ERHIInterfaceType::D3D11) {
        #if BOOST_OS_WINDOWS
            ID3D11DynamicRHI* d3d11_dynamic_rhi = GetID3D11DynamicRHI();
            SP_ASSERT(d3d11_dynamic_rhi);
            IDXGIAdapter* dxgi_adapter = d3d11_dynamic_rhi->RHIGetAdapter();
            SP_ASSERT(dxgi_adapter);

            TRefCountPtr<IDXGIAdapter3> dxgi_adapter_3;
            HRESULT status = dxgi_adapter->QueryInterface(IID_PPV_ARGS(dxgi_adapter_3.GetInitReference()));
            SP_ASSERT(!FAILED(status));

            addDXGIVideoMemoryInfo(stats, dxgi_adapter_3, DXGI_MEMORY_SEGMENT_GROUP_LOCAL);
            addDXGIVideoMemoryInfo(stats, dxgi_adapter_3, DXGI_MEMORY_SEGMENT_GROUP_NON_LOCAL);
        #endif
    } else if (interface_type == ERHIInterfaceType::Vulkan) {
        #if BOOST_OS_WINDOWS || BOOST_OS_LINUX
            IVulkanDynamicRHI* vulkan_dynamic_rhi = GetIVulkanDynamicRHI();
            SP_ASSERT(vulkan_dynamic_rhi);
            VkPhysicalDevice vk_physical_device = vulkan_dynamic_rhi->RHIGetVkPhysicalDevice();
            SP_ASSERT(vk_physical_device != VK_NULL_HANDLE);

            // VK_EXT_memory_budget's fields are only meaningful if the physical device actually supports it, and
            // chaining the extension struct in when it's unsupported triggers a validation error, so we check first.
            bool supports_memory_budget = false;
            for (const VkExtensionProperties& vk_extension_properties : vulkan_dynamic_rhi->RHIGetAllDeviceExtensions(vk_physical_device)) {
                if (FCStringAnsi::Strcmp(vk_extension_properties.extensionName, VK_EXT_MEMORY_BUDGET_EXTENSION_NAME) == 0) {
                    supports_memory_budget = true;
                    break;
                }
            }
            SP_ASSERT(supports_memory_budget);

            PFN_vkGetPhysicalDeviceMemoryProperties2 vk_get_physical_device_memory_properties_2 =
                reinterpret_cast<PFN_vkGetPhysicalDeviceMemoryProperties2>(vulkan_dynamic_rhi->RHIGetVkInstanceProcAddr("vkGetPhysicalDeviceMemoryProperties2"));
            if (!vk_get_physical_device_memory_properties_2) {
                vk_get_physical_device_memory_properties_2 =
                    reinterpret_cast<PFN_vkGetPhysicalDeviceMemoryProperties2>(vulkan_dynamic_rhi->RHIGetVkInstanceProcAddr("vkGetPhysicalDeviceMemoryProperties2KHR"));
            }
            SP_ASSERT(vk_get_physical_device_memory_properties_2);

            VkPhysicalDeviceMemoryBudgetPropertiesEXT vk_physical_device_memory_budget_properties = {};
            vk_physical_device_memory_budget_properties.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_MEMORY_BUDGET_PROPERTIES_EXT;

            VkPhysicalDeviceMemoryProperties2 vk_physical_device_memory_properties_2 = {};
            vk_physical_device_memory_properties_2.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_MEMORY_PROPERTIES_2;
            vk_physical_device_memory_properties_2.pNext = &vk_physical_device_memory_budget_properties;

            vk_get_physical_device_memory_properties_2(vk_physical_device, &vk_physical_device_memory_properties_2);

            const VkPhysicalDeviceMemoryProperties& vk_physical_device_memory_properties = vk_physical_device_memory_properties_2.memoryProperties;
            for (int i = 0; i < (int)vk_physical_device_memory_properties.memoryHeapCount; i++) {
                if (vk_physical_device_memory_properties.memoryHeaps[i].flags & VK_MEMORY_HEAP_DEVICE_LOCAL_BIT) {
                    stats.BudgetLocal += vk_physical_device_memory_budget_properties.heapBudget[i];
                    stats.UsedLocal += vk_physical_device_memory_budget_properties.heapUsage[i];
                } else {
                    stats.BudgetSystem += vk_physical_device_memory_budget_properties.heapBudget[i];
                    stats.UsedSystem += vk_physical_device_memory_budget_properties.heapUsage[i];
                }
            }
        #endif
    } else if (interface_type == ERHIInterfaceType::Metal) {
        #if BOOST_OS_MACOS
            MTL::Device* mtl_device = reinterpret_cast<MTL::Device*>(RHIGetNativeDevice());
            SP_ASSERT(mtl_device);
            stats.BudgetLocal = mtl_device->recommendedMaxWorkingSetSize();
            stats.UsedLocal = static_cast<uint64>(mtl_device->currentAllocatedSize());
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
