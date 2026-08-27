//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <ranges> // std::views::transform
#include <string>
#include <vector>

#include <boost/predef.h> // BOOST_OS_LINUX, BOOST_OS_MACOS, BOOST_OS_WINDOWS

#include <Containers/Array.h>      // TArray
#include <DynamicRHI.h>            // GDynamicRHI, RHIGetInterfaceType
#include <HAL/Platform.h>          // uint32, uint64
#include <Kismet/BlueprintFunctionLibrary.h>
#include <RHIDefinitions.h>        // ERHIInterfaceType
#include <UObject/ObjectMacros.h>  // GENERATED_BODY, UCLASS, UFUNCTION, UPROPERTY, USTRUCT

#if BOOST_OS_WINDOWS
    #include <ID3D11DynamicRHI.h> // GetID3D11DynamicRHI, ID3D11DynamicRHI, IDXGIAdapter, IDXGIAdapter3, DXGI_MEMORY_SEGMENT_GROUP_LOCAL,
                                  // DXGI_MEMORY_SEGMENT_GROUP_NON_LOCAL, DXGI_QUERY_VIDEO_MEMORY_INFO
    #include <ID3D12DynamicRHI.h> // CreateDXGIFactory2, GetID3D12DynamicRHI, ID3D12Device, ID3D12DynamicRHI, IDXGIFactory4
    
    // Windows macros, conflicts with vulkan declarations
    #pragma push_macro("interface")
    #pragma push_macro("small")
    #undef interface
    #undef small
    #include <IVulkanDynamicRHI.h> // GetIVulkanDynamicRHI, IVulkanDynamicRHI, VkExtensionProperties, VkPhysicalDevice,
                                   // VkPhysicalDeviceMemoryBudgetPropertiesEXT, VkPhysicalDeviceMemoryProperties2,
                                   // VK_EXT_MEMORY_BUDGET_EXTENSION_NAME, VK_MEMORY_HEAP_DEVICE_LOCAL_BIT
    #pragma pop_macro("interface")
    #pragma pop_macro("small")

    #include <MultiGPU.h>              // GNumExplicitGPUsForRendering, GVirtualMGPU
    #include <Templates/RefCounting.h> // TRefCountPtr
#elif BOOST_OS_MACOS
    // No metal headers needed currently
#elif BOOST_OS_LINUX
    #include <IVulkanDynamicRHI.h> // GetIVulkanDynamicRHI, IVulkanDynamicRHI, VkExtensionProperties, VkPhysicalDevice,
                                   // VkPhysicalDeviceMemoryBudgetPropertiesEXT, VkPhysicalDeviceMemoryProperties2,
                                   // VK_EXT_MEMORY_BUDGET_EXTENSION_NAME, VK_MEMORY_HEAP_DEVICE_LOCAL_BIT
#else
    #errors
#endif

#include "SpCore/Assert.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"

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

#if BOOST_OS_WINDOWS
// Resolves the IDXGIAdapter backing the D3D12 device at GPU node 0. The D3D12 RHI exposes an ID3D12Device but not the
// DXGI adapter, so we resolve it ourselves via the device's adapter LUID.
static TRefCountPtr<IDXGIAdapter> getD3D12DXGIAdapter()
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

    return dxgi_adapter;
}

// Fills stats from the given DXGI adapter, shared by our D3D11 and D3D12 code paths. Mimics
// UE::DXGIUtilities::GetD3DMemoryStats(...) in UE 5.8, see Engine/Source/Runtime/RHICore/Private/DXGIUtilities.cpp
static void getDXGIMemoryStats(IDXGIAdapter* dxgi_adapter, FSpRHIMemoryStats& stats)
{
    SP_ASSERT(dxgi_adapter);

    HRESULT status;

    TRefCountPtr<IDXGIAdapter3> dxgi_adapter_3;
    status = dxgi_adapter->QueryInterface(IID_PPV_ARGS(dxgi_adapter_3.GetInitReference()));
    SP_ASSERT(!FAILED(status));

    DXGI_QUERY_VIDEO_MEMORY_INFO local_memory_info;
    status = dxgi_adapter_3->QueryVideoMemoryInfo(0, DXGI_MEMORY_SEGMENT_GROUP_LOCAL, &local_memory_info);
    SP_ASSERT(!FAILED(status));

    DXGI_QUERY_VIDEO_MEMORY_INFO non_local_memory_info;
    status = dxgi_adapter_3->QueryVideoMemoryInfo(0, DXGI_MEMORY_SEGMENT_GROUP_NON_LOCAL, &non_local_memory_info);
    SP_ASSERT(!FAILED(status));

    // In case of multiple GPUs, use the memory info from the one with the highest local budget.
    if (!GVirtualMGPU) {
        for (uint32 index = 1; index < GNumExplicitGPUsForRendering; index++) {
            DXGI_QUERY_VIDEO_MEMORY_INFO temp_local_memory_info;
            status = dxgi_adapter_3->QueryVideoMemoryInfo(index, DXGI_MEMORY_SEGMENT_GROUP_LOCAL, &temp_local_memory_info);
            SP_ASSERT(!FAILED(status));

            DXGI_QUERY_VIDEO_MEMORY_INFO temp_non_local_memory_info;
            status = dxgi_adapter_3->QueryVideoMemoryInfo(index, DXGI_MEMORY_SEGMENT_GROUP_NON_LOCAL, &temp_non_local_memory_info);
            SP_ASSERT(!FAILED(status));

            if (temp_local_memory_info.Budget > local_memory_info.Budget) {
                local_memory_info = temp_local_memory_info;
                non_local_memory_info = temp_non_local_memory_info;
            }
        }
    }

    stats.BudgetLocal = local_memory_info.Budget;
    stats.BudgetSystem = non_local_memory_info.Budget;
    stats.UsedLocal = local_memory_info.CurrentUsage;
    stats.UsedSystem = non_local_memory_info.CurrentUsage;

    // Check if we're over budget.
    if (stats.UsedLocal > stats.BudgetLocal) {
        stats.AvailableLocal = 0;
        stats.DemotedLocal = stats.UsedLocal - stats.BudgetLocal;
    } else {
        stats.AvailableLocal = stats.BudgetLocal - stats.UsedLocal;
        stats.DemotedLocal = 0;
    }

    if (stats.UsedSystem > stats.BudgetSystem) {
        stats.AvailableSystem = 0;
        stats.DemotedSystem = stats.UsedSystem - stats.BudgetSystem;
    } else {
        stats.AvailableSystem = stats.BudgetSystem - stats.UsedSystem;
        stats.DemotedSystem = 0;
    }
}
#endif

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

    // Mimics ::RHIGetMemoryStats(...) in UE 5.8, see Engine/Source/Runtime/RHI/Public/DynamicRHI.h. In UE 5.8 the
    // per-backend logic below is implemented by each RHI's own RHIGetMemoryStats() override, but UE 5.5 doesn't
    // provide it, so we inline the equivalent logic here. Metal leaves the stats zeroed, matching UE 5.8's no-op
    // behavior on Metal. Not BlueprintCallable because uint64 is not a Blueprint type.
    UFUNCTION()
    static FSpRHIMemoryStats RHIGetMemoryStats()
    {
        FSpRHIMemoryStats stats;

        SP_ASSERT(GDynamicRHI);
        ERHIInterfaceType interface_type = RHIGetInterfaceType();

        if (interface_type == ERHIInterfaceType::D3D12) {
            #if BOOST_OS_WINDOWS
                TRefCountPtr<IDXGIAdapter> dxgi_adapter = getD3D12DXGIAdapter();
                getDXGIMemoryStats(dxgi_adapter, stats);
            #endif

        } else if (interface_type == ERHIInterfaceType::D3D11) {
            #if BOOST_OS_WINDOWS
                ID3D11DynamicRHI* d3d11_dynamic_rhi = GetID3D11DynamicRHI();
                SP_ASSERT(d3d11_dynamic_rhi);
                IDXGIAdapter* dxgi_adapter = d3d11_dynamic_rhi->RHIGetAdapter();
                SP_ASSERT(dxgi_adapter);
                getDXGIMemoryStats(dxgi_adapter, stats);
            #endif

        } else if (interface_type == ERHIInterfaceType::Vulkan) {
            #if BOOST_OS_WINDOWS || BOOST_OS_LINUX
                // Mimics FDeviceMemoryManager::UpdateMemoryProperties() in UE 5.8, see
                // Engine/Source/Runtime/VulkanRHI/Private/VulkanMemory.cpp
                IVulkanDynamicRHI* vulkan_dynamic_rhi = GetIVulkanDynamicRHI();
                SP_ASSERT(vulkan_dynamic_rhi);
                VkPhysicalDevice vk_physical_device = vulkan_dynamic_rhi->RHIGetVkPhysicalDevice();
                SP_ASSERT(vk_physical_device != VK_NULL_HANDLE);

                // Require VK_EXT_memory_budget, equivalent to the engine's FVulkanDevice::GetOptionalExtensions().HasMemoryBudget
                // check. Its heapBudget/heapUsage fields are only meaningful when the extension is present, and chaining
                // its struct in when it's unsupported triggers a validation error.
                std::vector<std::string> device_extension_names = Std::toVector<std::string>(
                    Unreal::toStdVector(vulkan_dynamic_rhi->RHIGetAllDeviceExtensions(vk_physical_device)) |
                    std::views::transform([](const VkExtensionProperties& vk_extension_properties) { return std::string(vk_extension_properties.extensionName); }));
                SP_ASSERT(Std::contains(device_extension_names, std::string(VK_EXT_MEMORY_BUDGET_EXTENSION_NAME)));

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
                for (uint32 i = 0; i < vk_physical_device_memory_properties.memoryHeapCount; i++) {
                    if (vk_physical_device_memory_properties.memoryHeaps[i].flags & VK_MEMORY_HEAP_DEVICE_LOCAL_BIT) {
                        stats.BudgetLocal += vk_physical_device_memory_budget_properties.heapBudget[i];
                        stats.UsedLocal += vk_physical_device_memory_budget_properties.heapUsage[i];
                    } else {
                        stats.BudgetSystem += vk_physical_device_memory_budget_properties.heapBudget[i];
                        stats.UsedSystem += vk_physical_device_memory_budget_properties.heapUsage[i];
                    }
                }

                if (stats.UsedLocal > stats.BudgetLocal) {
                    stats.AvailableLocal = 0;
                    stats.DemotedLocal = stats.UsedLocal - stats.BudgetLocal;
                } else {
                    stats.AvailableLocal = stats.BudgetLocal - stats.UsedLocal;
                    stats.DemotedLocal = 0;
                }

                if (stats.UsedSystem > stats.BudgetSystem) {
                    stats.AvailableSystem = 0;
                    stats.DemotedSystem = stats.UsedSystem - stats.BudgetSystem;
                } else {
                    stats.AvailableSystem = stats.BudgetSystem - stats.UsedSystem;
                    stats.DemotedSystem = 0;
                }
            #endif
        }

        return stats;
    }
};
