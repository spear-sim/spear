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
#include <RHIStats.h>              // FD3DMemoryStats
#include <UObject/ObjectMacros.h>  // GENERATED_BODY, UCLASS, UFUNCTION, UPROPERTY, USTRUCT
#include <Templates/RefCounting.h> // TRefCountPtr

#include "SpCore/Assert.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"

#if BOOST_OS_WINDOWS
    #include <IVulkanDynamicRHI.h> // GetIVulkanDynamicRHI, IVulkanDynamicRHI, VkExtensionProperties, VkPhysicalDevice,
                                   // VkPhysicalDeviceMemoryBudgetPropertiesEXT, VkPhysicalDeviceMemoryProperties2,
                                   // VK_EXT_MEMORY_BUDGET_EXTENSION_NAME, VK_MEMORY_HEAP_DEVICE_LOCAL_BIT
                                
    #include <ID3D11DynamicRHI.h> // GetID3D11DynamicRHI, ID3D11DynamicRHI, IDXGIAdapter, IDXGIAdapter3, DXGI_MEMORY_SEGMENT_GROUP_LOCAL,
                                  // DXGI_MEMORY_SEGMENT_GROUP_NON_LOCAL, DXGI_QUERY_VIDEO_MEMORY_INFO
    #include <ID3D12DynamicRHI.h> // CreateDXGIFactory2, GetID3D12DynamicRHI, ID3D12Device, ID3D12DynamicRHI, IDXGIFactory4
    #include <DXGIUtilities.h>     // FD3DMemoryStats, GetD3DMemoryStats
#elif BOOST_OS_MACOS
    // No metal headers needed currently
#elif BOOST_OS_LINUX
    #include <IVulkanDynamicRHI.h> // GetIVulkanDynamicRHI, IVulkanDynamicRHI, VkExtensionProperties, VkPhysicalDevice,
                                   // VkPhysicalDeviceMemoryBudgetPropertiesEXT, VkPhysicalDeviceMemoryProperties2,
                                   // VK_EXT_MEMORY_BUDGET_EXTENSION_NAME, VK_MEMORY_HEAP_DEVICE_LOCAL_BIT
#else
    #errors
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

#if BOOST_OS_WINDOWS
    static void getD3DMemoryStats(IDXGIAdapter* dxgi_adapter, FSpRHIMemoryStats& stats)
    {
        SP_ASSERT(dxgi_adapter);

        FD3DMemoryStats d3d_memory_stats;
        HRESULT status = UE::DXGIUtilities::GetD3DMemoryStats(dxgi_adapter, d3d_memory_stats);
        SP_ASSERT(!FAILED(status));

        stats.BudgetLocal = d3d_memory_stats.BudgetLocal;
        stats.BudgetSystem = d3d_memory_stats.BudgetSystem;
        stats.UsedLocal = d3d_memory_stats.UsedLocal;
        stats.UsedSystem = d3d_memory_stats.UsedSystem;
        stats.DemotedLocal = d3d_memory_stats.DemotedLocal;
        stats.DemotedSystem = d3d_memory_stats.DemotedSystem;
        stats.AvailableLocal = d3d_memory_stats.AvailableLocal;
        stats.AvailableSystem = d3d_memory_stats.AvailableSystem;
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
            ID3D12DynamicRHI* d3d12_dynamic_rhi = GetID3D12DynamicRHI();
            SP_ASSERT(d3d12_dynamic_rhi);

            // ID3D12DynamicRHI::RHIGetAdapterDescs() always returns exactly one entry,
            // see FD3D12DynamicRHIModule::FindAdapter in Engine/Source/Runtime/D3D12RHI/Private/Windows/WindowsD3D12Device.cpp
            TArray<FD3D12MinimalAdapterDesc> adapter_descs = d3d12_dynamic_rhi->RHIGetAdapterDescs();
            SP_ASSERT(adapter_descs.Num() == 1);
            LUID adapter_luid = adapter_descs[0].Desc.AdapterLuid;

            HRESULT status;

            TRefCountPtr<IDXGIFactory4> dxgi_factory;
            status = CreateDXGIFactory2(0, IID_PPV_ARGS(dxgi_factory.GetInitReference()));
            SP_ASSERT(!FAILED(status));

            TRefCountPtr<IDXGIAdapter> dxgi_adapter;
            status = dxgi_factory->EnumAdapterByLuid(adapter_luid, IID_PPV_ARGS(dxgi_adapter.GetInitReference()));
            SP_ASSERT(!FAILED(status));

            getD3DMemoryStats(dxgi_adapter, stats);
        #else
            SP_ASSERT(false);
        #endif

    } else if (interface_type == ERHIInterfaceType::D3D11) {
        #if BOOST_OS_WINDOWS
            ID3D11DynamicRHI* d3d11_dynamic_rhi = GetID3D11DynamicRHI();
            SP_ASSERT(d3d11_dynamic_rhi);
            IDXGIAdapter* dxgi_adapter = d3d11_dynamic_rhi->RHIGetAdapter();
            SP_ASSERT(dxgi_adapter);
            getD3DMemoryStats(dxgi_adapter, stats);
        #else
            SP_ASSERT(false);
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
