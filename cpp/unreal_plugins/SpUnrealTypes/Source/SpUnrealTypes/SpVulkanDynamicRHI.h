//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <boost/predef.h> // BOOST_OS_LINUX, BOOST_OS_WINDOWS

#include <ranges> // std::views::transform
#include <string>
#include <vector>

#include <Containers/Array.h>                // TArray
#include <Containers/StringConv.h>           // UTF8_TO_TCHAR
#include <Containers/UnrealString.h>         // FString
#include <HAL/Platform.h>                    // uint32, uint64
#include <Kismet/BlueprintFunctionLibrary.h> // UBlueprintFunctionLibrary
#include <UObject/ObjectMacros.h>            // GENERATED_BODY, UCLASS, UFUNCTION, UPROPERTY, USTRUCT
#if BOOST_OS_WINDOWS || BOOST_OS_LINUX
    #include <IVulkanDynamicRHI.h> // GetIVulkanDynamicRHI, IVulkanDynamicRHI, PFN_vkGetPhysicalDeviceMemoryProperties2, VK_EXT_MEMORY_BUDGET_EXTENSION_NAME, VkExtensionProperties, VkPhysicalDevice, VkPhysicalDeviceMemoryBudgetPropertiesEXT, VkPhysicalDeviceMemoryProperties2
#endif

#include "SpCore/Assert.h" // SP_ASSERT

// SpCore/Std.h and SpCore/Unreal.h use "interface" and "small" as ordinary identifiers. When this header is compiled
// in a translation unit that has already pulled in the Windows COM/RPC headers (e.g., alongside our D3D wrappers in
// SpRHIInterface.cpp), those headers leak "interface" (combaseapi.h) and "small" (rpcndr.h) as macros, which
// HideWindowsPlatformTypes.h does not undo. We undefine them here, after all Windows headers in the translation unit
// have been parsed, so the SpCore headers below compile regardless of include order.
#undef interface
#undef small
#include "SpCore/Std.h"    // Std::contains, Std::toVector
#include "SpCore/Unreal.h" // Unreal::toStdString, Unreal::toStdVector

#include "SpVulkanDynamicRHI.generated.h"

//
// Types in this file are intended to be faithful transcriptions of the Vulkan types we query, see VkExtensionProperties,
// VkPhysicalDeviceMemoryProperties2, and VkPhysicalDeviceMemoryBudgetPropertiesEXT.
//

USTRUCT()
struct FSpVkExtensionProperties
{
    GENERATED_BODY()

    UPROPERTY()
    FString ExtensionName;

    UPROPERTY()
    uint32 SpecVersion = 0;
};

USTRUCT()
struct FSpVkMemoryType
{
    GENERATED_BODY()

    UPROPERTY()
    uint32 PropertyFlags = 0;

    UPROPERTY()
    uint32 HeapIndex = 0;
};

USTRUCT()
struct FSpVkMemoryHeap
{
    GENERATED_BODY()

    UPROPERTY()
    uint64 Size = 0;

    UPROPERTY()
    uint32 Flags = 0;
};

USTRUCT()
struct FSpVkPhysicalDeviceMemoryProperties
{
    GENERATED_BODY()

    UPROPERTY()
    uint32 MemoryTypeCount = 0;

    UPROPERTY()
    TArray<FSpVkMemoryType> MemoryTypes;

    UPROPERTY()
    uint32 MemoryHeapCount = 0;

    UPROPERTY()
    TArray<FSpVkMemoryHeap> MemoryHeaps;
};

// From VK_EXT_memory_budget. HeapBudget and HeapUsage are indexed by heap, i.e., they line up with MemoryHeaps above.
USTRUCT()
struct FSpVkPhysicalDeviceMemoryBudgetProperties
{
    GENERATED_BODY()

    UPROPERTY()
    TArray<uint64> HeapBudget;

    UPROPERTY()
    TArray<uint64> HeapUsage;
};

USTRUCT()
struct FSpVkPhysicalDeviceMemoryProperties2
{
    GENERATED_BODY()

    UPROPERTY()
    FSpVkPhysicalDeviceMemoryProperties MemoryProperties;

    UPROPERTY()
    FSpVkPhysicalDeviceMemoryBudgetProperties MemoryBudget;
};

//
// This class mimics Unreal's IVulkanDynamicRHI, see Engine/Source/Runtime/VulkanRHI/Public/IVulkanDynamicRHI.h. We
// expose the low-level Vulkan physical device queries we need, transcribing native handles as uint64 values and
// native structs as faithful USTRUCT instances so they can participate in UFUNCTION signatures.
//

UCLASS()
class USpVulkanDynamicRHIInterface : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()
public:

    // Returns the VkPhysicalDevice, reinterpreted as a uint64 handle.
    UFUNCTION(Category="SPEAR") // uint64 is not supported for BlueprintCallable
    static uint64 RHIGetVkPhysicalDevice()
    {
        #if BOOST_OS_WINDOWS || BOOST_OS_LINUX
            IVulkanDynamicRHI* vulkan_dynamic_rhi = GetIVulkanDynamicRHI();
            SP_ASSERT(vulkan_dynamic_rhi);
            VkPhysicalDevice vk_physical_device = vulkan_dynamic_rhi->RHIGetVkPhysicalDevice();
            SP_ASSERT(vk_physical_device != VK_NULL_HANDLE);
            return reinterpret_cast<uint64>(vk_physical_device);
        #else
            SP_ASSERT(false);
            return 0;
        #endif
    }

    UFUNCTION(Category="SPEAR") // uint64 is not supported for BlueprintCallable
    static TArray<FSpVkExtensionProperties> RHIGetAllDeviceExtensions(uint64 PhysicalDevice)
    {
        TArray<FSpVkExtensionProperties> result;
        #if BOOST_OS_WINDOWS || BOOST_OS_LINUX
            IVulkanDynamicRHI* vulkan_dynamic_rhi = GetIVulkanDynamicRHI();
            SP_ASSERT(vulkan_dynamic_rhi);
            VkPhysicalDevice vk_physical_device = reinterpret_cast<VkPhysicalDevice>(PhysicalDevice);
            SP_ASSERT(vk_physical_device != VK_NULL_HANDLE);

            for (const VkExtensionProperties& vk_extension_properties : vulkan_dynamic_rhi->RHIGetAllDeviceExtensions(vk_physical_device)) {
                FSpVkExtensionProperties extension_properties;
                extension_properties.ExtensionName = FString(UTF8_TO_TCHAR(vk_extension_properties.extensionName));
                extension_properties.SpecVersion = vk_extension_properties.specVersion;
                result.Add(extension_properties);
            }
        #else
            SP_ASSERT(false);
        #endif
        return result;
    }

    UFUNCTION(Category="SPEAR") // uint64 is not supported for BlueprintCallable
    static FSpVkPhysicalDeviceMemoryProperties2 VkGetPhysicalDeviceMemoryProperties2(uint64 PhysicalDevice)
    {
        FSpVkPhysicalDeviceMemoryProperties2 result;
        #if BOOST_OS_WINDOWS || BOOST_OS_LINUX
            IVulkanDynamicRHI* vulkan_dynamic_rhi = GetIVulkanDynamicRHI();
            SP_ASSERT(vulkan_dynamic_rhi);
            VkPhysicalDevice vk_physical_device = reinterpret_cast<VkPhysicalDevice>(PhysicalDevice);
            SP_ASSERT(vk_physical_device != VK_NULL_HANDLE);

            // VK_EXT_memory_budget's fields are only meaningful if the physical device actually supports it, and
            // chaining the extension struct in when it's unsupported triggers a validation error, so we check first.
            std::vector<std::string> extension_names = Std::toVector<std::string>(
                Unreal::toStdVector(RHIGetAllDeviceExtensions(PhysicalDevice)) |
                std::views::transform([](const auto& extension_properties) { return Unreal::toStdString(extension_properties.ExtensionName); }));
            SP_ASSERT(Std::contains(extension_names, VK_EXT_MEMORY_BUDGET_EXTENSION_NAME));

            PFN_vkGetPhysicalDeviceMemoryProperties2 vk_get_physical_device_memory_properties_2 = nullptr;
            if (!vk_get_physical_device_memory_properties_2) {
                vk_get_physical_device_memory_properties_2 = reinterpret_cast<PFN_vkGetPhysicalDeviceMemoryProperties2>(vulkan_dynamic_rhi->RHIGetVkInstanceProcAddr("vkGetPhysicalDeviceMemoryProperties2"));
            } if (!vk_get_physical_device_memory_properties_2) {
                vk_get_physical_device_memory_properties_2 = reinterpret_cast<PFN_vkGetPhysicalDeviceMemoryProperties2>(vulkan_dynamic_rhi->RHIGetVkInstanceProcAddr("vkGetPhysicalDeviceMemoryProperties2KHR"));
            } if (!vk_get_physical_device_memory_properties_2) {
                SP_ASSERT(false);
            }

            VkPhysicalDeviceMemoryBudgetPropertiesEXT vk_physical_device_memory_budget_properties = {};
            vk_physical_device_memory_budget_properties.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_MEMORY_BUDGET_PROPERTIES_EXT;

            VkPhysicalDeviceMemoryProperties2 vk_physical_device_memory_properties_2 = {};
            vk_physical_device_memory_properties_2.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_MEMORY_PROPERTIES_2;
            vk_physical_device_memory_properties_2.pNext = &vk_physical_device_memory_budget_properties;

            vk_get_physical_device_memory_properties_2(vk_physical_device, &vk_physical_device_memory_properties_2);

            const VkPhysicalDeviceMemoryProperties& vk_physical_device_memory_properties = vk_physical_device_memory_properties_2.memoryProperties;

            result.MemoryProperties.MemoryTypeCount = vk_physical_device_memory_properties.memoryTypeCount;
            for (int i = 0; i < (int)vk_physical_device_memory_properties.memoryTypeCount; i++) {
                FSpVkMemoryType memory_type;
                memory_type.PropertyFlags = vk_physical_device_memory_properties.memoryTypes[i].propertyFlags;
                memory_type.HeapIndex = vk_physical_device_memory_properties.memoryTypes[i].heapIndex;
                result.MemoryProperties.MemoryTypes.Add(memory_type);
            }

            result.MemoryProperties.MemoryHeapCount = vk_physical_device_memory_properties.memoryHeapCount;
            for (int i = 0; i < (int)vk_physical_device_memory_properties.memoryHeapCount; i++) {
                FSpVkMemoryHeap memory_heap;
                memory_heap.Size = vk_physical_device_memory_properties.memoryHeaps[i].size;
                memory_heap.Flags = vk_physical_device_memory_properties.memoryHeaps[i].flags;
                result.MemoryProperties.MemoryHeaps.Add(memory_heap);

                result.MemoryBudget.HeapBudget.Add(vk_physical_device_memory_budget_properties.heapBudget[i]);
                result.MemoryBudget.HeapUsage.Add(vk_physical_device_memory_budget_properties.heapUsage[i]);
            }
        #else
            SP_ASSERT(false);
        #endif
        return result;
    }
};
