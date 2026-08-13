//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Kismet/BlueprintFunctionLibrary.h>
#include <Containers/UnrealString.h>
#include <DynamicRHI.h>        // GDynamicRHI
#include <HAL/Platform.h>      // int32, uint32, uint64
#include <RHI.h>               // IsRHIDevice*
#include <RHIDefinitions.h>    // ERHIInterfaceType
#include <RHIFeatureLevel.h>   // GMaxRHIFeatureLevel
#include <RHIGlobals.h>        // Global IDs and limits
#include <RHIShaderPlatform.h> // GMaxRHIShaderPlatform
#include <RHIStrings.h>        // LexToString

#include "SpRHIInterface.generated.h"

USTRUCT()
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

UCLASS()
class USpRHIInterface : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()
public:

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static FString GetRHIName()
    {
        return GDynamicRHI->GetName();
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static FString GetGPUName()
    {
        return GRHIAdapterName;
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static bool IsDeviceNVIDIA()
    {
        return IsRHIDeviceNVIDIA();
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static bool IsDeviceAMD()
    {
        return IsRHIDeviceAMD();
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static bool IsDeviceIntel()
    {
        return IsRHIDeviceIntel();
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static bool IsDeviceApple()
    {
        return IsRHIDeviceApple();
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static bool IsInterfaceDX11()
    {
        return GDynamicRHI->GetInterfaceType() == ERHIInterfaceType::D3D11;
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static bool IsInterfaceDX12()
    {
        return GDynamicRHI->GetInterfaceType() == ERHIInterfaceType::D3D12;
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static bool IsInterfaceVulkan()
    {
        return GDynamicRHI->GetInterfaceType() == ERHIInterfaceType::Vulkan;
    }

    UFUNCTION(Category="SPEAR")
    static uint32 GetVendorId()
    {
        return GRHIVendorId;
    }

    UFUNCTION(Category="SPEAR")
    static uint32 GetDeviceId()
    {
        return GRHIDeviceId;
    }

    UFUNCTION(Category="SPEAR")
    static uint32 GetDeviceRevision()
    {
        return GRHIDeviceRevision;
    }

    // The vendor's raw internal build identifier for the driver (format is vendor-specific).
    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static FString GetInternalDriverVersion()
    {
        return GRHIAdapterInternalDriverVersion;
    }

    // The human-readable driver version shown in vendor control panels (e.g. "31.0.15.3623").
    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static FString GetUserDriverVersion()
    {
        return GRHIAdapterUserDriverVersion;
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static FString GetDriverDate()
    {
        return GRHIAdapterDriverDate;
    }

    // Whether Unreal has flagged the currently installed driver as known-bad for this GPU.
    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static bool IsDriverOnDenyList()
    {
        return GRHIAdapterDriverOnDenyList;
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static bool SupportsRayTracing()
    {
        return GRHISupportsRayTracing;
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static int32 GetMaxTextureDimensions()
    {
        return GMaxTextureDimensions;
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static FString GetShaderPlatformName()
    {
        return LexToString(GMaxRHIShaderPlatform);
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static FString GetFeatureLevelName()
    {
        return LexToString(GMaxRHIFeatureLevel);
    }

    // Rough estimate of the total VRAM budget our process can allocate.
    UFUNCTION(Category="SPEAR")
    static uint64 GetTotalVideoMemoryBytes();

    // How much of GetTotalVideoMemoryBytes() is currently in use by this process.
    UFUNCTION(Category="SPEAR")
    static uint64 GetUsedVideoMemoryBytes();

    // The fixed, total amount of dedicated VRAM on the GPU.
    UFUNCTION(Category="SPEAR")
    static uint64 GetTotalPhysicalVideoMemoryBytes();

    // Total amount of the GPU-visible RAM.
    UFUNCTION(Category="SPEAR")
    static uint64 GetTotalSystemMemoryBytes();

    // How much of GetTotalSystemMemoryBytes() is currently in use by this process.
    UFUNCTION(Category="SPEAR")
    static uint64 GetUsedSystemMemoryBytes();

    // Takes a fresh snapshot of all currently tracked RHI.
    UFUNCTION(Category="SPEAR")
    static TArray<FSpRHIResourceStats> GetResourceStats();
};
