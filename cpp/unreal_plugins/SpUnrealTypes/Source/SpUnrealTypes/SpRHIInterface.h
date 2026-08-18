//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Kismet/BlueprintFunctionLibrary.h>
#include <Containers/Array.h>  // TArray
#include <Containers/UnrealString.h>
#include <DynamicRHI.h>        // GDynamicRHI
#include <HAL/Platform.h>      // uint64
#include <RHI.h>               // IsRHIDevice*
#include <RHIDefinitions.h>    // ERHIInterfaceType
#include <RHIFeatureLevel.h>   // GMaxRHIFeatureLevel
#include <RHIShaderPlatform.h> // GMaxRHIShaderPlatform
#include <RHIStrings.h>        // LexToString

#include "SpCore/Unreal.h"
#include "SpCore/Assert.h"     // SP_ASSERT
#include "SpRHIGlobals.h"

#include "SpRHIInterface.generated.h"

UENUM()
enum class ESpRHIInterfaceType
{
	Hidden = Unreal::getConstEnumValue(ERHIInterfaceType::Hidden),
	Null = Unreal::getConstEnumValue(ERHIInterfaceType::Null),
	D3D11 = Unreal::getConstEnumValue(ERHIInterfaceType::D3D11),
	D3D12 = Unreal::getConstEnumValue(ERHIInterfaceType::D3D12),
	Vulkan = Unreal::getConstEnumValue(ERHIInterfaceType::Vulkan),
	Metal = Unreal::getConstEnumValue(ERHIInterfaceType::Metal),
	Agx = Unreal::getConstEnumValue(ERHIInterfaceType::Agx),
	OpenGL = Unreal::getConstEnumValue(ERHIInterfaceType::OpenGL),
};

USTRUCT()
struct FSpRHIMemoryStats
{
    GENERATED_BODY()

    UPROPERTY()
    bool IsValid = false;

    UPROPERTY()
    uint64 BudgetBytes = 0;

    UPROPERTY()
    uint64 UsedBytes = 0;

    UPROPERTY()
    uint64 DedicatedBytes = 0;

    UPROPERTY()
    uint64 SystemBudgetBytes = 0;

    UPROPERTY()
    uint64 SystemUsedBytes = 0;
};


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
    static FString GetName()
    {
        SP_ASSERT(GDynamicRHI);
        return GDynamicRHI->GetName();
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static ESpRHIInterfaceType GetInterfaceType()
    {
        SP_ASSERT(GDynamicRHI);
        return static_cast<ESpRHIInterfaceType>(GDynamicRHI->GetInterfaceType());
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
    static ESpShaderPlatform GetShaderPlatform()
    {
        return static_cast<ESpShaderPlatform>(GMaxRHIShaderPlatform);
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static ESpRHIFeatureLevel GetFeatureLevel()
    {
        return static_cast<ESpRHIFeatureLevel>(GMaxRHIFeatureLevel);
    }

    // Queries the active RHI backend for a summary of GPU and system memory budget and usage.
    UFUNCTION(Category="SPEAR")
    static FSpRHIMemoryStats GetMemoryStats();

    // Takes a fresh snapshot of all currently tracked RHI resources.
    UFUNCTION(Category="SPEAR")
    static TArray<FSpRHIResourceStats> GetResourceStats();
};
