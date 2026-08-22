//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Containers/UnrealString.h> // FString
#include <HAL/Platform.h>            // int32, uint32, uint64
#include <UObject/ObjectMacros.h>    // GENERATED_BODY, UENUM, UPROPERTY, USTRUCT

#include "SpDXGI.generated.h"

//
// Types in this file are intended to be faithful transcriptions of the DXGI types shared by our D3D11 and D3D12
// wrappers, see DXGI_MEMORY_SEGMENT_GROUP, DXGI_ADAPTER_DESC, and DXGI_QUERY_VIDEO_MEMORY_INFO.
//

UENUM()
enum class ESpDXGIMemorySegmentGroup
{
    // Values match DXGI_MEMORY_SEGMENT_GROUP
    Local = 0,
    NonLocal = 1,
};

USTRUCT()
struct FSpDXGIAdapterDesc
{
    GENERATED_BODY()

    UPROPERTY()
    FString Description;

    UPROPERTY()
    uint32 VendorId = 0;

    UPROPERTY()
    uint32 DeviceId = 0;

    UPROPERTY()
    uint32 SubSysId = 0;

    UPROPERTY()
    uint32 Revision = 0;

    UPROPERTY()
    uint64 DedicatedVideoMemory = 0;

    UPROPERTY()
    uint64 DedicatedSystemMemory = 0;

    UPROPERTY()
    uint64 SharedSystemMemory = 0;

    // LUID is a 64-bit value split into a low and high part; see the LUID struct.
    UPROPERTY()
    uint32 AdapterLuidLowPart = 0;

    UPROPERTY()
    int32 AdapterLuidHighPart = 0;
};

USTRUCT()
struct FSpDXGIQueryVideoMemoryInfo
{
    GENERATED_BODY()

    UPROPERTY()
    uint64 Budget = 0;

    UPROPERTY()
    uint64 CurrentUsage = 0;

    UPROPERTY()
    uint64 AvailableForReservation = 0;

    UPROPERTY()
    uint64 CurrentReservation = 0;
};
