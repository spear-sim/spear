//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <boost/predef.h> // BOOST_OS_WINDOWS

#include <HAL/Platform.h>                    // uint64
#include <Kismet/BlueprintFunctionLibrary.h> // UBlueprintFunctionLibrary
#include <UObject/ObjectMacros.h>            // GENERATED_BODY, UCLASS, UFUNCTION
#if BOOST_OS_WINDOWS
    // ID3D11DynamicRHI.h pulls in the DXGI types below via D3D11ThirdParty.h, which wraps the raw Windows SDK headers
    // in AllowMicrosoftPlatformTypes.h / HideMicrosoftPlatformTypes.h. We must not include the raw dxgi headers directly.
    #include <ID3D11DynamicRHI.h>      // DXGI_ADAPTER_DESC, DXGI_MEMORY_SEGMENT_GROUP, DXGI_QUERY_VIDEO_MEMORY_INFO, GetID3D11DynamicRHI, ID3D11Device, ID3D11DynamicRHI, IDXGIAdapter, IDXGIAdapter3
    #include <Templates/RefCounting.h> // TRefCountPtr
#endif

#include "SpCore/Assert.h" // SP_ASSERT

#include "SpUnrealTypes/SpDXGI.h" // ESpDXGIMemorySegmentGroup, FSpDXGIAdapterDesc, FSpDXGIQueryVideoMemoryInfo

#include "SpD3D11DynamicRHI.generated.h"

//
// This class mimics Unreal's ID3D11DynamicRHI, see Engine/Source/Runtime/Windows/D3D11RHI/Public/ID3D11DynamicRHI.h.
// We expose the low-level D3D11 device and DXGI adapter queries we need, transcribing native handles as uint64
// values and native structs as faithful USTRUCT instances so they can participate in UFUNCTION signatures.
//

UCLASS()
class USpD3D11DynamicRHIInterface : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()
public:

    // Returns the ID3D11Device, reinterpreted as a uint64 handle.
    UFUNCTION(Category="SPEAR") // uint64 is not supported for BlueprintCallable
    static uint64 RHIGetDevice()
    {
        #if BOOST_OS_WINDOWS
            ID3D11DynamicRHI* d3d11_dynamic_rhi = GetID3D11DynamicRHI();
            SP_ASSERT(d3d11_dynamic_rhi);
            ID3D11Device* d3d11_device = d3d11_dynamic_rhi->RHIGetDevice();
            SP_ASSERT(d3d11_device);
            return reinterpret_cast<uint64>(d3d11_device);
        #else
            SP_ASSERT(false);
            return 0;
        #endif
    }

    // Returns the IDXGIAdapter backing the D3D11 device, reinterpreted as a uint64 handle.
    UFUNCTION(Category="SPEAR") // uint64 is not supported for BlueprintCallable
    static uint64 RHIGetAdapter()
    {
        #if BOOST_OS_WINDOWS
            ID3D11DynamicRHI* d3d11_dynamic_rhi = GetID3D11DynamicRHI();
            SP_ASSERT(d3d11_dynamic_rhi);
            IDXGIAdapter* dxgi_adapter = d3d11_dynamic_rhi->RHIGetAdapter();
            SP_ASSERT(dxgi_adapter);
            return reinterpret_cast<uint64>(dxgi_adapter);
        #else
            SP_ASSERT(false);
            return 0;
        #endif
    }

    UFUNCTION(Category="SPEAR") // uint64 is not supported for BlueprintCallable
    static FSpDXGIAdapterDesc GetAdapterDesc(uint64 DXGIAdapter)
    {
        FSpDXGIAdapterDesc result;
        #if BOOST_OS_WINDOWS
            IDXGIAdapter* dxgi_adapter = reinterpret_cast<IDXGIAdapter*>(DXGIAdapter);
            SP_ASSERT(dxgi_adapter);

            DXGI_ADAPTER_DESC dxgi_adapter_desc;
            HRESULT status = dxgi_adapter->GetDesc(&dxgi_adapter_desc);
            SP_ASSERT(!FAILED(status));

            result.Description = FString(dxgi_adapter_desc.Description);
            result.VendorId = dxgi_adapter_desc.VendorId;
            result.DeviceId = dxgi_adapter_desc.DeviceId;
            result.SubSysId = dxgi_adapter_desc.SubSysId;
            result.Revision = dxgi_adapter_desc.Revision;
            result.DedicatedVideoMemory = dxgi_adapter_desc.DedicatedVideoMemory;
            result.DedicatedSystemMemory = dxgi_adapter_desc.DedicatedSystemMemory;
            result.SharedSystemMemory = dxgi_adapter_desc.SharedSystemMemory;
            result.AdapterLuidLowPart = dxgi_adapter_desc.AdapterLuid.LowPart;
            result.AdapterLuidHighPart = dxgi_adapter_desc.AdapterLuid.HighPart;
        #else
            SP_ASSERT(false);
        #endif
        return result;
    }

    UFUNCTION(Category="SPEAR") // uint64 is not supported for BlueprintCallable
    static FSpDXGIQueryVideoMemoryInfo QueryVideoMemoryInfo(uint64 DXGIAdapter, ESpDXGIMemorySegmentGroup MemorySegmentGroup)
    {
        FSpDXGIQueryVideoMemoryInfo result;
        #if BOOST_OS_WINDOWS
            IDXGIAdapter* dxgi_adapter = reinterpret_cast<IDXGIAdapter*>(DXGIAdapter);
            SP_ASSERT(dxgi_adapter);

            TRefCountPtr<IDXGIAdapter3> dxgi_adapter_3;
            HRESULT status = dxgi_adapter->QueryInterface(IID_PPV_ARGS(dxgi_adapter_3.GetInitReference()));
            SP_ASSERT(!FAILED(status));

            DXGI_QUERY_VIDEO_MEMORY_INFO dxgi_query_video_memory_info;
            status = dxgi_adapter_3->QueryVideoMemoryInfo(0, static_cast<DXGI_MEMORY_SEGMENT_GROUP>(MemorySegmentGroup), &dxgi_query_video_memory_info);
            SP_ASSERT(!FAILED(status));

            result.Budget = dxgi_query_video_memory_info.Budget;
            result.CurrentUsage = dxgi_query_video_memory_info.CurrentUsage;
            result.AvailableForReservation = dxgi_query_video_memory_info.AvailableForReservation;
            result.CurrentReservation = dxgi_query_video_memory_info.CurrentReservation;
        #else
            SP_ASSERT(false);
        #endif
        return result;
    }
};
