//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <boost/predef.h> // BOOST_OS_MACOS

#include <HAL/Platform.h>                    // uint64
#include <Kismet/BlueprintFunctionLibrary.h> // UBlueprintFunctionLibrary
#include <UObject/ObjectMacros.h>            // GENERATED_BODY, UCLASS, UFUNCTION
#include "SpCore/Assert.h" // SP_ASSERT
#if BOOST_OS_MACOS
    #include <RHICommandList.h> // RHIGetNativeDevice
    THIRD_PARTY_INCLUDES_START
        #include <Metal/Metal.h>  // id<MTLDevice>
        #include <MetalInclude.h> // MTL::Device
    THIRD_PARTY_INCLUDES_END
#endif

#include "SpMetalDynamicRHI.generated.h"

UCLASS()
class USpMetalDynamicRHIInterface : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()
public:

    // Returns the id<MTLDevice>, reinterpreted as a uint64 handle.
    UFUNCTION(Category="SPEAR") // uint64 is not supported for BlueprintCallable
    static uint64 RHIGetDevice()
    {
        #if BOOST_OS_MACOS
            void* mtl_device = RHIGetNativeDevice();
            SP_ASSERT(mtl_device);
            return reinterpret_cast<uint64>(mtl_device);
        #else
            SP_ASSERT(false);
            return 0;
        #endif
    }

    UFUNCTION(Category="SPEAR") // uint64 is not supported for BlueprintCallable
    static uint64 RecommendedMaxWorkingSetSize(uint64 MTLDevice)
    {
        #if BOOST_OS_MACOS
            MTL::Device* mtl_device = reinterpret_cast<MTL::Device*>(MTLDevice);
            SP_ASSERT(mtl_device);
            return mtl_device->recommendedMaxWorkingSetSize();
        #else
            SP_ASSERT(false);
            return 0;
        #endif
    }

    UFUNCTION(Category="SPEAR") // uint64 is not supported for BlueprintCallable
    static uint64 CurrentAllocatedSize(uint64 MTLDevice)
    {
        #if BOOST_OS_MACOS
            MTL::Device* mtl_device = reinterpret_cast<MTL::Device*>(MTLDevice);
            SP_ASSERT(mtl_device);
            return static_cast<uint64>(mtl_device->currentAllocatedSize());
        #else
            SP_ASSERT(false);
            return 0;
        #endif
    }

    UFUNCTION(Category="SPEAR")
    static bool HasUnifiedMemory(uint64 MTLDevice)
    {
        #if BOOST_OS_MACOS
            MTL::Device* mtl_device = reinterpret_cast<MTL::Device*>(MTLDevice);
            SP_ASSERT(mtl_device);
            return mtl_device->hasUnifiedMemory();
        #else
            SP_ASSERT(false);
            return false;
        #endif
    }
};
