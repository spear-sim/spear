//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Kismet/BlueprintFunctionLibrary.h>
#include <RHIGlobals.h>        // Global IDs and limits
#include <RHIShaderPlatform.h>

#include "SpCore/Unreal.h"
#include "SpUnrealTypes/SpRHIFeatureLevel.h" // ESpRHIFeatureLevel

#include "SpRHIGlobals.generated.h"

// WinSock2.h workaround
#ifdef PF_MAX
#undef PF_MAX
#endif

//
// Structs in this file are intended to be identical to FRHIGlobals, see Engine/Source/Runtime/RHI/Public/RHIGlobals.h
//

UENUM(BlueprintType)
enum class ESpRHIBindlessSupport : uint8
{
    Unsupported = Unreal::getConstEnumValue(ERHIBindlessSupport::Unsupported),
    RayTracingOnly = Unreal::getConstEnumValue(ERHIBindlessSupport::RayTracingOnly),
    AllShaderTypes = Unreal::getConstEnumValue(ERHIBindlessSupport::AllShaderTypes),
    NumBits = Unreal::getConstEnumValue(ERHIBindlessSupport::NumBits),
};

UENUM(BlueprintType)
enum class ESpShaderPlatform : uint16
{
    SP_PCD3D_SM5 = Unreal::getConstEnumValue(EShaderPlatform::SP_PCD3D_SM5),
    SP_METAL = Unreal::getConstEnumValue(EShaderPlatform::SP_METAL),
    SP_METAL_MRT = Unreal::getConstEnumValue(EShaderPlatform::SP_METAL_MRT),
    SP_PCD3D_ES3_1 = Unreal::getConstEnumValue(EShaderPlatform::SP_PCD3D_ES3_1),
    SP_OPENGL_PCES3_1 = Unreal::getConstEnumValue(EShaderPlatform::SP_OPENGL_PCES3_1),
    SP_METAL_SM5 = Unreal::getConstEnumValue(EShaderPlatform::SP_METAL_SM5),
    SP_VULKAN_PCES3_1 = Unreal::getConstEnumValue(EShaderPlatform::SP_VULKAN_PCES3_1),
    SP_VULKAN_SM5 = Unreal::getConstEnumValue(EShaderPlatform::SP_VULKAN_SM5),
    SP_VULKAN_ES3_1_ANDROID = Unreal::getConstEnumValue(EShaderPlatform::SP_VULKAN_ES3_1_ANDROID),
    SP_METAL_MACES3_1 = Unreal::getConstEnumValue(EShaderPlatform::SP_METAL_MACES3_1),
    SP_OPENGL_ES3_1_ANDROID = Unreal::getConstEnumValue(EShaderPlatform::SP_OPENGL_ES3_1_ANDROID),
    SP_METAL_MRT_MAC = Unreal::getConstEnumValue(EShaderPlatform::SP_METAL_MRT_MAC),
    SP_METAL_TVOS = Unreal::getConstEnumValue(EShaderPlatform::SP_METAL_TVOS),
    SP_METAL_MRT_TVOS = Unreal::getConstEnumValue(EShaderPlatform::SP_METAL_MRT_TVOS),
    SP_StaticPlatform_First = Unreal::getConstEnumValue(EShaderPlatform::SP_StaticPlatform_First),
    SP_StaticPlatform_Last = Unreal::getConstEnumValue(EShaderPlatform::SP_StaticPlatform_Last),
    SP_VULKAN_SM5_ANDROID = Unreal::getConstEnumValue(EShaderPlatform::SP_VULKAN_SM5_ANDROID),
    SP_PCD3D_SM6 = Unreal::getConstEnumValue(EShaderPlatform::SP_PCD3D_SM6),
    SP_VULKAN_SM6 = Unreal::getConstEnumValue(EShaderPlatform::SP_VULKAN_SM6),
    SP_METAL_SM6 = Unreal::getConstEnumValue(EShaderPlatform::SP_METAL_SM6),
    SP_METAL_SIM = Unreal::getConstEnumValue(EShaderPlatform::SP_METAL_SIM),
    SP_CUSTOM_PLATFORM_FIRST = Unreal::getConstEnumValue(EShaderPlatform::SP_CUSTOM_PLATFORM_FIRST),
    SP_CUSTOM_PLATFORM_LAST = Unreal::getConstEnumValue(EShaderPlatform::SP_CUSTOM_PLATFORM_LAST),
    SP_NumPlatforms = Unreal::getConstEnumValue(EShaderPlatform::SP_NumPlatforms),
    SP_NumBits = Unreal::getConstEnumValue(EShaderPlatform::SP_NumBits),
};

UENUM(BlueprintType)
enum class ESpVRSImageDataType : uint8
{
    VRSImage_NotSupported = Unreal::getConstEnumValue(EVRSImageDataType::VRSImage_NotSupported),
    VRSImage_Palette = Unreal::getConstEnumValue(EVRSImageDataType::VRSImage_Palette),
    VRSImage_Fractional = Unreal::getConstEnumValue(EVRSImageDataType::VRSImage_Fractional),
};

UENUM(BlueprintType)
enum class ESpPixelFormat : uint8
{
    PF_Unknown = Unreal::getConstEnumValue(EPixelFormat::PF_Unknown),
    PF_A32B32G32R32F = Unreal::getConstEnumValue(EPixelFormat::PF_A32B32G32R32F),
    PF_B8G8R8A8 = Unreal::getConstEnumValue(EPixelFormat::PF_B8G8R8A8),
    PF_G8 = Unreal::getConstEnumValue(EPixelFormat::PF_G8),
    PF_G16 = Unreal::getConstEnumValue(EPixelFormat::PF_G16),
    PF_DXT1 = Unreal::getConstEnumValue(EPixelFormat::PF_DXT1),
    PF_DXT3 = Unreal::getConstEnumValue(EPixelFormat::PF_DXT3),
    PF_DXT5 = Unreal::getConstEnumValue(EPixelFormat::PF_DXT5),
    PF_UYVY = Unreal::getConstEnumValue(EPixelFormat::PF_UYVY),
    PF_FloatRGB = Unreal::getConstEnumValue(EPixelFormat::PF_FloatRGB),
    PF_FloatRGBA = Unreal::getConstEnumValue(EPixelFormat::PF_FloatRGBA),
    PF_DepthStencil = Unreal::getConstEnumValue(EPixelFormat::PF_DepthStencil),
    PF_ShadowDepth = Unreal::getConstEnumValue(EPixelFormat::PF_ShadowDepth),
    PF_R32_FLOAT = Unreal::getConstEnumValue(EPixelFormat::PF_R32_FLOAT),
    PF_G16R16 = Unreal::getConstEnumValue(EPixelFormat::PF_G16R16),
    PF_G16R16F = Unreal::getConstEnumValue(EPixelFormat::PF_G16R16F),
    PF_G16R16F_FILTER = Unreal::getConstEnumValue(EPixelFormat::PF_G16R16F_FILTER),
    PF_G32R32F = Unreal::getConstEnumValue(EPixelFormat::PF_G32R32F),
    PF_A2B10G10R10 = Unreal::getConstEnumValue(EPixelFormat::PF_A2B10G10R10),
    PF_A16B16G16R16 = Unreal::getConstEnumValue(EPixelFormat::PF_A16B16G16R16),
    PF_D24 = Unreal::getConstEnumValue(EPixelFormat::PF_D24),
    PF_R16F = Unreal::getConstEnumValue(EPixelFormat::PF_R16F),
    PF_R16F_FILTER = Unreal::getConstEnumValue(EPixelFormat::PF_R16F_FILTER),
    PF_BC5 = Unreal::getConstEnumValue(EPixelFormat::PF_BC5),
    PF_V8U8 = Unreal::getConstEnumValue(EPixelFormat::PF_V8U8),
    PF_A1 = Unreal::getConstEnumValue(EPixelFormat::PF_A1),
    PF_FloatR11G11B10 = Unreal::getConstEnumValue(EPixelFormat::PF_FloatR11G11B10),
    PF_A8 = Unreal::getConstEnumValue(EPixelFormat::PF_A8),
    PF_R32_UINT = Unreal::getConstEnumValue(EPixelFormat::PF_R32_UINT),
    PF_R32_SINT = Unreal::getConstEnumValue(EPixelFormat::PF_R32_SINT),
    PF_PVRTC2 = Unreal::getConstEnumValue(EPixelFormat::PF_PVRTC2),
    PF_PVRTC4 = Unreal::getConstEnumValue(EPixelFormat::PF_PVRTC4),
    PF_R16_UINT = Unreal::getConstEnumValue(EPixelFormat::PF_R16_UINT),
    PF_R16_SINT = Unreal::getConstEnumValue(EPixelFormat::PF_R16_SINT),
    PF_R16G16B16A16_UINT = Unreal::getConstEnumValue(EPixelFormat::PF_R16G16B16A16_UINT),
    PF_R16G16B16A16_SINT = Unreal::getConstEnumValue(EPixelFormat::PF_R16G16B16A16_SINT),
    PF_R5G6B5_UNORM = Unreal::getConstEnumValue(EPixelFormat::PF_R5G6B5_UNORM),
    PF_R8G8B8A8 = Unreal::getConstEnumValue(EPixelFormat::PF_R8G8B8A8),
    PF_A8R8G8B8 = Unreal::getConstEnumValue(EPixelFormat::PF_A8R8G8B8),
    PF_BC4 = Unreal::getConstEnumValue(EPixelFormat::PF_BC4),
    PF_R8G8 = Unreal::getConstEnumValue(EPixelFormat::PF_R8G8),
    PF_ATC_RGB = Unreal::getConstEnumValue(EPixelFormat::PF_ATC_RGB),
    PF_ATC_RGBA_E = Unreal::getConstEnumValue(EPixelFormat::PF_ATC_RGBA_E),
    PF_ATC_RGBA_I = Unreal::getConstEnumValue(EPixelFormat::PF_ATC_RGBA_I),
    PF_X24_G8 = Unreal::getConstEnumValue(EPixelFormat::PF_X24_G8),
    PF_ETC1 = Unreal::getConstEnumValue(EPixelFormat::PF_ETC1),
    PF_ETC2_RGB = Unreal::getConstEnumValue(EPixelFormat::PF_ETC2_RGB),
    PF_ETC2_RGBA = Unreal::getConstEnumValue(EPixelFormat::PF_ETC2_RGBA),
    PF_R32G32B32A32_UINT = Unreal::getConstEnumValue(EPixelFormat::PF_R32G32B32A32_UINT),
    PF_R16G16_UINT = Unreal::getConstEnumValue(EPixelFormat::PF_R16G16_UINT),
    PF_ASTC_4x4 = Unreal::getConstEnumValue(EPixelFormat::PF_ASTC_4x4),
    PF_ASTC_6x6 = Unreal::getConstEnumValue(EPixelFormat::PF_ASTC_6x6),
    PF_ASTC_8x8 = Unreal::getConstEnumValue(EPixelFormat::PF_ASTC_8x8),
    PF_ASTC_10x10 = Unreal::getConstEnumValue(EPixelFormat::PF_ASTC_10x10),
    PF_ASTC_12x12 = Unreal::getConstEnumValue(EPixelFormat::PF_ASTC_12x12),
    PF_BC6H = Unreal::getConstEnumValue(EPixelFormat::PF_BC6H),
    PF_BC7 = Unreal::getConstEnumValue(EPixelFormat::PF_BC7),
    PF_R8_UINT = Unreal::getConstEnumValue(EPixelFormat::PF_R8_UINT),
    PF_L8 = Unreal::getConstEnumValue(EPixelFormat::PF_L8),
    PF_XGXR8 = Unreal::getConstEnumValue(EPixelFormat::PF_XGXR8),
    PF_R8G8B8A8_UINT = Unreal::getConstEnumValue(EPixelFormat::PF_R8G8B8A8_UINT),
    PF_R8G8B8A8_SNORM = Unreal::getConstEnumValue(EPixelFormat::PF_R8G8B8A8_SNORM),
    PF_R16G16B16A16_UNORM = Unreal::getConstEnumValue(EPixelFormat::PF_R16G16B16A16_UNORM),
    PF_R16G16B16A16_SNORM = Unreal::getConstEnumValue(EPixelFormat::PF_R16G16B16A16_SNORM),
    PF_PLATFORM_HDR_0 = Unreal::getConstEnumValue(EPixelFormat::PF_PLATFORM_HDR_0),
    PF_PLATFORM_HDR_1 = Unreal::getConstEnumValue(EPixelFormat::PF_PLATFORM_HDR_1),
    PF_PLATFORM_HDR_2 = Unreal::getConstEnumValue(EPixelFormat::PF_PLATFORM_HDR_2),
    PF_NV12 = Unreal::getConstEnumValue(EPixelFormat::PF_NV12),
    PF_R32G32_UINT = Unreal::getConstEnumValue(EPixelFormat::PF_R32G32_UINT),
    PF_ETC2_R11_EAC = Unreal::getConstEnumValue(EPixelFormat::PF_ETC2_R11_EAC),
    PF_ETC2_RG11_EAC = Unreal::getConstEnumValue(EPixelFormat::PF_ETC2_RG11_EAC),
    PF_R8 = Unreal::getConstEnumValue(EPixelFormat::PF_R8),
    PF_B5G5R5A1_UNORM = Unreal::getConstEnumValue(EPixelFormat::PF_B5G5R5A1_UNORM),
    PF_ASTC_4x4_HDR = Unreal::getConstEnumValue(EPixelFormat::PF_ASTC_4x4_HDR),	
    PF_ASTC_6x6_HDR = Unreal::getConstEnumValue(EPixelFormat::PF_ASTC_6x6_HDR),	
    PF_ASTC_8x8_HDR = Unreal::getConstEnumValue(EPixelFormat::PF_ASTC_8x8_HDR),	
    PF_ASTC_10x10_HDR = Unreal::getConstEnumValue(EPixelFormat::PF_ASTC_10x10_HDR),	
    PF_ASTC_12x12_HDR = Unreal::getConstEnumValue(EPixelFormat::PF_ASTC_12x12_HDR),
    PF_G16R16_SNORM = Unreal::getConstEnumValue(EPixelFormat::PF_G16R16_SNORM),
    PF_R8G8_UINT = Unreal::getConstEnumValue(EPixelFormat::PF_R8G8_UINT),
    PF_R32G32B32_UINT = Unreal::getConstEnumValue(EPixelFormat::PF_R32G32B32_UINT),
    PF_R32G32B32_SINT = Unreal::getConstEnumValue(EPixelFormat::PF_R32G32B32_SINT),
    PF_R32G32B32F = Unreal::getConstEnumValue(EPixelFormat::PF_R32G32B32F),
    PF_R8_SINT = Unreal::getConstEnumValue(EPixelFormat::PF_R8_SINT),
    PF_R64_UINT = Unreal::getConstEnumValue(EPixelFormat::PF_R64_UINT),
    PF_R9G9B9EXP5 = Unreal::getConstEnumValue(EPixelFormat::PF_R9G9B9EXP5),
    PF_P010 = Unreal::getConstEnumValue(EPixelFormat::PF_P010),
    PF_ASTC_4x4_NORM_RG = Unreal::getConstEnumValue(EPixelFormat::PF_ASTC_4x4_NORM_RG),
    PF_ASTC_6x6_NORM_RG = Unreal::getConstEnumValue(EPixelFormat::PF_ASTC_6x6_NORM_RG),
    PF_ASTC_8x8_NORM_RG = Unreal::getConstEnumValue(EPixelFormat::PF_ASTC_8x8_NORM_RG),
    PF_ASTC_10x10_NORM_RG = Unreal::getConstEnumValue(EPixelFormat::PF_ASTC_10x10_NORM_RG),
    PF_ASTC_12x12_NORM_RG = Unreal::getConstEnumValue(EPixelFormat::PF_ASTC_12x12_NORM_RG),
    PF_R16G16_SINT = Unreal::getConstEnumValue(EPixelFormat::PF_R16G16_SINT),
    PF_MAX = Unreal::getConstEnumValue(EPixelFormat::PF_MAX),
};

UENUM(Flags)
enum class ESpRequestedGPUCrash
{
    None = Unreal::getConstEnumValue(ERequestedGPUCrash::None),
    Type_Hang = Unreal::getConstEnumValue(ERequestedGPUCrash::Type_Hang),
    Type_PageFault = Unreal::getConstEnumValue(ERequestedGPUCrash::Type_PageFault),
    Type_PlatformBreak = Unreal::getConstEnumValue(ERequestedGPUCrash::Type_PlatformBreak),
    Type_Assert = Unreal::getConstEnumValue(ERequestedGPUCrash::Type_Assert),
    Queue_Direct = Unreal::getConstEnumValue(ERequestedGPUCrash::Queue_Direct),
    Queue_Compute = Unreal::getConstEnumValue(ERequestedGPUCrash::Queue_Compute),
};
ENUM_CLASS_FLAGS(ESpRequestedGPUCrash);


USTRUCT()
struct FSpReservedResources
{
    GENERATED_BODY()

    /**
    * True if the RHI supports reserved (AKA tiled, virtual or sparse) resources and operations related to them.
    * Buffers and 2D textures (without mips) can be created with ReservedResource flag.
    */
    UPROPERTY()
    bool Supported = false;

    /**
    * True if the RHI supports creating volume textures with ReservedResource flag.
    */
    UPROPERTY()
    bool SupportsVolumeTextures = false;

    /**
    * Smallest mip dimension of reserved texture arrays must be greater or equal to this value.
    * Tiled/reserved resources with both more than one array slice and any mipmap that
    * has a dimension less than a tile in extent are not supported by some hardware.
    * This is a conservative value chosen by the engine, independent of the texture format for simplicity.
    */
    UPROPERTY()
    int32 TextureArrayMinimumMipDimension = 256;

    /**
    * Size that corresponds to a minimum unit of physical memory that may be mapped to a region of a reserved resource.
    * High-level code should aim to allocate reserved resources such that their size is a multiple of this tile size.
    * Guaranteed to be the same value on all platforms, regardless of the native virtual memory page size.
    */
    UPROPERTY()
    int32 TileSizeInBytes = 65536;
};

USTRUCT()
struct FSpGpuInfo
{
    GENERATED_BODY()

    /**
     * only set if RHI has the information (after init of the RHI and only if RHI has that information, never changes after that)
     * e.g. "NVIDIA GeForce GTX 670"
     */
    UPROPERTY()
    FString AdapterName;

    UPROPERTY()
    FString AdapterInternalDriverVersion;

    UPROPERTY()
    FString AdapterUserDriverVersion;

    UPROPERTY()
    FString AdapterDriverDate;

    UPROPERTY()
    bool AdapterDriverOnDenyList = false;

    UPROPERTY()
    uint32 DeviceId = 0;

    UPROPERTY()
    uint32 DeviceRevision = 0;

    /** true if the GPU is AMD's Pre-GCN architecture */
    UPROPERTY()
    bool IsAMDPreGCNArchitecture = false;

    // 0 means not defined yet, use functions like IsRHIDeviceAMD() to access
    UPROPERTY()
    uint32 VendorId = 0;
};


USTRUCT()
struct FSpRayTracing
{
    GENERATED_BODY()

    /**
    * Whether or not the RHI supports ray tracing on current hardware (acceleration structure building and new ray tracing-specific shader types).
    * SupportsRayTracingShaders and SupportsInlineRayTracing must also be checked before dispatching ray tracing workloads.
    */
    UPROPERTY()
    bool Supported = false;

    /**
    * Whether or not the RHI supports ray tracing raygen, miss and hit shaders (i.e. full ray tracing pipeline).
    * The RHI may support inline ray tracing from compute shaders, but not the full pipeline.
    */
    UPROPERTY()
    bool SupportsShaders = false;

    /** Whether or not the RHI supports adding new shaders to an existing RT PSO. */
    UPROPERTY()
    bool SupportsPSOAdditions = false;

    /** Whether or not the RHI supports indirect ray tracing dispatch commands. */
    UPROPERTY()
    bool SupportsDispatchIndirect = false;

    /** Whether or not the RHI supports async building ray tracing acceleration structures. */
    UPROPERTY()
    bool SupportsAsyncBuildAccelerationStructure = false;

    /** Whether or not the RHI supports the AMD Hit Token extension. */
    UPROPERTY()
    bool SupportsAMDHitToken = false;

    /** Whether or not the RHI supports inline ray tracing in compute shaders, without a full ray tracing pipeline. */
    UPROPERTY()
    bool SupportsInlineRayTracing = false;
    
    /** Whether or not the RHI requires a SBT for inline ray tracing in compute shaders to fetch geometry information */
    UPROPERTY()
    bool RequiresInlineRayTracingSBT = false;
    
    /** Whether or not the RHI supports inlined callbacks */
    UPROPERTY()
    bool SupportsInlinedCallbacks = false;

    /** Wether an extra uniform buffer parameter is required when loose parameters are present, or if they are stored directly in the shader record. */
    UPROPERTY()
    bool SupportsLooseParamsInShaderRecord = false;

    /** Required alignment for ray tracing acceleration structures. */
    UPROPERTY()
    uint32 AccelerationStructureAlignment = 0;

    /** Required alignment for ray tracing scratch buffers. */
    UPROPERTY()
    uint32 ScratchBufferAlignment = 0;

    /** Required alignment for ray tracing shader binding table buffer. */
    UPROPERTY()
    uint32 ShaderTableAlignment = 0;

    /** Size of an individual element in the ray tracing instance buffer. This defines the required stride and alignment of structured buffers of instances. */
    UPROPERTY()
    uint32 InstanceDescriptorSize = 0;
};


USTRUCT()
struct FSpVariableRateShading
{
    GENERATED_BODY()

    /** Whether or not the RHI can support per-draw Variable Rate Shading. */
    UPROPERTY()
    bool SupportsPipeline = false;

    /** Whether or not the Variable Rate Shading can be done at larger (2x4 or 4x2 or 4x4) sizes. */
    UPROPERTY()
    bool SupportsLargerSizes = false;

    /** Whether or not the RHI can support image-based Variable Rate Shading. */
    UPROPERTY()
    bool SupportsAttachment = false;

    /** Whether or not the RHI can support complex combiner operatations between per-draw (pipeline) VRS and image VRS. */
    UPROPERTY()
    bool SupportsComplexCombinerOps = false;

    /** Whether or not the RHI can support shading rate attachments as array textures. */
    UPROPERTY()
    bool SupportsAttachmentArrayTextures = false;

    /** Maximum tile width in a screen space texture that can be used to drive Variable Rate Shading. */
    UPROPERTY()
    int32 ImageTileMaxWidth = 0;

    /** Maximum tile height in a screen space texture that can be used to drive Variable Rate Shading. */
    UPROPERTY()
    int32 ImageTileMaxHeight = 0;

    /** Minimum tile width in a screen space texture that can be used to drive Variable Rate Shading. */
    UPROPERTY()
    int32 ImageTileMinWidth = 0;

    /** Minimum tile height in a screen space texture that can be used to drive Variable Rate Shading. */
    UPROPERTY()
    int32 ImageTileMinHeight = 0;

    /** Data type contained in a shading-rate image for image-based Variable Rate Shading. */
    UPROPERTY()
    ESpVRSImageDataType ImageDataType = ESpVRSImageDataType::NotSupported;

    /** Image format for the shading rate image for image-based Variable Rate Shading. */
    UPROPERTY()
    ESpPixelFormat ImageFormat = ESpPixelFormat::Unknown;

    /** Whether Variable Rate Shading deferred shading rate texture update is supported. */
    UPROPERTY()
    bool SupportsLateUpdate = false;

};

USTRUCT()
struct FSpShaderBundles
{
    GENERATED_BODY()

    /** Whether current RHI supports native shader bundle dispatch. */
    UPROPERTY()
    bool SupportsDispatch = false;

    /** Whether current RHI supports shader bundle dispatch using work graphs. */
    UPROPERTY()
    bool SupportsWorkGraphDispatch = false;

    /** Whether current RHI supports shader bundle dispatch and RHI parallel translate. */
    UPROPERTY()
    bool SupportsParallel = false;

    /** Whether the current RHI requires shared bindless parameters. */
    UPROPERTY()
    bool RequiresSharedBindlessParameters = false;
};

USTRUCT()
struct FSpRHIGlobals
{
    GENERATED_BODY()

    static constexpr int MaxMSAASampleOffsets = 1 + 2 + 4 + 8 + 16;

    /** True if the render hardware has been initialized. */
    UPROPERTY()
    bool IsRHIInitialized = false;

    /** Optimal number of persistent thread groups to fill the GPU. */
    UPROPERTY()
    int32 PersistentThreadGroupCount = 0;

    /** The maximum number of mip-maps that a texture can contain. */
    UPROPERTY()
    int32 MaxTextureMipCount = MAX_TEXTURE_MIP_COUNT;

    /** true if this platform has quad buffer stereo support. */
    UPROPERTY()
    bool SupportsQuadBufferStereo = false;

    /** true if the RHI supports textures that may be bound as both a render target and a shader resource. */
    UPROPERTY()
    bool SupportsRenderDepthTargetableShaderResources = true;

    /** true if the RHI supports Draw Indirect */
    UPROPERTY()
    bool SupportsDrawIndirect = true;

    /** true if the RHI supports Multi Draw Indirect, a variant of Draw Indirect with variable number of sub-commands generated by GPU */
    UPROPERTY()
    bool SupportsMultiDrawIndirect = false;

    /** Whether the RHI can send commands to the device context from multiple threads. Used in the GPU readback to avoid stalling the RHI threads. */
    UPROPERTY()
    bool SupportsMultithreading = false;

    /** Whether RHIGetRenderQueryResult can be safely called off the render thread. */
    UPROPERTY()
    bool SupportsAsyncGetRenderQueryResult = false;

    UPROPERTY()
    FSpGpuInfo GpuInfo;

    // true if the RHI supports Pixel Shader UAV
    UPROPERTY()
    bool SupportsPixelShaderUAVs = true;

    // true if the RHI supports Vertex Shader UAV
    UPROPERTY()
    bool SupportsVertexShaderUAVs = false;

    /** true if PF_G8 render targets are supported */
    UPROPERTY()
    bool SupportsRenderTargetFormat_PF_G8 = true;

    /** true if PF_FloatRGBA render targets are supported */
    UPROPERTY()
    bool SupportsRenderTargetFormat_PF_FloatRGBA = true;

    /** true if mobile framebuffer fetch is supported */
    UPROPERTY()
    bool SupportsShaderFramebufferFetch = false;

    /** true if mobile framebuffer fetch is supported from MRT's*/
    UPROPERTY()
    bool SupportsShaderMRTFramebufferFetch = false;

    /** true if mobile framebuffer fetch can be used for programmable blending, does not imply that framebuffer fetch is supported*/
    UPROPERTY()
    bool SupportsShaderFramebufferFetchProgrammableBlending = true;

    /** true if mobile pixel local storage is supported */
    UPROPERTY()
    bool SupportsPixelLocalStorage = false;

    /** true if mobile depth & stencil fetch is supported */
    UPROPERTY()
    bool SupportsShaderDepthStencilFetch = false;

    /** true if RQT_AbsoluteTime is supported by RHICreateRenderQuery */
    UPROPERTY()
    bool SupportsTimestampRenderQueries = false;

    /** true if RQT_AbsoluteTime is supported by RHICreateRenderQuery */
    UPROPERTY()
    bool SupportsGPUTimestampBubblesRemoval = false;

    /** true if RHIGetGPUFrameCycles removes CPu generated bubbles. */
    UPROPERTY()
    bool SupportsFrameCyclesBubblesRemoval = false;

    /** true if RHIGetGPUUsage() is supported. */
    UPROPERTY()
    bool SupportsGPUUsage = false;

    /** true if the GPU supports hidden surface removal in hardware. */
    UPROPERTY()
    bool HardwareHiddenSurfaceRemoval = false;

    /** true if the RHI supports asynchronous creation of texture resources */
    UPROPERTY()
    bool SupportsAsyncTextureCreation = false;

    /** true if the RHI supports quad topology (PT_QuadList). */
    UPROPERTY()
    bool SupportsQuadTopology = false;

    /** true if the RHI supports rectangular topology (PT_RectList). */
    UPROPERTY()
    bool SupportsRectTopology = false;

    /** true if the RHI supports primitive shaders. */
    UPROPERTY()
    bool SupportsPrimitiveShaders = false;

    /** true if the RHI supports 64 bit uint atomics. */
    UPROPERTY()
    bool SupportsAtomicUInt64 = false;

    /** true if the RHI supports optimal low level pipeline state sort keys. */
    UPROPERTY()
    bool SupportsPipelineStateSortKey = false;

    /** Temporary. When OpenGL is running in a separate thread, it cannot yet do things like initialize shaders that are first discovered in a rendering task. It is doable, it just isn't done. */
    UPROPERTY()
    bool SupportsParallelRenderingTasksWithSeparateRHIThread = true;

    /** If an RHI is so slow, that it is the limiting factor for the entire frame, we can kick early to try to give it as much as possible. */
    UPROPERTY()
    bool RHIThreadNeedsKicking = false;

    /** The maximum number of in-flight GPU queries the current RHI can handle without stalling and waiting for the GPU. Used to tune the occlusion culler to avoid stalls. */
    UPROPERTY()
    int32 MaximumInFlightQueries = MAX_int32;

    /** Some RHIs can only do visible or not occlusion queries. */
    UPROPERTY()
    bool SupportsExactOcclusionQueries = true;

    /** True if and only if the GPU support rendering to volume textures (2D Array, 3D). Some OpenGL 3.3 cards support SM4, but can't render to volume textures. */
    UPROPERTY()
    bool SupportsVolumeTextureRendering = true;

    /** True if the RHI supports separate blend states per render target. */
    UPROPERTY()
    bool SupportsSeparateRenderTargetBlendState = false;

    /** True if the RHI supports dual src blending. */
    UPROPERTY()
    bool SupportsDualSrcBlending = true;

    /** True if the RHI has artifacts with atlased CSM depths. */
    UPROPERTY()
    bool NeedsUnatlasedCSMDepthsWorkaround = false;

    /** Whether to initialize 3D textures using a bulk data (or through a mip update if false). */
    UPROPERTY()
    bool SupportsTexture3D = true;

    /** true if bulk data should be used with 3d textures */
    UPROPERTY()
    bool UseTexture3DBulkData = false;

    /** true if the RHI supports mobile multi-view */
    UPROPERTY()
    bool SupportsMobileMultiView = false;

    /** true if the RHI supports image external */
    UPROPERTY()
    bool SupportsImageExternal = false;

    /** true if the RHI supports 256bit MRT */
    UPROPERTY()
    bool SupportsWideMRT = true;

    /** True if the RHI and current hardware supports supports depth bounds testing */
    UPROPERTY()
    bool SupportsDepthBoundsTest = false;

    /** True if the RHI supports explicit access to depth target HTile meta data. */
    UPROPERTY()
    bool SupportsExplicitHTile = false;

    /** True if the RHI supports explicit access to MSAA target FMask meta data. */
    UPROPERTY()
    bool SupportsExplicitFMask = false;

    /** True if the RHI supports resummarizing depth target HTile meta data. */
    UPROPERTY()
    bool SupportsResummarizeHTile = false;

    /** True if the RHI supports depth target unordered access views. */
    UPROPERTY()
    bool SupportsDepthUAV = false;

    /** True if the RHI and current hardware supports efficient AsyncCompute (by default we assume false and later we can enable this for more hardware) */
    UPROPERTY()
    bool SupportsEfficientAsyncCompute = false;

    /** True if the RHI supports aliasing transient resources on the async compute pipe. */
    UPROPERTY()
    bool SupportsAsyncComputeTransientAliasing = false;

    /** True if the RHI supports getting the result of occlusion queries when on a thread other than the render thread */
    UPROPERTY()
    bool SupportsParallelOcclusionQueries = false;

    /** true if the RHI requires a valid RT bound during UAV scatter operation inside the pixel shader */
    UPROPERTY()
    bool RequiresRenderTargetForPixelShaderUAVs = false;

    /** true if the RHI supports unordered access view format aliasing */
    UPROPERTY()
    bool SupportsUAVFormatAliasing = false;

    /** true if the RHI supports texture views (data aliasing) */
    UPROPERTY()
    bool SupportsTextureViews = true;

    /** true if the pointer returned by Lock is a persistent direct pointer to gpu memory */
    UPROPERTY()
    bool SupportsDirectGPUMemoryLock = false;

    /** true if the multi-threaded shader creation is supported by (or desirable for) the RHI. */
    UPROPERTY()
    bool SupportsMultithreadedShaderCreation = true;

    /** Does the RHI support parallel resource commands (i.e. create / lock / unlock) on non-immediate command list APIs, recorded off the render thread. */
    UPROPERTY()
    bool SupportsMultithreadedResources = false;

    /** Does this RHI need to wait for deletion of resources due to ref counting. */
    UPROPERTY()
    bool NeedsExtraDeletionLatency = false;

    /** Allow opt-out default RHI resource deletion latency for streaming textures */
    UPROPERTY()
    bool ForceNoDeletionLatencyForStreamingTextures = false;

    /** The maximum size allowed for a computeshader dispatch. */
    UPROPERTY()
    int32 MaxComputeDispatchDimension = ((1 << 16) - 1);

    /** If true, then avoiding loading shader code and instead force the "native" path, which sends a library and a hash instead. */
    UPROPERTY()
    bool LazyShaderCodeLoading = false;

    /** If true, then it is possible to turn on LazyShaderCodeLoading. */
    UPROPERTY()
    bool SupportsLazyShaderCodeLoading = false;

    /** true if the RHI supports UpdateFromBufferTexture method */
    UPROPERTY()
    bool SupportsUpdateFromBufferTexture = false;

    /** The maximum size to allow for the shadow depth buffer in the X dimension.  This must be larger or equal to GMaxShadowDepthBufferSizeY. */
    UPROPERTY()
    int32 MaxShadowDepthBufferSizeX = 2048;

    /** The maximum size to allow for the shadow depth buffer in the Y dimension. */
    UPROPERTY()
    int32 MaxShadowDepthBufferSizeY = 2048;

    /** The maximum size allowed for 2D textures in both dimensions. */
    UPROPERTY()
    int32 MaxTextureDimensions = 2048;

    /** The maximum size allowed for 2D textures in both dimensions. */
    UPROPERTY()
    int64 MaxBufferDimensions = (1 << 27);

    /** The maximum size allowed for a contant buffer. */
    UPROPERTY()
    int64 MaxConstantBufferByteSize = (1 << 27);

    /** The maximum size allowed for Shared Compute Memory. */
    UPROPERTY()
    int64 MaxComputeSharedMemory = (1 << 15);

    /** The maximum size allowed for 3D textures in all three dimensions. */
    UPROPERTY()
    int32 MaxVolumeTextureDimensions = 2048;

    /** The maximum size allowed for cube textures. */
    UPROPERTY()
    int32 MaxCubeTextureDimensions = 2048;

    /** Whether RW texture buffers are supported */
    UPROPERTY()
    bool SupportsRWTextureBuffers = true;

    /** Whether a raw (ByteAddress) buffer view can be created for any buffer, regardless of its EBufferUsageFlags::ByteAddressBuffer flag. */
    UPROPERTY()
    bool SupportsRawViewsForAnyBuffer = false;

    /** Whether depth or stencil can individually be set to CopySrc/Dest access. */
    UPROPERTY()
    bool SupportsSeparateDepthStencilCopyAccess = true;

    /** Support using async thread for texture stream out operations */
    UPROPERTY()
    bool SupportsAsyncTextureStreamOut = false;

    /** The Maximum number of layers in a 1D or 2D texture array. */
    UPROPERTY()
    int32 MaxTextureArrayLayers = 256;

    UPROPERTY()
    int32 MaxTextureSamplers = 16;

    /** The maximum work group invocations allowed for compute shader. */
    UPROPERTY()
    int32 MaxWorkGroupInvocations = 1024;

    /** true if we are running with the NULL RHI */
    UPROPERTY()
    bool UsingNullRHI = false;

    /**
     *	The size to check against for Draw*UP call vertex counts.
     *	If greater than this value, the draw call will not occur.
     */
    UPROPERTY()
    int32 DrawUPVertexCheckCount = MAX_int32;

    /**
     *	The size to check against for Draw*UP call index counts.
     *	If greater than this value, the draw call will not occur.
     */
    UPROPERTY()
    int32 DrawUPIndexCheckCount = MAX_int32;

    // Not supported for now
    // FVertexElementTypeSupportInfo VertexElementTypeSupport;

    /** Whether the next frame should profile the GPU. */
    UPROPERTY()
    bool TriggerGPUProfile = false;

    /** Whether we are profiling GPU hitches. */
    UPROPERTY()
    bool TriggerGPUHitchProfile = false;

    /** Whether an intentional GPU crash has been scheduled. */
    UPROPERTY()
    ESpRequestedGPUCrash TriggerGPUCrash = ESpRequestedGPUCrash::None;

    /** Non-empty if we are performing a gpu trace. Also says where to place trace file. */
    UPROPERTY()
    FString GPUTraceFileName;

    /** True if the RHI supports texture streaming */
    UPROPERTY()
    bool SupportsTextureStreaming = false;

    /** Amount of memory allocated by streaming textures. In kilobytes. */
    UPROPERTY()
    uint64 StreamingTextureMemorySizeInKB = 0;

    /** Amount of memory allocated by non streaming textures. In kilobytes. */
    UPROPERTY()
    uint64 NonStreamingTextureMemorySizeInKB = 0;

    /** Current texture streaming pool size, in bytes. 0 means unlimited. */
    UPROPERTY()
    int64 TexturePoolSize = 0 * 1024 * 1024;

    /** In percent. If non-zero, the texture pool size is a percentage of TotalGraphicsMemory. */
    UPROPERTY()
    int32 PoolSizeVRAMPercentage = 0;

    /** Amount of local video memory demoted to system memory. In bytes. */
    UPROPERTY()
    uint64 DemotedLocalMemorySize = 0;

    /** Amount of memory allocated by buffers */
    UPROPERTY()
    uint64 BufferMemorySize = 0;

    /** Amount of memory allocated by uniform buffers */
    UPROPERTY()
    uint64 UniformBufferMemorySize = 0;

    /** Whether or not the RHI can handle a non-zero BaseVertexIndex - extra SetStreamSource calls will be needed if this is false */
    UPROPERTY()
    bool SupportsBaseVertexIndex = true;

    /** Whether or not the RHI can handle a non-zero FirstInstance to DrawIndexedPrimitive and friends - extra SetStreamSource calls will be needed if this is false */
    UPROPERTY()
    bool SupportsFirstInstance = false;

    /** Whether or not the RHI can handle dynamic resolution or not. */
    UPROPERTY()
    bool SupportsDynamicResolution = false;

    UPROPERTY()
    FSpRayTracing RayTracing;

    /** Whether or not the RHI supports shader wave operations (shader model 6.0). */
    UPROPERTY()
    bool SupportsWaveOperations = false;

    /** Whether or not the current GPU is integrated into to CPU */
    UPROPERTY()
    bool DeviceIsIntegrated = false;

    /**
    * Specifies the minimum and maximum number of lanes in the SIMD wave that this GPU can support. I.e. 32 on NVIDIA, 64 on AMD.
    * Valid values are in range [4..128] (as per SM 6.0 specification) or 0 if unknown.
    * Rendering code must always check SupportsWaveOperations in addition to wave min/max size.
    */
    UPROPERTY()
    int32 MinimumWaveSize = 0;
    UPROPERTY()
    int32 MaximumWaveSize = 0;

    /** Whether or not the RHI supports 16bit VALU and resource loads (HLSL shader model 6.2). */
    UPROPERTY()
    bool SupportsNative16BitOps = false;

    /** Whether or not the RHI supports an RHI thread.
    Requirements for RHI thread
    * Microresources (those in RHIStaticStates.h) need to be able to be created by any thread at any time and be able to work with a radically simplified rhi resource lifecycle. CreateSamplerState, CreateRasterizerState, CreateDepthStencilState, CreateBlendState
    * CreateUniformBuffer needs to be threadsafe
    * GetRenderQueryResult should be threadsafe, but this isn't required. If it isn't threadsafe, then you need to flush yourself in the RHI
    * GetViewportBackBuffer and AdvanceFrameForGetViewportBackBuffer need to be threadsafe and need to support the fact that the render thread has a different concept of "current backbuffer" than the RHI thread. Without an RHIThread this is moot due to the next two items.
    * AdvanceFrameForGetViewportBackBuffer needs be added as an RHI method and this needs to work with GetViewportBackBuffer to give the render thread the right back buffer even though many commands relating to the beginning and end of the frame are queued.
    * BeginDrawingViewport, and 5 or so other frame advance methods are queued with an RHIThread. Without an RHIThread, these just flush internally.
    ***/
    UPROPERTY()
    bool SupportsRHIThread = false;

    /* as above, but we run the commands on arbitrary task threads */
    UPROPERTY()
    bool SupportsRHIOnTaskThread = false;

    /** Whether or not the RHI supports parallel RHIThread executes / translates
    Requirements:
    * RHICreateBoundShaderState & RHICreateGraphicsPipelineState is threadsafe and GetCachedBoundShaderState must not be used. GetCachedBoundShaderState_Threadsafe has a slightly different protocol.
    ***/
    UPROPERTY()
    bool SupportsParallelRHIExecute = false;

    /** Whether or not the RHI can perform MSAA sample load. */
    UPROPERTY()
    bool SupportsMSAADepthSampleAccess = false;

    /** Whether or not the RHI can render to the backbuffer with a custom depth/stencil surface bound. */
    UPROPERTY()
    bool SupportsBackBufferWithCustomDepthStencil = true;

    /** Whether or not HDR is currently enabled */
    UPROPERTY()
    bool IsHDREnabled = false;

    /** Whether the present adapter/display offers HDR output capabilities. */
    UPROPERTY()
    bool SupportsHDROutput = false;

    /** The maximum number of groups that can be dispatched in each dimensions. */
    UPROPERTY()
    FIntVector MaxDispatchThreadGroupsPerDimension = FIntVector::ZeroValue;

    UPROPERTY()
    FSpVariableRateShading VariableRateShading;

    /** Format used for the backbuffer when outputting to a HDR display. */
    UPROPERTY()
    ESpPixelFormat HDRDisplayOutputFormat = ESpPixelFormat::FloatRGBA;

    /** Counter incremented once on each frame present. Used to support game thread synchronization with swap chain frame flips. */
    UPROPERTY()
    uint64 PresentCounter = 1;

    /** True if the RHI supports setting the render target array index from any shader stage */
    UPROPERTY()
    bool SupportsArrayIndexFromAnyShader = false;

    /** True if the pipeline file cache can be used with this RHI */
    UPROPERTY()
    bool SupportsPipelineFileCache = false;

    /** True if the RHI support PSO precaching */
    UPROPERTY()
    bool SupportsPSOPrecaching = false;

    /** True if the RHI supports setting the stencil ref at pixel granularity from the pixel shader */
    UPROPERTY()
    bool SupportsStencilRefFromPixelShader = false;

    /** True if the RHI supports raster order views. */
    UPROPERTY()
    bool SupportsRasterOrderViews = false;

    /** Whether current RHI supports overestimated conservative rasterization. */
    UPROPERTY()
    bool SupportsConservativeRasterization = false;

    /** Whether current RHI supports shader root constants. */
    UPROPERTY()
    bool SupportsShaderRootConstants = false;

    UPROPERTY()
    FSpShaderBundles ShaderBundles;

    /** true if the RHI supports Mesh and Amplification shaders with tier0 capability */
    UPROPERTY()
    bool SupportsMeshShadersTier0 = false;

    /** true if the RHI supports Mesh and Amplification shaders with tier1 capability */
    UPROPERTY()
    bool SupportsMeshShadersTier1 = false;

    /** Whether current RHI supports work graphs with tier1 capability. */
    UPROPERTY()
    bool SupportsShaderWorkGraphsTier1 = false;

    /**
    * True if the RHI supports reading system timer in shaders via GetShaderTimestamp().
    * Individual shaders must be compiled with an appropriate vendor extension and check PLATFORM_SUPPORTS_SHADER_TIMESTAMP.
    */
    UPROPERTY()
    bool SupportsShaderTimestamp = false;

    UPROPERTY()
    bool SupportsEfficientUploadOnResourceCreation = false;

    /** true if the RHI supports RLM_WriteOnly_NoOverwrite */
    UPROPERTY()
    bool SupportsMapWriteNoOverwrite = false;

    /** Tables of all MSAA sample offset for all MSAA supported. Use GetMSAASampleOffsets() to read it. */
    UPROPERTY()
    FVector2f DefaultMSAASampleOffsets[MaxMSAASampleOffsets];

    /** True if the RHI supports pipeline precompiling from any thread. */
    UPROPERTY()
    bool SupportsAsyncPipelinePrecompile = true;

    /** Whether dynamic (bindless) resources are supported */
    UPROPERTY()
    ESpRHIBindlessSupport BindlessSupport = ESpRHIBindlessSupport::Unsupported;

    UPROPERTY()
    FSpReservedResources ReservedResources;

    /** Table for finding out which shader platform corresponds to a given feature level for this RHI. */
    UPROPERTY()
    ESpShaderPlatform ShaderPlatformForFeatureLevel[(int)ESpRHIFeatureLevel::Num];
    
    /** True if the RHI has initialized a device with the debug layer enabled. */
    UPROPERTY()
    bool IsDebugLayerEnabled = false;

    /** True if the RHI needs shader unbinds (SetShaderUnbinds). RHIs that don't need them can avoid creating extra commands. */
    UPROPERTY()
    bool NeedsShaderUnbinds = false;

    /** True if the RHI supports shaders with barycentrics */
    UPROPERTY()
    bool SupportsBarycentricsSemantic = false;

    /** True if RHI supports MSAA resolve with a custom shader */
    UPROPERTY()
    bool SupportsMSAAShaderResolve = false;

    /** Whether Depth Stencil MSAA Resolve Targets are supported. */
    UPROPERTY()
    bool SupportsDepthStencilResolve = false;

    /** True if RHI supports Linear texture format in 3D/Cube/Array texture */
    UPROPERTY()
    bool SupportLinearTextureVolumeFormat = true;
};


UCLASS()
class USpRHIGlobalsInterface : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()
public:
    UFUNCTION(Category="SPEAR")
    static FSpGpuInfo GetGpuInfo() {
        FSpGpuInfo result;
        result.AdapterName = GRHIGlobals.GpuInfo.AdapterName;
        result.AdapterInternalDriverVersion = GRHIGlobals.GpuInfo.AdapterInternalDriverVersion;
        result.AdapterUserDriverVersion = GRHIGlobals.GpuInfo.AdapterUserDriverVersion;
        result.AdapterDriverDate = GRHIGlobals.GpuInfo.AdapterDriverDate;
        result.AdapterDriverOnDenyList = GRHIGlobals.GpuInfo.AdapterDriverOnDenyList;
        result.DeviceId = GRHIGlobals.GpuInfo.DeviceId;
        result.DeviceRevision = GRHIGlobals.GpuInfo.DeviceRevision;
        result.IsAMDPreGCNArchitecture = GRHIGlobals.GpuInfo.IsAMDPreGCNArchitecture;
        result.VendorId = GRHIGlobals.GpuInfo.VendorId;
        return result;
    }

    UFUNCTION(Category="SPEAR")
    static FSpRayTracing GetRayTracing() {
        FSpRayTracing result;
        result.Supported = GRHIGlobals.RayTracing.Supported;
        result.SupportsShaders = GRHIGlobals.RayTracing.SupportsShaders;
        result.SupportsPSOAdditions = GRHIGlobals.RayTracing.SupportsPSOAdditions;
        result.SupportsDispatchIndirect = GRHIGlobals.RayTracing.SupportsDispatchIndirect;
        result.SupportsAsyncBuildAccelerationStructure = GRHIGlobals.RayTracing.SupportsAsyncBuildAccelerationStructure;
        result.SupportsAMDHitToken = GRHIGlobals.RayTracing.SupportsAMDHitToken;
        result.SupportsInlineRayTracing = GRHIGlobals.RayTracing.SupportsInlineRayTracing;
        result.RequiresInlineRayTracingSBT = GRHIGlobals.RayTracing.RequiresInlineRayTracingSBT;
        result.SupportsInlinedCallbacks = GRHIGlobals.RayTracing.SupportsInlinedCallbacks;
        result.SupportsLooseParamsInShaderRecord = GRHIGlobals.RayTracing.SupportsLooseParamsInShaderRecord;
        result.AccelerationStructureAlignment = GRHIGlobals.RayTracing.AccelerationStructureAlignment;
        result.ScratchBufferAlignment = GRHIGlobals.RayTracing.ScratchBufferAlignment;
        result.ShaderTableAlignment = GRHIGlobals.RayTracing.ShaderTableAlignment;
        result.InstanceDescriptorSize = GRHIGlobals.RayTracing.InstanceDescriptorSize;
        return result;
    }

    UFUNCTION(Category="SPEAR")
    static FSpVariableRateShading GetVariableRateShading() {
        FSpVariableRateShading result;
        result.SupportsPipeline = GRHIGlobals.VariableRateShading.SupportsPipeline;
        result.SupportsLargerSizes = GRHIGlobals.VariableRateShading.SupportsLargerSizes;
        result.SupportsAttachment = GRHIGlobals.VariableRateShading.SupportsAttachment;
        result.SupportsComplexCombinerOps = GRHIGlobals.VariableRateShading.SupportsComplexCombinerOps;
        result.SupportsAttachmentArrayTextures = GRHIGlobals.VariableRateShading.SupportsAttachmentArrayTextures;
        result.ImageTileMaxWidth = GRHIGlobals.VariableRateShading.ImageTileMaxWidth;
        result.ImageTileMaxHeight = GRHIGlobals.VariableRateShading.ImageTileMaxHeight;
        result.ImageTileMinWidth = GRHIGlobals.VariableRateShading.ImageTileMinWidth;
        result.ImageTileMinHeight = GRHIGlobals.VariableRateShading.ImageTileMinHeight;
        result.ImageDataType = static_cast<ESpVRSImageDataType>(GRHIGlobals.VariableRateShading.ImageDataType);
        result.ImageFormat = static_cast<ESpPixelFormat>(GRHIGlobals.VariableRateShading.ImageFormat);
        result.SupportsLateUpdate = GRHIGlobals.VariableRateShading.SupportsLateUpdate;
        return result;
    }

    UFUNCTION(Category="SPEAR")
    static FSpShaderBundles GetShaderBundles() {
        FSpShaderBundles result;
        result.SupportsDispatch = GRHIGlobals.ShaderBundles.SupportsDispatch;
        result.SupportsWorkGraphDispatch = GRHIGlobals.ShaderBundles.SupportsWorkGraphDispatch;
        result.SupportsParallel = GRHIGlobals.ShaderBundles.SupportsParallel;
        result.RequiresSharedBindlessParameters = GRHIGlobals.ShaderBundles.RequiresSharedBindlessParameters;
        return result;
    }

    UFUNCTION(Category="SPEAR")
    static FSpReservedResources GetReservedResources() {
        FSpReservedResources result;
        result.Supported = GRHIGlobals.ReservedResources.Supported;
        result.SupportsVolumeTextures = GRHIGlobals.ReservedResources.SupportsVolumeTextures;
        result.TextureArrayMinimumMipDimension = GRHIGlobals.ReservedResources.TextureArrayMinimumMipDimension;
        result.TileSizeInBytes = GRHIGlobals.ReservedResources.TileSizeInBytes;
        return result;
    }

    UFUNCTION(Category="SPEAR")
    static FSpRHIGlobals Get() {
        FSpRHIGlobals result;

        result.GpuInfo = GetGpuInfo();
        result.RayTracing = GetRayTracing();
        result.VariableRateShading = GetVariableRateShading();
        result.ShaderBundles = GetShaderBundles();
        result.ReservedResources = GetReservedResources();

        result.IsRHIInitialized = GRHIGlobals.IsRHIInitialized;
        result.PersistentThreadGroupCount = GRHIGlobals.PersistentThreadGroupCount;
        result.MaxTextureMipCount = GRHIGlobals.MaxTextureMipCount;
        result.SupportsQuadBufferStereo = GRHIGlobals.SupportsQuadBufferStereo;
        result.SupportsRenderDepthTargetableShaderResources = GRHIGlobals.SupportsRenderDepthTargetableShaderResources;
        result.SupportsDrawIndirect = GRHIGlobals.SupportsDrawIndirect;
        result.SupportsMultiDrawIndirect = GRHIGlobals.SupportsMultiDrawIndirect;
        result.SupportsMultithreading = GRHIGlobals.SupportsMultithreading;
        result.SupportsAsyncGetRenderQueryResult = GRHIGlobals.SupportsAsyncGetRenderQueryResult;
        result.SupportsPixelShaderUAVs = GRHIGlobals.SupportsPixelShaderUAVs;
        result.SupportsVertexShaderUAVs = GRHIGlobals.SupportsVertexShaderUAVs;
        result.SupportsRenderTargetFormat_PF_G8 = GRHIGlobals.SupportsRenderTargetFormat_PF_G8;
        result.SupportsRenderTargetFormat_PF_FloatRGBA = GRHIGlobals.SupportsRenderTargetFormat_PF_FloatRGBA;
        result.SupportsShaderFramebufferFetch = GRHIGlobals.SupportsShaderFramebufferFetch;
        result.SupportsShaderMRTFramebufferFetch = GRHIGlobals.SupportsShaderMRTFramebufferFetch;
        result.SupportsShaderFramebufferFetchProgrammableBlending = GRHIGlobals.SupportsShaderFramebufferFetchProgrammableBlending;
        result.SupportsPixelLocalStorage = GRHIGlobals.SupportsPixelLocalStorage;
        result.SupportsShaderDepthStencilFetch = GRHIGlobals.SupportsShaderDepthStencilFetch;
        result.SupportsTimestampRenderQueries = GRHIGlobals.SupportsTimestampRenderQueries;
        result.SupportsGPUTimestampBubblesRemoval = GRHIGlobals.SupportsGPUTimestampBubblesRemoval;
        result.SupportsFrameCyclesBubblesRemoval = GRHIGlobals.SupportsFrameCyclesBubblesRemoval;
        result.SupportsGPUUsage = GRHIGlobals.SupportsGPUUsage;
        result.HardwareHiddenSurfaceRemoval = GRHIGlobals.HardwareHiddenSurfaceRemoval;
        result.SupportsAsyncTextureCreation = GRHIGlobals.SupportsAsyncTextureCreation;
        result.SupportsQuadTopology = GRHIGlobals.SupportsQuadTopology;
        result.SupportsRectTopology = GRHIGlobals.SupportsRectTopology;
        result.SupportsPrimitiveShaders = GRHIGlobals.SupportsPrimitiveShaders;
        result.SupportsAtomicUInt64 = GRHIGlobals.SupportsAtomicUInt64;
        result.SupportsPipelineStateSortKey = GRHIGlobals.SupportsPipelineStateSortKey;
        result.SupportsParallelRenderingTasksWithSeparateRHIThread = GRHIGlobals.SupportsParallelRenderingTasksWithSeparateRHIThread;
        result.RHIThreadNeedsKicking = GRHIGlobals.RHIThreadNeedsKicking;
        result.MaximumInFlightQueries = GRHIGlobals.MaximumInFlightQueries;
        result.SupportsExactOcclusionQueries = GRHIGlobals.SupportsExactOcclusionQueries;
        result.SupportsVolumeTextureRendering = GRHIGlobals.SupportsVolumeTextureRendering;
        result.SupportsSeparateRenderTargetBlendState = GRHIGlobals.SupportsSeparateRenderTargetBlendState;
        result.SupportsDualSrcBlending = GRHIGlobals.SupportsDualSrcBlending;
        result.NeedsUnatlasedCSMDepthsWorkaround = GRHIGlobals.NeedsUnatlasedCSMDepthsWorkaround;
        result.SupportsTexture3D = GRHIGlobals.SupportsTexture3D;
        result.UseTexture3DBulkData = GRHIGlobals.UseTexture3DBulkData;
        result.SupportsMobileMultiView = GRHIGlobals.SupportsMobileMultiView;
        result.SupportsImageExternal = GRHIGlobals.SupportsImageExternal;
        result.SupportsWideMRT = GRHIGlobals.SupportsWideMRT;
        result.SupportsDepthBoundsTest = GRHIGlobals.SupportsDepthBoundsTest;
        result.SupportsExplicitHTile = GRHIGlobals.SupportsExplicitHTile;
        result.SupportsExplicitFMask = GRHIGlobals.SupportsExplicitFMask;
        result.SupportsResummarizeHTile = GRHIGlobals.SupportsResummarizeHTile;
        result.SupportsDepthUAV = GRHIGlobals.SupportsDepthUAV;
        result.SupportsEfficientAsyncCompute = GRHIGlobals.SupportsEfficientAsyncCompute;
        result.SupportsAsyncComputeTransientAliasing = GRHIGlobals.SupportsAsyncComputeTransientAliasing;
        result.SupportsParallelOcclusionQueries = GRHIGlobals.SupportsParallelOcclusionQueries;
        result.RequiresRenderTargetForPixelShaderUAVs = GRHIGlobals.RequiresRenderTargetForPixelShaderUAVs;
        result.SupportsUAVFormatAliasing = GRHIGlobals.SupportsUAVFormatAliasing;
        result.SupportsTextureViews = GRHIGlobals.SupportsTextureViews;
        result.SupportsDirectGPUMemoryLock = GRHIGlobals.SupportsDirectGPUMemoryLock;
        result.SupportsMultithreadedShaderCreation = GRHIGlobals.SupportsMultithreadedShaderCreation;
        result.SupportsMultithreadedResources = GRHIGlobals.SupportsMultithreadedResources;
        result.NeedsExtraDeletionLatency = GRHIGlobals.NeedsExtraDeletionLatency;
        result.ForceNoDeletionLatencyForStreamingTextures = GRHIGlobals.ForceNoDeletionLatencyForStreamingTextures;
        result.MaxComputeDispatchDimension = GRHIGlobals.MaxComputeDispatchDimension;
        result.LazyShaderCodeLoading = GRHIGlobals.LazyShaderCodeLoading;
        result.SupportsLazyShaderCodeLoading = GRHIGlobals.SupportsLazyShaderCodeLoading;
        result.SupportsUpdateFromBufferTexture = GRHIGlobals.SupportsUpdateFromBufferTexture;
        result.MaxShadowDepthBufferSizeX = GRHIGlobals.MaxShadowDepthBufferSizeX;
        result.MaxShadowDepthBufferSizeY = GRHIGlobals.MaxShadowDepthBufferSizeY;
        result.MaxTextureDimensions = GRHIGlobals.MaxTextureDimensions;
        result.MaxBufferDimensions = GRHIGlobals.MaxBufferDimensions;
        result.MaxConstantBufferByteSize = GRHIGlobals.MaxConstantBufferByteSize;
        result.MaxComputeSharedMemory = GRHIGlobals.MaxComputeSharedMemory;
        result.MaxVolumeTextureDimensions = GRHIGlobals.MaxVolumeTextureDimensions;
        result.MaxCubeTextureDimensions = GRHIGlobals.MaxCubeTextureDimensions;
        result.SupportsRWTextureBuffers = GRHIGlobals.SupportsRWTextureBuffers;
        result.SupportsRawViewsForAnyBuffer = GRHIGlobals.SupportsRawViewsForAnyBuffer;
        result.SupportsSeparateDepthStencilCopyAccess = GRHIGlobals.SupportsSeparateDepthStencilCopyAccess;
        result.SupportsAsyncTextureStreamOut = GRHIGlobals.SupportsAsyncTextureStreamOut;
        result.MaxTextureArrayLayers = GRHIGlobals.MaxTextureArrayLayers;
        result.MaxTextureSamplers = GRHIGlobals.MaxTextureSamplers;
        result.MaxWorkGroupInvocations = GRHIGlobals.MaxWorkGroupInvocations;
        result.UsingNullRHI = GRHIGlobals.UsingNullRHI;
        result.DrawUPVertexCheckCount = GRHIGlobals.DrawUPVertexCheckCount;
        result.DrawUPIndexCheckCount = GRHIGlobals.DrawUPIndexCheckCount;
        result.TriggerGPUProfile = GRHIGlobals.TriggerGPUProfile;
        result.TriggerGPUHitchProfile = GRHIGlobals.TriggerGPUHitchProfile;
        result.TriggerGPUCrash = static_cast<ESpRequestedGPUCrash>(GRHIGlobals.TriggerGPUCrash);
        result.GPUTraceFileName = GRHIGlobals.GPUTraceFileName;
        result.SupportsTextureStreaming = GRHIGlobals.SupportsTextureStreaming;
        result.StreamingTextureMemorySizeInKB = GRHIGlobals.StreamingTextureMemorySizeInKB;
        result.NonStreamingTextureMemorySizeInKB = GRHIGlobals.NonStreamingTextureMemorySizeInKB;
        result.TexturePoolSize = GRHIGlobals.TexturePoolSize;
        result.PoolSizeVRAMPercentage = GRHIGlobals.PoolSizeVRAMPercentage;
        result.DemotedLocalMemorySize = GRHIGlobals.DemotedLocalMemorySize;
        result.BufferMemorySize = GRHIGlobals.BufferMemorySize;
        result.UniformBufferMemorySize = GRHIGlobals.UniformBufferMemorySize;
        result.SupportsBaseVertexIndex = GRHIGlobals.SupportsBaseVertexIndex;
        result.SupportsFirstInstance = GRHIGlobals.SupportsFirstInstance;
        result.SupportsDynamicResolution = GRHIGlobals.SupportsDynamicResolution;
        result.SupportsWaveOperations = GRHIGlobals.SupportsWaveOperations;
        result.DeviceIsIntegrated = GRHIGlobals.DeviceIsIntegrated;
        result.MinimumWaveSize = GRHIGlobals.MinimumWaveSize;
        result.MaximumWaveSize = GRHIGlobals.MaximumWaveSize;
        result.SupportsNative16BitOps = GRHIGlobals.SupportsNative16BitOps;
        result.SupportsRHIThread = GRHIGlobals.SupportsRHIThread;
        result.SupportsRHIOnTaskThread = GRHIGlobals.SupportsRHIOnTaskThread;
        result.SupportsParallelRHIExecute = GRHIGlobals.SupportsParallelRHIExecute;
        result.SupportsMSAADepthSampleAccess = GRHIGlobals.SupportsMSAADepthSampleAccess;
        result.SupportsBackBufferWithCustomDepthStencil = GRHIGlobals.SupportsBackBufferWithCustomDepthStencil;
        result.IsHDREnabled = GRHIGlobals.IsHDREnabled;
        result.SupportsHDROutput = GRHIGlobals.SupportsHDROutput;
        result.MaxDispatchThreadGroupsPerDimension = GRHIGlobals.MaxDispatchThreadGroupsPerDimension;
        result.HDRDisplayOutputFormat = static_cast<ESpPixelFormat>(GRHIGlobals.HDRDisplayOutputFormat);
        result.PresentCounter = GRHIGlobals.PresentCounter;
        result.SupportsArrayIndexFromAnyShader = GRHIGlobals.SupportsArrayIndexFromAnyShader;
        result.SupportsPipelineFileCache = GRHIGlobals.SupportsPipelineFileCache;
        result.SupportsPSOPrecaching = GRHIGlobals.SupportsPSOPrecaching;
        result.SupportsStencilRefFromPixelShader = GRHIGlobals.SupportsStencilRefFromPixelShader;
        result.SupportsRasterOrderViews = GRHIGlobals.SupportsRasterOrderViews;
        result.SupportsConservativeRasterization = GRHIGlobals.SupportsConservativeRasterization;
        result.SupportsShaderRootConstants = GRHIGlobals.SupportsShaderRootConstants;
        result.SupportsMeshShadersTier0 = GRHIGlobals.SupportsMeshShadersTier0;
        result.SupportsMeshShadersTier1 = GRHIGlobals.SupportsMeshShadersTier1;
        result.SupportsShaderWorkGraphsTier1 = GRHIGlobals.SupportsShaderWorkGraphsTier1;
        result.SupportsShaderTimestamp = GRHIGlobals.SupportsShaderTimestamp;
        result.SupportsEfficientUploadOnResourceCreation = GRHIGlobals.SupportsEfficientUploadOnResourceCreation;
        result.SupportsMapWriteNoOverwrite = GRHIGlobals.SupportsMapWriteNoOverwrite;
        result.SupportsAsyncPipelinePrecompile = GRHIGlobals.SupportsAsyncPipelinePrecompile;
        result.BindlessSupport = static_cast<ESpRHIBindlessSupport>(GRHIGlobals.BindlessSupport);
        result.IsDebugLayerEnabled = GRHIGlobals.IsDebugLayerEnabled;
        result.NeedsShaderUnbinds = GRHIGlobals.NeedsShaderUnbinds;
        result.SupportsBarycentricsSemantic = GRHIGlobals.SupportsBarycentricsSemantic;
        result.SupportsMSAAShaderResolve = GRHIGlobals.SupportsMSAAShaderResolve;
        result.SupportsDepthStencilResolve = GRHIGlobals.SupportsDepthStencilResolve;
        result.SupportLinearTextureVolumeFormat = GRHIGlobals.SupportLinearTextureVolumeFormat;
        
        for (int i = 0; i < FSpRHIGlobals::MaxMSAASampleOffsets; i++) {
            result.DefaultMSAASampleOffsets[i] = GRHIGlobals.DefaultMSAASampleOffsets[i];
        }

        for (int i = 0; i < (int)ESpRHIFeatureLevel::Num; i++) {
            result.ShaderPlatformForFeatureLevel[i] = static_cast<ESpShaderPlatform>(GRHIGlobals.ShaderPlatformForFeatureLevel[i]);
        }

        return result;
    }
};
