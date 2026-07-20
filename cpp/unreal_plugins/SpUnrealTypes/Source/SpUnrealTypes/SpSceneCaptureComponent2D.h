//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <map>
#include <memory> // std::unique_ptr
#include <string>
#include <utility> // std::pair
#include <vector>

#include <Components/SceneCaptureComponent2D.h>
#include <Containers/Array.h>
#include <Containers/EnumAsByte.h>
#include <Containers/IndirectArray.h>
#include <Containers/Map.h>
#include <Containers/UnrealString.h>
#include <Delegates/IDelegateInstance.h>  // FDelegateHandle
#include <Engine/EngineTypes.h>           // EEndPlayReason
#include <Engine/TextureRenderTarget2D.h> // ETextureRenderTargetFormat
#include <HAL/Platform.h>                 // int32, uint32
#include <RHIGPUReadback.h>               // FRHIGPUTextureReadback
#include <RendererInterface.h>            // IPooledRenderTarget
#include <SceneTypes.h>                   // FSceneViewStateReference
#include <SceneView.h>                    // FSceneViewFamily
#include <SceneViewExtension.h>           // FAutoRegister, FSceneViewExtensionBase, FSceneViewExtensions
#include <Templates/RefCounting.h>        // TRefCountPtr
#include <Templates/SharedPointer.h>      // TSharedPtr
#include <Templates/SharedPointerFwd.h>   // ESPMode
#include <TextureResource.h>              // FTextureRenderTargetResource
#include <UObject/ObjectMacros.h>         // GENERATED_BODY, UCLASS, UFUNCTION, UPROPERTY

#include "SpCore/Assert.h"
#include "SpCore/Boost.h"
#include "SpCore/SharedMemory.h"
#include "SpCore/SpArray.h"
#include "SpCore/SpFuncComponent.h"
#include "SpCore/Std.h"

#include "SpUnrealTypes/SpMeshProxyComponentManager.h"

#include "SpSceneCaptureComponent2D.generated.h"

class FRDGBuilder;
class FRHICommandListImmediate;
class FRHITexture;
class UMaterial;

class FSpSceneViewExtensionBase : public FSceneViewExtensionBase
{
public:
    FSpSceneViewExtensionBase(const FAutoRegister& auto_register, USpSceneCaptureComponent2D* component);

    void SetupViewFamily(FSceneViewFamily& in_view_family) override;
    void SetupView(FSceneViewFamily& in_view_family, FSceneView& in_view) override;
    void BeginRenderViewFamily(FSceneViewFamily& in_view_family) override;
    void PostRenderViewFamily_RenderThread(FRDGBuilder& graph_builder, FSceneViewFamily& in_view_family) override;

protected:
    virtual void setupViewFamily(FSceneViewFamily& view_family) {};
    virtual void setupView(FSceneViewFamily& view_family, FSceneView& view) {};
    virtual void beginRenderViewFamily(FSceneViewFamily& view_family) {};
    virtual void postRenderViewFamily_RenderThread(FRDGBuilder& graph_builder, FSceneViewFamily& view_family) {};

    USpSceneCaptureComponent2D* getComponent() { SP_ASSERT(component_); return component_; };

private:
    bool shouldHandleViewFamily(const FSceneViewFamily* view_family) const;
    bool shouldHandleView(const FSceneViewFamily* view_family, const FSceneView* view) const;

    USpSceneCaptureComponent2D* component_ = nullptr;
};

class FSpSceneViewExtension : public FSpSceneViewExtensionBase
{
public:
    FSpSceneViewExtension(const FAutoRegister& auto_register, USpSceneCaptureComponent2D* component);
protected:
    void setupView(FSceneViewFamily& view_family, FSceneView& view) override;
    void postRenderViewFamily_RenderThread(FRDGBuilder& graph_builder, FSceneViewFamily& view_family) override;
};

UENUM()
enum class ESpBufferingMode : uint8
{
    SingleBuffered = 0,
    DoubleBuffered = 1,
    TripleBuffered = 2
};

USTRUCT(BlueprintType)
struct FSpUserSceneTextureMaterialDesc
{
    GENERATED_BODY()

    // Post-process material.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    UMaterial* Material = nullptr;

    // The UserSceneTexture output name specified in the material editor.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    FString InternalName;

    // The resolution divisor specified in the material editor. The transient texture's extent is the render
    // extent divided by this rounded up per component, so it lets us allocate shared memory buffers ahead of
    // time.

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    int32 ResolutionDivisorWidth = 1;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    int32 ResolutionDivisorHeight = 1;
};

// We need meta=(BlueprintSpawnableComponent) for the component to show up when using the "+Add" button in the editor.
UCLASS(ClassGroup="SPEAR", HideCategories=(Activation, AssetUserData, Collision, Cooking, LOD, Navigation, Physics, Rendering, Tags), meta=(BlueprintSpawnableComponent))
class USpSceneCaptureComponent2D : public USceneCaptureComponent2D
{
    GENERATED_BODY()
public:
    USpSceneCaptureComponent2D();
    ~USpSceneCaptureComponent2D() = default;

    // UActorComponent interface
    void BeginPlay() override;
    void EndPlay(const EEndPlayReason::Type end_play_reason) override;

    UFUNCTION(BlueprintCallable, CallInEditor, Category="SPEAR")
    void Initialize();
    UFUNCTION(BlueprintCallable, CallInEditor, Category="SPEAR")
    void Terminate();
    UFUNCTION(BlueprintCallable, Category="SPEAR")
    bool IsInitialized();

    // requests that the path tracer restarts rendering from scratch; only has an effect if bUseSceneViewExtension is true
    UFUNCTION(BlueprintCallable, CallInEditor, Category="SPEAR")
    void RequestPathTracerReset();

    // called from FSpSceneViewExtensionBase::shouldHandleView(...) when deciding whether or not this component matches the current view
    const TIndirectArray<FSceneViewStateReference>& getViewStates() const { return ViewStates; }

    // called from FSpSceneViewExtension::setupView(...) to set up the view before rendering
    void setupView(FSceneViewFamily& view_family, FSceneView& view);

    // called from FSpSceneViewExtension::postRenderViewFamily_RenderThread(...) to do post-render work on the render thread
    void postRenderViewFamily_RenderThread(FRDGBuilder& graph_builder, FSceneViewFamily& view_family);

    // BlueprintReadWrite is incompatible with uint32 so we use int32

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    int32 Width = 512;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    int32 Height = 512;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    int32 NumChannelsPerPixel = 4;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    ESpArrayDataType ChannelDataType = ESpArrayDataType::UInt8;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    bool bOverrideTextureRenderTargetFormat = false;

    // TEnumAsByte avoids: "Error: You cannot use the raw enum name as a type for member variables, instead use TEnumAsByte or a C++11 enum class with an explicit underlying type."
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    TEnumAsByte<ETextureRenderTargetFormat> TextureRenderTargetFormat = ETextureRenderTargetFormat::RTF_RGBA8_SRGB;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    bool bOverrideTextureRenderTargetSRGB = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    bool bTextureRenderTargetSRGB = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    bool bOverrideTextureRenderTargetForceLinearGamma = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    bool bTextureRenderTargetForceLinearGamma = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    bool bOverrideTextureRenderTargetGamma = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    float TextureRenderTargetGamma = 0.0;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    UMaterial* Material = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    bool bUseSharedMemory = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    bool bReadPixelData = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    ESpBufferingMode BufferingMode = ESpBufferingMode::SingleBuffered;

    // The universe of possible post-process materials that can be enabled dynamically keyed by a "public name".
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    TMap<FString, FSpUserSceneTextureMaterialDesc> UserSceneTextureMaterialDescs;

    // Selects which materials are active by populating this array with "public names" (i.e., keys into UserSceneTextureMaterialDescs).
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    TArray<FString> UserSceneTextureNames;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    bool bPrintReadbackSpinWaitInfo = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    TSubclassOf<ASpMeshProxyComponentManager> MeshProxyComponentManagerClass;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    bool bUseSceneViewExtension = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    bool bPrintFrameTime = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    bool bPrintFrameTimeEveryFrame = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    bool bReadPixelsEveryFrame = false;

private:

    // State common to TextureReadbackDesc (persistent) and TextureReadbackMinimalDesc (transient). Factoring it into a
    // base lets getPackedArray() have a single implementation.
    struct TextureReadbackDescBase
    {
        // metadata that is used when the pixel data is being read back
        int32 width_ = 0;
        int32 height_ = 0;
        int32 num_channels_per_pixel_ = 0;
        SpArrayDataType channel_data_type_ = SpArrayDataType::Invalid;

        SpArraySharedMemoryView shared_memory_view_; // only used when shared memory is enabled
    };

    // std::atomic makes TextureReadbackDesc non-copyable and non-movable, so populate maps with try_emplace(), not insert()
    struct TextureReadbackDesc : TextureReadbackDescBase
    {
        // owns the shared memory region whose view lives in the base (only used when shared memory is enabled)
        std::unique_ptr<SharedMemoryRegion> shared_memory_region_ = nullptr;

        // scratchpad buffer for staging-to-CPU data (only used when shared memory is disabled and only when double-buffered or triple-buffered)
        std::vector<uint8_t, SpAlignedAllocator<uint8_t, 4096>> scratchpad_;

        // RHI GPU texture readback resources (single-buffered and double-buffered are hard-coded to only use index 0, triple-buffered uses index 0 and 1)
        std::array<std::unique_ptr<FRHIGPUTextureReadback>, 2> rhi_gpu_texture_readbacks_;

        // Double-buffered: GT increments in enqueueCopyPixelsDoubleBuffered(), RT decrements in postRenderViewFamily_RenderThread()
        // Triple-buffered: GT increments in enqueueCopyPixelsTripleBuffered(), RT decrements in enqueueCopyPixelsTripleBuffered()'s render command
        std::atomic<int> num_readbacks_pending_ = 0;

        // Triple-buffered
        int readback_enqueue_index_ = 0; // GT-only: ring-buffer index, advanced in requestSwapRHIGPUTextureReadbacks()
        bool readback_primed_ = false;   // GT-only: one-way latch, set true by the first requestSwapRHIGPUTextureReadbacks() call

        // A user scene texture's extracted transient render target (null for the main texture, which reads from
        // TextureTarget). Written on the render thread by postRenderViewFamily_RenderThread() every frame. Its address
        // is stable for the desc's lifetime, but its value is not, so only the render thread reads its value (via the
        // minimal desc's pooled_render_target_ptr_); the game thread only ever takes its address.
        TRefCountPtr<IPooledRenderTarget> pooled_render_target_;
    };

    // A minimal and movable view of readback data required on the rendering thread. This data structure is not
    // intended to be persistent: its readback pointers, destination, and source handle are re-resolved every frame.
    // It must be assembled on the game thread every frame to be used for a single readback.
    struct TextureReadbackMinimalDesc : TextureReadbackDescBase
    {
        FRHIGPUTextureReadback* current_readback_ = nullptr; // GPU-to-staging copy target for this frame (used by all buffering modes)
        FRHIGPUTextureReadback* prev_readback_ = nullptr;    // staging-to-CPU copy source for the in-command copy; nullptr if there is no in-command copy this frame

        // Stable handles to the GPU-to-staging source, recorded on the game thread and resolved to a live FRHITexture
        // on the render thread (see enqueueCopyPixelsFromGPUToStaging_RenderThread). Exactly one is non-null. We never
        // resolve the live texture on the game thread. For the main texture that would be unnecessary, and for a user
        // scene texture, it would race with the render thread reassigning pooled_render_target_ every frame.
        FTextureRenderTargetResource* render_target_resource_ptr_ = nullptr;    // main texture: render thread calls GetRenderTargetTexture()
        TRefCountPtr<IPooledRenderTarget>* pooled_render_target_ptr_ = nullptr; // user scene texture: render thread dereferences and calls GetRHI()

        void* dest_ptr_ = nullptr; // CPU destination for the in-command staging-to-CPU copy

        // points at the persistent desc's pending counter (incremented on the game thread, decremented on the render thread); only used by double- and triple-buffered
        std::atomic<int>* num_readbacks_pending_ = nullptr;
    };

    // callbacks
    void beginFrameHandler();
    void endFrameHandler();

    // single-buffered mode
    SpFuncDataBundle readPixelsSingleBuffered(std::map<std::string, TextureReadbackMinimalDesc>& texture_readback_minimal_descs);

    // double-buffered mode
    void enqueueCopyPixelsDoubleBuffered(std::map<std::string, TextureReadbackMinimalDesc>& texture_readback_minimal_descs);
    SpFuncDataBundle readPixelsDoubleBuffered();

    // triple-buffered mode
    void enqueueCopyPixelsTripleBuffered(std::map<std::string, TextureReadbackMinimalDesc>& texture_readback_minimal_descs);
    SpFuncDataBundle readPixelsTripleBuffered();

    // game thread helpers
    std::map<std::string, TextureReadbackMinimalDesc> requestUpdateAndGetTextureReadbackMinimalDescs();
    std::pair<FRHIGPUTextureReadback*, FRHIGPUTextureReadback*> requestSwapRHIGPUTextureReadbacks(TextureReadbackDesc& texture_readback_desc); // returns {current, prev}; advances the triple-buffered ring-buffer state
    TextureReadbackMinimalDesc getTextureReadbackMinimalDesc(
        TextureReadbackDesc& texture_readback_desc,
        FRHIGPUTextureReadback* current_readback,
        FRHIGPUTextureReadback* prev_readback,
        FTextureRenderTargetResource* render_target_resource_ptr,
        TRefCountPtr<IPooledRenderTarget>* pooled_render_target_ptr);
    SpPackedArray getPackedArray(const TextureReadbackDescBase& texture_readback_desc_base);
    SpPackedArray readPixelsImpl(const TextureReadbackDesc& texture_readback_desc);

    // render thread helpers
    void enqueueCopyPixelsFromGPUToStaging_RenderThread(FRHICommandListImmediate& rhi_command_list_immediate, const TextureReadbackMinimalDesc& texture_readback_minimal_desc);
    void copyPixelsFromStagingToCPU_RenderThread(FRHIGPUTextureReadback* readback, void* dest_ptr, const TextureReadbackDescBase& texture_readback_desc_base);

    // miscellaneous helpers
    void updateFrameTime();

    UPROPERTY(VisibleAnywhere, Category="SPEAR")
    bool bIsInitialized = false;

    UPROPERTY(VisibleAnywhere, Category="SPEAR")
    USpFuncComponent* SpFuncComponent = nullptr;

    bool is_initialized_ = false;

    // Scene view extension
    TSharedPtr<FSpSceneViewExtension, ESPMode::ThreadSafe> scene_view_extension_ = nullptr;

    // Texture readback state for main texture and user scene textures
    TextureReadbackDesc texture_readback_desc_;
    std::map<std::string, TextureReadbackDesc> user_scene_texture_readback_descs_;

    // Path tracer state
    bool request_path_tracer_reset_ = false;

    // Additional state for measuring "standalone" and "standalone + extra work" frame rates.
    FDelegateHandle begin_frame_handle_;
    FDelegateHandle end_frame_handle_;
    SpFuncDataBundle scratchpad_data_bundle_;
    boost::circular_buffer<double> previous_time_deltas_;
    std::chrono::time_point<std::chrono::high_resolution_clock> previous_time_point_;
    int frame_index_ = 0;
};
