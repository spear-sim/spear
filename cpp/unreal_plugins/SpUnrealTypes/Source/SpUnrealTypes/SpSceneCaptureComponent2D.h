//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <map>
#include <memory>  // std::unique_ptr
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
class UMaterialInterface;

//
// USpSceneCaptureComponent2D is the component that SPEAR uses to render images and return them to Python. The purpose
// of this overview is to connect the code below with the camera sensor described in the SPEAR paper (see the README for
// a link), and in the programming model described in SpServices/EngineService.h.
//
// Reading pixels
//
// The primary responsibility of USpSceneCaptureComponent2D is to copy rendered images from the GPU to the CPU, and to
// return them to the user's Python code with a predictable amount of latency. USpSceneCaptureComponent2D derives from
// UE's USceneCaptureComponent2D, which renders the scene from the component's point of view into a render target on the
// GPU once per frame. USpSceneCaptureComponent2D adds the machinery to copy that render target back to the CPU, and
// exposes this machinery to the user's Python code through two SpFuncs, "enqueue_read_pixels" and "read_pixels". (In
// SPEAR, an SpFunc is a named C++ lambda function that is registered on a scene component that the user's Python code
// can call like an ordinary method on the Python component, e.g., component.read_pixels()). Internally, SpFuncs are
// called through the "sp_func_service.call_function" game thread entry point, so they always execute on the game
// thread.)
//
// UE renders each frame on a dedicated thread, which we refer to as the render thread. As the game thread executes a
// frame, it enqueues commands for the render thread, and the render thread executes these commands in the order they
// were enqueued, concurrently with the game thread. The render thread, in turn, enqueues commands for the GPU, and the
// GPU executes these commands asynchronously with respect to both threads.
//
// Copying an already-rendered image from the GPU to the CPU happens in two steps. First, the render thread enqueues a
// GPU command that copies the GPU buffer into a staging buffer (i.e., a buffer that the render thread is allowed to
// read from via a Lock() and Unlock() interface). We refer to this step as the GPU-to-staging copy. Second, the render
// thread calls Lock() on the staging buffer and copies its contents into a CPU buffer that is managed by
// USpSceneCaptureComponent2D. We refer to this step as the staging-to-CPU copy. The GPU can't execute the
// GPU-to-staging copy until it has finished rendering an image to the GPU buffer, and Lock() blocks until the GPU has
// finished executing the GPU-to-staging copy. If the GPU-to-staging copy happens too late after the frame is finished
// rendering, or if the staging-to-CPU copy happens too soon after the GPU-to-staging copy has been enqueued, then the
// end-to-end copy operation will block waiting for the GPU. So the key design question is how to arrange the
// GPU-to-staging and staging-to-CPU copy operations with respect to the rest of the rendering work that must happen
// during each frame. USpSceneCaptureComponent2D supports three buffering modes, each of which addresses this question
// differently in a way that makes a different latency-vs-throughput tradeoff.
//
//     - SingleBuffered: The user's Python code calls read_pixels() in its instance.end_frame() context. At this point,
//                       the game thread has already enqueued the necessary commands to render the current frame to the
//                       GPU buffer. read_pixels() enqueues the GPU-to-staging copy and the staging-to-CPU copy on the
//                       rendering thread and blocks until the render thread has completed all of them. Therefore,
//                       read_pixels() returns the image that was rendered during the current frame. SingleBuffered mode
//                       is the most convenient mode, because it doesn't require any setup work, and the user's Python
//                       code receives the most up-to-date possible each image with 0 frames of latency. But it is also
//                       the slowest, because it must wait for the GPU to finish rendering the current frame, and for
//                       the GPU-to-staging copy, and for the staging-to-CPU copy.
//
//     - DoubleBuffered: Instead of waiting for all of this work in read_pixels(), DoubleBuffered mode assumes that the
//                       user's Python code is willing to receive each image with 1 frame of latency in exchange for
//                       higher throughput. In this mode, the user's Python code must call enqueue_read_pixels() in its
//                       instance.begin_frame() context, which enqueues the GPU-to-staging copy and returns immediately.
//                       The GPU-to-staging copy executes at the start of the frame using the previous frame's GPU
//                       buffer. Importantly, the render thread (and even the GPU itself) make progress with the rest of
//                       the frame while the GPU-to-staging copy is happening. Then, the staging-to-CPU copy is
//                       performed in a post-frame callback on the render thread. The user's Python code still calls
//                       read_pixels() in its instance.end_frame() context, which must still wait until the
//                       staging-to-CPU copy has finished, but this requires much less waiting than in SingleBuffered
//                       mode. The tradeoff is that read_pixels() returns the image that was rendered during the
//                       previous frame. Ideally we would prefer to schedule the staging-to-CPU copy somewhere in the
//                       middle of the rendering thread's per-frame work, instead of near the end in our post-frame
//                       rendering callback. More specifically, we would prefer for the staging-to-CPU copy to happen
//                       late enough that the GPU-to-staging copy has had time to finish to avoid stalling the rendering
//                       thread, but early enough that we can overlap the staging-to-CPU copy with other per-frame
//                       rendering thread work. But this is not straightforward using Unreal's rendering thread
//                       callbacks (there is no middle-of-frame rendering thread callback), so we settle for doing the
//                       staging-to-CPU copy in a post-frame rendering thread callback.
//
//     - TripleBuffered: Like DoubleBuffered mode, TripleBuffered mode requires the user's Python code to call
//                       enqueue_read_pixels() in its instance.begin_frame() context and read_pixels() in its
//                       instance.end_frame() context. But in this mode, the GPU-to-staging copy and the staging-to-CPU
//                       copy are pipelined, and both are enqueued at the start of the frame. This gives the GPU an
//                       entire frame to do GPU-to-staging copy, and likewise with the staging-to-CPU copy. Importantly,
//                       the rendering thread (and the GPU) can make progress with the rest of the frame while both
//                       copies are happening concurrently. The user's Python code still calls read_pixels() in its
//                       instance.end_frame() context, which must still wait until the staging-to-CPU copy has finished,
//                       but in practice this always finishes much faster than rendering, so the data is ready in the
//                       final destination buffer by the time the user calls read_pixels(). The tradeoff is an
//                       additional frame of latency (i.e., read_pixels() returns the image that was rendered two frames
//                       ago), and USpSceneCaptureComponent2D must maintain two staging buffers, since one is being
//                       copied to, and the other is being copied from, concurrently, during each frame.
//
// An important consequence of this design is that, in DoubleBuffered and TripleBuffered modes, read_pixels() doesn't
// return a meaningful image until enqueue_read_pixels() has been called once or twice, respectively, which we refer to
// as priming the readback pipeline (see examples/render_image_buffered). Additionally, the latencies described above
// are measured in UE frames. Consecutive user transactions are only guaranteed to execute on consecutive frames when
// the user's Python code calls instance.end_frame(single_step=True) (see EngineService.h), so this is the only case
// where DoubleBuffered and TripleBuffered modes return images with a fixed amount of transaction latency.
//
// User Scene Textures
//
// In addition to a final beauty image, our implementation can return intermediate images that are written by
// post-process materials during rendering, which UE refers to as User Scene Textures (USTs). USTs are useful for
// returning additional per-pixel data that is computed during the same overall rendering pass as the final image. From
// the user's perspective, USTs behave exactly like the final image. They are read back using the same buffering mode as
// the final image data, with the same amount of latency, and read_pixels() returns them alongside the final image.
//
// The only complication is that UE allocates each UST as a transient texture that only exists while the render thread
// is rendering the frame. So, during each frame, our implementation enqueues a GPU-to-GPU copy for each UST into a
// persistent GPU texture owned by our implementation in a post-frame rendering thread callback. Once these GPU-to-GPU
// copies have been enqueued, the rest of our implementation can proceed as though the USTs were rendered directly to
// our persistent textures, just like the final image data, and our persistent textures interact with all of our
// buffering modes identically to the final image data.
//
// Shared memory
//
// By default, our implementation returns images to the user's Python code via shared memory, i.e., a region of memory
// that is mapped into both the UE process and the user's Python process. Shared memory enables our implementation to
// return large images to the user's Python code efficiently, because the images don't need to be serialized and sent
// over an RPC connection to the user's Python code. Our use of shared memory doesn't fundamentally change how any of
// our buffering modes work, it only changes the CPU destination of our staging-to-CPU copy operations. When shared
// memory is enabled, our implementation performs the staging-to-CPU copies directly into shared memory regions, and
// read_pixels() returns SpPackedArrays that refer to these regions, so returning images to the user's Python code
// doesn't require any additional copying.
//
// To facilitate the use of shared memory, we must introduce a volatility contract with the user that governs when the
// user is allowed to read from shared memory, and when our implementation is allowed to write to shared memory. This
// contract is necessary because the render thread writes into memory that the user's Python code reads, without any
// synchronization beyond the programming model described in SpServices/EngineService.h and the interprocess shared
// memory fence guarantees in SpCore/SharedMemory.h. Each buffering mode has a distinct contract.
//
//     - SingleBuffered: Our implementation writes to the shared memory region during each call to read_pixels(), and
//                       read_pixels() doesn't return until the write has finished. So the user can safely access the
//                       shared memory region in between calls to read_pixels().
//
//     - DoubleBuffered: Our implementation writes to the shared memory region at some point after each call to
//                       enqueue_read_pixels(), and may continue writing to the shared memory region until read_pixels()
//                       returns. So the user can safely access the shared memory region after read_pixels() returns and
//                       before calling enqueue_read_pixels() again, but must not access the shared memory in between
//                       calls to enqueue_read_pixels() and read_pixels().
//
//     - TripleBuffered: Same as DoubleBuffered mode.
//

class FSpSceneViewExtensionBase : public FSceneViewExtensionBase
{
public:
    FSpSceneViewExtensionBase(const FAutoRegister& auto_register);
    ~FSpSceneViewExtensionBase() override;

    void initialize(USpSceneCaptureComponent2D* component) { SP_ASSERT(component); component_ = component; }
    void terminate() { component_ = nullptr; }

    bool IsActiveThisFrame_Internal(const FSceneViewExtensionContext& context) const override;
    void SetupViewFamily(FSceneViewFamily& in_view_family) override;
    void SetupView(FSceneViewFamily& in_view_family, FSceneView& in_view) override;
    void BeginRenderViewFamily(FSceneViewFamily& in_view_family) override;
    void PostRenderViewFamily_RenderThread(FRDGBuilder& graph_builder, FSceneViewFamily& in_view_family) override;

protected:
    virtual void setupViewFamily(FSceneViewFamily& view_family) {};
    virtual void setupView(FSceneViewFamily& view_family, FSceneView& view) {};
    virtual void beginRenderViewFamily(FSceneViewFamily& view_family) {};
    virtual void postRenderViewFamily_RenderThread(FRDGBuilder& graph_builder, FSceneViewFamily& view_family) {};

    USpSceneCaptureComponent2D* getComponent() const { SP_ASSERT(component_); return component_; };

private:
    bool shouldHandleViewFamily(const FSceneViewFamily* view_family) const;
    bool shouldHandleView(const FSceneViewFamily* view_family, const FSceneView* view) const;

    USpSceneCaptureComponent2D* component_ = nullptr;
};

class FSpSceneViewExtension : public FSpSceneViewExtensionBase
{
public:
    FSpSceneViewExtension(const FAutoRegister& auto_register);
    ~FSpSceneViewExtension() override;

protected:
    void setupView(FSceneViewFamily& view_family, FSceneView& view) override;
    void postRenderViewFamily_RenderThread(FRDGBuilder& graph_builder, FSceneViewFamily& view_family) override;
};

UENUM(BlueprintType)
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

    // Post-process material (may be a UMaterial or a UMaterialInstance, since post-process blendables accept either).
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    UMaterialInterface* Material = nullptr;

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
    bool IsInitialized() const;

    UFUNCTION()
    TArray<uint64> GetViewStates(); // uint64 not supported for BlueprintCallable, can't be const, FSceneViewStateInterface is not a UCLASS so we can't return FSceneViewStateInterface* and we return uint64 instead

    // Functions for setting deferred state that gets consumed in an FSceneViewExtension callback
    UFUNCTION(BlueprintCallable, CallInEditor, Category="SPEAR")
    void RequestPathTracerReset();
    UFUNCTION(BlueprintCallable, Category="SPEAR")
    void RequestSetOfflineRender(bool bOverrideIsOfflineRender, bool bIsOfflineRender);

    // called from FSpSceneViewExtensionBase::shouldHandleView(...) when deciding whether or not this component matches the current view
    const TIndirectArray<FSceneViewStateReference>& getViewStates() const { return ViewStates; }

    // called from FSpSceneViewExtension::setupView(...) to set up the view before rendering
    void setupView(FSceneViewFamily& view_family, FSceneView& view);

    // called from FSpSceneViewExtension::postRenderViewFamily_RenderThread(...) to do post-render work on the render thread
    void postRenderViewFamily_RenderThread(FRDGBuilder& graph_builder, FSceneViewFamily& view_family);

private:
    bool showFlagsNeedSceneViewExtension() const;

public:
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
    UMaterialInterface* Material = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    TSubclassOf<ASpMeshProxyComponentManager> MeshProxyComponentManagerClass;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    ESpBufferingMode BufferingMode = ESpBufferingMode::SingleBuffered;

    // The universe of possible post-process materials that can be enabled dynamically keyed by a "public name".
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    TMap<FString, FSpUserSceneTextureMaterialDesc> UserSceneTextureMaterialDescs;

    // Selects which materials are active by populating this array with "public names" (i.e., keys into UserSceneTextureMaterialDescs).
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    TArray<FString> UserSceneTextureNames;

    // When disabled, the main (primary) texture is excluded entirely from the returned data (the "data" entry is
    // omitted), so only the user scene textures are read. Unlike bReadPixelData below (a developer/benchmarking flag
    // that skips the internal GPU-to-CPU readback without changing the shape of the returned data), this flag is
    // user-facing and deliberately changes the shape of the returned data.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    bool bReadPrimaryPixelData = true;

    // Can be disabled for debugging

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    bool bUseSharedMemory = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    bool bReadPixelData = true;

    // Can be enabled for debugging

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    bool bPrintFrameTimeEveryFrame = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    bool bReadPixelsEveryFrame = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    bool bPrintReadbackSpinWaitInfo = false;

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
        std::unique_ptr<SharedMemoryRegion> shared_memory_region_;

        // scratchpad buffer for staging-to-CPU data (only used when shared memory is disabled and only when double-buffered or triple-buffered)
        std::vector<uint8_t, SpAlignedAllocator<uint8_t, 4096>> scratchpad_;

        // RHI GPU texture readback resources (single-buffered and double-buffered are hard-coded to only use index 0, triple-buffered uses index 0 and 1)
        std::array<std::unique_ptr<FRHIGPUTextureReadback>, 2> rhi_gpu_texture_readbacks_;

        // Read-side wait counter. Incremented on the game thread at enqueue time so readPixelsImpl()'s game-thread
        // spin-wait always observes an outstanding readback (it must be bumped before read_pixels() can run, so we can't
        // defer it to the render thread), and decremented on the render thread once the staging-to-CPU copy is done.
        // Double-buffered: GT increments in enqueueReadPixelsDoubleBuffered(), RT decrements in postRenderViewFamily_RenderThread()
        // Triple-buffered: GT increments in enqueueReadPixelsTripleBuffered(), RT decrements in enqueueReadPixelsTripleBuffered()'s render command
        std::atomic<int> num_readbacks_pending_ = 0;

        // Double-buffered consume gate. Incremented on the render thread inside enqueueReadPixelsDoubleBuffered()'s
        // command, immediately after the GPU-to-staging EnqueueCopy() populates the staging texture, and decremented on
        // the render thread in postRenderViewFamily_RenderThread() when the staging-to-CPU copy consumes it. Because the
        // GPU-to-staging copy runs in an async command but the staging-to-CPU copy runs in a separate render-thread
        // callback (postRenderViewFamily_RenderThread), we can't gate the consume on num_readbacks_pending_: under
        // r.OneFrameThreadLag>0 the render thread runs behind the game thread, so a postRenderViewFamily callback from an
        // in-flight frame can fire after the game thread bumped num_readbacks_pending_ but before the command populated
        // the staging texture, and would then Lock() an empty slot. Gating on this render-thread-only counter guarantees
        // we only consume a slot whose EnqueueCopy() has actually executed. Triple-buffered does not need this because
        // its consume runs inside the same command, in render-thread FIFO order behind the slot's populating command.
        std::atomic<int> num_staging_copies_pending_ = 0;

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
        std::atomic<int>* num_readbacks_pending_ptr_ = nullptr;

        // points at the persistent desc's render-thread consume-gate counter (incremented and decremented on the render thread); only used by double-buffered
        std::atomic<int>* num_staging_copies_pending_ptr_ = nullptr;
    };

    // callbacks
    void beginFrameHandler();
    void endFrameHandler();

    // single-buffered mode
    SpFuncDataBundle readPixelsSingleBuffered(std::map<std::string, TextureReadbackMinimalDesc>& texture_readback_minimal_descs);

    // double-buffered mode
    void enqueueReadPixelsDoubleBuffered(std::map<std::string, TextureReadbackMinimalDesc>& texture_readback_minimal_descs);
    SpFuncDataBundle readPixelsDoubleBuffered();

    // triple-buffered mode
    void enqueueReadPixelsTripleBuffered(std::map<std::string, TextureReadbackMinimalDesc>& texture_readback_minimal_descs);
    SpFuncDataBundle readPixelsTripleBuffered();

    // game thread helpers
    std::map<std::string, TextureReadbackMinimalDesc> requestUpdateAndGetTextureReadbackMinimalDescs();
    std::pair<FRHIGPUTextureReadback*, FRHIGPUTextureReadback*> requestSwapRHIGPUTextureReadbacks(TextureReadbackDesc& texture_readback_desc); // returns {current, prev}; advances the triple-buffered ring-buffer state
    TextureReadbackMinimalDesc getTextureReadbackMinimalDesc(
        TextureReadbackDesc& texture_readback_desc,
        FRHIGPUTextureReadback* current_readback,
        FRHIGPUTextureReadback* prev_readback,
        FTextureRenderTargetResource* render_target_resource_ptr,
        TRefCountPtr<IPooledRenderTarget>* pooled_render_target_ptr) const;
    SpPackedArray getPackedArray(const TextureReadbackDescBase& texture_readback_desc_base) const;
    SpPackedArray readPixelsImpl(const TextureReadbackDesc& texture_readback_desc) const;

    // render thread helpers
    void enqueueCopyPixelsFromGPUToStaging_RenderThread(FRHICommandListImmediate& rhi_command_list_immediate, const TextureReadbackMinimalDesc& texture_readback_minimal_desc) const;
    void copyPixelsFromStagingToCPU_RenderThread(FRHIGPUTextureReadback* readback, void* dest_ptr, const TextureReadbackDescBase& texture_readback_desc_base) const;

    // miscellaneous helpers
    void updateFrameTime();

    UPROPERTY(Transient, VisibleAnywhere, Category="SPEAR")
    bool bIsInitialized = false;

    UPROPERTY(VisibleAnywhere, Category="SPEAR")
    USpFuncComponent* SpFuncComponent = nullptr;

    // The user's state that Initialize() modifies, saved so Terminate() can restore it exactly. These are UPROPERTYs
    // so the garbage collector clears references to destroyed objects, and Transient because they're only meaningful
    // between Initialize() and Terminate().

    UPROPERTY(Transient)
    TObjectPtr<UTextureRenderTarget2D> SavedTextureTarget = nullptr;

    UPROPERTY(Transient)
    FWeightedBlendables SavedWeightedBlendables;

    UPROPERTY(Transient)
    ESceneCapturePrimitiveRenderMode SavedPrimitiveRenderMode = ESceneCapturePrimitiveRenderMode::PRM_LegacySceneCapture;

    UPROPERTY(Transient)
    TArray<TObjectPtr<AActor>> SavedShowOnlyActors;

    UPROPERTY(Transient)
    TArray<TObjectPtr<AActor>> SavedHiddenActors;

    UPROPERTY(Transient)
    bool bSavedAlwaysPersistRenderingState = false;

    bool is_initialized_ = false;

    // Scene view extension
    TSharedPtr<FSpSceneViewExtension, ESPMode::ThreadSafe> scene_view_extension_ = nullptr;

    // Texture readback state for main texture and user scene textures
    TextureReadbackDesc texture_readback_desc_;
    std::map<std::string, TextureReadbackDesc> user_scene_texture_readback_descs_;

    // Deferred state to be consumed in an FSceneViewExtension callback
    bool request_path_tracer_reset_ = false;
    bool request_override_is_offline_render_ = false;
    bool request_is_offline_render_ = false;

    // Additional state for measuring "standalone" and "standalone + extra work" frame rates.
    FDelegateHandle begin_frame_handle_;
    FDelegateHandle end_frame_handle_;
    SpFuncDataBundle scratchpad_data_bundle_;
    boost::circular_buffer<double> previous_time_deltas_;
    std::chrono::time_point<std::chrono::high_resolution_clock> previous_time_point_;
    int frame_index_ = 0;
};
