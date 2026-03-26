//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <memory> // std::unique_ptr
#include <vector>

#include <Components/SceneCaptureComponent2D.h>
#include <Containers/Array.h>
#include <Containers/EnumAsByte.h>
#include <Delegates/IDelegateInstance.h>  // FDelegateHandle
#include <Engine/EngineTypes.h>           // EEndPlayReason
#include <Engine/TextureRenderTarget2D.h> // ETextureRenderTargetFormat
#include <HAL/Platform.h>                 // int32, uint32
#include <RHIGPUReadback.h>               // FRHIGPUTextureReadback
#include <SceneView.h>                    // FSceneViewFamily
#include <SceneViewExtension.h>           // FAutoRegister, FSceneViewExtensionBase, FSceneViewExtensions
#include <Templates/SharedPointer.h>      // TSharedPtr
#include <Templates/SharedPointerFwd.h>   // ESPMode
#include <TextureResource.h>              // FTextureRenderTargetResource
#include <UObject/ObjectMacros.h>         // GENERATED_BODY, UCLASS, UFUNCTION, UPROPERTY

#include "SpCore/Boost.h"
#include "SpCore/SharedMemory.h"
#include "SpCore/SpArray.h"
#include "SpCore/SpFuncComponent.h"
#include "SpCore/Std.h"

#include "SpSceneCaptureComponent2D.generated.h"

class FRDGBuilder;
class FRHICommandListImmediate;
class UMaterial;
class UMaterialInstanceDynamic;

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
    virtual void postRenderViewFamily_RenderThread(FSceneViewFamily& view_family) {};

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
    void postRenderViewFamily_RenderThread(FSceneViewFamily& view_family) override;
};

UENUM()
enum class ESpBufferingMode : uint8
{
    SingleBuffered = 0,
    DoubleBuffered = 1,
    TripleBuffered = 2
};

// We need meta=(BlueprintSpawnableComponent) for the component to show up when using the "+Add" button in the editor.
UCLASS(ClassGroup="SPEAR", HideCategories=(Activation, AssetUserData, Collision, Cooking, LOD, Navigation, Physics, Rendering, Tags), meta=(BlueprintSpawnableComponent))
class USpSceneCaptureComponent2D : public USceneCaptureComponent2D
{
    GENERATED_BODY()
public:
    USpSceneCaptureComponent2D();
    ~USpSceneCaptureComponent2D() override;

    // UActorComponent interface
    void BeginPlay() override;
    void EndPlay(const EEndPlayReason::Type end_play_reason) override;

    UFUNCTION(BlueprintCallable, CallInEditor, Category="SPEAR")
    void Initialize();
    UFUNCTION(BlueprintCallable, CallInEditor, Category="SPEAR")
    void Terminate();
    UFUNCTION(BlueprintCallable, Category="SPEAR")
    bool IsInitialized();

    // called from FSpSceneViewExtension::postRenderViewFamily_RenderThread to do post-render work on the render thread
    void postRender_RenderThread();

    int32 getNumViewStates() { return ViewStates.Num(); }

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

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    bool bPrintReadbackSpinWaitInfo = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    bool bHidePrimitiveProxyComponentManagers = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    TArray<FString> AllowedProxyComponentModalities;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    bool bUseSceneViewExtension = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    bool bPrintFrameTime = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    bool bPrintFrameTimeEveryFrame = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    bool bReadPixelsEveryFrame = false;

private:
    // callbacks
    void beginFrameHandler();
    void endFrameHandler();

    // single-buffered mode
    SpPackedArray readPixelsSingleBuffered();

    // double-buffered mode
    void enqueueCopyPixelsDoubleBuffered();
    SpPackedArray readPixelsDoubleBuffered();

    // triple-buffered mode
    void enqueueCopyPixelsTripleBuffered();
    SpPackedArray readPixelsTripleBuffered();

    // game thread helpers
    SpPackedArray getPackedArray();
    SpPackedArray readPixelsImpl();

    // render thread helpers
    void enqueueCopyPixelsFromGPUToStagingAndImmediateFlush_RenderThread(FRHIGPUTextureReadback* readback, FRHICommandListImmediate& rhi_command_list_immediate, FTextureRenderTargetResource* render_target_resource);
    void copyPixelsFromStagingToCPU_RenderThread(FRHIGPUTextureReadback* readback, void* dest_ptr);

    // miscellaneous helpers
    void updateFrameTime();

    UPROPERTY(VisibleAnywhere, Category="SPEAR")
    bool bIsInitialized = false;

    UPROPERTY(VisibleAnywhere, Category="SPEAR")
    USpFuncComponent* SpFuncComponent = nullptr;

    bool is_initialized_ = false;
    TSharedPtr<FSpSceneViewExtension, ESPMode::ThreadSafe> scene_view_extension_ = nullptr;
    UMaterialInstanceDynamic* material_instance_dynamic_ = nullptr;
    std::unique_ptr<SharedMemoryRegion> shared_memory_region_ = nullptr;
    SpArraySharedMemoryView shared_memory_view_;

    // Scratchpad buffer for staging-to-CPU pixel data (only allocated when !bUseSharedMemory).
    std::vector<uint8_t, SpAlignedAllocator<uint8_t, 4096>> scratchpad_;

    // State for buffered readback (SingleBuffered and DoubleBuffered use index 0 only, TripleBuffered alternates uses both in an alternating fashion).
    std::array<std::unique_ptr<FRHIGPUTextureReadback>, 2> readback_buffers_;
    int readback_enqueue_index_ = 0;              // GT-only: used by TripleBuffered to alternate buffers
    bool readback_primed_ = false;                // GT-only: one-way latch, true after first enqueueCopyPixelsTripleBuffered call
    std::atomic<int> readback_pending_ = 0;       // GT increments in enqueueCopyPixelsDoubleBuffered/enqueueCopyPixelsTripleBuffered, RT decrements in postRender_RenderThread/enqueueCopyPixelsTripleBuffered

    // Additional state for measuring "standalone" and "standalone + extra work" frame rates.
    FDelegateHandle begin_frame_handle_;
    FDelegateHandle end_frame_handle_;
    SpPackedArray scratchpad_packed_array_;
    boost::circular_buffer<double> previous_time_deltas_;
    std::chrono::time_point<std::chrono::high_resolution_clock> previous_time_point_;
    int frame_index_ = 0;
};
