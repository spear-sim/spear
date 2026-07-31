//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpUnrealTypes/SpSceneCaptureComponent2D.h"

#include <stdint.h> // uint64_t

#include <chrono>
#include <cstring> // std::memcpy
#include <memory>  // std::construct_at, std::destroy_at, std::make_unique
#include <numeric> // std::accumulate
#include <ranges>  // std::views::filter, std::views::transform
#include <utility> // std::move, std::pair

#include <Components/SceneCaptureComponent2D.h>
#include <Containers/Array.h>
#include <Delegates/IDelegateInstance.h> // FDelegateHandle
#include <Materials/Material.h>
#include <Math/Color.h>                  // FLinearColor
#include <Math/UnrealMathUtility.h>      // FMath
#include <RenderGraphBuilder.h>          // FRDGTextureRef
#include <RendererInterface.h>           // FPooledRenderTargetDesc, IPooledRenderTarget
#include <RenderingThread.h>             // ENQUEUE_RENDER_COMMAND, FlushRenderingCommands
#include <RHICommandList.h>              // EImmediateFlushType, ERHIAccess, FRHICommandListImmediate, FRHITexture, FRHITransitionInfo
#include <SceneManagement.h>             // FSceneViewStateInterface
#include <SceneTextures.h>               // FSceneTextures, FTransientUserSceneTexture
#include <SceneView.h>                   // FSceneViewFamily
#include <SceneViewExtension.h>          // FAutoRegister, FSceneViewExtensionBase, FSceneViewExtensions
#include <Templates/RefCounting.h>       // TRefCountPtr
#include <Templates/SharedPointer.h>     // TSharedPtr
#include <TextureResource.h>             // FTextureRenderTargetResource
#include <UObject/UObjectGlobals.h>      // NewObject

// private headers need custom include paths in SpModuleRules.Build.cs
#include <SceneRendering.h> // FViewFamilyInfo

#include "SpCore/Assert.h"
#include "SpCore/Boost.h"
#include "SpCore/Log.h"
#include "SpCore/SharedMemory.h"
#include "SpCore/SpArray.h"
#include "SpCore/SpFuncComponent.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"
#include "SpCore/UnrealUtils.h"

#include "SpUnrealTypes/SpMeshProxyComponentManager.h"

FSpSceneViewExtensionBase::FSpSceneViewExtensionBase(const FAutoRegister& auto_register) : FSceneViewExtensionBase(auto_register)
{
    SP_LOG_CURRENT_FUNCTION();
}

FSpSceneViewExtensionBase::~FSpSceneViewExtensionBase()
{
    SP_LOG_CURRENT_FUNCTION();
}

bool FSpSceneViewExtensionBase::IsActiveThisFrame_Internal(const FSceneViewExtensionContext& context) const
{
    // It is possible for an FSceneViewExtensionBase instance to outlive its owning component if IsActiveThisFrame_Internal(...)
    // returns true, so we return false after the owning component has called terminate().
    return component_ != nullptr;
}

void FSpSceneViewExtensionBase::SetupViewFamily(FSceneViewFamily& in_view_family)
{
    if (shouldHandleViewFamily(&in_view_family)) {
        setupViewFamily(in_view_family);
    }
}

void FSpSceneViewExtensionBase::SetupView(FSceneViewFamily& in_view_family, FSceneView& in_view)
{
    if (shouldHandleView(&in_view_family, &in_view)) {
        setupView(in_view_family, in_view);
    }
}

void FSpSceneViewExtensionBase::BeginRenderViewFamily(FSceneViewFamily& in_view_family)
{
    if (shouldHandleViewFamily(&in_view_family)) {
        beginRenderViewFamily(in_view_family);
    }
}

void FSpSceneViewExtensionBase::PostRenderViewFamily_RenderThread(FRDGBuilder& graph_builder, FSceneViewFamily& in_view_family)
{
    if (shouldHandleViewFamily(&in_view_family)) {
        postRenderViewFamily_RenderThread(graph_builder, in_view_family);
    }
}

bool FSpSceneViewExtensionBase::shouldHandleViewFamily(const FSceneViewFamily* view_family) const
{
    if (!component_) {
        return false;
    }

    SP_ASSERT(view_family);
    for (int i = 0; i < view_family->Views.Num(); i++) {
        const FSceneView* view = view_family->Views[i];
        if (shouldHandleView(view_family, view)) {
            return true;
        }
    }
    return false;
}

bool FSpSceneViewExtensionBase::shouldHandleView(const FSceneViewFamily* view_family, const FSceneView* view) const
{
    SP_ASSERT(view_family);
    SP_ASSERT(view);

    if (!component_) {
        return false;
    }

    FSceneViewStateInterface* view_state = view->State;
    if (!view_state) {
        return false;
    }

    int32 view_state_key = view->State->GetViewKey();

    for (int32 i = 0; i < getComponent()->getViewStates().Num(); i++) {
        FSceneViewStateInterface* component_view_state = getComponent()->GetViewState(i);
        if (component_view_state) {
            int32 component_view_state_key = component_view_state->GetViewKey();
            if (component_view_state_key > 0 && view_state_key > 0) { // compare keys if they're both valid
                if (component_view_state_key == view_state_key) {
                    return true;
                }
            } else { // otherwise compare pointers as a fallback to handle the case where one or both keys haven't been initialized yet
                if (component_view_state == view_state) {
                    return true;
                }
            }
        }
    }

    return false;
}

FSpSceneViewExtension::FSpSceneViewExtension(const FAutoRegister& auto_register) : FSpSceneViewExtensionBase(auto_register)
{
    SP_LOG_CURRENT_FUNCTION();
}

FSpSceneViewExtension::~FSpSceneViewExtension()
{
    SP_LOG_CURRENT_FUNCTION();
}

void FSpSceneViewExtension::setupView(FSceneViewFamily& view_family, FSceneView& view)
{
    getComponent()->setupView(view_family, view);
}

void FSpSceneViewExtension::postRenderViewFamily_RenderThread(FRDGBuilder& graph_builder, FSceneViewFamily& view_family)
{
    getComponent()->postRenderViewFamily_RenderThread(graph_builder, view_family);
}

USpSceneCaptureComponent2D::USpSceneCaptureComponent2D()
{
    SpFuncComponent = UnrealUtils::createSceneComponentInsideOwnerConstructor<USpFuncComponent>(this, "sp_func_component");
    SP_ASSERT(SpFuncComponent);

    // we want to be able to capture the scene even when the game is paused; note that we don't set
    // PrimaryActorTick.TickGroup because we don't want to interfere with when the base class component
    // gets ticked
    PrimaryComponentTick.bCanEverTick = true;
    PrimaryComponentTick.bTickEvenWhenPaused = true;

    SetVisibility(false); // disable rendering to texture
}

void USpSceneCaptureComponent2D::BeginPlay()
{
    USceneCaptureComponent2D::BeginPlay();
    bIsInitialized = false;
}

void USpSceneCaptureComponent2D::EndPlay(const EEndPlayReason::Type end_play_reason)
{
    Terminate();
    USceneCaptureComponent2D::EndPlay(end_play_reason);
}

void USpSceneCaptureComponent2D::Initialize()
{
    if (IsInitialized()) {
        return;
    }

    TextureTarget = NewObject<UTextureRenderTarget2D>(this);
    SP_ASSERT(TextureTarget);
    TextureTarget->ClearColor = FLinearColor(1.0f, 1.0f, 0.0f, 1.0f); // bright yellow

    if (bOverrideTextureRenderTargetFormat) {
        TextureTarget->RenderTargetFormat = TextureRenderTargetFormat;
    }

    TextureTarget->InitAutoFormat(Width, Height);

    if (bOverrideTextureRenderTargetSRGB) {
        TextureTarget->SRGB = bTextureRenderTargetSRGB;
    }

    if (bOverrideTextureRenderTargetForceLinearGamma) {
        TextureTarget->bForceLinearGamma = bTextureRenderTargetForceLinearGamma;
    }

    if (bOverrideTextureRenderTargetGamma) {
        TextureTarget->TargetGamma = TextureRenderTargetGamma;
    }

    bool clear_render_target = true;
    TextureTarget->UpdateResourceImmediate(clear_render_target);

    if (Material) {
        PostProcessSettings.AddBlendable(Material, 1.0f);
    }

    if (MeshProxyComponentManagerClass) {
        PrimitiveRenderMode = ESceneCapturePrimitiveRenderMode::PRM_UseShowOnlyList;
        for (auto manager : UnrealUtils::findActorsByClass<ASpMeshProxyComponentManager>(MeshProxyComponentManagerClass, GetWorld())) {
            ShowOnlyActors.Add(manager);
        }
    } else {
        for (auto manager : UnrealUtils::findActorsByType<ASpMeshProxyComponentManager>(GetWorld())) {
            HiddenActors.Add(manager);
        }
    }

    if (BufferingMode == ESpBufferingMode::DoubleBuffered || UserSceneTextureNames.Num() > 0 || showFlagsNeedSceneViewExtension()) {
        bAlwaysPersistRenderingState = true; // ensure that the underlying view-state data is stable across frames so FSpSceneViewExtensionBase can match view-state data to this component
        scene_view_extension_ = FSceneViewExtensions::NewExtension<FSpSceneViewExtension>();
        scene_view_extension_->initialize(this);
    }

    // allocate memory

    {
        SpArrayDataType channel_data_type = Unreal::getEnumValueAs<SpArrayDataType, ESpArrayDataType>(ChannelDataType);
        uint64_t num_bytes = Height*Width*NumChannelsPerPixel*SpArrayDataTypeUtils::getSizeOf(channel_data_type);

        texture_readback_desc_.width_ = Width;
        texture_readback_desc_.height_ = Height;
        texture_readback_desc_.num_channels_per_pixel_ = NumChannelsPerPixel;
        texture_readback_desc_.channel_data_type_ = channel_data_type;

        if (bUseSharedMemory) {
            SP_ASSERT(!texture_readback_desc_.shared_memory_region_);
            texture_readback_desc_.shared_memory_region_ = std::make_unique<SharedMemoryRegion>(num_bytes);
            SP_ASSERT(texture_readback_desc_.shared_memory_region_);
            std::string shared_memory_name = "smem:sp_scene_capture_component_2d";
            texture_readback_desc_.shared_memory_view_ = SpArraySharedMemoryView(texture_readback_desc_.shared_memory_region_->getView(), shared_memory_name, SpArraySharedMemoryUsageFlags::ReturnValue);
            SpFuncComponent->registerSharedMemoryView(texture_readback_desc_.shared_memory_view_); // name needs to be unique per USpFuncComponent
        } else if (BufferingMode == ESpBufferingMode::DoubleBuffered || BufferingMode == ESpBufferingMode::TripleBuffered) {
            texture_readback_desc_.scratchpad_.resize(num_bytes);
        }
    }

    // allocate readback buffers

    if (BufferingMode == ESpBufferingMode::SingleBuffered || BufferingMode == ESpBufferingMode::DoubleBuffered) {
        texture_readback_desc_.rhi_gpu_texture_readbacks_ = {
            std::make_unique<FRHIGPUTextureReadback>(Unreal::toFName("rhi_gpu_texture_readback_A")), nullptr};
    } else if (BufferingMode == ESpBufferingMode::TripleBuffered) {
        texture_readback_desc_.rhi_gpu_texture_readbacks_ = {
            std::make_unique<FRHIGPUTextureReadback>(Unreal::toFName("rhi_gpu_texture_readback_A")),
            std::make_unique<FRHIGPUTextureReadback>(Unreal::toFName("rhi_gpu_texture_readback_B")) };
    }

    //
    // user scene textures
    //

    for (auto& name : UserSceneTextureNames) {

        std::string name_string = Unreal::toStdString(name);

        // construct the desc in place because TextureReadbackDesc is non-movable (it holds a std::atomic)
        auto [itr, inserted] = user_scene_texture_readback_descs_.try_emplace(name_string);
        SP_ASSERT(inserted);
        TextureReadbackDesc& texture_readback_desc = itr->second;

        SP_ASSERT(UserSceneTextureMaterialDescs.Contains(name));
        const FSpUserSceneTextureMaterialDesc& user_scene_texture_material_desc = UserSceneTextureMaterialDescs[name];

        // validate user_scene_texture_material_desc
        SP_ASSERT(user_scene_texture_material_desc.Material);
        SP_ASSERT(user_scene_texture_material_desc.ResolutionDivisorWidth >= 1 && user_scene_texture_material_desc.ResolutionDivisorHeight >= 1);

        PostProcessSettings.AddBlendable(user_scene_texture_material_desc.Material, 1.0f);

        // allocate memory

        int height = FMath::DivideAndRoundUp(Height, user_scene_texture_material_desc.ResolutionDivisorHeight);
        int width = FMath::DivideAndRoundUp(Width, user_scene_texture_material_desc.ResolutionDivisorWidth);
        int num_channels_per_pixel = 4;
        SpArrayDataType channel_data_type = SpArrayDataType::Float16;
        uint64_t num_bytes = height*width*num_channels_per_pixel*SpArrayDataTypeUtils::getSizeOf(channel_data_type);

        texture_readback_desc.width_ = width;
        texture_readback_desc.height_ = height;
        texture_readback_desc.num_channels_per_pixel_ = num_channels_per_pixel;
        texture_readback_desc.channel_data_type_ = channel_data_type;

        if (bUseSharedMemory) {
            texture_readback_desc.shared_memory_region_ = std::make_unique<SharedMemoryRegion>(num_bytes);
            SP_ASSERT(texture_readback_desc.shared_memory_region_);
            std::string shared_memory_name = "smem:sp_scene_capture_component_2d:" + name_string;
            texture_readback_desc.shared_memory_view_ = SpArraySharedMemoryView(texture_readback_desc.shared_memory_region_->getView(), shared_memory_name, SpArraySharedMemoryUsageFlags::ReturnValue);
            SpFuncComponent->registerSharedMemoryView(texture_readback_desc.shared_memory_view_); // name needs to be unique per USpFuncComponent
        } else if (BufferingMode == ESpBufferingMode::DoubleBuffered || BufferingMode == ESpBufferingMode::TripleBuffered) {
            texture_readback_desc.scratchpad_.resize(num_bytes);
        }

        // allocate readback buffers

        if (BufferingMode == ESpBufferingMode::SingleBuffered || BufferingMode == ESpBufferingMode::DoubleBuffered) {
            texture_readback_desc.rhi_gpu_texture_readbacks_ = {
                std::make_unique<FRHIGPUTextureReadback>(Unreal::toFName("rhi_gpu_texture_readback_A_" + name_string)), nullptr};
        } else if (BufferingMode == ESpBufferingMode::TripleBuffered) {
            texture_readback_desc.rhi_gpu_texture_readbacks_ = {
                std::make_unique<FRHIGPUTextureReadback>(Unreal::toFName("rhi_gpu_texture_readback_A_" + name_string)),
                std::make_unique<FRHIGPUTextureReadback>(Unreal::toFName("rhi_gpu_texture_readback_B_" + name_string)) };
        }
    }

    // initialize state for measuring "standalone" and "standalone + extra work" frame rates

    if (bPrintFrameTimeEveryFrame || bReadPixelsEveryFrame) {
        begin_frame_handle_ = FCoreDelegates::OnBeginFrame.AddUObject(this, &USpSceneCaptureComponent2D::beginFrameHandler);
    }

    if (bReadPixelsEveryFrame) {
        end_frame_handle_ = FCoreDelegates::OnEndFrame.AddUObject(this, &USpSceneCaptureComponent2D::endFrameHandler);
    }

    if (bPrintFrameTimeEveryFrame) {
        int num_samples = 100;
        previous_time_deltas_.set_capacity(num_samples);
    }

    // register SpFuncs

    SpFuncComponent->registerFunc("enqueue_copy", [this](SpFuncDataBundle& args) -> SpFuncDataBundle {
        SP_ASSERT(!bReadPixelsEveryFrame);

        if (BufferingMode == ESpBufferingMode::DoubleBuffered) {
            std::map<std::string, TextureReadbackMinimalDesc> texture_readback_minimal_descs = requestUpdateAndGetTextureReadbackMinimalDescs();
            enqueueCopyPixelsDoubleBuffered(texture_readback_minimal_descs);
        } else if (BufferingMode == ESpBufferingMode::TripleBuffered) {
            std::map<std::string, TextureReadbackMinimalDesc> texture_readback_minimal_descs = requestUpdateAndGetTextureReadbackMinimalDescs(); // in triple-buffered case, must be called exactly once per enqueue
            enqueueCopyPixelsTripleBuffered(texture_readback_minimal_descs);
        } else {
            SP_ASSERT(false);
        }

        return SpFuncDataBundle();
    });

    SpFuncComponent->registerFunc("read_pixels", [this](SpFuncDataBundle& args) -> SpFuncDataBundle {
        if (BufferingMode == ESpBufferingMode::SingleBuffered) {
            std::map<std::string, TextureReadbackMinimalDesc> texture_readback_minimal_descs = requestUpdateAndGetTextureReadbackMinimalDescs();
            return readPixelsSingleBuffered(texture_readback_minimal_descs);
        } else if (BufferingMode == ESpBufferingMode::DoubleBuffered) {
            return readPixelsDoubleBuffered();
        } else if (BufferingMode == ESpBufferingMode::TripleBuffered) {
            return readPixelsTripleBuffered();
        } else {
            SP_ASSERT(false);
            return SpFuncDataBundle();
        }
    });

    request_path_tracer_reset_ = false;
    is_initialized_ = true;
    bIsInitialized = true;

    SetVisibility(true); // enable rendering to texture
}

void USpSceneCaptureComponent2D::Terminate()
{
    if (!IsInitialized()) {
        return;
    }

    SetVisibility(false); // disable rendering to texture

    request_path_tracer_reset_ = false;
    is_initialized_ = false;
    bIsInitialized = false;

    // unregister SpFuncs
    SpFuncComponent->unregisterFunc("enqueue_copy");
    SpFuncComponent->unregisterFunc("read_pixels");

    // terminate state for measuring "standalone" and "standalone + extra work" frame rates
    if (bPrintFrameTimeEveryFrame || bReadPixelsEveryFrame) {
        FCoreDelegates::OnBeginFrame.Remove(begin_frame_handle_);
        begin_frame_handle_.Reset();
    }
    if (bReadPixelsEveryFrame) {
        FCoreDelegates::OnEndFrame.Remove(end_frame_handle_);
        end_frame_handle_.Reset();
    }

    //
    // user scene textures
    //

    if (UserSceneTextureNames.Num() > 0) {
        FlushRenderingCommands();

        for (auto& name : UserSceneTextureNames) {
            SP_ASSERT(UserSceneTextureMaterialDescs.Contains(name));

            if (bUseSharedMemory) {
                std::string name_string = Unreal::toStdString(name);
                SP_ASSERT(user_scene_texture_readback_descs_.contains(name_string));

                TextureReadbackDesc& texture_readback_desc = user_scene_texture_readback_descs_.at(name_string);
                SP_ASSERT(texture_readback_desc.shared_memory_region_);

                SpFuncComponent->unregisterSharedMemoryView(texture_readback_desc.shared_memory_view_);
            }

            const FSpUserSceneTextureMaterialDesc& user_scene_texture_material_desc = UserSceneTextureMaterialDescs[name];
            PostProcessSettings.RemoveBlendable(user_scene_texture_material_desc.Material);
        }

        user_scene_texture_readback_descs_.clear();
    }

    // de-allocate memory
    if (bUseSharedMemory) {
        SP_ASSERT(texture_readback_desc_.shared_memory_region_);
        SpFuncComponent->unregisterSharedMemoryView(texture_readback_desc_.shared_memory_view_);
    }

    // reset in-place because texture_readback_desc_ is non-movable and non-copyable
    std::destroy_at(&texture_readback_desc_);
    std::construct_at(&texture_readback_desc_);

    if (BufferingMode == ESpBufferingMode::DoubleBuffered || UserSceneTextureNames.Num() > 0 || showFlagsNeedSceneViewExtension()) {
        SP_ASSERT(scene_view_extension_);
        scene_view_extension_->terminate();
        scene_view_extension_ = nullptr;
        bAlwaysPersistRenderingState = false;
    }

    ShowOnlyActors.Empty();
    HiddenActors.Empty();

    if (Material) {
        PostProcessSettings.RemoveBlendable(Material);
    }

    TextureTarget = nullptr;
}

bool USpSceneCaptureComponent2D::IsInitialized()
{
    return is_initialized_;
}

TArray<uint64> USpSceneCaptureComponent2D::GetViewStates() // can't be const because GetViewState(...) is non-const
{
    TArray<uint64> view_states;
    for (int32 i = 0; i < ViewStates.Num(); i++) {
        FSceneViewStateInterface* view_state = GetViewState(i);
        view_states.Add(reinterpret_cast<uint64>(view_state));
    }
    return view_states;
}

void USpSceneCaptureComponent2D::RequestPathTracerReset()
{
    SP_ASSERT(IsInitialized());
    FlushRenderingCommands(); // force rendering thread to be fully up-to-date before resetting the path tracer
    request_path_tracer_reset_ = true;
}

void USpSceneCaptureComponent2D::setupView(FSceneViewFamily& view_family, FSceneView& view)
{
    const TArray<FEngineShowFlagsSetting>& engine_show_flag_settings = GetShowFlagSettings();

    for (auto& engine_show_flag_setting : engine_show_flag_settings) {
        if (Unreal::toStdString(engine_show_flag_setting.ShowFlagName) == "LightingOnlyOverride" && engine_show_flag_setting.Enabled) {
            view.DiffuseOverrideParameter = FVector4f(GEngine->LightingOnlyBrightness.R, GEngine->LightingOnlyBrightness.G, GEngine->LightingOnlyBrightness.B, 0.0f);
            view.SpecularOverrideParameter = FVector4f(0.0f, 0.0f, 0.0f, 0.0f);
        }

        if (Unreal::toStdString(engine_show_flag_setting.ShowFlagName) == "Specular" && !engine_show_flag_setting.Enabled) {
            view.SpecularOverrideParameter = FVector4f(0.0f, 0.0f, 0.0f, 0.0f);
        }
    }

    if (request_path_tracer_reset_) {
        view.bForcePathTracerReset = true;
        request_path_tracer_reset_ = false;
    }
}

void USpSceneCaptureComponent2D::postRenderViewFamily_RenderThread(FRDGBuilder& graph_builder, FSceneViewFamily& view_family)
{
    // Copy each active UserSceneTexture's transient render-graph texture into our persistent pooled render target
    // so it can be read back to the CPU later. The view family is a FViewFamilyInfo (a renderer-private type), whose
    // GetSceneTextures() gives us the UserSceneTextures map. That map is keyed by the material's internal output
    // name (FSpUserSceneTextureMaterialDesc::InternalName), whereas our readback desc map is keyed by public name.

    if (UserSceneTextureNames.Num() > 0) {
        SP_ASSERT(view_family.bIsViewFamilyInfo); // guard the static_cast below
        const FSceneTextures& scene_textures = static_cast<FViewFamilyInfo&>(view_family).GetSceneTextures();

        for (auto& name : UserSceneTextureNames) {
            SP_ASSERT(UserSceneTextureMaterialDescs.Contains(name));
            const FSpUserSceneTextureMaterialDesc& user_scene_texture_material_desc = UserSceneTextureMaterialDescs[name];

            std::string name_string = Unreal::toStdString(name);
            SP_ASSERT(user_scene_texture_readback_descs_.contains(name_string));
            TextureReadbackDesc& texture_readback_desc = user_scene_texture_readback_descs_.at(name_string);

            std::string internal_name_string = Unreal::toStdString(user_scene_texture_material_desc.InternalName);
            FName internal_name_fname = Unreal::toFName(internal_name_string);
            SP_ASSERT(scene_textures.UserSceneTextures.Contains(internal_name_fname));
            const TArray<FTransientUserSceneTexture>& transient_user_scene_textures = scene_textures.UserSceneTextures[internal_name_fname];
            const FTransientUserSceneTexture& transient_user_scene_texture = transient_user_scene_textures[0]; // the engine swaps the most recently written resolution to index 0 in FindOrAddUserSceneTexture()

            // Verify the configured resolution divisor matches the actual transient resource, and that the transient
            // extent is at least the divisor-derived dims we sized our buffers for. The transient can be larger: the
            // engine quantizes the scene-texture extent up to a multiple of 4 (8 under Substrate, possibly more via a
            // suggested divisor) in QuantizeSceneBufferSize(), so the transient is DivideAndRoundUp(quantized extent,
            // ResolutionDivisor). That padding is on the right/bottom; the view still renders into the top-left rect,
            // so copyPixelsFromStagingToCPU_RenderThread() reads back exactly our top-left (width, height) sub-region
            // using the staging row pitch. Checked here every frame, so all buffering modes rely on it.

            SP_ASSERT(transient_user_scene_texture.ResolutionDivisor.X == user_scene_texture_material_desc.ResolutionDivisorWidth);
            SP_ASSERT(transient_user_scene_texture.ResolutionDivisor.Y == user_scene_texture_material_desc.ResolutionDivisorHeight);
            SP_ASSERT(transient_user_scene_texture.Texture->Desc.Extent.X >= FMath::DivideAndRoundUp(Width, user_scene_texture_material_desc.ResolutionDivisorWidth));
            SP_ASSERT(transient_user_scene_texture.Texture->Desc.Extent.Y >= FMath::DivideAndRoundUp(Height, user_scene_texture_material_desc.ResolutionDivisorHeight));
            SP_ASSERT(transient_user_scene_texture.Texture->Desc.Format == PF_FloatRGBA);

            // Extract the transient texture into the desc's persistent pooled render target, leaving it in SRVMask
            // so copyPixelsFromStagingToCPU_RenderThread()'s SRVMask->CopySrc transition is valid on readback. The
            // key already exists (populated in Initialize()) and std::map node addresses are stable, so the address
            // we hand to QueueTextureExtraction() stays valid until graph execution.

            TRefCountPtr<IPooledRenderTarget>* pooled_render_target_ptr = &(texture_readback_desc.pooled_render_target_);
            graph_builder.QueueTextureExtraction(transient_user_scene_texture.Texture, pooled_render_target_ptr, ERHIAccess::SRVMask);
        }
    }

    // Double-buffered staging-to-CPU copy step happens here. Copy each pending staging readback to the CPU and 
    // decrement its pending counter. Triple-buffered staging-to-CPU completes inside its own enqueue command
    // instead.

    if (BufferingMode == ESpBufferingMode::DoubleBuffered) {
        if (bReadPixelData) {

            // Gate the staging-to-CPU consume on num_staging_copies_pending_ (bumped on the render thread once the matching
            // EnqueueCopy() executed), not num_readbacks_pending_ (bumped on the game thread at enqueue time), so we never
            // Lock() a staging texture that hasn't been populated yet. num_staging_copies_pending_ <= num_readbacks_pending_
            // always, so when we consume we decrement both: num_staging_copies_pending_ closes the gate, and
            // num_readbacks_pending_ releases readPixelsImpl()'s game-thread wait.

            SP_ASSERT(texture_readback_desc_.num_staging_copies_pending_ >= 0);
            if (texture_readback_desc_.num_staging_copies_pending_ > 0) {
                void* dest_ptr = nullptr;
                if (bUseSharedMemory) {
                    dest_ptr = texture_readback_desc_.shared_memory_view_.data_;
                } else {
                    dest_ptr = texture_readback_desc_.scratchpad_.data();
                }

                FRHIGPUTextureReadback* readback = texture_readback_desc_.rhi_gpu_texture_readbacks_.at(0).get(); // always use index 0 for DoubleBuffered
                SP_ASSERT(readback);

                copyPixelsFromStagingToCPU_RenderThread(readback, dest_ptr, texture_readback_desc_);
                texture_readback_desc_.num_staging_copies_pending_--;
                texture_readback_desc_.num_readbacks_pending_--;
                SP_ASSERT(texture_readback_desc_.num_staging_copies_pending_ >= 0);
                SP_ASSERT(texture_readback_desc_.num_readbacks_pending_ >= 0);
            }

            for (auto& name : UserSceneTextureNames) {
                std::string name_string = Unreal::toStdString(name);
                SP_ASSERT(user_scene_texture_readback_descs_.contains(name_string));
                TextureReadbackDesc& texture_readback_desc = user_scene_texture_readback_descs_.at(name_string);

                SP_ASSERT(texture_readback_desc.num_staging_copies_pending_ >= 0);
                if (texture_readback_desc.num_staging_copies_pending_ > 0) {
                    void* dest_ptr = nullptr;
                    if (bUseSharedMemory) {
                        dest_ptr = texture_readback_desc.shared_memory_view_.data_;
                    } else {
                        dest_ptr = texture_readback_desc.scratchpad_.data();
                    }

                    FRHIGPUTextureReadback* readback = texture_readback_desc.rhi_gpu_texture_readbacks_.at(0).get(); // always use index 0 for DoubleBuffered
                    SP_ASSERT(readback);

                    copyPixelsFromStagingToCPU_RenderThread(readback, dest_ptr, texture_readback_desc);
                    texture_readback_desc.num_staging_copies_pending_--;
                    texture_readback_desc.num_readbacks_pending_--;
                    SP_ASSERT(texture_readback_desc.num_staging_copies_pending_ >= 0);
                    SP_ASSERT(texture_readback_desc.num_readbacks_pending_ >= 0);
                }
            }
        }
    }
}

bool USpSceneCaptureComponent2D::showFlagsNeedSceneViewExtension()
{
    std::vector<std::string> setting_names_that_need_sve_when_enabled = {"LightingOnlyOverride", "PathTracing"};
    std::vector<std::string> setting_names_that_need_sve_when_disabled = {"Specular"};

    std::vector<FEngineShowFlagsSetting> show_flag_settings = Unreal::toStdVector(GetShowFlagSettings());

    return
        Std::any(
            show_flag_settings |
            std::views::filter([](const auto& setting) { return setting.Enabled; }) |
            std::views::transform([](const auto& setting) { return Unreal::toStdString(setting.ShowFlagName); }) |
            std::views::transform([&setting_names_that_need_sve_when_enabled](const auto& name) { return Std::contains(setting_names_that_need_sve_when_enabled, name); })) ||
        Std::any(
            show_flag_settings |
            std::views::filter([](const auto& setting) { return !setting.Enabled; }) |
            std::views::transform([](const auto& setting) { return Unreal::toStdString(setting.ShowFlagName); }) |
            std::views::transform([&setting_names_that_need_sve_when_disabled](const auto& name) { return Std::contains(setting_names_that_need_sve_when_disabled, name); }));
}

//
// callbacks
//

void USpSceneCaptureComponent2D::beginFrameHandler()
{
    if (bPrintFrameTimeEveryFrame) {
        updateFrameTime();
    }

    if (bReadPixelsEveryFrame) {

        // single-buffered does all its work in endFrameHandler(), so there is nothing to enqueue here
        if (BufferingMode == ESpBufferingMode::SingleBuffered) {
            return;
        }

        if (BufferingMode == ESpBufferingMode::DoubleBuffered) {
            std::map<std::string, TextureReadbackMinimalDesc> texture_readback_minimal_descs = requestUpdateAndGetTextureReadbackMinimalDescs();
            enqueueCopyPixelsDoubleBuffered(texture_readback_minimal_descs);
        } else if (BufferingMode == ESpBufferingMode::TripleBuffered) {
            std::map<std::string, TextureReadbackMinimalDesc> texture_readback_minimal_descs = requestUpdateAndGetTextureReadbackMinimalDescs(); // in triple-buffered case, must be called exactly once per enqueue
            enqueueCopyPixelsTripleBuffered(texture_readback_minimal_descs);
        } else {
            SP_ASSERT(false);
        }
    }
}

void USpSceneCaptureComponent2D::endFrameHandler()
{
    if (bReadPixelsEveryFrame) {
        if (BufferingMode == ESpBufferingMode::SingleBuffered) {
            std::map<std::string, TextureReadbackMinimalDesc> texture_readback_minimal_descs = requestUpdateAndGetTextureReadbackMinimalDescs();
            scratchpad_data_bundle_ = readPixelsSingleBuffered(texture_readback_minimal_descs);
        } else if (BufferingMode == ESpBufferingMode::DoubleBuffered) {
            scratchpad_data_bundle_ = readPixelsDoubleBuffered();
        } else if (BufferingMode == ESpBufferingMode::TripleBuffered) {
            scratchpad_data_bundle_ = readPixelsTripleBuffered();
        } else {
            SP_ASSERT(false);
        }
    }
}

//
// single-buffered mode
//

SpFuncDataBundle USpSceneCaptureComponent2D::readPixelsSingleBuffered(std::map<std::string, TextureReadbackMinimalDesc>& texture_readback_minimal_descs)
{
    SP_ASSERT(BufferingMode == ESpBufferingMode::SingleBuffered);
    SP_ASSERT(IsInitialized());

    SpFuncDataBundle return_values;

    // Build every packed array and resolve its CPU destination pointer into the corresponding minimal desc, then move
    // the whole map into the batched command below.

    for (auto& [name, texture_readback_minimal_desc] : texture_readback_minimal_descs) {
        SpPackedArray packed_array = getPackedArray(texture_readback_minimal_desc);
        if (bReadPixelData) {
            texture_readback_minimal_desc.dest_ptr_ = packed_array.view_;
            SP_ASSERT(texture_readback_minimal_desc.dest_ptr_);
        }
        Std::insert(return_values.packed_arrays_, name, std::move(packed_array));
    }

    if (bReadPixelData) {

        // Enqueue every GPU-to-staging copy first, then flush once, then copy every staging buffer to the CPU. The
        // whole batch lives in a single ENQUEUE_RENDER_COMMAND because copyPixelsFromStagingToCPU_RenderThread()
        // blocks on each readback's Lock(), so we can't interleave a Lock() before the one flush that dispatches all
        // the copies. Everything is synchronous (we FlushRenderingCommands() below), so the borrowed readback and
        // src_texture pointers stay valid for the duration of the command, as do the dest_ptrs (which point into the
        // packed arrays we already moved into return_values, whose backing buffers are stable across the move).

        ENQUEUE_RENDER_COMMAND(SpReadPixelsSingleBuffered)(
            [this, texture_readback_minimal_descs = std::move(texture_readback_minimal_descs)](FRHICommandListImmediate& command_list) {
                for (auto& [name, texture_readback_minimal_desc] : texture_readback_minimal_descs) {
                    enqueueCopyPixelsFromGPUToStaging_RenderThread(command_list, texture_readback_minimal_desc);
                }
                command_list.ImmediateFlush(EImmediateFlushType::DispatchToRHIThread);
                for (auto& [name, texture_readback_minimal_desc] : texture_readback_minimal_descs) {
                    copyPixelsFromStagingToCPU_RenderThread(texture_readback_minimal_desc.prev_readback_, texture_readback_minimal_desc.dest_ptr_, texture_readback_minimal_desc);
                }
            });

        FlushRenderingCommands();
    }

    return return_values;
}

//
// double-buffered mode
//

void USpSceneCaptureComponent2D::enqueueCopyPixelsDoubleBuffered(std::map<std::string, TextureReadbackMinimalDesc>& texture_readback_minimal_descs)
{
    SP_ASSERT(BufferingMode == ESpBufferingMode::DoubleBuffered);
    SP_ASSERT(IsInitialized());

    if (bReadPixelData) {

        // Increment the pending counter for every texture (decremented later on the render thread in
        // postRenderViewFamily_RenderThread(), waited on in readPixelsImpl()). num_readbacks_pending_ points at the
        // persistent desc's counter, since the minimal descs are transient.

        for (auto& [name, texture_readback_minimal_desc] : texture_readback_minimal_descs) {
            SP_ASSERT(texture_readback_minimal_desc.num_readbacks_pending_ptr_);
            SP_ASSERT(*texture_readback_minimal_desc.num_readbacks_pending_ptr_ >= 0);
            (*texture_readback_minimal_desc.num_readbacks_pending_ptr_)++;
        }

        // Enqueue every GPU-to-staging copy in a single command, then do a single flush. This command is async, so each
        // source texture must stay valid until it runs; each minimal desc holds its pooled render target by value,
        // which keeps a user scene texture's source alive (null for the main texture, whose source TextureTarget owns
        // and keeps alive for its lifetime). The staging-to-CPU copy and the matching pending-counter decrement happen
        // later in postRenderViewFamily_RenderThread().

        ENQUEUE_RENDER_COMMAND(SpEnqueueCopyPixelsDoubleBuffered)(
            [this, texture_readback_minimal_descs = std::move(texture_readback_minimal_descs)](FRHICommandListImmediate& command_list) {
                for (auto& [name, texture_readback_minimal_desc] : texture_readback_minimal_descs) {
                    enqueueCopyPixelsFromGPUToStaging_RenderThread(command_list, texture_readback_minimal_desc);

                    // The staging texture is now populated, so the staging-to-CPU consume in postRenderViewFamily_RenderThread()
                    // is safe. Bump the render-thread consume gate here (not on the game thread) so a postRenderViewFamily
                    // callback that fires before this command executes cannot Lock() an empty slot (see num_staging_copies_pending_).

                    SP_ASSERT(texture_readback_minimal_desc.num_staging_copies_pending_ptr_);
                    SP_ASSERT(*texture_readback_minimal_desc.num_staging_copies_pending_ptr_ >= 0);
                    (*texture_readback_minimal_desc.num_staging_copies_pending_ptr_)++;
                }
                command_list.ImmediateFlush(EImmediateFlushType::DispatchToRHIThread);
            });
    }
}

SpFuncDataBundle USpSceneCaptureComponent2D::readPixelsDoubleBuffered()
{
    SP_ASSERT(BufferingMode == ESpBufferingMode::DoubleBuffered);
    SP_ASSERT(IsInitialized());

    // Wait for and read back the CPU-side data for the main texture and every user scene texture. The staging-to-CPU
    // copy already happened on the render thread (in postRenderViewFamily_RenderThread()), so we just read the
    // persistent descs' buffers here; readPixelsImpl() spin-waits on each pending counter first.

    SpFuncDataBundle return_values;
    Std::insert(return_values.packed_arrays_, "data", readPixelsImpl(texture_readback_desc_));
    for (auto& [name, texture_readback_desc] : user_scene_texture_readback_descs_) {
        Std::insert(return_values.packed_arrays_, name, readPixelsImpl(texture_readback_desc));
    }
    return return_values;
}

//
// triple-buffered mode
//

void USpSceneCaptureComponent2D::enqueueCopyPixelsTripleBuffered(std::map<std::string, TextureReadbackMinimalDesc>& texture_readback_minimal_descs)
{
    SP_ASSERT(BufferingMode == ESpBufferingMode::TripleBuffered);
    SP_ASSERT(IsInitialized());

    if (bReadPixelData) {

        // A non-null prev_readback_ means this texture wrote a readback on a previous frame that is now ready to copy
        // back this frame (i.e. it has been primed; requestUpdateAndGetTextureReadbackMinimalDescs() resolves this).
        // Increment the pending counter for exactly those textures; the matching decrement happens in the command below.
        // num_readbacks_pending_ points at the persistent desc's counter, since the minimal descs are transient.

        for (auto& [name, texture_readback_minimal_desc] : texture_readback_minimal_descs) {
            if (texture_readback_minimal_desc.prev_readback_) {
                SP_ASSERT(texture_readback_minimal_desc.num_readbacks_pending_ptr_);
                SP_ASSERT(*texture_readback_minimal_desc.num_readbacks_pending_ptr_ >= 0);
                (*texture_readback_minimal_desc.num_readbacks_pending_ptr_)++;
            }
        }

        // Enqueue every GPU-to-staging copy (into current_readback_) in a single command, do a single flush, then copy
        // every previous frame's staging buffer (prev_readback_) to the CPU and decrement its pending counter. The
        // whole batch lives in one command because copyPixelsFromStagingToCPU_RenderThread() blocks on Lock(), so we
        // can't interleave a Lock() before the one flush that dispatches all the copies. This command is async, so
        // each minimal desc holds its pooled render target by value to keep a user scene texture's source alive (null
        // for the main texture, whose source TextureTarget owns and keeps alive for its lifetime).

        ENQUEUE_RENDER_COMMAND(SpEnqueueCopyPixelsTripleBuffered)(
            [this, texture_readback_minimal_descs = std::move(texture_readback_minimal_descs)](FRHICommandListImmediate& command_list) {
                for (auto& [name, texture_readback_minimal_desc] : texture_readback_minimal_descs) {
                    enqueueCopyPixelsFromGPUToStaging_RenderThread(command_list, texture_readback_minimal_desc);
                }
                command_list.ImmediateFlush(EImmediateFlushType::DispatchToRHIThread);
                for (auto& [name, texture_readback_minimal_desc] : texture_readback_minimal_descs) {
                    if (texture_readback_minimal_desc.prev_readback_) {
                        copyPixelsFromStagingToCPU_RenderThread(texture_readback_minimal_desc.prev_readback_, texture_readback_minimal_desc.dest_ptr_, texture_readback_minimal_desc);
                        SP_ASSERT(texture_readback_minimal_desc.num_readbacks_pending_ptr_);
                        (*texture_readback_minimal_desc.num_readbacks_pending_ptr_)--;
                        SP_ASSERT(*texture_readback_minimal_desc.num_readbacks_pending_ptr_ >= 0);
                    }
                }
            });
    }
}

SpFuncDataBundle USpSceneCaptureComponent2D::readPixelsTripleBuffered()
{
    SP_ASSERT(BufferingMode == ESpBufferingMode::TripleBuffered);
    SP_ASSERT(IsInitialized());

    // Wait for and read back the CPU-side data for the main texture and every user scene texture. The staging-to-CPU
    // copy already happened on the render thread (inside the enqueue command, see enqueueCopyPixelsTripleBuffered()),
    // so we just read the persistent descs' buffers here; readPixelsImpl() spin-waits on each pending counter first.

    SpFuncDataBundle return_values;
    Std::insert(return_values.packed_arrays_, "data", readPixelsImpl(texture_readback_desc_));
    for (auto& [name, texture_readback_desc] : user_scene_texture_readback_descs_) {
        Std::insert(return_values.packed_arrays_, name, readPixelsImpl(texture_readback_desc));
    }
    return return_values;
}

//
// game thread helpers
//

std::map<std::string, USpSceneCaptureComponent2D::TextureReadbackMinimalDesc> USpSceneCaptureComponent2D::requestUpdateAndGetTextureReadbackMinimalDescs()
{
    SP_ASSERT(IsInitialized());

    // Assemble a TextureReadbackMinimalDesc for the main texture and every active user scene texture, keyed by public
    // name, so a single batched render command can process all of them. Used by every buffering mode: single-buffered
    // readback (readPixelsSingleBuffered), double-buffered enqueue (enqueueCopyPixelsDoubleBuffered), and
    // triple-buffered enqueue (enqueueCopyPixelsTripleBuffered).

    // The current/prev readback pointers are resolved per buffering mode by requestSwapRHIGPUTextureReadbacks(). For
    // triple-buffered that advances the per-desc ring-buffer state (readback_enqueue_index_/readback_primed_), so this
    // function must be called exactly once per enqueue.

    // We record only a stable source handle for each desc (the render target resource for the main texture; the
    // address of pooled_render_target_ for a user scene texture) and never resolve the live source texture here. The
    // render thread resolves it (see enqueueCopyPixelsFromGPUToStaging_RenderThread). This is what lets us avoid an
    // unsynchronized game-thread read of pooled_render_target_, which the render thread reassigns every frame.

    std::map<std::string, TextureReadbackMinimalDesc> texture_readback_minimal_descs;

    // main texture: record the stable resource pointer (its GetRenderTargetTexture() is resolved later on the render thread)

    {
        FTextureRenderTargetResource* render_target_resource_ptr = TextureTarget->GameThread_GetRenderTargetResource();
        SP_ASSERT(render_target_resource_ptr);
        auto [current_readback, prev_readback] = requestSwapRHIGPUTextureReadbacks(texture_readback_desc_);
        TextureReadbackMinimalDesc texture_readback_minimal_desc = getTextureReadbackMinimalDesc(texture_readback_desc_, current_readback, prev_readback, render_target_resource_ptr, nullptr);
        Std::insert(texture_readback_minimal_descs, "data", std::move(texture_readback_minimal_desc));
    }

    // user scene textures: record the address of pooled_render_target_ (its GetRHI() is resolved later on the render thread)
    for (auto& name : UserSceneTextureNames) {
        std::string name_string = Unreal::toStdString(name);
        SP_ASSERT(user_scene_texture_readback_descs_.contains(name_string));
        TextureReadbackDesc& texture_readback_desc = user_scene_texture_readback_descs_.at(name_string);

        TRefCountPtr<IPooledRenderTarget>* pooled_render_target_ptr = &(texture_readback_desc.pooled_render_target_);
        auto [current_readback, prev_readback] = requestSwapRHIGPUTextureReadbacks(texture_readback_desc);
        TextureReadbackMinimalDesc texture_readback_minimal_desc = getTextureReadbackMinimalDesc(texture_readback_desc, current_readback, prev_readback, nullptr, pooled_render_target_ptr);
        Std::insert(texture_readback_minimal_descs, name_string, std::move(texture_readback_minimal_desc));
    }

    return texture_readback_minimal_descs;
}

std::pair<FRHIGPUTextureReadback*, FRHIGPUTextureReadback*> USpSceneCaptureComponent2D::requestSwapRHIGPUTextureReadbacks(TextureReadbackDesc& texture_readback_desc)
{
    // Resolve this frame's {current, prev} readback pointers according to the buffering mode. current_readback_ is the
    // GPU-to-staging target; prev_readback_ is the in-command staging-to-CPU source, or null if there is no in-command
    // copy this frame. For triple-buffered this advances the per-desc ring-buffer state, so it must be called exactly
    // once per enqueue.

    if (BufferingMode == ESpBufferingMode::SingleBuffered) {
        FRHIGPUTextureReadback* readback = texture_readback_desc.rhi_gpu_texture_readbacks_.at(0).get();
        return {readback, readback}; // single-buffered enqueues into and copies from the same readback
    } else if (BufferingMode == ESpBufferingMode::DoubleBuffered) {
        return {texture_readback_desc.rhi_gpu_texture_readbacks_.at(0).get(), nullptr}; // double-buffered has no in-command copy (it copies later in postRenderViewFamily_RenderThread)
    } else {
        SP_ASSERT(BufferingMode == ESpBufferingMode::TripleBuffered);
        int current_index = texture_readback_desc.readback_enqueue_index_;
        int prev_index = (current_index + 1) % 2;
        FRHIGPUTextureReadback* current_readback = texture_readback_desc.rhi_gpu_texture_readbacks_.at(current_index).get();
        FRHIGPUTextureReadback* prev_readback = nullptr;
        if (texture_readback_desc.readback_primed_) {
            prev_readback = texture_readback_desc.rhi_gpu_texture_readbacks_.at(prev_index).get(); // copy from the readback we wrote last frame
        }
        texture_readback_desc.readback_enqueue_index_ = prev_index; // advance the ring buffer
        texture_readback_desc.readback_primed_ = true;
        return {current_readback, prev_readback};
    }
}

USpSceneCaptureComponent2D::TextureReadbackMinimalDesc USpSceneCaptureComponent2D::getTextureReadbackMinimalDesc(
    TextureReadbackDesc& texture_readback_desc,
    FRHIGPUTextureReadback* current_readback,
    FRHIGPUTextureReadback* prev_readback,
    FTextureRenderTargetResource* render_target_resource_ptr,
    TRefCountPtr<IPooledRenderTarget>* pooled_render_target_ptr)
{
    SP_ASSERT(IsInitialized());
    SP_ASSERT(current_readback);

    // Exactly one source handle must be set: the render target resource for the main texture, or the address of a user
    // scene texture's pooled_render_target_. The render thread later resolves whichever one is set to a live FRHITexture
    // (see enqueueCopyPixelsFromGPUToStaging_RenderThread).

    SP_ASSERT(render_target_resource_ptr || pooled_render_target_ptr); // at least one handle is set
    if (render_target_resource_ptr) {
        SP_ASSERT(!pooled_render_target_ptr); // mutually exclusive with the user-scene-texture handle
    }
    if (pooled_render_target_ptr) {
        SP_ASSERT(!render_target_resource_ptr); // mutually exclusive with the main-texture handle
    }

    TextureReadbackMinimalDesc texture_readback_minimal_desc;
    texture_readback_minimal_desc.width_ = texture_readback_desc.width_;
    texture_readback_minimal_desc.height_ = texture_readback_desc.height_;
    texture_readback_minimal_desc.num_channels_per_pixel_ = texture_readback_desc.num_channels_per_pixel_;
    texture_readback_minimal_desc.channel_data_type_ = texture_readback_desc.channel_data_type_;
    texture_readback_minimal_desc.shared_memory_view_ = texture_readback_desc.shared_memory_view_;
    texture_readback_minimal_desc.current_readback_ = current_readback;
    texture_readback_minimal_desc.prev_readback_ = prev_readback;
    texture_readback_minimal_desc.num_readbacks_pending_ptr_ = &texture_readback_desc.num_readbacks_pending_;
    texture_readback_minimal_desc.num_staging_copies_pending_ptr_ = &texture_readback_desc.num_staging_copies_pending_;
    texture_readback_minimal_desc.render_target_resource_ptr_ = render_target_resource_ptr;
    texture_readback_minimal_desc.pooled_render_target_ptr_ = pooled_render_target_ptr;

    // Destination for the in-command staging-to-CPU copy. Only triple-buffered resolves it here (to the persistent
    // shared-memory or scratchpad buffer that readPixelsImpl() later returns). Single-buffered mode resolves its own
    // destination (the returned packed array's buffer) in readPixelsSingleBuffered(), and double-buffered copies
    // later in postRenderViewFamily_RenderThread(), so it never uses dest_ptr_.

    if (BufferingMode == ESpBufferingMode::TripleBuffered) {
        if (bUseSharedMemory) {
            texture_readback_minimal_desc.dest_ptr_ = texture_readback_desc.shared_memory_view_.data_;
        } else {
            texture_readback_minimal_desc.dest_ptr_ = texture_readback_desc.scratchpad_.data();
        }
    }

    return texture_readback_minimal_desc;
}

SpPackedArray USpSceneCaptureComponent2D::getPackedArray(const TextureReadbackDescBase& texture_readback_desc_base)
{
    SP_ASSERT(IsInitialized());
    SP_ASSERT(texture_readback_desc_base.width_ >= 0);
    SP_ASSERT(texture_readback_desc_base.height_ >= 0);

    int32 width = texture_readback_desc_base.width_;
    int32 height = texture_readback_desc_base.height_;
    int32 num_channels_per_pixel = texture_readback_desc_base.num_channels_per_pixel_;
    SpArrayDataType channel_data_type = texture_readback_desc_base.channel_data_type_;
    uint64_t num_bytes = height*width*num_channels_per_pixel*SpArrayDataTypeUtils::getSizeOf(channel_data_type);

    SpPackedArray packed_array;
    packed_array.shape_ = {static_cast<uint64_t>(height), static_cast<uint64_t>(width), static_cast<uint64_t>(num_channels_per_pixel)}; // explicit cast needed on Windows
    packed_array.data_type_ = channel_data_type;

    if (bUseSharedMemory) {
        packed_array.view_ = texture_readback_desc_base.shared_memory_view_.data_;
        packed_array.data_source_ = SpArrayDataSource::Shared;
        packed_array.shared_memory_name_ = texture_readback_desc_base.shared_memory_view_.name_;
        packed_array.shared_memory_usage_flags_ = texture_readback_desc_base.shared_memory_view_.usage_flags_;
    } else {
        Std::resizeUninitialized(packed_array.data_, num_bytes);
        packed_array.view_ = packed_array.data_.data();
        packed_array.data_source_ = SpArrayDataSource::Internal;
    }

    return packed_array;
}

SpPackedArray USpSceneCaptureComponent2D::readPixelsImpl(const TextureReadbackDesc& texture_readback_desc)
{
    SP_ASSERT(IsInitialized());

    if (bReadPixelData) {
        SP_ASSERT(texture_readback_desc.num_readbacks_pending_ >= 0);
        int64_t spin_wait_iterations = 0;
        while (texture_readback_desc.num_readbacks_pending_ > 0) {
            spin_wait_iterations++;
            if (spin_wait_iterations % (100*1000*1000) == 0) {
                SP_LOG("ERROR: Spin wait in readPixelsImpl() appears to be deadlocked.");
                SP_ASSERT(false);
            }
        }
        if (bPrintReadbackSpinWaitInfo && spin_wait_iterations > 0) {
            SP_LOG("WARNING: Readback spin-waited for ", spin_wait_iterations, " iterations in readPixelsImpl().");
        }
    }

    SpPackedArray packed_array = getPackedArray(texture_readback_desc);

    if (!bUseSharedMemory && bReadPixelData) {
        uint64_t num_bytes = texture_readback_desc.height_*texture_readback_desc.width_*texture_readback_desc.num_channels_per_pixel_*SpArrayDataTypeUtils::getSizeOf(texture_readback_desc.channel_data_type_);
        std::memcpy(packed_array.data_.data(), texture_readback_desc.scratchpad_.data(), num_bytes);
    }

    return packed_array;
}

//
// render thread helpers
//

void USpSceneCaptureComponent2D::enqueueCopyPixelsFromGPUToStaging_RenderThread(FRHICommandListImmediate& command_list, const TextureReadbackMinimalDesc& texture_readback_minimal_desc)
{
    SP_ASSERT(texture_readback_minimal_desc.current_readback_);

    // Resolve the live source texture from the stable handle the game thread recorded. Exactly one handle is set: the
    // render target resource for the main texture, or a pointer to the persistent desc's pooled_render_target_ for a
    // user scene texture. Both accessors return render-thread-side RHI, so this is the correct thread to call them on.

    FRHITexture* src_texture = nullptr;
    TRefCountPtr<IPooledRenderTarget> pooled_render_target; // keeps a user scene texture's source alive through EnqueueCopy

    SP_ASSERT(texture_readback_minimal_desc.render_target_resource_ptr_ || texture_readback_minimal_desc.pooled_render_target_ptr_); // at least one handle is set
    if (texture_readback_minimal_desc.render_target_resource_ptr_) {           // main texture
        SP_ASSERT(!texture_readback_minimal_desc.pooled_render_target_ptr_);   // mutually exclusive with the user-scene-texture handle
        src_texture = texture_readback_minimal_desc.render_target_resource_ptr_->GetRenderTargetTexture();
    }
    if (texture_readback_minimal_desc.pooled_render_target_ptr_) {             // user scene texture
        SP_ASSERT(!texture_readback_minimal_desc.render_target_resource_ptr_); // mutually exclusive with the main-texture handle
        pooled_render_target = *texture_readback_minimal_desc.pooled_render_target_ptr_;
        SP_ASSERT(pooled_render_target);
        src_texture = pooled_render_target->GetRHI();
    }

    SP_ASSERT(src_texture);

    command_list.Transition(FRHITransitionInfo(src_texture, ERHIAccess::SRVMask, ERHIAccess::CopySrc));
    texture_readback_minimal_desc.current_readback_->EnqueueCopy(command_list, src_texture);
}

void USpSceneCaptureComponent2D::copyPixelsFromStagingToCPU_RenderThread(FRHIGPUTextureReadback* readback, void* dest_ptr, const TextureReadbackDescBase& texture_readback_desc_base)
{
    SP_ASSERT(readback);
    SP_ASSERT(dest_ptr);

    int32 width = texture_readback_desc_base.width_;
    int32 height = texture_readback_desc_base.height_;
    int32 num_channels_per_pixel = texture_readback_desc_base.num_channels_per_pixel_;
    SpArrayDataType channel_data_type = texture_readback_desc_base.channel_data_type_;

    uint64_t num_bytes = height*width*num_channels_per_pixel*SpArrayDataTypeUtils::getSizeOf(channel_data_type);
    int32_t bytes_per_pixel = num_channels_per_pixel*SpArrayDataTypeUtils::getSizeOf(channel_data_type);
    int32_t row_bytes = width*bytes_per_pixel;

    // Lock() blocks internally until the readback is GPU-complete before mapping (e.g., on Metal RHIMapStagingSurface()
    // does SubmitAndBlockUntilGPUIdle() + a CPU wait on the fence), so the mapped pointer is always valid data and we
    // do not need any RT-side readiness wait here. Lock() returns null only if the readback's staging texture was never
    // populated by EnqueueCopy(); callers must guarantee the matching EnqueueCopy() has executed on the render thread
    // before we reach here. That ordering is enforced by the render-thread consume gate (num_staging_copies_pending_)
    // in postRenderViewFamily_RenderThread() for DoubleBuffered, and by construction in enqueueCopyPixelsTripleBuffered()
    // and readPixelsSingleBuffered(), where the EnqueueCopy() and this copy run in the same command.

    int32 row_pitch_in_pixels = 0;
    void* src_ptr = readback->Lock(row_pitch_in_pixels);
    SP_ASSERT(src_ptr);
    SP_ASSERT(row_pitch_in_pixels >= width);

    int32_t src_row_pitch_bytes = row_pitch_in_pixels*bytes_per_pixel;

    if (src_row_pitch_bytes == row_bytes) {
        std::memcpy(dest_ptr, src_ptr, num_bytes);
    } else {
        uint8_t* src = static_cast<uint8_t*>(src_ptr);
        uint8_t* dst = static_cast<uint8_t*>(dest_ptr);
        for (int32_t y = 0; y < height; y++) {
            std::memcpy(dst + y*row_bytes, src + y*src_row_pitch_bytes, row_bytes);
        }
    }

    readback->Unlock();
}

//
// miscellaneous helpers
//

void USpSceneCaptureComponent2D::updateFrameTime()
{
    std::chrono::time_point current_time_point = std::chrono::high_resolution_clock::now();
    double time_delta_seconds = std::chrono::duration<double>(current_time_point - previous_time_point_).count();
    previous_time_deltas_.push_back(time_delta_seconds);
    previous_time_point_ = current_time_point;

    frame_index_++;
    int num_samples_per_print_statement = 100;
    if (frame_index_ % num_samples_per_print_statement == 0) {
        double time_delta_average_seconds = std::accumulate(previous_time_deltas_.begin(), previous_time_deltas_.end(), 0.0) / previous_time_deltas_.size();
        SP_LOG("Time delta (milliseconds): ", 1000.0*time_delta_average_seconds, " (", 1.0/time_delta_average_seconds, " frames per second)");
        frame_index_ = 0;
    }
}
