//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpUnrealTypes/SpSceneCaptureComponent2D.h"

#include <stdint.h> // uint64_t

#include <chrono>
#include <cstring> // std::memcpy
#include <memory>  // std::align, std::make_unique
#include <utility> // std::move

#include <Components/SceneCaptureComponent2D.h>
#include <Containers/Array.h>
#include <Delegates/IDelegateInstance.h> // FDelegateHandle
#include <Materials/Material.h>
#include <Materials/MaterialInstanceDynamic.h>
#include <Math/Color.h>                  // FColor, FLinearColor
#include <Math/Float16Color.h>
#include <RenderingThread.h>             // ENQUEUE_RENDER_COMMAND
#include <RHICommandList.h>              // EImmediateFlushType, FRHICommandListImmediate
#include <RHIDefinitions.h>              // ERangeCompressionMode
#include <RHITypes.h>                    // FReadSurfaceDataFlags
#include <SceneManagement.h>             // FSceneViewStateInterface
#include <SceneView.h>                   // FSceneViewFamily
#include <SceneViewExtension.h>          // FAutoRegister, FSceneViewExtensionBase, FSceneViewExtensions
#include <Templates/SharedPointer.h>     // TSharedPtr
#include <TextureResource.h>             // FTextureRenderTargetResource
#include <UObject/UObjectGlobals.h>      // NewObject

#include "SpCore/Assert.h"
#include "SpCore/Boost.h"
#include "SpCore/Log.h"
#include "SpCore/SharedMemory.h"
#include "SpCore/SpArray.h"
#include "SpCore/SpFuncComponent.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"
#include "SpCore/UnrealArrayUpdateDataPtrScope.h"
#include "SpCore/UnrealUtils.h"

#include "SpUnrealTypes/SpPrimitiveProxyComponentManager.h"

FSpSceneViewExtensionBase::FSpSceneViewExtensionBase(const FAutoRegister& auto_register, USpSceneCaptureComponent2D* component) : FSceneViewExtensionBase(auto_register)
{
    SP_LOG_CURRENT_FUNCTION();

    SP_ASSERT(component);
    component_ = component;
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
        postRenderViewFamily_RenderThread(in_view_family);
    }
}

bool FSpSceneViewExtensionBase::shouldHandleViewFamily(const FSceneViewFamily* view_family) const
{
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
    SP_ASSERT(component_);
    SP_ASSERT(view_family);
    SP_ASSERT(view);

    FSceneViewStateInterface* view_state = view->State;
    if (!view_state) {
        return false;
    }

    int32 view_state_key = view->State->GetViewKey();

    for (int32 i = 0; i < component_->getNumViewStates(); i++) {
        FSceneViewStateInterface* component_view_state = component_->GetViewState(i);
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

FSpSceneViewExtension::FSpSceneViewExtension(const FAutoRegister& auto_register, USpSceneCaptureComponent2D* component) : FSpSceneViewExtensionBase(auto_register, component)
{
    SP_LOG_CURRENT_FUNCTION();
}

void FSpSceneViewExtension::setupView(FSceneViewFamily& view_family, FSceneView& view)
{
    const TArray<FEngineShowFlagsSetting>& engine_show_flag_settings = getComponent()->GetShowFlagSettings();

    for (auto& engine_show_flag_setting : engine_show_flag_settings) {
        if (Unreal::toStdString(engine_show_flag_setting.ShowFlagName) == "LightingOnlyOverride" && engine_show_flag_setting.Enabled) {
            view.DiffuseOverrideParameter = FVector4f(GEngine->LightingOnlyBrightness.R, GEngine->LightingOnlyBrightness.G, GEngine->LightingOnlyBrightness.B, 0.0f);
            view.SpecularOverrideParameter = FVector4f(0.0f, 0.0f, 0.0f, 0.0f);
        }

        if (Unreal::toStdString(engine_show_flag_setting.ShowFlagName) == "Specular" && !engine_show_flag_setting.Enabled) {
            view.SpecularOverrideParameter = FVector4f(0.0f, 0.0f, 0.0f, 0.0f);
        }
    }
}

void FSpSceneViewExtension::postRenderViewFamily_RenderThread(FSceneViewFamily& view_family)
{
    getComponent()->postRender_RenderThread();
}

USpSceneCaptureComponent2D::USpSceneCaptureComponent2D()
{
    SP_LOG_CURRENT_FUNCTION();

    SpFuncComponent = UnrealUtils::createSceneComponentInsideOwnerConstructor<USpFuncComponent>(this, "sp_func_component");
    SP_ASSERT(SpFuncComponent);

    // we want to be able to capture the scene even when the game is paused; note that we don't set
    // PrimaryActorTick.TickGroup because we don't want to interfere with when the base class component
    // gets ticked
    PrimaryComponentTick.bCanEverTick = true;
    PrimaryComponentTick.bTickEvenWhenPaused = true;

    SetVisibility(false); // disable rendering to texture
}

USpSceneCaptureComponent2D::~USpSceneCaptureComponent2D()
{
    SP_LOG_CURRENT_FUNCTION();
}

void USpSceneCaptureComponent2D::BeginPlay()
{
    SP_LOG_CURRENT_FUNCTION();

    USceneCaptureComponent2D::BeginPlay();
    bIsInitialized = false;
}

void USpSceneCaptureComponent2D::EndPlay(const EEndPlayReason::Type end_play_reason)
{
    SP_LOG_CURRENT_FUNCTION();

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
    TextureTarget->ClearColor = FLinearColor(1.0f, 0.0f, 1.0f, 1.0f); // bright pink

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
        material_instance_dynamic_ = UMaterialInstanceDynamic::Create(Material, this);
        PostProcessSettings.AddBlendable(material_instance_dynamic_, 1.0f);
    }

    if (bHidePrimitiveProxyComponentManagers) {
        PrimitiveRenderMode = ESceneCapturePrimitiveRenderMode::PRM_RenderScenePrimitives; // use HiddenActors list
        std::vector<ASpPrimitiveProxyComponentManager*> primitive_proxy_component_managers = UnrealUtils::findActorsByType<ASpPrimitiveProxyComponentManager>(GetWorld());
        for (auto primitive_proxy_component_manager : primitive_proxy_component_managers) {
            HiddenActors.Add(primitive_proxy_component_manager);
        }
    }

    if (AllowedProxyComponentModalities.Num() > 0) {
        HiddenActors.Empty(); // clear HiddenActors because all actors will be hidden by default
        PrimitiveRenderMode = ESceneCapturePrimitiveRenderMode::PRM_UseShowOnlyList; // use ShowOnlyActors list

        std::vector<std::string> allowed_modalities = Std::toVector<std::string>(
            Unreal::toStdVector(AllowedProxyComponentModalities) |
            std::views::transform([](auto& str) { return Unreal::toStdString(str); }));

        std::vector<ASpPrimitiveProxyComponentManager*> primitive_proxy_component_managers = Std::toVector<ASpPrimitiveProxyComponentManager*>(
            UnrealUtils::findActorsByType<ASpPrimitiveProxyComponentManager>(GetWorld()) |
            std::views::filter([&allowed_modalities](auto manager) { return Std::contains(allowed_modalities, manager->getModalityName()); }));

        for (auto primitive_proxy_component_manager : primitive_proxy_component_managers) {
            ShowOnlyActors.Add(primitive_proxy_component_manager);
        }
    }

    // ensure that the underlying view state data is stable across frames so FSpSceneViewExtensionBase can
    // match view state data to this component
    bAlwaysPersistRenderingState = true;

    if (bUseSceneViewExtension || BufferingMode != ESpBufferingMode::SingleBuffered) {
        scene_view_extension_ = FSceneViewExtensions::NewExtension<FSpSceneViewExtension>(this);
    }

    // initialize state for measuring "standalone" and "standalone + extra work" frame rates

    if (bPrintFrameTimeEveryFrame) {
        begin_frame_handle_ = FCoreDelegates::OnBeginFrame.AddUObject(this, &USpSceneCaptureComponent2D::beginFrameHandler);
    }

    if (bReadPixelsEveryFrame) {
        end_frame_handle_ = FCoreDelegates::OnEndFrame.AddUObject(this, &USpSceneCaptureComponent2D::endFrameHandler);
    }

    if (bPrintFrameTimeEveryFrame) {
        int num_samples = 100;
        previous_time_deltas_.set_capacity(num_samples);
    }

    // allocate memory

    SpArrayDataType channel_data_type = Unreal::getEnumValueAs<SpArrayDataType, ESpArrayDataType>(ChannelDataType);
    uint64_t num_bytes = Height*Width*NumChannelsPerPixel*SpArrayDataTypeUtils::getSizeOf(channel_data_type);

    if (bUseSharedMemory) {
        SP_ASSERT(!shared_memory_region_);
        shared_memory_region_ = std::make_unique<SharedMemoryRegion>(num_bytes);
        SP_ASSERT(shared_memory_region_);
        shared_memory_view_ = SpArraySharedMemoryView(shared_memory_region_->getView(), "smem:sp_scene_capture_component_2d", SpArraySharedMemoryUsageFlags::ReturnValue);
        SpFuncComponent->registerSharedMemoryView(shared_memory_view_); // name needs to be unique per USpFuncComponent
    }

    if (NumChannelsPerPixel == 4 && channel_data_type == SpArrayDataType::UInt8) {
        scratchpad_array_color_.Reserve(Height*Width);
        SP_ASSERT(Height*Width <= static_cast<int64_t>(scratchpad_array_color_.Max()));
        SP_ASSERT(num_bytes <= scratchpad_array_color_.GetAllocatedSize());
    } else if (NumChannelsPerPixel == 4 && channel_data_type == SpArrayDataType::Float16) {
        scratchpad_array_float_16_color_.Reserve(Height*Width);
        SP_ASSERT(Height*Width <= static_cast<int64_t>(scratchpad_array_float_16_color_.Max()));
        SP_ASSERT(num_bytes <= scratchpad_array_float_16_color_.GetAllocatedSize());
    } else if (NumChannelsPerPixel == 4 && channel_data_type == SpArrayDataType::Float32) {
        scratchpad_array_linear_color_.Reserve(Height*Width);
        SP_ASSERT(Height*Width <= static_cast<int64_t>(scratchpad_array_linear_color_.Max()));
        SP_ASSERT(num_bytes <= scratchpad_array_linear_color_.GetAllocatedSize());
    } else {
        SP_ASSERT(false);
    }

    // allocate readback buffers
    if (BufferingMode == ESpBufferingMode::DoubleBuffered) {
        readback_buffers_.at(0) = std::make_unique<FRHIGPUTextureReadback>(TEXT("rhi_gpu_texture_readback_A"));
        readback_pending_ = false;
    } else if (BufferingMode == ESpBufferingMode::TripleBuffered) {
        readback_buffers_.at(0) = std::make_unique<FRHIGPUTextureReadback>(TEXT("rhi_gpu_texture_readback_A"));
        readback_buffers_.at(1) = std::make_unique<FRHIGPUTextureReadback>(TEXT("rhi_gpu_texture_readback_B"));
        readback_enqueue_index_ = 0;
        readback_primed_ = false;
        readback_pending_ = false;
    }

    // register SpFuncs
    SpFuncComponent->registerFunc("read_pixels", [this](SpFuncDataBundle& args) -> SpFuncDataBundle {
        SpFuncDataBundle return_values;
        SpPackedArray packed_array;
        if (BufferingMode == ESpBufferingMode::SingleBuffered) {
            packed_array = readPixelsSingleBuffered();
        } else if (BufferingMode == ESpBufferingMode::DoubleBuffered) {
            packed_array = readPixelsDoubleBuffered();
        } else if (BufferingMode == ESpBufferingMode::TripleBuffered) {
            packed_array = readPixelsTripleBuffered();
        } else {
            SP_ASSERT(false);
        }
        Std::insert(return_values.packed_arrays_, "data", std::move(packed_array));
        return return_values;
    });

    SpFuncComponent->registerFunc("enqueue_copy", [this](SpFuncDataBundle& args) -> SpFuncDataBundle {
        SP_ASSERT(BufferingMode != ESpBufferingMode::SingleBuffered);
        SP_ASSERT(!bReadPixelsEveryFrame);
        if (BufferingMode == ESpBufferingMode::DoubleBuffered) {
            enqueueCopyPixelsFromGPUToStaging();
        } else if (BufferingMode == ESpBufferingMode::TripleBuffered) {
            enqueueCopyPixelsTripleBuffered();
        } else {
            SP_ASSERT(false);
        }
        return SpFuncDataBundle();
    });

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

    is_initialized_ = false;
    bIsInitialized = false;

    // unregister SpFuncs
    SpFuncComponent->unregisterFunc("read_pixels");
    SpFuncComponent->unregisterFunc("enqueue_copy");

    // deallocate readback buffers
    readback_pending_ = false;
    readback_primed_ = false;
    readback_enqueue_index_ = 0;
    readback_buffers_.at(0).reset();
    readback_buffers_.at(1).reset();

    // deallocate memory
    if (shared_memory_region_) {
        SpFuncComponent->unregisterSharedMemoryView(shared_memory_view_);
        shared_memory_view_ = SpArraySharedMemoryView();
        shared_memory_region_ = nullptr;
    }

    // remove callbacks

    if (bPrintFrameTimeEveryFrame) {
        FCoreDelegates::OnBeginFrame.Remove(begin_frame_handle_);
        begin_frame_handle_.Reset();
    }

    if (bReadPixelsEveryFrame) {
        FCoreDelegates::OnEndFrame.Remove(end_frame_handle_);
        end_frame_handle_.Reset();
    }

    bAlwaysPersistRenderingState = false;
    scene_view_extension_ = nullptr;

    ShowOnlyActors.Empty();
    HiddenActors.Empty();

    if (material_instance_dynamic_) {
        PostProcessSettings.RemoveBlendable(material_instance_dynamic_);
    }

    TextureTarget = nullptr;
}

bool USpSceneCaptureComponent2D::IsInitialized()
{
    return is_initialized_;
}

void USpSceneCaptureComponent2D::beginFrameHandler()
{
    if (bPrintFrameTimeEveryFrame) {
        updateFrameTime();
    }
}

void USpSceneCaptureComponent2D::endFrameHandler()
{
    if (bReadPixelsEveryFrame) {
        scratchpad_packed_array_ = readPixelsSingleBuffered();
    }
}

void USpSceneCaptureComponent2D::enqueueCopyPixelsFromGPUToStaging()
{
    SP_ASSERT(IsInitialized());
    SP_ASSERT(BufferingMode == ESpBufferingMode::DoubleBuffered);
    SP_ASSERT(!bReadPixelsEveryFrame);

    readback_pending_ = true;

    if (bReadPixelData) {
        FTextureRenderTargetResource* render_target_resource = TextureTarget->GameThread_GetRenderTargetResource();
        SP_ASSERT(render_target_resource);

        ENQUEUE_RENDER_COMMAND(SpEnqueueCopyPixelsFromGPUToStaging)(
            [this, render_target_resource](FRHICommandListImmediate& rhi_command_list_immediate) {
                enqueueCopyPixelsFromGPUToStaging_RenderThread(rhi_command_list_immediate, render_target_resource);
            });
    }
}

void USpSceneCaptureComponent2D::enqueueCopyPixelsTripleBuffered()
{
    SP_ASSERT(IsInitialized());
    SP_ASSERT(BufferingMode == ESpBufferingMode::TripleBuffered);
    SP_ASSERT(!bReadPixelsEveryFrame);

    int prev_index = (readback_enqueue_index_ + 1) % 2;
    int current_index = readback_enqueue_index_;
    bool has_previous = readback_primed_;

    if (has_previous) {
        readback_pending_ = true;
    }

    if (bReadPixelData) {
        FTextureRenderTargetResource* render_target_resource = TextureTarget->GameThread_GetRenderTargetResource();
        SP_ASSERT(render_target_resource);

        FRHIGPUTextureReadback* prev_readback = has_previous ? readback_buffers_.at(prev_index).get() : nullptr;
        FRHIGPUTextureReadback* current_readback = readback_buffers_.at(current_index).get();
        SP_ASSERT(current_readback);

        ENQUEUE_RENDER_COMMAND(SpEnqueueCopyPixelsTripleBuffered)(
            [this, prev_readback, current_readback, render_target_resource](FRHICommandListImmediate& rhi_command_list_immediate) {
                if (prev_readback) {
                    copyPixelsFromStagingToCPU_RenderThread(prev_readback);
                }
                current_readback->EnqueueCopy(rhi_command_list_immediate, render_target_resource->GetRenderTargetTexture());
                rhi_command_list_immediate.ImmediateFlush(EImmediateFlushType::DispatchToRHIThread);
                if (prev_readback) {
                    readback_pending_ = false;
                }
            });
    } else {
        if (has_previous) {
            readback_pending_ = false;
        }
    }

    readback_enqueue_index_ = (readback_enqueue_index_ + 1) % 2;
    readback_primed_ = true;
}

void USpSceneCaptureComponent2D::enqueueCopyPixelsFromGPUToStaging_RenderThread(FRHICommandListImmediate& rhi_command_list_immediate, FTextureRenderTargetResource* render_target_resource)
{
    SP_ASSERT(readback_buffers_.at(0));
    readback_buffers_.at(0)->EnqueueCopy(rhi_command_list_immediate, render_target_resource->GetRenderTargetTexture());
    rhi_command_list_immediate.ImmediateFlush(EImmediateFlushType::DispatchToRHIThread);
}

void USpSceneCaptureComponent2D::copyPixelsFromStagingToCPU_RenderThread(FRHIGPUTextureReadback* readback)
{
    SP_ASSERT(readback);

    int64_t spin_wait_iterations = 0;
    while (!readback->IsReady()) {
        spin_wait_iterations++;
    }
    if (bPrintReadbackSpinWaitInfo && spin_wait_iterations > 0) {
        SP_LOG("WARNING: Readback spin-waited for ", spin_wait_iterations, " iterations in copyPixelsFromStagingToCPU_RenderThread(...).");
    }

    SpArrayDataType channel_data_type = Unreal::getEnumValueAs<SpArrayDataType, ESpArrayDataType>(ChannelDataType);
    uint64_t num_bytes = Height*Width*NumChannelsPerPixel*SpArrayDataTypeUtils::getSizeOf(channel_data_type);
    int32_t bytes_per_pixel = NumChannelsPerPixel * SpArrayDataTypeUtils::getSizeOf(channel_data_type);
    int32_t row_bytes = Width * bytes_per_pixel;

    void* dest_ptr = nullptr;
    if (bUseSharedMemory) {
        dest_ptr = shared_memory_view_.data_;
    } else {
        if (NumChannelsPerPixel == 4 && channel_data_type == SpArrayDataType::UInt8) {
            dest_ptr = scratchpad_array_color_.GetData();
        } else if (NumChannelsPerPixel == 4 && channel_data_type == SpArrayDataType::Float16) {
            dest_ptr = scratchpad_array_float_16_color_.GetData();
        } else if (NumChannelsPerPixel == 4 && channel_data_type == SpArrayDataType::Float32) {
            dest_ptr = scratchpad_array_linear_color_.GetData();
        } else {
            SP_ASSERT(false);
        }
    }
    SP_ASSERT(dest_ptr);

    int32 row_pitch_in_pixels = 0;
    void* src_ptr = readback->Lock(row_pitch_in_pixels);
    SP_ASSERT(src_ptr);
    SP_ASSERT(row_pitch_in_pixels >= Width);

    int32_t src_row_pitch_bytes = row_pitch_in_pixels * bytes_per_pixel;

    if (src_row_pitch_bytes == row_bytes) {
        std::memcpy(dest_ptr, src_ptr, num_bytes);
    } else {
        uint8_t* src = static_cast<uint8_t*>(src_ptr);
        uint8_t* dst = static_cast<uint8_t*>(dest_ptr);
        for (int32_t y = 0; y < Height; y++) {
            std::memcpy(dst + y * row_bytes, src + y * src_row_pitch_bytes, row_bytes);
        }
    }

    readback->Unlock();
}

void USpSceneCaptureComponent2D::postRender_RenderThread()
{
    if (BufferingMode != ESpBufferingMode::DoubleBuffered) {
        return;
    }

    if (!readback_pending_) {
        return;
    }

    if (bReadPixelData) {
        copyPixelsFromStagingToCPU_RenderThread(readback_buffers_.at(0).get());
    }

    readback_pending_ = false;
}

SpPackedArray USpSceneCaptureComponent2D::readPixelsSingleBuffered()
{
    SP_ASSERT(IsInitialized());
    SP_ASSERT(Height >= 0);
    SP_ASSERT(Width >= 0);
    SP_ASSERT(NumChannelsPerPixel == 4);

    SpPackedArray packed_array;

    SpArrayDataType channel_data_type = Unreal::getEnumValueAs<SpArrayDataType, ESpArrayDataType>(ChannelDataType);
    uint64_t num_bytes = Height*Width*NumChannelsPerPixel*SpArrayDataTypeUtils::getSizeOf(channel_data_type);

    packed_array.shape_ = {static_cast<uint64_t>(Height), static_cast<uint64_t>(Width), static_cast<uint64_t>(NumChannelsPerPixel)}; // explicit cast needed on Windows
    packed_array.data_type_ = channel_data_type;

    void* dest_ptr = nullptr;
    if (bUseSharedMemory) {
        packed_array.view_ = shared_memory_view_.data_;
        packed_array.data_source_ = SpArrayDataSource::Shared;
        packed_array.shared_memory_name_ = shared_memory_view_.name_;
        packed_array.shared_memory_usage_flags_ = shared_memory_view_.usage_flags_;
        dest_ptr = shared_memory_view_.data_;
    } else {
        Std::resizeUninitialized(packed_array.data_, num_bytes);
        packed_array.view_ = packed_array.data_.data();
        packed_array.data_source_ = SpArrayDataSource::Internal;
        dest_ptr = packed_array.data_.data();
    }
    SP_ASSERT(dest_ptr);

    if (bReadPixelData) {
        FTextureRenderTargetResource* texture_render_target_resource = TextureTarget->GameThread_GetRenderTargetResource();
        SP_ASSERT(texture_render_target_resource);

        // ReadPixels assumes 4 channels per pixel, 1 uint8 per channel
        if (NumChannelsPerPixel == 4 && channel_data_type == SpArrayDataType::UInt8) {

            FReadSurfaceDataFlags read_surface_flags = FReadSurfaceDataFlags(ERangeCompressionMode::RCM_UNorm);
            if (bOverrideSetLinearToGamma) {
                read_surface_flags.SetLinearToGamma(bSetLinearToGamma);
            }

            UnrealArrayUpdateDataPtrScope scope(scratchpad_array_color_, dest_ptr, num_bytes);
            bool success = texture_render_target_resource->ReadPixels(scratchpad_array_color_, read_surface_flags);
            SP_ASSERT(success);

        // ReadFloat16Pixels assumes 4 channels per pixel, 1 float16 per channel
        } else if (NumChannelsPerPixel == 4 && channel_data_type == SpArrayDataType::Float16) {

            // we never want to change floating point values when reading
            FReadSurfaceDataFlags read_surface_flags = FReadSurfaceDataFlags(ERangeCompressionMode::RCM_MinMax);
            read_surface_flags.SetLinearToGamma(false);

            UnrealArrayUpdateDataPtrScope scope(scratchpad_array_float_16_color_, dest_ptr, num_bytes);
            bool success = texture_render_target_resource->ReadFloat16Pixels(scratchpad_array_float_16_color_, read_surface_flags);
            SP_ASSERT(success);

        // ReadLinearColorPixels assumes 4 channels per pixel, 1 float32 per channel
        } else if (NumChannelsPerPixel == 4 && channel_data_type == SpArrayDataType::Float32) {
            SP_ASSERT(BOOST_OS_WINDOWS, "ERROR: NumChannelsPerPixel == 4 && channel_data_type == SpArrayDataType::Float32 is only supported on Windows.");

            // we never want to change floating point values when reading
            FReadSurfaceDataFlags read_surface_flags = FReadSurfaceDataFlags(ERangeCompressionMode::RCM_MinMax);
            read_surface_flags.SetLinearToGamma(false);

            UnrealArrayUpdateDataPtrScope scope(scratchpad_array_linear_color_, dest_ptr, num_bytes);
            bool success = texture_render_target_resource->ReadLinearColorPixels(scratchpad_array_linear_color_, read_surface_flags);
            SP_ASSERT(success);

        } else {
            SP_ASSERT(false);
        }
    }

    return packed_array;
}

SpPackedArray USpSceneCaptureComponent2D::readPixelsDoubleBuffered()
{
    SP_ASSERT(BufferingMode == ESpBufferingMode::DoubleBuffered);
    SP_ASSERT(!bReadPixelsEveryFrame);
    return readPixelsBufferedImpl();
}

SpPackedArray USpSceneCaptureComponent2D::readPixelsTripleBuffered()
{
    SP_ASSERT(BufferingMode == ESpBufferingMode::TripleBuffered);
    SP_ASSERT(!bReadPixelsEveryFrame);
    return readPixelsBufferedImpl();
}

SpPackedArray USpSceneCaptureComponent2D::readPixelsBufferedImpl()
{
    SP_ASSERT(IsInitialized());
    SP_ASSERT(Height >= 0);
    SP_ASSERT(Width >= 0);
    SP_ASSERT(NumChannelsPerPixel == 4);

    int64_t spin_wait_iterations = 0;
    while (readback_pending_) {
        spin_wait_iterations++;
    }
    if (bPrintReadbackSpinWaitInfo && spin_wait_iterations > 0) {
        SP_LOG("WARNING: Readback spin-waited for ", spin_wait_iterations, " iterations in readPixelsBufferedImpl().");
    }

    SpPackedArray packed_array;

    SpArrayDataType channel_data_type = Unreal::getEnumValueAs<SpArrayDataType, ESpArrayDataType>(ChannelDataType);
    uint64_t num_bytes = Height*Width*NumChannelsPerPixel*SpArrayDataTypeUtils::getSizeOf(channel_data_type);

    packed_array.shape_ = {static_cast<uint64_t>(Height), static_cast<uint64_t>(Width), static_cast<uint64_t>(NumChannelsPerPixel)}; // explicit cast needed on Windows
    packed_array.data_type_ = channel_data_type;

    if (bUseSharedMemory) {
        packed_array.view_ = shared_memory_view_.data_;
        packed_array.data_source_ = SpArrayDataSource::Shared;
        packed_array.shared_memory_name_ = shared_memory_view_.name_;
        packed_array.shared_memory_usage_flags_ = shared_memory_view_.usage_flags_;
    } else {
        Std::resizeUninitialized(packed_array.data_, num_bytes);
        packed_array.view_ = packed_array.data_.data();
        packed_array.data_source_ = SpArrayDataSource::Internal;
        if (bReadPixelData) {
            void* scratchpad_ptr = nullptr;
            if (NumChannelsPerPixel == 4 && channel_data_type == SpArrayDataType::UInt8) {
                scratchpad_ptr = scratchpad_array_color_.GetData();
            } else if (NumChannelsPerPixel == 4 && channel_data_type == SpArrayDataType::Float16) {
                scratchpad_ptr = scratchpad_array_float_16_color_.GetData();
            } else if (NumChannelsPerPixel == 4 && channel_data_type == SpArrayDataType::Float32) {
                scratchpad_ptr = scratchpad_array_linear_color_.GetData();
            } else {
                SP_ASSERT(false);
            }
            std::memcpy(packed_array.data_.data(), scratchpad_ptr, num_bytes);
        }
    }

    return packed_array;
}

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
