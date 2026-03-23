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
#include <RenderingThread.h>             // ENQUEUE_RENDER_COMMAND, FlushRenderingCommands
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

    if (bUseSceneViewExtension) {
        scene_view_extension_ = FSceneViewExtensions::NewExtension<FSpSceneViewExtension>(this);
    }

    // initialize state for measuring "standalone" and "standalone + extra work" frame rates

    if (bPrintFrameTimeEveryFrame || bReadPixelsEveryFrame) {
        begin_frame_handle_ = FCoreDelegates::OnBeginFrame.AddUObject(this, &USpSceneCaptureComponent2D::beginFrameHandler);
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

    // allocate readback buffers for double-buffered readback
    if (bUseDoubleBufferedReadback) {
        SP_ASSERT(!bReadPixelsEveryFrame);

        for (int i = 0; i < NUM_READBACK_BUFFERS; i++) {
            readback_buffers_.at(i) = std::make_unique<FRHIGPUTextureReadback>(FName(*FString::Printf(TEXT("SpReadback_%d"), i)));
        }
        readback_pending_.set_capacity(NUM_READBACK_BUFFERS);
        readback_enqueue_index_ = 0;
        readback_scratchpad_ready_ = false;
    }

    // register SpFuncs
    SpFuncComponent->registerFunc("enqueue_copy", [this](SpFuncDataBundle& args) -> SpFuncDataBundle {
        enqueueCopy();
        return SpFuncDataBundle();
    });

    SpFuncComponent->registerFunc("read_pixels", [this](SpFuncDataBundle& args) -> SpFuncDataBundle {
        SpFuncDataBundle return_values;
        SpPackedArray packed_array = readPixels();
        Std::insert(return_values.packed_arrays_, "data", std::move(packed_array));
        return return_values;
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
    SpFuncComponent->unregisterFunc("enqueue_copy");
    SpFuncComponent->unregisterFunc("read_pixels");

    // deallocate readback buffers
    readback_scratchpad_ready_ = false;
    readback_pending_.clear();
    readback_enqueue_index_ = 0;
    for (int i = 0; i < NUM_READBACK_BUFFERS; i++) {
        readback_buffers_.at(i).reset();
    }

    // deallocate memory
    if (shared_memory_region_) {
        SpFuncComponent->unregisterSharedMemoryView(shared_memory_view_);
        shared_memory_view_ = SpArraySharedMemoryView();
        shared_memory_region_ = nullptr;
    }

    // remove callbacks

    if (bPrintFrameTimeEveryFrame || bReadPixelsEveryFrame) {
        FCoreDelegates::OnBeginFrame.Remove(begin_frame_handle_);
        begin_frame_handle_.Reset();
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

    if (bReadPixelsEveryFrame) {
        scratchpad_packed_array_ = readPixels(); // assign to a scratchpad to make sure the call to readPixels() doesn't get optimized out
    }
}

SpPackedArray USpSceneCaptureComponent2D::readPixels()
{
    if (bUseDoubleBufferedReadback) {
        return readPixelsDoubleBuffered();
    }
    return readPixelsSingleBuffered();
}

void USpSceneCaptureComponent2D::enqueueCopy()
{
    SP_ASSERT(IsInitialized());
    SP_ASSERT(bUseDoubleBufferedReadback);
    SP_ASSERT(!readback_pending_.full());

    if (bReadPixelData) {
        FTextureRenderTargetResource* render_target_resource = TextureTarget->GameThread_GetRenderTargetResource();
        SP_ASSERT(render_target_resource);

        FRHIGPUTextureReadback* readback = readback_buffers_.at(readback_enqueue_index_).get();
        SP_ASSERT(readback);

        FRHIGPUTextureReadback* prev_readback = nullptr;
        if (!readback_pending_.empty()) {
            prev_readback = readback_buffers_.at(readback_pending_.front()).get();
        }

        SpArrayDataType channel_data_type = Unreal::getEnumValueAs<SpArrayDataType, ESpArrayDataType>(ChannelDataType);
        uint64_t num_bytes = Height*Width*NumChannelsPerPixel*SpArrayDataTypeUtils::getSizeOf(channel_data_type);
        int32_t bytes_per_pixel = NumChannelsPerPixel * SpArrayDataTypeUtils::getSizeOf(channel_data_type);
        int32_t row_bytes = Width * bytes_per_pixel;
        int32_t width = Width;
        int32_t height = Height;

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
        SP_ASSERT(scratchpad_ptr);

        std::atomic<bool>* scratchpad_ready = &readback_scratchpad_ready_;

        bool print_spin_wait_info = bPrintDoubleBufferedSpinWaitInfo;

        ENQUEUE_RENDER_COMMAND(SpEnqueueCopy)(
            [readback, render_target_resource, prev_readback, scratchpad_ptr, scratchpad_ready,
             num_bytes, bytes_per_pixel, row_bytes, width, height, print_spin_wait_info](FRHICommandListImmediate& RHICmdList) {

                if (prev_readback) {
                    int64_t spin_wait_iterations = 0;
                    while (!prev_readback->IsReady()) { spin_wait_iterations++; }
                    if (print_spin_wait_info && spin_wait_iterations > 0) {
                        SP_LOG("WARNING: Double-buffered readback waited for ", spin_wait_iterations, " spin-wait iterations.");
                    }

                    int32 row_pitch_in_pixels = 0;
                    void* src_ptr = prev_readback->Lock(row_pitch_in_pixels);
                    SP_ASSERT(src_ptr);
                    SP_ASSERT(row_pitch_in_pixels >= width);

                    int32_t src_row_pitch_bytes = row_pitch_in_pixels * bytes_per_pixel;

                    if (src_row_pitch_bytes == row_bytes) {
                        std::memcpy(scratchpad_ptr, src_ptr, num_bytes);
                    } else {
                        uint8_t* src = static_cast<uint8_t*>(src_ptr);
                        uint8_t* dst = static_cast<uint8_t*>(scratchpad_ptr);
                        for (int32_t y = 0; y < height; y++) {
                            std::memcpy(dst + y * row_bytes, src + y * src_row_pitch_bytes, row_bytes);
                        }
                    }

                    prev_readback->Unlock();
                    (*scratchpad_ready) = true;
                }

                readback->EnqueueCopy(RHICmdList, render_target_resource->GetRenderTargetTexture());
            });
    }

    readback_pending_.push_back(readback_enqueue_index_);
    readback_enqueue_index_ = (readback_enqueue_index_ + 1) % NUM_READBACK_BUFFERS;
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
    SP_ASSERT(IsInitialized());
    SP_ASSERT(bUseDoubleBufferedReadback);
    SP_ASSERT(!readback_pending_.empty());
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
        if (!readback_scratchpad_ready_) {
            SP_LOG("WARNING: Double-buffered readback scratchpad not ready, flushing rendering commands...");
            FlushRenderingCommands();
            SP_ASSERT(readback_scratchpad_ready_);
        }

        readback_scratchpad_ready_ = false;

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
        SP_ASSERT(scratchpad_ptr);

        std::memcpy(dest_ptr, scratchpad_ptr, num_bytes);
    }

    readback_pending_.pop_front();

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
