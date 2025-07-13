//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpComponents/SpSceneCaptureComponent2D.h"

#include <memory>  // std::align, std::make_unique, std::memcpy
#include <utility> // std::move

#include <Components/SceneCaptureComponent2D.h>
#include <Containers/Array.h>
#include <Materials/Material.h>
#include <Materials/MaterialInstanceDynamic.h>
#include <Math/Color.h>             // FColor, FLinearColor
#include <Math/Float16Color.h>
#include <RHIDefinitions.h>         // ERangeCompressionMode
#include <RHITypes.h>               // FReadSurfaceDataFlags
#include <TextureResource.h>        // FTextureRenderTargetResource
#include <UObject/UObjectGlobals.h> // NewObject

#include "SpCore/Assert.h"
#include "SpCore/Boost.h"
#include "SpCore/Log.h"
#include "SpCore/SharedMemory.h"
#include "SpCore/SpArray.h"
#include "SpCore/SpFuncComponent.h"
#include "SpCore/SpFuncDataBundle.h"
#include "SpCore/Unreal.h"

USpSceneCaptureComponent2D::USpSceneCaptureComponent2D()
{
    SP_LOG_CURRENT_FUNCTION();

    SpFuncComponent = Unreal::createSceneComponentInsideOwnerConstructor<USpFuncComponent>(this, "sp_func_component");
    SP_ASSERT(SpFuncComponent);

    // we want to be able to capture the scene even when the game is paused; note that we don't set
    // PrimaryActorTick.TickGroup because we don't want to interfere with when the base class component
    // gets ticked
    PrimaryComponentTick.bCanEverTick = true;
    PrimaryComponentTick.bTickEvenWhenPaused = true;
}

USpSceneCaptureComponent2D::~USpSceneCaptureComponent2D()
{
    SP_LOG_CURRENT_FUNCTION();
}

void USpSceneCaptureComponent2D::Initialize()
{
    if (initialized_) {
        return;
    }

    SpArrayDataType channel_data_type = Unreal::getEnumValueAs<SpArrayDataType, ESpArrayDataType>(ChannelDataType);
    int num_bytes = Height*Width*NumChannelsPerPixel*SpArrayDataTypeUtils::getSizeOf(channel_data_type);

    auto texture_render_target_2d = NewObject<UTextureRenderTarget2D>(this);
    SP_ASSERT(texture_render_target_2d);
    texture_render_target_2d->RenderTargetFormat = TextureRenderTargetFormat;
    texture_render_target_2d->ClearColor = FLinearColor(255, 0, 255); // bright pink
    texture_render_target_2d->InitAutoFormat(Width, Height);
    bool clear_render_target = true;
    texture_render_target_2d->UpdateResourceImmediate(clear_render_target);
    TextureTarget = texture_render_target_2d;
    SetVisibility(true);

    if (Material) {
        material_instance_dynamic_ = UMaterialInstanceDynamic::Create(Material, this);
        PostProcessSettings.AddBlendable(material_instance_dynamic_, 1.0f);
    }

    if (bUseSharedMemory) {
        SP_ASSERT(!shared_memory_region_);
        shared_memory_region_ = std::make_unique<SharedMemoryRegion>(num_bytes);
        SP_ASSERT(shared_memory_region_);
        shared_memory_view_ = SpArraySharedMemoryView(shared_memory_region_->getView(), SpArraySharedMemoryUsageFlags::ReturnValue);
        SpFuncComponent->registerSharedMemoryView("smem:sp_scene_capture_component_2d", shared_memory_view_); // name needs to be unique per USpFuncComponent
    }

    if (NumChannelsPerPixel == 4 && channel_data_type == SpArrayDataType::UInt8) {
        scratchpad_color_.Reserve(Height*Width);
        SP_ASSERT(Height*Width <= static_cast<int64_t>(scratchpad_color_.Max()), "Height == %d, Width == %d, Height*Width == %d, static_cast<int64_t>(scratchpad_color_.Max()) == %d", Height, Width, Height*Width, static_cast<int64_t>(scratchpad_color_.Max()));
        SP_ASSERT(num_bytes <= scratchpad_color_.GetAllocatedSize(), "num_bytes == %d, scratchpad_color_.GetAllocatedSize() == %d", num_bytes, scratchpad_color_.GetAllocatedSize());
    } else if (NumChannelsPerPixel == 4 && channel_data_type == SpArrayDataType::Float16) {
        scratchpad_float_16_color_.Reserve(Height*Width);
        SP_ASSERT(Height*Width <= static_cast<int64_t>(scratchpad_float_16_color_.Max()), "Height == %d, Width == %d, Height*Width == %d, static_cast<int64_t>(scratchpad_float_16_color_.Max()) == %d", Height, Width, Height*Width, static_cast<int64_t>(scratchpad_float_16_color_.Max()));
        SP_ASSERT(num_bytes <= scratchpad_float_16_color_.GetAllocatedSize(), "num_bytes == %d, scratchpad_float_16_color_.GetAllocatedSize() == %d", num_bytes, scratchpad_float_16_color_.GetAllocatedSize());
    } else if (NumChannelsPerPixel == 4 && channel_data_type == SpArrayDataType::Float32) {
        scratchpad_linear_color_.Reserve(Height*Width);
        SP_ASSERT(Height*Width <= static_cast<int64_t>(scratchpad_linear_color_.Max()), "Height == %d, Width == %d, Height*Width == %d, static_cast<int64_t>(scratchpad_linear_color_.Max()) == %d", Height, Width, Height*Width, static_cast<int64_t>(scratchpad_linear_color_.Max()));
        SP_ASSERT(num_bytes <= scratchpad_linear_color_.GetAllocatedSize(), "num_bytes == %d, scratchpad_linear_color_.GetAllocatedSize() == %d", num_bytes, scratchpad_linear_color_.GetAllocatedSize());
    } else {
        SP_ASSERT(false);
    }

    SpFuncComponent->registerFunc("read_pixels", [this, channel_data_type, num_bytes](SpFuncDataBundle& args) -> SpFuncDataBundle {

        SP_ASSERT(initialized_);

        SpPackedArray packed_array;
        packed_array.shape_ = {Height, Width, NumChannelsPerPixel};
        packed_array.data_type_ = channel_data_type;

        void* dest_ptr = nullptr;
        if (bUseSharedMemory) {
            packed_array.view_ = shared_memory_view_.data_;
            packed_array.data_source_ = SpArrayDataSource::Shared;
            packed_array.shared_memory_name_ = "smem:sp_scene_capture_component_2d";
            packed_array.shared_memory_usage_flags_ = shared_memory_view_.usage_flags_;
            dest_ptr = shared_memory_view_.data_;

        } else {
            packed_array.data_.reserve(num_bytes);
            packed_array.view_ = packed_array.data_.data();
            packed_array.data_source_ = SpArrayDataSource::Internal;

            if (NumChannelsPerPixel == 4 && channel_data_type == SpArrayDataType::UInt8) {
                dest_ptr = scratchpad_color_.GetData();
            } else if (NumChannelsPerPixel == 4 && channel_data_type == SpArrayDataType::Float16) {
                dest_ptr = scratchpad_float_16_color_.GetData();
            } else if (NumChannelsPerPixel == 4 && channel_data_type == SpArrayDataType::Float32) {
                SP_ASSERT(BOOST_OS_WINDOWS, "ERROR: NumChannelsPerPixel == 4 && channel_data_type == SpArrayDataType::Float32 is only supported on Windows.");
                dest_ptr = scratchpad_linear_color_.GetData();
            } else {
                SP_ASSERT(false);
            }
        }
        
        SP_ASSERT(dest_ptr);

        if (bReadPixelData) {
            FTextureRenderTargetResource* texture_render_target_resource = TextureTarget->GameThread_GetRenderTargetResource();
            SP_ASSERT(texture_render_target_resource);

            // ReadPixels assumes 4 channels per pixel, 1 uint8 per channel
            if (NumChannelsPerPixel == 4 && channel_data_type == SpArrayDataType::UInt8) {

                FColor* scratchpad_data_ptr = scratchpad_color_.GetData();
                UpdateArrayDataPtr(scratchpad_color_, dest_ptr, num_bytes);

                bool success = texture_render_target_resource->ReadPixels(scratchpad_color_);
                SP_ASSERT(success);

                UpdateArrayDataPtr(scratchpad_color_, scratchpad_data_ptr, num_bytes);

                if (!bUseSharedMemory) {
                    std::memcpy(packed_array.view_, dest_ptr, num_bytes); // need extra memcpy in this case because dest_ptr isn't necessarily aligned to FColor boundaries
                }

            // ReadFloat16Pixels assumes 4 channels per pixel, 1 float16 per channel
            } else if (NumChannelsPerPixel == 4 && channel_data_type == SpArrayDataType::Float16) {

                ERangeCompressionMode compression_mode = ERangeCompressionMode::RCM_MinMax;
                FReadSurfaceDataFlags read_surface_flags = FReadSurfaceDataFlags(compression_mode);
                read_surface_flags.SetLinearToGamma(false);

                FFloat16Color* scratchpad_data_ptr = scratchpad_float_16_color_.GetData();
                UpdateArrayDataPtr(scratchpad_float_16_color_, dest_ptr, num_bytes);

                bool success = texture_render_target_resource->ReadFloat16Pixels(scratchpad_float_16_color_, read_surface_flags);
                SP_ASSERT(success);

                UpdateArrayDataPtr(scratchpad_float_16_color_, scratchpad_data_ptr, num_bytes);

                if (!bUseSharedMemory) {
                    std::memcpy(packed_array.view_, dest_ptr, num_bytes); // need extra memcpy in this case because dest_ptr isn't necessarily aligned to FFloat16Color boundaries
                }

            // ReadLinearColorPixels assumes 4 channels per pixel, 1 float32 per channel
            } else if (NumChannelsPerPixel == 4 && channel_data_type == SpArrayDataType::Float32) {
                SP_ASSERT(BOOST_OS_WINDOWS, "ERROR: NumChannelsPerPixel == 4 && channel_data_type == SpArrayDataType::Float32 is only supported on Windows.");

                ERangeCompressionMode compression_mode = ERangeCompressionMode::RCM_MinMax;
                FReadSurfaceDataFlags read_surface_flags = FReadSurfaceDataFlags(compression_mode);
                read_surface_flags.SetLinearToGamma(false);

                FLinearColor* scratchpad_data_ptr = scratchpad_linear_color_.GetData();
                UpdateArrayDataPtr(scratchpad_linear_color_, dest_ptr, num_bytes);

                bool success = texture_render_target_resource->ReadLinearColorPixels(scratchpad_linear_color_, read_surface_flags);
                SP_ASSERT(success);

                UpdateArrayDataPtr(scratchpad_linear_color_, scratchpad_data_ptr, num_bytes);

                if (!bUseSharedMemory) {
                    std::memcpy(packed_array.view_, dest_ptr, num_bytes); // need extra memcpy in this case because dest_ptr isn't necessarily aligned to FLinearColor boundaries
                }

            } else {
                SP_ASSERT(false);
            }
        }

        SpFuncDataBundle return_values;
        return_values.packed_arrays_ = {{"data", std::move(packed_array)}};
        return return_values;
    });

    initialized_ = true;
}

void USpSceneCaptureComponent2D::Terminate()
{
    if (!initialized_) {
        return;
    }

    initialized_ = false;

    SpFuncComponent->unregisterFunc("read_pixels");

    if (bUseSharedMemory) {
        SP_ASSERT(shared_memory_region_);
        SpFuncComponent->unregisterSharedMemoryView("smem:sp_scene_capture_component_2d");
        shared_memory_view_ = SpArraySharedMemoryView();
        shared_memory_region_ = nullptr;
    }

    if (Material) {
        SP_ASSERT(material_instance_dynamic_);
        PostProcessSettings.RemoveBlendable(material_instance_dynamic_);
    }

    TextureTarget = nullptr;

    SetVisibility(false);
}

bool USpSceneCaptureComponent2D::IsInitialized()
{
    return initialized_;
}
