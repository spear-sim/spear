//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpComponents/SpSceneCaptureComponent2D.h"

#include <memory> // std::make_unique

#include <Components/SceneCaptureComponent2D.h>
#include <Materials/MaterialInstanceDynamic.h>
#include <Math/Color.h>
#include <UObject/UObjectGlobals.h> // NewObject

#include "SpCore/Assert.h"
#include "SpCore/Log.h"
#include "SpCore/SharedMemoryRegion.h"
#include "SpCore/SpFuncArray.h"
#include "SpCore/Unreal.h"

#include "SpComponents/SpFuncComponent.h"

USpSceneCaptureComponent2D::USpSceneCaptureComponent2D()
{
    SP_LOG_CURRENT_FUNCTION();

    SpFuncComponent = Unreal::createComponentInsideOwnerConstructor<USpFuncComponent>(this, "sp_func_component");
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
        SpFuncArrayDataType channel_data_type = Unreal::getEnumValueAs<SpFuncArrayDataType, ESpFuncArrayDataType>(ChannelDataType);
        int num_bytes = Height*Width*NumChannelsPerPixel*SpFuncArrayDataTypeUtils::getSizeOf(channel_data_type);
        shared_memory_region_ = std::make_unique<SharedMemoryRegion>(num_bytes);
        SP_ASSERT(shared_memory_region_);
        shared_memory_view_ = SpFuncSharedMemoryView(shared_memory_region_->getView(), SpFuncSharedMemoryUsageFlags::ReturnValue);
        SpFuncComponent->registerSharedMemoryView("smem:sp_scene_capture_component_2d", shared_memory_view_); // name needs to be unique per USpFuncComponent
    }

    SpFuncComponent->registerFunc("read_pixels", [this](SpFuncDataBundle& args) -> SpFuncDataBundle {

        SP_ASSERT(initialized_);

        SpFuncArrayDataType channel_data_type = Unreal::getEnumValueAs<SpFuncArrayDataType, ESpFuncArrayDataType>(ChannelDataType);

        SpFuncPackedArray packed_array;
        packed_array.shape_ = {Height, Width, NumChannelsPerPixel};
        packed_array.data_type_ = channel_data_type;

        void* dest_ptr = nullptr;
        
        if (bUseSharedMemory) {
            packed_array.view_ = shared_memory_view_.data_;
            packed_array.data_source_ = SpFuncArrayDataSource::Shared;
            packed_array.shared_memory_name_ = "smem:sp_scene_capture_component_2d";
            packed_array.shared_memory_usage_flags_ = shared_memory_view_.usage_flags_;
            dest_ptr = shared_memory_view_.data_;

        } else {
            int num_bytes = Height*Width*NumChannelsPerPixel*SpFuncArrayDataTypeUtils::getSizeOf(channel_data_type);
            packed_array.data_.resize(num_bytes);
            packed_array.view_ = packed_array.data_.data();
            packed_array.data_source_ = SpFuncArrayDataSource::Internal;
            dest_ptr = packed_array.data_.data();
        }
        SP_ASSERT(dest_ptr);

        if (bReadPixelData) {
            FTextureRenderTargetResource* texture_render_target_resource = TextureTarget->GameThread_GetRenderTargetResource();
            SP_ASSERT(texture_render_target_resource);

            // ReadPixelsPtr assumes 4 channels per pixel, 1 byte per channel
            if (NumChannelsPerPixel == 4 && SpFuncArrayDataTypeUtils::getSizeOf(channel_data_type) == 1) {
                texture_render_target_resource->ReadPixelsPtr(static_cast<FColor*>(dest_ptr));

            // ReadLinearColorPixelsPtr assumes 4 channels per pixel, 4 bytes per channel
            } else if (NumChannelsPerPixel == 4 && SpFuncArrayDataTypeUtils::getSizeOf(channel_data_type) == 4) {
                texture_render_target_resource->ReadLinearColorPixelsPtr(static_cast<FLinearColor*>(dest_ptr));

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
        shared_memory_view_ = SpFuncSharedMemoryView();
        shared_memory_region_ = nullptr;
    }

    if (Material) {
        SP_ASSERT(material_instance_dynamic_);
        PostProcessSettings.RemoveBlendable(material_instance_dynamic_);
    }

    TextureTarget = nullptr;

    SetVisibility(false);
}
