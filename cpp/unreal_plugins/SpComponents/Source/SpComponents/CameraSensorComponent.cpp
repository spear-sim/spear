//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpComponents/CameraSensorComponent.h"

#include <stdint.h> // uint8_t

#include <limits> // std::numeric_limits
#include <map>
#include <string>
#include <utility> // std::move
#include <vector>

#include <Camera/CameraComponent.h>
#include <Components/SceneCaptureComponent2D.h>
#include <Engine/TextureRenderTarget2D.h> // ETextureRenderTargetFormat
#include <GameFramework/Actor.h>
#include <Materials/MaterialInstanceDynamic.h>
#include <UObject/UObjectGlobals.h> // LoadObject, NewObject

#include "SpCore/ArrayDesc.h"
#include "SpCore/Assert.h"
#include "SpCore/Boost.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"

struct FColor;
struct FLinearColor;

// Unfortunately, Unreal's ReadPixels functions all assume 4 channels of output. For greater efficiency, we could
// potentially combine depth and/or world-space position and/or normal data into a single rendering pass. Similarly,
// we could also combine segmentation data and instance data into a single pass.
const std::map<std::string, int> RENDER_PASS_NUM_CHANNELS = {
    {"depth", 4},
    {"final_color", 4},
    {"normal", 4},
    {"segmentation", 4}
};

const std::map<std::string, int> RENDER_PASS_NUM_BYTES_PER_CHANNEL = {
    {"depth", 4},
    {"final_color", 1},
    {"normal", 4},
    {"segmentation", 1}
};

const std::map<std::string, ETextureRenderTargetFormat> RENDER_PASS_TEXTURE_RENDER_TARGET_FORMAT = {
    {"depth", ETextureRenderTargetFormat::RTF_RGBA32f},
    {"final_color", ETextureRenderTargetFormat::RTF_RGBA8_SRGB},
    {"normal", ETextureRenderTargetFormat::RTF_RGBA32f},
    {"segmentation", ETextureRenderTargetFormat::RTF_RGBA8}
};

const std::map<std::string, std::string> RENDER_PASS_MATERIAL = {
    {"depth", "/SpServices/Materials/PPM_Depth.PPM_Depth"},
    {"normal", "/SpServices/Materials/PPM_Normal.PPM_Normal"},
    {"segmentation", "/SpServices/Materials/PPM_Segmentation.PPM_Segmentation"}
};

const std::map<std::string, double> RENDER_PASS_LOW = {
    {"depth", 0.0},
    {"final_color", 0.0},
    {"normal", std::numeric_limits<double>::lowest()}, // Unreal can return normals that are not unit length
    {"segmentation", 0.0}
};

const std::map<std::string, double> RENDER_PASS_HIGH = {
    {"depth", std::numeric_limits<double>::max()},
    {"final_color", 255.0},
    {"normal", std::numeric_limits<double>::max()}, // Unreal can return normals that are not unit length
    {"segmentation", 255.0}
};

const std::map<std::string, DataType> RENDER_PASS_CHANNEL_DATATYPE = {
    {"depth", DataType::Float32},
    {"final_color", DataType::UInteger8},
    {"normal", DataType::Float32},
    {"segmentation", DataType::UInteger8}
};

UCameraSensorComponent::UCameraSensorComponent()
{
    cpp_component_ = Unreal::createComponentInsideOwnerConstructor<UCppFuncComponent>(this, "cpp_component");

    cpp_component_->registerFunc("camera_sensor.render", [this](CppFuncPackage& args) -> CppFuncPackage {
        CppFuncView<uint8_t> render_pass_name_view("render_pass_name");
        CppFuncUtils::setViewsFromItems({render_pass_name_view.getPtr()}, args.items_);

        std::string render_pass_name = std::string(reinterpret_cast<char const*>(render_pass_name_view.getView().data()));
        render_pass_name             = render_pass_name.substr(0, render_pass_name_view.getView().size());
        SP_ASSERT(Std::containsKey(render_pass_descs_, render_pass_name));

        CppFuncData<uint8_t> shared_data("shared_data");
        RenderPassDesc& render_pass_desc = render_pass_descs_[render_pass_name];
        auto& view                       = render_pass_desc.shared_memory_region_->getView();
        shared_data.setData(render_pass_name, view, render_pass_desc.num_bytes_);

        FTextureRenderTargetResource* texture_render_target_resource = render_pass_desc.scene_capture_component_2d_->TextureTarget->GameThread_GetRenderTargetResource();
        SP_ASSERT(texture_render_target_resource);

        if (render_pass_name == "final_color" || render_pass_name == "segmentation") {
            // ReadPixelsPtr assumes 4 channels per pixel, 1 byte per channel, so it can be used to read
            // the following ETextureRenderTargetFormat formats:
            //     final_color:  RTF_RGBA8
            //     segmentation: RTF_RGBA8_SRGB
            texture_render_target_resource->ReadPixelsPtr(reinterpret_cast<FColor*>(view.data_));
        } else if (render_pass_name == "depth" || render_pass_name == "normal") {
            texture_render_target_resource->ReadLinearColorPixelsPtr(static_cast<FLinearColor*>(view.data_));
        } else {
            SP_ASSERT(false);
        }

        // Return CppFuncData objects.
        CppFuncPackage return_values;
        return_values.items_ = CppFuncUtils::moveDataToItems({shared_data.getPtr()});
        return return_values;
    });
}

UCameraSensorComponent::~UCameraSensorComponent()
{
}

void UCameraSensorComponent::setup(
    UCameraComponent* camera_component, TArray<FString> render_pass_names, int camera_width, int camera_height, float camera_fov)
{
    camera_component_ = camera_component;

    for (auto& render_pass_name_fstring : render_pass_names) {
        std::string render_pass_name = Unreal::toStdString(render_pass_name_fstring);
        RenderPassDesc render_pass_desc;

        render_pass_desc.width_     = camera_width;
        render_pass_desc.height_ = camera_height;
        render_pass_desc.num_bytes_ =
            camera_height * camera_width * RENDER_PASS_NUM_CHANNELS.at(render_pass_name) * RENDER_PASS_NUM_BYTES_PER_CHANNEL.at(render_pass_name);

        // create TextureRenderTarget2D
        auto texture_render_target_2d = NewObject<UTextureRenderTarget2D>(GetOwner(), Unreal::toFName("texture_render_target_2d_" + render_pass_name));
        SP_ASSERT(texture_render_target_2d);

        bool clear_render_target                     = true;
        texture_render_target_2d->RenderTargetFormat = RENDER_PASS_TEXTURE_RENDER_TARGET_FORMAT.at(render_pass_name);
        texture_render_target_2d->InitAutoFormat(camera_width, camera_height);
        texture_render_target_2d->UpdateResourceImmediate(clear_render_target);

        // create SceneCaptureComponent2D
        render_pass_desc.scene_capture_component_2d_ = Unreal::createComponentOutsideOwnerConstructor<USceneCaptureComponent2D>(
            GetOwner(), camera_component, "scene_capture_component_2d_" + render_pass_name);
        SP_ASSERT(render_pass_desc.scene_capture_component_2d_);
        render_pass_desc.scene_capture_component_2d_->TextureTarget = texture_render_target_2d;
        render_pass_desc.scene_capture_component_2d_->FOVAngle      = camera_fov;
        render_pass_desc.scene_capture_component_2d_->CaptureSource = ESceneCaptureSource::SCS_FinalToneCurveHDR;
        render_pass_desc.scene_capture_component_2d_->SetVisibility(true);

        if (render_pass_name == "final_color") {
            // need to override these settings to obtain the same rendering quality as in a default game viewport
            render_pass_desc.scene_capture_component_2d_->PostProcessSettings.bOverride_DynamicGlobalIlluminationMethod = true;
            render_pass_desc.scene_capture_component_2d_->PostProcessSettings.DynamicGlobalIlluminationMethod       = EDynamicGlobalIlluminationMethod::Lumen;
            render_pass_desc.scene_capture_component_2d_->PostProcessSettings.bOverride_ReflectionMethod            = true;
            render_pass_desc.scene_capture_component_2d_->PostProcessSettings.ReflectionMethod                      = EReflectionMethod::Lumen;
            render_pass_desc.scene_capture_component_2d_->PostProcessSettings.bOverride_LumenSurfaceCacheResolution = true;
            render_pass_desc.scene_capture_component_2d_->PostProcessSettings.LumenSurfaceCacheResolution           = 1.0f;
        } else {
            // TODO (MR): turn off as many rendering features as possible for efficiency
            auto material = LoadObject<UMaterial>(nullptr, *Unreal::toFString(RENDER_PASS_MATERIAL.at(render_pass_name)));
            SP_ASSERT(material);
            render_pass_desc.scene_capture_component_2d_->PostProcessSettings.AddBlendable(UMaterialInstanceDynamic::Create(material, GetOwner()), 1.0f);
        }

        // create shared_memory_object
        render_pass_desc.shared_memory_region_ = std::make_unique<SharedMemoryRegion>(render_pass_desc.num_bytes_);
        SP_ASSERT(render_pass_desc.shared_memory_region_);

        CppFuncSharedMemoryView shared_memory_view(render_pass_desc.shared_memory_region_->getView(), CppFuncSharedMemoryUsageFlags::ReturnValue);
        cpp_component_->registerSharedMemoryView(render_pass_name, shared_memory_view);

        // update render_pass_descs_
        Std::insert(render_pass_descs_, render_pass_name, std::move(render_pass_desc));
    }
}

void UCameraSensorComponent::setup0(UCameraComponent* camera_component, TArray<FString> render_pass_names)
{
    int width  = 320;
    int height = 240;
    float fov = 90;
    setup(camera_component, render_pass_names, width, height, fov);
}

TArray<FColor> UCameraSensorComponent::getObservation(FString render_pass_name) const
{
    TArray<FColor> observation;

    std::string render_pass_name0 = Unreal::toStdString(render_pass_name);

    const RenderPassDesc& render_pass_desc = render_pass_descs_.at(render_pass_name0);

    FTextureRenderTargetResource* texture_render_target_resource =
        render_pass_desc.scene_capture_component_2d_->TextureTarget->GameThread_GetRenderTargetResource();
    SP_ASSERT(texture_render_target_resource);

    if (render_pass_name == "final_color" || render_pass_name == "segmentation") {
        // ReadPixelsPtr assumes 4 channels per pixel, 1 byte per channel, so it can be used to read
        // the following ETextureRenderTargetFormat formats:
        //     final_color:  RTF_RGBA8
        //     segmentation: RTF_RGBA8_SRGB
        observation.SetNum(render_pass_desc.width_ * render_pass_desc.height_);
        texture_render_target_resource->ReadPixelsPtr(observation.GetData());
    } else if (render_pass_name == "depth" || render_pass_name == "normal") {
        SP_ASSERT(false);
    } else {
        SP_ASSERT(false);
    }

    return observation;
}

TArray<uint8> UCameraSensorComponent::getObservationV2(FString render_pass_name) const
{
    TArray<FColor> observation = getObservation(render_pass_name);
    TArray<uint8> obs;
    obs.SetNum(observation.Num() * 4, false);
    FMemory::Memcpy(obs.GetData(), observation.GetData(), observation.Num() * 4);
    return obs;
}
