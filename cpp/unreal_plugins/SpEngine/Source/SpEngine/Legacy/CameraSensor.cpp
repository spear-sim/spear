//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpEngine/Legacy/CameraSensor.h"

#include <stdint.h> // uint8_t

#include <limits>  // std::numeric_limits
#include <map>
#include <string>
#include <utility> // std::move
#include <vector>

#include <Camera/CameraComponent.h>
#include <Components/SceneCaptureComponent2D.h>
#include <Engine/TextureRenderTarget2D.h> // ETextureRenderTargetFormat
#include <GameFramework/Actor.h>
#include <Materials/MaterialInstanceDynamic.h>
#include <UObject/UObjectGlobals.h>       // LoadObject, NewObject

#include "SpCore/ArrayDesc.h" // DataType
#include "SpCore/Assert.h"
#include "SpCore/Boost.h"
#include "SpCore/Config.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"

struct FColor;
struct FLinearColor;

// Unfortunately, Unreal's ReadPixels functions all assume 4 channels of output. For greater efficiency, we could
// potentially combine depth and/or world-space position and/or normal data into a single rendering pass. Similarly,
// we could also combine segmentation data and instance data into a single pass.
const std::map<std::string, int> RENDER_PASS_NUM_CHANNELS = {
    {"depth",        4},
    {"final_color",  4},
    {"normal",       4},
    {"segmentation", 4}};

const std::map<std::string, int> RENDER_PASS_NUM_BYTES_PER_CHANNEL = {
    {"depth",        4},
    {"final_color",  1},
    {"normal",       4},
    {"segmentation", 1}};

const std::map<std::string, ETextureRenderTargetFormat> RENDER_PASS_TEXTURE_RENDER_TARGET_FORMAT = {
    {"depth",        ETextureRenderTargetFormat::RTF_RGBA32f},
    {"final_color",  ETextureRenderTargetFormat::RTF_RGBA8_SRGB},
    {"normal",       ETextureRenderTargetFormat::RTF_RGBA32f},
    {"segmentation", ETextureRenderTargetFormat::RTF_RGBA8}};

const std::map<std::string, std::string> RENDER_PASS_MATERIAL = {
    {"depth",        "/SpEngine/Materials/PPM_Depth.PPM_Depth"},
    {"normal",       "/SpEngine/Materials/PPM_Normal.PPM_Normal"},
    {"segmentation", "/SpEngine/Materials/PPM_Segmentation.PPM_Segmentation"}};

const std::map<std::string, double> RENDER_PASS_LOW = {
    {"depth",        0.0},
    {"final_color",  0.0},
    {"normal",       std::numeric_limits<double>::lowest()}, // Unreal can return normals that are not unit length
    {"segmentation", 0.0}};

const std::map<std::string, double> RENDER_PASS_HIGH = {
    {"depth",        std::numeric_limits<double>::max()},
    {"final_color",  255.0},
    {"normal",       std::numeric_limits<double>::max()}, // Unreal can return normals that are not unit length
    {"segmentation", 255.0}};

const std::map<std::string, DataType> RENDER_PASS_CHANNEL_DATATYPE = {
    {"depth",        DataType::Float32},
    {"final_color",  DataType::UInteger8},
    {"normal",       DataType::Float32},
    {"segmentation", DataType::UInteger8}};

CameraSensor::CameraSensor(
    UCameraComponent* camera_component, const std::vector<std::string>& render_pass_names, unsigned int width, unsigned int height, float fov)
{
    SP_ASSERT(camera_component);

    actor_ = camera_component->GetWorld()->SpawnActor<AActor>();
    SP_ASSERT(actor_);
    
    for (auto& render_pass_name : render_pass_names) {
        RenderPassDesc render_pass_desc;

        render_pass_desc.width_ = width;
        render_pass_desc.height_ = height;
        render_pass_desc.num_bytes_ = height * width * RENDER_PASS_NUM_CHANNELS.at(render_pass_name) * RENDER_PASS_NUM_BYTES_PER_CHANNEL.at(render_pass_name);

        // create TextureRenderTarget2D
        auto texture_render_target_2d = NewObject<UTextureRenderTarget2D>(actor_, Unreal::toFName("texture_render_target_2d_" + render_pass_name));
        SP_ASSERT(texture_render_target_2d);

        bool clear_render_target = true;
        texture_render_target_2d->RenderTargetFormat = RENDER_PASS_TEXTURE_RENDER_TARGET_FORMAT.at(render_pass_name);
        texture_render_target_2d->InitAutoFormat(width, height);
        texture_render_target_2d->UpdateResourceImmediate(clear_render_target);

        // create SceneCaptureComponent2D
        render_pass_desc.scene_capture_component_2d_ = 
            Unreal::createComponentOutsideOwnerConstructor<USceneCaptureComponent2D>(actor_, camera_component, "scene_capture_component_2d_" + render_pass_name);
        SP_ASSERT(render_pass_desc.scene_capture_component_2d_);
        render_pass_desc.scene_capture_component_2d_->TextureTarget = texture_render_target_2d;
        render_pass_desc.scene_capture_component_2d_->FOVAngle = fov;
        render_pass_desc.scene_capture_component_2d_->CaptureSource = ESceneCaptureSource::SCS_FinalToneCurveHDR;
        render_pass_desc.scene_capture_component_2d_->SetVisibility(true);

        if (render_pass_name == "final_color") {
            // need to override these settings to obtain the same rendering quality as in a default game viewport
            render_pass_desc.scene_capture_component_2d_->PostProcessSettings.bOverride_DynamicGlobalIlluminationMethod = true;
            render_pass_desc.scene_capture_component_2d_->PostProcessSettings.DynamicGlobalIlluminationMethod           = EDynamicGlobalIlluminationMethod::Lumen;
            render_pass_desc.scene_capture_component_2d_->PostProcessSettings.bOverride_ReflectionMethod = true;
            render_pass_desc.scene_capture_component_2d_->PostProcessSettings.ReflectionMethod           = EReflectionMethod::Lumen;
            render_pass_desc.scene_capture_component_2d_->PostProcessSettings.bOverride_LumenSurfaceCacheResolution = true;
            render_pass_desc.scene_capture_component_2d_->PostProcessSettings.LumenSurfaceCacheResolution           = 1.0f;
        } else {
            // TODO (MR): turn off as many rendering features as possible for efficiency
            auto material = LoadObject<UMaterial>(nullptr, *Unreal::toFString(RENDER_PASS_MATERIAL.at(render_pass_name)));
            SP_ASSERT(material);
            render_pass_desc.scene_capture_component_2d_->PostProcessSettings.AddBlendable(UMaterialInstanceDynamic::Create(material, actor_), 1.0f);
        }

        // create shared_memory_object
        if (Config::get<bool>("SP_ENGINE.LEGACY.CAMERA_SENSOR.USE_SHARED_MEMORY")) {
            render_pass_desc.shared_memory_name_ = "camera." + render_pass_name;

            #if BOOST_OS_WINDOWS
                render_pass_desc.shared_memory_id_ = render_pass_desc.shared_memory_name_; // don't use leading slash on Windows
                boost::interprocess::windows_shared_memory windows_shared_memory(
                    boost::interprocess::create_only,
                    render_pass_desc.shared_memory_id_.c_str(),
                    boost::interprocess::read_write,
                    render_pass_desc.num_bytes_);
                render_pass_desc.shared_memory_mapped_region_ = boost::interprocess::mapped_region(windows_shared_memory, boost::interprocess::read_write);
            #elif BOOST_OS_MACOS || BOOST_OS_LINUX
                render_pass_desc.shared_memory_id_ = "/" + render_pass_desc.shared_memory_name_; // use leading slash on macOS and Linux
                boost::interprocess::shared_memory_object::remove(render_pass_desc.shared_memory_id_.c_str());
                boost::interprocess::shared_memory_object shared_memory_object(
                    boost::interprocess::create_only,
                    render_pass_desc.shared_memory_id_.c_str(),
                    boost::interprocess::read_write);
                shared_memory_object.truncate(render_pass_desc.num_bytes_);
                render_pass_desc.shared_memory_mapped_region_ = boost::interprocess::mapped_region(shared_memory_object, boost::interprocess::read_write);
            #else
                #error
            #endif
        }

        // update render_pass_descs_
        Std::insert(render_pass_descs_, render_pass_name, std::move(render_pass_desc));
    }
}

CameraSensor::~CameraSensor()
{
    for (auto& [render_pass_name, render_pass_desc] : render_pass_descs_) {
        if (Config::get<bool>("SIMULATION_CONTROLLER.CAMERA_SENSOR.USE_SHARED_MEMORY")) {
            #if BOOST_OS_MACOS || BOOST_OS_LINUX
                boost::interprocess::shared_memory_object::remove(render_pass_desc.shared_memory_id_.c_str());
            #endif
        }
    }

    SP_ASSERT(actor_);
    actor_->Destroy();
    actor_ = nullptr;
}

std::map<std::string, ArrayDesc> CameraSensor::getObservationSpace() const
{
    std::map<std::string, ArrayDesc> observation_space;

    for (auto& [render_pass_name, render_pass_desc] : render_pass_descs_) {
        ArrayDesc array_desc;
        array_desc.low_ = RENDER_PASS_LOW.at(render_pass_name);
        array_desc.high_ = RENDER_PASS_HIGH.at(render_pass_name);
        array_desc.shape_ = {render_pass_desc.height_, render_pass_desc.width_, RENDER_PASS_NUM_CHANNELS.at(render_pass_name)};
        array_desc.datatype_ = RENDER_PASS_CHANNEL_DATATYPE.at(render_pass_name);
        array_desc.use_shared_memory_ = Config::get<bool>("SP_ENGINE.LEGACY.CAMERA_SENSOR.USE_SHARED_MEMORY");
        array_desc.shared_memory_name_ = render_pass_desc.shared_memory_name_;
        Std::insert(observation_space, "camera." + render_pass_name, std::move(array_desc));
    }

    return observation_space;
}

std::map<std::string, std::vector<uint8_t>> CameraSensor::getObservation() const
{
    std::map<std::string, std::vector<uint8_t>> observation;

    for (auto& [render_pass_name, render_pass_desc] : render_pass_descs_) {

        void* dest_ptr = nullptr;
        if (Config::get<bool>("SP_ENGINE.LEGACY.CAMERA_SENSOR.USE_SHARED_MEMORY")) {
            dest_ptr = render_pass_desc.shared_memory_mapped_region_.get_address();
        } else {
            Std::insert(observation, "camera." + render_pass_name, {});
            observation.at("camera." + render_pass_name).resize(render_pass_desc.num_bytes_);
            dest_ptr = observation.at("camera." + render_pass_name).data();
        }
        SP_ASSERT(dest_ptr);

        if (Config::get<bool>("SP_ENGINE.LEGACY.CAMERA_SENSOR.READ_SURFACE_DATA")) {
            FTextureRenderTargetResource* texture_render_target_resource =
                render_pass_desc.scene_capture_component_2d_->TextureTarget->GameThread_GetRenderTargetResource();
            SP_ASSERT(texture_render_target_resource);

            if (render_pass_name == "final_color" || render_pass_name == "segmentation") {
                // ReadPixelsPtr assumes 4 channels per pixel, 1 byte per channel, so it can be used to read
                // the following ETextureRenderTargetFormat formats:
                //     final_color:  RTF_RGBA8
                //     segmentation: RTF_RGBA8_SRGB
                texture_render_target_resource->ReadPixelsPtr(static_cast<FColor*>(dest_ptr));
            } else if (render_pass_name == "depth" || render_pass_name == "normal") {
                // ReadLinearColorPixelsPtr assumes 4 channels per pixel, 4 bytes per channel, so it can be used
                // to read the following ETextureRenderTargetFormat formats:
                //     depth:  RTF_RGBA32f
                //     normal: RTF_RGBA32f
                texture_render_target_resource->ReadLinearColorPixelsPtr(static_cast<FLinearColor*>(dest_ptr));
            } else {
                SP_ASSERT(false);
            }
        }
    }

    return observation;
}
