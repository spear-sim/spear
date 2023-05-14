//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SimulationController/CameraSensor.h"

#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include <boost/predef.h>

#include <Camera/CameraComponent.h>
#include <Components/SceneCaptureComponent2D.h>
#include <Containers/Array.h>
#include <Engine/EngineTypes.h>
#include <Engine/TextureRenderTarget2D.h>
#include <Engine/World.h>
#include <GameFramework/Actor.h>
#include <Materials/MaterialInstanceDynamic.h>
#include <TextureResource.h>
#include <UObject/UObjectGlobals.h>

#include "CoreUtils/ArrayDesc.h"
#include "CoreUtils/Assert.h"
#include "CoreUtils/Config.h"
#include "CoreUtils/Std.h"
#include "CoreUtils/Unreal.h"
#include "SimulationController/BoostInterprocess.h"

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
    {"depth",        "/SimulationController/Materials/M_Depth.M_Depth"},
    {"normal",       "/SimulationController/Materials/M_Normal.M_Normal"},
    {"segmentation", "/SimulationController/Materials/M_Segmentation.M_Segmentation"}};

const std::map<std::string, float> RENDER_PASS_LOW = {
    {"depth",        0.0f},
    {"final_color",  0.0f},
    {"normal",       std::numeric_limits<float>::lowest()}, // Unreal can return normals that are not unit length
    {"segmentation", 0.0f}};

const std::map<std::string, float> RENDER_PASS_HIGH = {
    {"depth",        std::numeric_limits<float>::max()},
    {"final_color",  255.0f},
    {"normal",       std::numeric_limits<float>::max()}, // Unreal can return normals that are not unit length
    {"segmentation", 255.0f}};

const std::map<std::string, DataType> RENDER_PASS_CHANNEL_DATATYPE = {
    {"depth",        DataType::Float32},
    {"final_color",  DataType::UInteger8},
    {"normal",       DataType::Float32},
    {"segmentation", DataType::UInteger8}};

CameraSensor::CameraSensor(
    UCameraComponent* camera_component, const std::vector<std::string>& render_pass_names, unsigned int width, unsigned int height, float fov)
{
    SP_ASSERT(camera_component);

    parent_actor_ = camera_component->GetWorld()->SpawnActor<AActor>();
    SP_ASSERT(parent_actor_);
    
    for (auto& render_pass_name : render_pass_names) {
        RenderPassDesc render_pass_desc;

        render_pass_desc.width_ = width;
        render_pass_desc.height_ = height;
        render_pass_desc.num_bytes_ = height * width * RENDER_PASS_NUM_CHANNELS.at(render_pass_name) * RENDER_PASS_NUM_BYTES_PER_CHANNEL.at(render_pass_name);

        // create TextureRenderTarget2D
        render_pass_desc.texture_render_target_ = NewObject<UTextureRenderTarget2D>(parent_actor_);
        SP_ASSERT(render_pass_desc.texture_render_target_);

        bool clear_render_target = true;
        render_pass_desc.texture_render_target_->RenderTargetFormat = RENDER_PASS_TEXTURE_RENDER_TARGET_FORMAT.at(render_pass_name);
        render_pass_desc.texture_render_target_->InitAutoFormat(width, height);
        render_pass_desc.texture_render_target_->UpdateResourceImmediate(clear_render_target);
        SP_ASSERT(render_pass_desc.texture_render_target_->GameThread_GetRenderTargetResource());

        // create SceneCaptureComponent2D
        render_pass_desc.scene_capture_component_ = NewObject<USceneCaptureComponent2D>(parent_actor_);
        SP_ASSERT(render_pass_desc.scene_capture_component_);

        render_pass_desc.scene_capture_component_->TextureTarget = render_pass_desc.texture_render_target_;
        render_pass_desc.scene_capture_component_->FOVAngle = fov;
        render_pass_desc.scene_capture_component_->CaptureSource = ESceneCaptureSource::SCS_FinalToneCurveHDR;

        if (render_pass_name == "final_color") {
            // need to override these settings to obtain the same rendering quality as in a default game viewport
            render_pass_desc.scene_capture_component_->PostProcessSettings.bOverride_DynamicGlobalIlluminationMethod = true;
            render_pass_desc.scene_capture_component_->PostProcessSettings.DynamicGlobalIlluminationMethod           = EDynamicGlobalIlluminationMethod::Lumen;
            render_pass_desc.scene_capture_component_->PostProcessSettings.bOverride_ReflectionMethod = true;
            render_pass_desc.scene_capture_component_->PostProcessSettings.ReflectionMethod           = EReflectionMethod::Lumen;
            render_pass_desc.scene_capture_component_->PostProcessSettings.bOverride_LumenSurfaceCacheResolution = true;
            render_pass_desc.scene_capture_component_->PostProcessSettings.LumenSurfaceCacheResolution           = 1.0f;
        } else {
            // TODO: turn off as many rendering features as possible for efficiency
            UMaterial* material = LoadObject<UMaterial>(nullptr, *Unreal::toFString(RENDER_PASS_MATERIAL.at(render_pass_name)));
            SP_ASSERT(material);
            render_pass_desc.scene_capture_component_->PostProcessSettings.AddBlendable(UMaterialInstanceDynamic::Create(material, parent_actor_), 1.0f);
        }

        render_pass_desc.scene_capture_component_->AttachToComponent(camera_component, FAttachmentTransformRules::SnapToTargetNotIncludingScale);
        render_pass_desc.scene_capture_component_->SetVisibility(true);
        render_pass_desc.scene_capture_component_->RegisterComponent();

        // create shared_memory_object
        if (Config::get<bool>("SIMULATION_CONTROLLER.CAMERA_SENSOR.USE_SHARED_MEMORY")) {
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
        render_pass_descs_[render_pass_name] = std::move(render_pass_desc);
    }
}

CameraSensor::~CameraSensor()
{
    for (auto& render_pass_desc : render_pass_descs_) {
        if (Config::get<bool>("SIMULATION_CONTROLLER.CAMERA_SENSOR.USE_SHARED_MEMORY")) {
            #if BOOST_OS_MACOS || BOOST_OS_LINUX
                boost::interprocess::shared_memory_object::remove(render_pass_desc.second.shared_memory_id_.c_str());
            #endif
        }

        SP_ASSERT(render_pass_desc.second.scene_capture_component_);
        render_pass_desc.second.scene_capture_component_->MarkAsGarbage();
        render_pass_desc.second.scene_capture_component_ = nullptr;

        SP_ASSERT(render_pass_desc.second.texture_render_target_);
        render_pass_desc.second.texture_render_target_->MarkAsGarbage();
        render_pass_desc.second.texture_render_target_ = nullptr;
    }
    render_pass_descs_.clear();

    SP_ASSERT(parent_actor_);
    parent_actor_->Destroy();
    parent_actor_ = nullptr;
}

std::map<std::string, ArrayDesc> CameraSensor::getObservationSpace(const std::vector<std::string>& observation_components) const
{
    std::map<std::string, ArrayDesc> observation_space;

    if (Std::contains(observation_components, "camera")) {
        for (auto& render_pass_desc : render_pass_descs_) {
            ArrayDesc array_desc;
            array_desc.low_ = RENDER_PASS_LOW.at(render_pass_desc.first);
            array_desc.high_ = RENDER_PASS_HIGH.at(render_pass_desc.first);
            array_desc.shape_ = {render_pass_desc.second.height_, render_pass_desc.second.width_, RENDER_PASS_NUM_CHANNELS.at(render_pass_desc.first)};
            array_desc.datatype_ = RENDER_PASS_CHANNEL_DATATYPE.at(render_pass_desc.first);
            array_desc.use_shared_memory_ = Config::get<bool>("SIMULATION_CONTROLLER.CAMERA_SENSOR.USE_SHARED_MEMORY");
            array_desc.shared_memory_name_ = render_pass_desc.second.shared_memory_name_;
            observation_space["camera." + render_pass_desc.first] = std::move(array_desc);
        }
    }

    return observation_space;
}

std::map<std::string, std::vector<uint8_t>> CameraSensor::getObservation(const std::vector<std::string>& observation_components) const
{
    std::map<std::string, std::vector<uint8_t>> observation;

    if (Std::contains(observation_components, "camera")) {
        for (auto& render_pass_desc : render_pass_descs_) {

            void* dest_ptr = nullptr;
            if (Config::get<bool>("SIMULATION_CONTROLLER.CAMERA_SENSOR.USE_SHARED_MEMORY")) {
                dest_ptr = render_pass_desc.second.shared_memory_mapped_region_.get_address();
            } else {
                observation["camera." + render_pass_desc.first] = {};
                observation.at("camera." + render_pass_desc.first).resize(render_pass_desc.second.num_bytes_);
                dest_ptr = observation.at("camera." + render_pass_desc.first).data();
            }
            SP_ASSERT(dest_ptr);

            if (Config::get<bool>("SIMULATION_CONTROLLER.CAMERA_SENSOR.READ_SURFACE_DATA")) {
                FTextureRenderTargetResource* texture_render_target_resource =
                    render_pass_desc.second.scene_capture_component_->TextureTarget->GameThread_GetRenderTargetResource();
                SP_ASSERT(texture_render_target_resource);

                if (render_pass_desc.first == "final_color" || render_pass_desc.first == "segmentation") {
                    // ReadPixelsPtr assumes 4 channels per pixel, 1 byte per channel, so it can be used to read
                    // the following ETextureRenderTargetFormat formats:
                    //     final_color:  RTF_RGBA8
                    //     segmentation: RTF_RGBA8_SRGB
                    texture_render_target_resource->ReadPixelsPtr(static_cast<FColor*>(dest_ptr));
                } else if (render_pass_desc.first == "depth" || render_pass_desc.first == "normal") {
                    // ReadLinearColorPixelsPtr assumes 4 channels per pixel, 4 byte per channel, so it can be used
                    // to read the following ETextureRenderTargetFormat formats:
                    //     depth:  RTF_RGBA32f
                    //     normal: RTF_RGBA32f
                    texture_render_target_resource->ReadLinearColorPixelsPtr(static_cast<FLinearColor*>(dest_ptr));
                } else {
                    SP_ASSERT(false);
                }
            }
        }
    }

    return observation;
}
