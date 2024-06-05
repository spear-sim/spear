//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpCore/CameraSensorComponent.h"

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
const std::map<std::string, int> RENDER_PASS_NUM_CHANNELS = {{"depth", 4}, {"final_color", 4}, {"normal", 4}, {"segmentation", 4}};

const std::map<std::string, int> RENDER_PASS_NUM_BYTES_PER_CHANNEL = {{"depth", 4}, {"final_color", 1}, {"normal", 4}, {"segmentation", 1}};

const std::map<std::string, ETextureRenderTargetFormat> RENDER_PASS_TEXTURE_RENDER_TARGET_FORMAT = {
    {"depth", ETextureRenderTargetFormat::RTF_RGBA32f},
    {"final_color", ETextureRenderTargetFormat::RTF_RGBA8_SRGB},
    {"normal", ETextureRenderTargetFormat::RTF_RGBA32f},
    {"segmentation", ETextureRenderTargetFormat::RTF_RGBA8}};

const std::map<std::string, std::string> RENDER_PASS_MATERIAL = {{"depth", "/SpServices/Materials/PPM_Depth.PPM_Depth"},
                                                                 {"normal", "/SpServices/Materials/PPM_Normal.PPM_Normal"},
                                                                 {"segmentation", "/SpServices/Materials/PPM_Segmentation.PPM_Segmentation"}};

const std::map<std::string, double> RENDER_PASS_LOW = {
    {"depth", 0.0},
    {"final_color", 0.0},
    {"normal", std::numeric_limits<double>::lowest()}, // Unreal can return normals that are not unit length
    {"segmentation", 0.0}};

const std::map<std::string, double> RENDER_PASS_HIGH = {{"depth", std::numeric_limits<double>::max()},
                                                        {"final_color", 255.0},
                                                        {"normal", std::numeric_limits<double>::max()}, // Unreal can return normals that are not unit length
                                                        {"segmentation", 255.0}};

const std::map<std::string, DataType> RENDER_PASS_CHANNEL_DATATYPE = {
    {"depth", DataType::Float32}, {"final_color", DataType::UInteger8}, {"normal", DataType::Float32}, {"segmentation", DataType::UInteger8}};

UCameraSensorComponent::UCameraSensorComponent()
{
}

UCameraSensorComponent::~UCameraSensorComponent()
{
    for (auto& [render_pass_name, render_pass_desc] : render_pass_descs_) {
        if (Config::get<bool>("SP_SERVICES.LEGACY.CAMERA_SENSOR.USE_SHARED_MEMORY")) {
#if BOOST_OS_MACOS || BOOST_OS_LINUX
            boost::interprocess::shared_memory_object::remove(render_pass_desc.shared_memory_id_.c_str());
#endif
        }
    }
}

void UCameraSensorComponent::setup(UCameraComponent* camera_component)
{
    camera_component_                          = camera_component;
    std::vector<std::string> render_pass_names = {"final_color"};
    int width                                  = 512;
    int height                                 = 512;
    float fov                                  = 90;
    for (auto& render_pass_name : render_pass_names) {
        RenderPassDesc render_pass_desc;

        render_pass_desc.width_     = width;
        render_pass_desc.height_    = height;
        render_pass_desc.num_bytes_ = height * width * RENDER_PASS_NUM_CHANNELS.at(render_pass_name) * RENDER_PASS_NUM_BYTES_PER_CHANNEL.at(render_pass_name);

        // create TextureRenderTarget2D
        auto texture_render_target_2d = NewObject<UTextureRenderTarget2D>(GetOwner(), Unreal::toFName("texture_render_target_2d_" + render_pass_name));
        SP_ASSERT(texture_render_target_2d);

        bool clear_render_target                     = true;
        texture_render_target_2d->RenderTargetFormat = RENDER_PASS_TEXTURE_RENDER_TARGET_FORMAT.at(render_pass_name);
        texture_render_target_2d->InitAutoFormat(width, height);
        texture_render_target_2d->UpdateResourceImmediate(clear_render_target);

        // create SceneCaptureComponent2D
        render_pass_desc.scene_capture_component_2d_ = Unreal::createComponentOutsideOwnerConstructor<USceneCaptureComponent2D>(
            GetOwner(), camera_component, "scene_capture_component_2d_" + render_pass_name);
        SP_ASSERT(render_pass_desc.scene_capture_component_2d_);
        render_pass_desc.scene_capture_component_2d_->TextureTarget = texture_render_target_2d;
        render_pass_desc.scene_capture_component_2d_->FOVAngle      = fov;
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
        if (Config::get<bool>("SP_SERVICES.LEGACY.CAMERA_SENSOR.USE_SHARED_MEMORY")) {
            render_pass_desc.shared_memory_name_ = "camera." + render_pass_name;

#if BOOST_OS_WINDOWS
            int shared_memory_num_bytes            = 1024;
            render_pass_desc.shared_memory_region_ = std::make_unique<SharedMemoryRegion>(shared_memory_num_bytes);
            SP_ASSERT(render_pass_desc.shared_memory_region_);
            CppFuncSharedMemoryView shared_memory_view(render_pass_desc.shared_memory_region_->getView(),
                                                       CppFuncSharedMemoryUsageFlags::Arg | CppFuncSharedMemoryUsageFlags::ReturnValue);
            registerSharedMemoryView(render_pass_desc.shared_memory_name_, shared_memory_view);

#elif BOOST_OS_MACOS || BOOST_OS_LINUX
            render_pass_desc.shared_memory_id_ = "/" + render_pass_desc.shared_memory_name_; // use leading slash on macOS and Linux
            boost::interprocess::shared_memory_object::remove(render_pass_desc.shared_memory_id_.c_str());
            boost::interprocess::shared_memory_object shared_memory_object(boost::interprocess::create_only, render_pass_desc.shared_memory_id_.c_str(),
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

    registerFunc("hello_world.camera", [this](CppFuncPackage& args) -> CppFuncPackage {
        // Store the Unreal data in a CppFuncData object to efficiently return it to Python.
        // Here we use shared memory for the most efficient possible communication.
        CppFuncPackage return_values;
        for (auto& [render_pass_name, render_pass_desc] : render_pass_descs_) {

            CppFuncData<uint8_t> unreal_data(render_pass_desc.shared_memory_name_);
            unreal_data.setData("my_shared_memory", render_pass_desc.shared_memory_region_->getView(), render_pass_desc.num_bytes_);

            TArray<FColor> dummy_observation;
            dummy_observation.SetNum(render_pass_desc.width_ * render_pass_desc.height_);

            FTextureRenderTargetResource* texture_render_target_resource =
                render_pass_desc.scene_capture_component_2d_->TextureTarget->GameThread_GetRenderTargetResource();
            SP_ASSERT(texture_render_target_resource);
            texture_render_target_resource->ReadPixelsPtr(static_cast<FColor*>(render_pass_desc.shared_memory_region_->getView().data_));

            return_values.items_ = CppFuncUtils::moveDataToItems({unreal_data.getPtr()});
        }
        // Return CppFuncData objects.
        return return_values;
    });
}

TArray<FColor> UCameraSensorComponent::getObservation() const
{
    TArray<FColor> dummy_observation;
    std::map<std::string, std::vector<uint8_t>> observation;

    for (auto& [render_pass_name, render_pass_desc] : render_pass_descs_) {
        void* dest_ptr = nullptr;
        // if (Config::get<bool>("SP_SERVICES.LEGACY.CAMERA_SENSOR.USE_SHARED_MEMORY")) {
        //     //dest_ptr = render_pass_desc.shared_memory_mapped_region_.get_address();
        // } else {
        Std::insert(observation, "camera." + render_pass_name, {});
        observation.at("camera." + render_pass_name).resize(render_pass_desc.num_bytes_);
        dest_ptr = observation.at("camera." + render_pass_name).data();
        //}
        SP_ASSERT(dest_ptr);

        if (Config::get<bool>("SP_SERVICES.LEGACY.CAMERA_SENSOR.READ_SURFACE_DATA")) {
            FTextureRenderTargetResource* texture_render_target_resource =
                render_pass_desc.scene_capture_component_2d_->TextureTarget->GameThread_GetRenderTargetResource();
            SP_ASSERT(texture_render_target_resource);

            if (render_pass_name == "final_color" || render_pass_name == "segmentation") {
                // ReadPixelsPtr assumes 4 channels per pixel, 1 byte per channel, so it can be used to read
                // the following ETextureRenderTargetFormat formats:
                //     final_color:  RTF_RGBA8
                //     segmentation: RTF_RGBA8_SRGB
                dummy_observation.SetNum(render_pass_desc.width_ * render_pass_desc.height_);
                texture_render_target_resource->ReadPixelsPtr(dummy_observation.GetData());
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

    // return observation;
    return dummy_observation;
}
