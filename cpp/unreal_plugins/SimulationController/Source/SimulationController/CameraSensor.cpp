//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SimulationController/CameraSensor.h"

#include <cstring>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include <boost/predef.h>

#include <Camera/CameraComponent.h>
#include <Components/SceneCaptureComponent2D.h>
#include <Containers/Array.h>
#include <Engine/Engine.h>
#include <Engine/TextureRenderTarget2D.h>
#include <Engine/World.h>
#include <EngineUtils.h>
#include <GameFramework/Actor.h>
#include <Materials/MaterialInstanceDynamic.h>
#include <Math/Color.h>
#include <UObject/UObjectGlobals.h>

#include "CoreUtils/Assert.h"
#include "CoreUtils/Config.h"
#include "CoreUtils/Std.h"
#include "CoreUtils/Unreal.h"
#include "SimulationController/BoostInterprocess.h"
#include "SimulationController/Box.h"

const std::string MATERIALS_PATH = "/SimulationController/PostProcessMaterials";

const std::map<std::string, float> RENDER_PASS_LOW = {
    {"depth",           0.0f},
    {"depth_glsl",      0.0f},
    {"final_color",     0.0f},
    {"lens_distortion", 0.0f},
    {"normals",         0.0f},
    {"segmentation",    0.0f}};

const std::map<std::string, float> RENDER_PASS_HIGH = {
    {"depth",           std::numeric_limits<float>::max()},
    {"depth_glsl",      std::numeric_limits<float>::max()},
    {"final_color",     255.0f},
    {"lens_distortion", 255.0f},
    {"normals",         255.0f},
    {"segmentation",    255.0f}};

const std::map<std::string, int> RENDER_PASS_NUM_CHANNELS = {
    {"depth",           1},
    {"depth_glsl",      1},
    {"final_color",     4},
    {"lens_distortion", 4},
    {"normals",         4},
    {"segmentation",    4}};

const std::map<std::string, DataType> RENDER_PASS_DATATYPE = {
    {"depth",           DataType::Float32},
    {"depth_glsl",      DataType::Float32},
    {"final_color",     DataType::UInteger8},
    {"lens_distortion", DataType::UInteger8},
    {"normals",         DataType::UInteger8},
    {"segmentation",    DataType::UInteger8}};

const std::map<std::string, int> RENDER_PASS_NUM_BYTES_PER_CHANNEL = {
    {"depth",           4},
    {"depth_glsl",      4},
    {"final_color",     1},
    {"lens_distortion", 1},
    {"normals",         1},
    {"segmentation",    1}};

CameraSensor::CameraSensor(UCameraComponent* camera_component, const std::vector<std::string>& render_pass_names, unsigned int width, unsigned int height)
{
    ASSERT(camera_component);

    parent_actor_ = camera_component->GetWorld()->SpawnActor<AActor>();
    ASSERT(parent_actor_);
    
    for (auto& render_pass_name : render_pass_names) {
        RenderPass render_pass;

        // create TextureRenderTarget2D
        render_pass.texture_render_target_ = NewObject<UTextureRenderTarget2D>(parent_actor_);
        ASSERT(render_pass.texture_render_target_);

        // TODO: allow returning floating point data instead of hardcoding to PF_B8G8R8A8
        bool force_linear_gamma = false;
        bool clear_render_target = true;
        render_pass.texture_render_target_->RenderTargetFormat = ETextureRenderTargetFormat::RTF_RGBA8;
        render_pass.texture_render_target_->TargetGamma = GEngine->GetDisplayGamma();
        render_pass.texture_render_target_->SRGB = true;
        render_pass.texture_render_target_->bGPUSharedFlag = true;
        render_pass.texture_render_target_->bAutoGenerateMips = false;
        render_pass.texture_render_target_->InitCustomFormat(width, height, PF_B8G8R8A8, force_linear_gamma);
        render_pass.texture_render_target_->UpdateResourceImmediate(clear_render_target);

        // create SceneCaptureComponent2D
        render_pass.scene_capture_component_ = NewObject<USceneCaptureComponent2D>(parent_actor_);
        ASSERT(render_pass.scene_capture_component_);

        if (Config::get<std::string>("SIMULATION_CONTROLLER.CAMERA_SENSOR.SCENE_CAPTURE_COMPONENT_CAPTURE_SOURCE") == "SCS_FinalColorHDR") {
            render_pass.scene_capture_component_->CaptureSource = ESceneCaptureSource::SCS_FinalColorHDR;
        } else if (Config::get<std::string>("SIMULATION_CONTROLLER.CAMERA_SENSOR.SCENE_CAPTURE_COMPONENT_CAPTURE_SOURCE") == "SCS_FinalToneCurveHDR") {
            render_pass.scene_capture_component_->CaptureSource = ESceneCaptureSource::SCS_FinalToneCurveHDR;
        } else {
            ASSERT(false);
        }

        render_pass.scene_capture_component_->TextureTarget = render_pass.texture_render_target_;
        render_pass.scene_capture_component_->AttachToComponent(camera_component, FAttachmentTransformRules::SnapToTargetNotIncludingScale);
        render_pass.scene_capture_component_->SetVisibility(true);
        render_pass.scene_capture_component_->RegisterComponent();

        if (render_pass_name == "final_color") {
            initializeSceneCaptureComponentFinalColor(render_pass.scene_capture_component_);
        } else {
            initializeSceneCaptureComponentNonFinalColor(render_pass.scene_capture_component_, render_pass_name);
        }

        // create shared_memory_object
        if (Config::get<bool>("SIMULATION_CONTROLLER.CAMERA_SENSOR.USE_SHARED_MEMORY")) {
            render_pass.shared_memory_num_bytes_ =
                height * width * RENDER_PASS_NUM_CHANNELS.at(render_pass_name) * RENDER_PASS_NUM_BYTES_PER_CHANNEL.at(render_pass_name);
            render_pass.shared_memory_name_ = "camera." + render_pass_name;

            #if BOOST_OS_WINDOWS
                render_pass.shared_memory_id_ = render_pass.shared_memory_name_; // don't use leading slash on Windows
                boost::interprocess::windows_shared_memory windows_shared_memory(
                    boost::interprocess::create_only,
                    render_pass.shared_memory_id_.c_str(),
                    boost::interprocess::read_write,
                    render_pass.shared_memory_num_bytes_);
                render_pass.shared_memory_mapped_region_ = boost::interprocess::mapped_region(windows_shared_memory, boost::interprocess::read_write);
            #elif BOOST_OS_MACOS || BOOST_OS_UNIX
                render_pass.shared_memory_id_ = "/" + render_pass.shared_memory_name_; // use leading slash on macOS and Linux
                boost::interprocess::shared_memory_object::remove(render_pass.shared_memory_id_.c_str());
                boost::interprocess::shared_memory_object shared_memory_object(
                    boost::interprocess::create_only,
                    render_pass.shared_memory_id_.c_str(),
                    boost::interprocess::read_write);
                shared_memory_object.truncate(render_pass.shared_memory_num_bytes_);
                render_pass.shared_memory_mapped_region_ = boost::interprocess::mapped_region(shared_memory_object, boost::interprocess::read_write);
            #else
                #error
            #endif
        }

        // update render_passes_
        render_passes_[render_pass_name] = std::move(render_pass);
    }

    width_ = width;
    height_ = height;
}

CameraSensor::~CameraSensor()
{
    height_ = -1;
    width_ = -1;

    for (auto& render_pass : render_passes_) {
        if (Config::get<bool>("SIMULATION_CONTROLLER.CAMERA_SENSOR.USE_SHARED_MEMORY")) {
            #if BOOST_OS_MACOS || BOOST_OS_LINUX
                boost::interprocess::shared_memory_object::remove(render_pass.second.shared_memory_id_.c_str());
            #endif
        }

        ASSERT(render_pass.second.scene_capture_component_);
        render_pass.second.scene_capture_component_->MarkPendingKill();
        render_pass.second.scene_capture_component_ = nullptr;

        ASSERT(render_pass.second.texture_render_target_);
        render_pass.second.texture_render_target_->MarkPendingKill();
        render_pass.second.texture_render_target_ = nullptr;
    }
    render_passes_.clear();

    ASSERT(parent_actor_);
    parent_actor_->Destroy();
    parent_actor_ = nullptr;
}

std::map<std::string, Box> CameraSensor::getObservationSpace(const std::vector<std::string>& observation_components) const
{
    std::map<std::string, Box> observation_space;

    if (Std::contains(observation_components, "camera")) {
        for (auto& render_pass : render_passes_) {
            Box box;
            box.low_ = RENDER_PASS_LOW.at(render_pass.first);
            box.high_ = RENDER_PASS_HIGH.at(render_pass.first);
            box.shape_ = {height_, width_, RENDER_PASS_NUM_CHANNELS.at(render_pass.first)};
            box.datatype_ = RENDER_PASS_DATATYPE.at(render_pass.first);
            box.use_shared_memory_ = Config::get<bool>("SIMULATION_CONTROLLER.CAMERA_SENSOR.USE_SHARED_MEMORY");
            box.shared_memory_name_ = render_pass.second.shared_memory_name_;
            observation_space["camera." + render_pass.first] = std::move(box);
        }
    }

    return observation_space;
}

std::map<std::string, std::vector<uint8_t>> CameraSensor::getObservation(const std::vector<std::string>& observation_components) const
{
    std::map<std::string, std::vector<uint8_t>> observation;

    if (Std::contains(observation_components, "camera")) {

        std::map<std::string, TArray<FColor>> render_data = getRenderData();
        for (auto& render_data_component : render_data) {

            if (render_data_component.first == "final_color"     ||
                render_data_component.first == "lens_distortion" ||
                render_data_component.first == "normals"         ||
                render_data_component.first == "segmentation") {

                if (Config::get<bool>("SIMULATION_CONTROLLER.CAMERA_SENSOR.USE_SHARED_MEMORY")) {
                    std::memcpy(
                        render_passes_.at(render_data_component.first).shared_memory_mapped_region_.get_address(),
                        render_data_component.second.GetData(),
                        render_passes_.at(render_data_component.first).shared_memory_num_bytes_);
                } else {
                    observation["camera." + render_data_component.first] = Unreal::reinterpret_as<uint8_t>(render_data_component.second);
                }

            } else if (render_data_component.first == "depth" ||
                       render_data_component.first == "depth_glsl") {

                std::vector<float> float_depth = getFloatDepthFromColorDepth(render_data_component.second);

                if (Config::get<bool>("SIMULATION_CONTROLLER.CAMERA_SENSOR.USE_SHARED_MEMORY")) {
                    std::memcpy(
                        render_passes_.at(render_data_component.first).shared_memory_mapped_region_.get_address(),
                        float_depth.data(),
                        render_passes_.at(render_data_component.first).shared_memory_num_bytes_);
                } else {
                    observation["camera." + render_data_component.first] = Std::reinterpret_as<uint8_t>(float_depth);
                }

            } else {
                ASSERT(false);
            }
        }
    }

    return observation;
}

void CameraSensor::initializeSceneCaptureComponentFinalColor(USceneCaptureComponent2D* scene_capture_component)
{
    // update auto-exposure settings
    scene_capture_component->PostProcessSettings.bOverride_AutoExposureSpeedUp =
        Config::get<bool> ("SIMULATION_CONTROLLER.CAMERA_SENSOR.FINAL_COLOR_OVERRIDE_AUTO_EXPOSURE_SPEED_UP");
    scene_capture_component->PostProcessSettings.AutoExposureSpeedUp =
        Config::get<float>("SIMULATION_CONTROLLER.CAMERA_SENSOR.FINAL_COLOR_AUTO_EXPOSURE_SPEED_UP");
    scene_capture_component->PostProcessSettings.bOverride_AutoExposureSpeedDown =
        Config::get<bool> ("SIMULATION_CONTROLLER.CAMERA_SENSOR.FINAL_COLOR_OVERRIDE_AUTO_EXPOSURE_SPEED_DOWN");
    scene_capture_component->PostProcessSettings.AutoExposureSpeedDown =
        Config::get<float>("SIMULATION_CONTROLLER.CAMERA_SENSOR.FINAL_COLOR_AUTO_EXPOSURE_SPEED_DOWN");

    // enable raytracing features
    scene_capture_component->bUseRayTracingIfEnabled =
        Config::get<bool>("SIMULATION_CONTROLLER.CAMERA_SENSOR.FINAL_COLOR_USE_RAYTRACING_IF_ENABLED");

    // update indirect lighting 
    scene_capture_component->PostProcessSettings.bOverride_IndirectLightingIntensity =
        Config::get<bool> ("SIMULATION_CONTROLLER.CAMERA_SENSOR.FINAL_COLOR_OVERRIDE_INDIRECT_LIGHTING_INTENSITY");
    scene_capture_component->PostProcessSettings.IndirectLightingIntensity =
        Config::get<float>("SIMULATION_CONTROLLER.CAMERA_SENSOR.FINAL_COLOR_INDIRECT_LIGHTING_INTENSITY");

    // update raytracing global illumination
    scene_capture_component->PostProcessSettings.bOverride_RayTracingGI =
        Config::get<bool>("SIMULATION_CONTROLLER.CAMERA_SENSOR.FINAL_COLOR_OVERRIDE_RAYTRACING_GI");

    auto raytracing_gi_type = Config::get<std::string>("SIMULATION_CONTROLLER.CAMERA_SENSOR.FINAL_COLOR_RAYTRACING_GI_TYPE");
    if (raytracing_gi_type == "BruteForce") {
        scene_capture_component->PostProcessSettings.RayTracingGIType = ERayTracingGlobalIlluminationType::BruteForce;
    } else if (raytracing_gi_type != "") {
        ASSERT(false);
    }

    scene_capture_component->PostProcessSettings.bOverride_RayTracingGIMaxBounces =
        Config::get<bool>("SIMULATION_CONTROLLER.CAMERA_SENSOR.FINAL_COLOR_OVERRIDE_RAYTRACING_GI_MAX_BOUNCES");
    scene_capture_component->PostProcessSettings.RayTracingGIMaxBounces =
        Config::get<unsigned long>("SIMULATION_CONTROLLER.CAMERA_SENSOR.FINAL_COLOR_RAYTRACING_GI_MAX_BOUNCES");
    scene_capture_component->PostProcessSettings.bOverride_RayTracingGISamplesPerPixel =
        Config::get<bool>("SIMULATION_CONTROLLER.CAMERA_SENSOR.FINAL_COLOR_OVERRIDE_RAYTRACING_GI_SAMPLES_PER_PIXEL");
    scene_capture_component->PostProcessSettings.RayTracingGISamplesPerPixel =
        Config::get<unsigned long>("SIMULATION_CONTROLLER.CAMERA_SENSOR.FINAL_COLOR_RAYTRACING_GI_SAMPLES_PER_PIXEL");

    // update raytracing ambient occlusion
    scene_capture_component->PostProcessSettings.bOverride_RayTracingAO =
        Config::get<bool>("SIMULATION_CONTROLLER.CAMERA_SENSOR.FINAL_COLOR_OVERRIDE_RAYTRACING_AO");
    scene_capture_component->PostProcessSettings.RayTracingAO =
        Config::get<unsigned long>("SIMULATION_CONTROLLER.CAMERA_SENSOR.FINAL_COLOR_RAYTRACING_AO");
    scene_capture_component->PostProcessSettings.bOverride_RayTracingAOSamplesPerPixel =
        Config::get<bool>("SIMULATION_CONTROLLER.CAMERA_SENSOR.FINAL_COLOR_OVERRIDE_RAYTRACING_AO_SAMPLES_PER_PIXEL");
    scene_capture_component->PostProcessSettings.RayTracingAOSamplesPerPixel =
        Config::get<unsigned long>("SIMULATION_CONTROLLER.CAMERA_SENSOR.FINAL_COLOR_RAYTRACING_AO_SAMPLES_PER_PIXEL");
    scene_capture_component->PostProcessSettings.bOverride_RayTracingAOIntensity =
        Config::get<bool>("SIMULATION_CONTROLLER.CAMERA_SENSOR.FINAL_COLOR_OVERRIDE_RAYTRACING_AO_INTENSITY");
    scene_capture_component->PostProcessSettings.RayTracingAOIntensity =
        Config::get<float>("SIMULATION_CONTROLLER.CAMERA_SENSOR.FINAL_COLOR_RAYTRACING_AO_INTENSITY");
    scene_capture_component->PostProcessSettings.bOverride_RayTracingAORadius =
        Config::get<bool>("SIMULATION_CONTROLLER.CAMERA_SENSOR.FINAL_COLOR_OVERRIDE_RAYTRACING_AO_RADIUS");
    scene_capture_component->PostProcessSettings.RayTracingAORadius =
        Config::get<float>("SIMULATION_CONTROLLER.CAMERA_SENSOR.FINAL_COLOR_RAYTRACING_AO_RADIUS");

    // update raytracing reflections
    scene_capture_component->PostProcessSettings.bOverride_ReflectionsType =
        Config::get<bool>("SIMULATION_CONTROLLER.CAMERA_SENSOR.FINAL_COLOR_OVERRIDE_REFLECTIONS_TYPE");

    auto reflections_type = Config::get<std::string>("SIMULATION_CONTROLLER.CAMERA_SENSOR.FINAL_COLOR_REFLECTIONS_TYPE");
    if (reflections_type == "RayTracing") {
        scene_capture_component->PostProcessSettings.ReflectionsType = EReflectionsType::RayTracing; 
    } else if (reflections_type != "") {
        ASSERT(false); 
    }

    scene_capture_component->PostProcessSettings.bOverride_RayTracingReflectionsMaxBounces =
        Config::get<bool>("SIMULATION_CONTROLLER.CAMERA_SENSOR.FINAL_COLOR_OVERRIDE_RAYTRACING_REFLECTIONS_MAX_BOUNCES");
    scene_capture_component->PostProcessSettings.RayTracingReflectionsMaxBounces =
        Config::get<unsigned long>("SIMULATION_CONTROLLER.CAMERA_SENSOR.FINAL_COLOR_RAYTRACING_REFLECTIONS_MAX_BOUNCES");
    scene_capture_component->PostProcessSettings.bOverride_RayTracingReflectionsMaxRoughness =
        Config::get<bool>("SIMULATION_CONTROLLER.CAMERA_SENSOR.FINAL_COLOR_OVERRIDE_RAYTRACING_REFLECTIONS_MAX_ROUGHNESS");
    scene_capture_component->PostProcessSettings.RayTracingReflectionsMaxRoughness =
        Config::get<float>("SIMULATION_CONTROLLER.CAMERA_SENSOR.FINAL_COLOR_RAYTRACING_REFLECTIONS_MAX_ROUGHNESS");
    scene_capture_component->PostProcessSettings.bOverride_RayTracingReflectionsSamplesPerPixel =
        Config::get<bool>("SIMULATION_CONTROLLER.CAMERA_SENSOR.FINAL_COLOR_OVERRIDE_RAYTRACING_REFLECTIONS_SAMPLES_PER_PIXEL");
    scene_capture_component->PostProcessSettings.RayTracingReflectionsSamplesPerPixel =
        Config::get<unsigned long>("SIMULATION_CONTROLLER.CAMERA_SENSOR.FINAL_COLOR_RAYTRACING_REFLECTIONS_SAMPLES_PER_PIXEL");
    scene_capture_component->PostProcessSettings.bOverride_RayTracingReflectionsTranslucency =
        Config::get<bool>("SIMULATION_CONTROLLER.CAMERA_SENSOR.FINAL_COLOR_OVERRIDE_RAYTRACING_REFLECTIONS_TRANSLUCENCY");
    scene_capture_component->PostProcessSettings.RayTracingReflectionsTranslucency =
        Config::get<unsigned long>("SIMULATION_CONTROLLER.CAMERA_SENSOR.FINAL_COLOR_RAYTRACING_REFLECTIONS_TRANSLUCENCY");

    // update show flags

    // enabled by FEngineShowFlags::EnableAdvancedFeatures()
    scene_capture_component->ShowFlags.SetAmbientOcclusion(
        Config::get<bool>("SIMULATION_CONTROLLER.CAMERA_SENSOR.FINAL_COLOR_SET_AMBIENT_OCCLUSION"));
    scene_capture_component->ShowFlags.SetCameraImperfections(
        Config::get<bool>("SIMULATION_CONTROLLER.CAMERA_SENSOR.FINAL_COLOR_SET_CAMERA_IMPERFECTIONS"));
    scene_capture_component->ShowFlags.SetColorGrading(
        Config::get<bool>("SIMULATION_CONTROLLER.CAMERA_SENSOR.FINAL_COLOR_SET_COLOR_GRADING"));
    scene_capture_component->ShowFlags.SetDepthOfField(
        Config::get<bool>("SIMULATION_CONTROLLER.CAMERA_SENSOR.FINAL_COLOR_SET_DEPTH_OF_FIELD"));
    scene_capture_component->ShowFlags.SetDistanceFieldAO(
        Config::get<bool>("SIMULATION_CONTROLLER.CAMERA_SENSOR.FINAL_COLOR_SET_DISTANCE_FIELD_AO"));
    scene_capture_component->ShowFlags.SetEyeAdaptation(
        Config::get<bool>("SIMULATION_CONTROLLER.CAMERA_SENSOR.FINAL_COLOR_SET_EYE_ADAPTATION"));
    scene_capture_component->ShowFlags.SetGrain(
        Config::get<bool>("SIMULATION_CONTROLLER.CAMERA_SENSOR.FINAL_COLOR_SET_GRAIN"));
    scene_capture_component->ShowFlags.SetIndirectLightingCache(
        Config::get<bool>("SIMULATION_CONTROLLER.CAMERA_SENSOR.FINAL_COLOR_SET_INDIRECT_LIGHTING_CACHE"));
    scene_capture_component->ShowFlags.SetLensFlares(
        Config::get<bool>("SIMULATION_CONTROLLER.CAMERA_SENSOR.FINAL_COLOR_SET_LENS_FLARES"));
    scene_capture_component->ShowFlags.SetLightShafts(
        Config::get<bool>("SIMULATION_CONTROLLER.CAMERA_SENSOR.FINAL_COLOR_SET_LIGHT_SHAFTS"));
    scene_capture_component->ShowFlags.SetScreenSpaceReflections(
        Config::get<bool>("SIMULATION_CONTROLLER.CAMERA_SENSOR.FINAL_COLOR_SET_SCREEN_SPACE_REFLECTIONS"));
    scene_capture_component->ShowFlags.SetSeparateTranslucency(
        Config::get<bool>("SIMULATION_CONTROLLER.CAMERA_SENSOR.FINAL_COLOR_SET_SEPARATE_TRANSLUCENCY"));
    scene_capture_component->ShowFlags.SetTemporalAA(
        Config::get<bool>("SIMULATION_CONTROLLER.CAMERA_SENSOR.FINAL_COLOR_SET_TEMPORAL_AA"));
    scene_capture_component->ShowFlags.SetVignette(
        Config::get<bool>("SIMULATION_CONTROLLER.CAMERA_SENSOR.FINAL_COLOR_SET_VIGNETTE"));

    // not enabled by FEngineShowFlags::EnableAdvancedFeatures()
    scene_capture_component->ShowFlags.SetAntiAliasing(
        Config::get<bool>("SIMULATION_CONTROLLER.CAMERA_SENSOR.FINAL_COLOR_SET_ANTI_ALIASING"));
    scene_capture_component->ShowFlags.SetRayTracedDistanceFieldShadows(
        Config::get<bool>("SIMULATION_CONTROLLER.CAMERA_SENSOR.FINAL_COLOR_SET_RAYTRACED_DISTANCE_FIELD_SHADOWS"));
    scene_capture_component->ShowFlags.SetDynamicShadows(
        Config::get<bool>("SIMULATION_CONTROLLER.CAMERA_SENSOR.FINAL_COLOR_SET_DYNAMIC_SHADOWS"));
}

void CameraSensor::initializeSceneCaptureComponentNonFinalColor(USceneCaptureComponent2D* scene_capture_component, const std::string& render_pass_name)
{
    auto material = LoadObject<UMaterial>(nullptr, *FString::Printf(
        TEXT("%s/%s.%s"), *Unreal::toFString(MATERIALS_PATH), *Unreal::toFString(render_pass_name), *Unreal::toFString(render_pass_name)));
    ASSERT(material);
    scene_capture_component->PostProcessSettings.AddBlendable(UMaterialInstanceDynamic::Create(material, scene_capture_component), 1.0f);
    scene_capture_component->ShowFlags.SetPostProcessMaterial(true);
}

std::map<std::string, TArray<FColor>> CameraSensor::getRenderData() const
{
    std::map<std::string, TArray<FColor>> render_data;

    // get data from all passes
    for (auto& render_pass : render_passes_) {
        FTextureRenderTargetResource* target_resource = render_pass.second.scene_capture_component_->TextureTarget->GameThread_GetRenderTargetResource();
        ASSERT(target_resource);

        FRHITexture* rhi_texture = target_resource->GetRenderTargetTexture();
        ASSERT(rhi_texture);
        FIntRect rect(0, 0, target_resource->GetSizeXY().X, target_resource->GetSizeXY().Y);
        TArray<FColor> render_data_component;
        FReadSurfaceDataFlags read_surface_data_flags(RCM_UNorm, CubeFace_MAX);
        read_surface_data_flags.SetLinearToGamma(false);

        if (Config::get<bool>("SIMULATION_CONTROLLER.CAMERA_SENSOR.SKIP_READ_SURFACE_DATA")) {
            render_data_component.SetNum(target_resource->GetSizeXY().X*target_resource->GetSizeXY().Y);
        } else {
            ENQUEUE_RENDER_COMMAND(ReadSurfaceDataCommand) (
                [rhi_texture, rect, &render_data_component, read_surface_data_flags](FRHICommandListImmediate& rhi_command_list_immediate) -> void {
                    rhi_command_list_immediate.ReadSurfaceData(rhi_texture, rect, render_data_component, read_surface_data_flags);
                }
            );

            FRenderCommandFence render_command_fence;
            render_command_fence.BeginFence(true);
            render_command_fence.Wait(true);
        }

        render_data[render_pass.first] = std::move(render_data_component);
    }

    return render_data;
}

std::vector<float> CameraSensor::getFloatDepthFromColorDepth(TArray<FColor>& color_depth)
{
    std::vector<float> float_depth;
    for (int i = 0; i < color_depth.Num(); i++) {
        float depth = color_depth[i].R  + (color_depth[i].G * 256.0f) + (color_depth[i].B * 256.0f * 256.0f); 
        float normalized_depth = depth / ((256.0f * 256.0f * 256.0f) - 1.0f);
        float dist = normalized_depth * 10.0f;
        float_depth.push_back(dist);
    }
    return float_depth;
}
