#include "CameraSensor.h"

#include <utility>

#include <Camera/CameraComponent.h>
#include <Components/SceneCaptureComponent2D.h>
#include <Engine/Engine.h>
#include <Engine/TextureRenderTarget2D.h>
#include <Engine/World.h>
#include <EngineUtils.h>
#include <GameFramework/Actor.h>
#include <Materials/MaterialInstanceDynamic.h>
#include <Math/Color.h>
#include <UObject/UObjectGlobals.h>

#include "Assert/Assert.h"
#include "Box.h"
#include "Config.h"
#include "Serialize.h"

const std::string MATERIALS_PATH = "/SimulationController/PostProcessMaterials";

const std::map<std::string, int>      OBSERVATION_COMPONENT_NUM_CHANNELS = {{"final_color", 3},                   {"segmentation", 3},                   {"depth_glsl", 1},                 {"normals", 1}};
const std::map<std::string, DataType> OBSERVATION_COMPONENT_DTYPE        = {{"final_color", DataType::UInteger8}, {"segmentation", DataType::UInteger8}, {"depth_glsl", DataType::Float32}, {"normals", DataType::UInteger8}};

CameraSensor::CameraSensor(UCameraComponent* camera_component, const std::vector<std::string>& render_pass_names, unsigned int width, unsigned int height)
{
    ASSERT(camera_component);

    new_object_parent_actor_ = camera_component->GetWorld()->SpawnActor<AActor>();
    ASSERT(new_object_parent_actor_);

    for (auto& render_pass_name : render_pass_names) {

        RenderPass render_pass;

        // create TextureRenderTarget2D
        render_pass.texture_render_target_ = NewObject<UTextureRenderTarget2D>(new_object_parent_actor_, *FString::Printf(TEXT("TextureRenderTarget2D_%s"), UTF8_TO_TCHAR(render_pass_name.c_str())));
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
        render_pass.scene_capture_component_ = NewObject<USceneCaptureComponent2D>(new_object_parent_actor_, *FString::Printf(TEXT("SceneCaptureComponent2D_%s"), UTF8_TO_TCHAR(render_pass_name.c_str())));
        ASSERT(render_pass.scene_capture_component_);

        render_pass.scene_capture_component_->TextureTarget = render_pass.texture_render_target_;
        render_pass.scene_capture_component_->AttachToComponent(camera_component, FAttachmentTransformRules::SnapToTargetNotIncludingScale);
        render_pass.scene_capture_component_->SetVisibility(true);
        render_pass.scene_capture_component_->RegisterComponent();

        if (render_pass_name != "final_color") {
            auto material = LoadObject<UMaterial>(nullptr, *FString::Printf(TEXT("%s/%s.%s"), UTF8_TO_TCHAR(MATERIALS_PATH.c_str()), UTF8_TO_TCHAR(render_pass_name.c_str()), UTF8_TO_TCHAR(render_pass_name.c_str())));
            ASSERT(material);
            render_pass.scene_capture_component_->PostProcessSettings.AddBlendable(UMaterialInstanceDynamic::Create(material, render_pass.scene_capture_component_), 1.0f);
            render_pass.scene_capture_component_->ShowFlags.SetPostProcessMaterial(true);
        }

        if (render_pass_name == "final_color") {
            initializeSceneCaptureComponentFinalColor(render_pass.scene_capture_component_);
        }

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
        ASSERT(render_pass.second.scene_capture_component_);
        render_pass.second.scene_capture_component_->MarkPendingKill();
        render_pass.second.scene_capture_component_ = nullptr;

        ASSERT(render_pass.second.texture_render_target_);
        render_pass.second.texture_render_target_->MarkPendingKill();
        render_pass.second.texture_render_target_ = nullptr;
    }
    render_passes_.clear();

    ASSERT(new_object_parent_actor_);
    new_object_parent_actor_->Destroy();
    new_object_parent_actor_ = nullptr;
}

std::map<std::string, Box> CameraSensor::getObservationSpace() const
{
    std::map<std::string, Box> observation_space;
    Box box;

    for (auto& render_pass : render_passes_) {
        box.low = 0;
        box.high = 255;
        box.shape = {height_, width_, OBSERVATION_COMPONENT_NUM_CHANNELS.at(render_pass.first)};
        box.dtype = OBSERVATION_COMPONENT_DTYPE.at(render_pass.first);
        observation_space[render_pass.first] = std::move(box);
    }

    return observation_space;
}

std::map<std::string, std::vector<uint8_t>> CameraSensor::getObservation() const
{
    std::map<std::string, std::vector<uint8_t>> observation;

    std::map<std::string, TArray<FColor>> render_data = getRenderData();
    for (auto& render_data_component : render_data) {

        if (render_data_component.first == "final_color" || render_data_component.first == "segmentation" || render_data_component.first == "normals") {

            std::vector<uint8_t> observation_vector(height_ * width_ * 3);
            TArray<FColor>& render_data_component_array = render_data_component.second;

            for (int i = 0; i < render_data_component_array.Num(); i++) {
                observation_vector[3 * i + 0] = render_data_component_array[i].R;
                observation_vector[3 * i + 1] = render_data_component_array[i].G;
                observation_vector[3 * i + 2] = render_data_component_array[i].B;
            }

            observation[render_data_component.first] = std::move(observation_vector);

        } else if (render_data_component.first == "depth_glsl") {
            std::vector<float> observation_vector = getFloatDepthFromColorDepth(render_data_component.second);
            observation[render_data_component.first] = Serialize::toUint8(observation_vector);

        } else {
            ASSERT(false);
        }
    }

    return observation;
}

void CameraSensor::initializeSceneCaptureComponentFinalColor(USceneCaptureComponent2D* scene_capture_component)
{
    // update auto-exposure settings
    scene_capture_component->PostProcessSettings.bOverride_AutoExposureSpeedUp   = Config::getValue<bool> ({"SIMULATION_CONTROLLER", "CAMERA_SENSOR", "FINAL_COLOR_OVERRIDE_AUTO_EXPOSURE_SPEED_UP"});
    scene_capture_component->PostProcessSettings.AutoExposureSpeedUp             = Config::getValue<float>({"SIMULATION_CONTROLLER", "CAMERA_SENSOR", "FINAL_COLOR_AUTO_EXPOSURE_SPEED_UP"});
    scene_capture_component->PostProcessSettings.bOverride_AutoExposureSpeedDown = Config::getValue<bool> ({"SIMULATION_CONTROLLER", "CAMERA_SENSOR", "FINAL_COLOR_OVERRIDE_AUTO_EXPOSURE_SPEED_DOWN"});
    scene_capture_component->PostProcessSettings.AutoExposureSpeedDown           = Config::getValue<float>({"SIMULATION_CONTROLLER", "CAMERA_SENSOR", "FINAL_COLOR_AUTO_EXPOSURE_SPEED_DOWN"});

    // enable raytracing features
    scene_capture_component->bUseRayTracingIfEnabled = Config::getValue<bool>({"SIMULATION_CONTROLLER", "CAMERA_SENSOR", "FINAL_COLOR_USE_RAYTRACING_IF_ENABLED"});

    // update indirect lighting 
    scene_capture_component->PostProcessSettings.bOverride_IndirectLightingIntensity = Config::getValue<bool> ({"SIMULATION_CONTROLLER", "CAMERA_SENSOR", "FINAL_COLOR_OVERRIDE_INDIRECT_LIGHTING_INTENSITY"});
    scene_capture_component->PostProcessSettings.IndirectLightingIntensity           = Config::getValue<float>({"SIMULATION_CONTROLLER", "CAMERA_SENSOR", "FINAL_COLOR_INDIRECT_LIGHTING_INTENSITY"});

    // update raytracing global illumination
    scene_capture_component->PostProcessSettings.bOverride_RayTracingGI = Config::getValue<bool>({"SIMULATION_CONTROLLER", "CAMERA_SENSOR", "FINAL_COLOR_OVERRIDE_RAYTRACING_GI"});
    auto raytracing_gi_type = Config::getValue<std::string>({"SIMULATION_CONTROLLER", "CAMERA_SENSOR", "FINAL_COLOR_RAYTRACING_GI_TYPE"});
    if (raytracing_gi_type == "BruteForce") {
        scene_capture_component->PostProcessSettings.RayTracingGIType = ERayTracingGlobalIlluminationType::BruteForce;
    } else if (raytracing_gi_type != "") {
        ASSERT(false);
    }

    scene_capture_component->PostProcessSettings.bOverride_RayTracingGIMaxBounces      = Config::getValue<bool>         ({"SIMULATION_CONTROLLER", "CAMERA_SENSOR", "FINAL_COLOR_OVERRIDE_RAYTRACING_GI_MAX_BOUNCES"});
    scene_capture_component->PostProcessSettings.RayTracingGIMaxBounces                = Config::getValue<unsigned long>({"SIMULATION_CONTROLLER", "CAMERA_SENSOR", "FINAL_COLOR_RAYTRACING_GI_MAX_BOUNCES"});
    scene_capture_component->PostProcessSettings.bOverride_RayTracingGISamplesPerPixel = Config::getValue<bool>         ({"SIMULATION_CONTROLLER", "CAMERA_SENSOR", "FINAL_COLOR_OVERRIDE_RAYTRACING_GI_SAMPLES_PER_PIXEL"});
    scene_capture_component->PostProcessSettings.RayTracingGISamplesPerPixel           = Config::getValue<unsigned long>({"SIMULATION_CONTROLLER", "CAMERA_SENSOR", "FINAL_COLOR_RAYTRACING_GI_SAMPLES_PER_PIXEL"});

    // update raytracing ambient occlusion
    scene_capture_component->PostProcessSettings.bOverride_RayTracingAO                = Config::getValue<bool>         ({"SIMULATION_CONTROLLER", "CAMERA_SENSOR", "FINAL_COLOR_OVERRIDE_RAYTRACING_AO"});
    scene_capture_component->PostProcessSettings.RayTracingAO                          = Config::getValue<unsigned long>({"SIMULATION_CONTROLLER", "CAMERA_SENSOR", "FINAL_COLOR_RAYTRACING_AO"});
    scene_capture_component->PostProcessSettings.bOverride_RayTracingAOSamplesPerPixel = Config::getValue<bool>         ({"SIMULATION_CONTROLLER", "CAMERA_SENSOR", "FINAL_COLOR_OVERRIDE_RAYTRACING_AO_SAMPLES_PER_PIXEL"});
    scene_capture_component->PostProcessSettings.RayTracingAOSamplesPerPixel           = Config::getValue<unsigned long>({"SIMULATION_CONTROLLER", "CAMERA_SENSOR", "FINAL_COLOR_RAYTRACING_AO_SAMPLES_PER_PIXEL"});
    scene_capture_component->PostProcessSettings.bOverride_RayTracingAOIntensity       = Config::getValue<bool>         ({"SIMULATION_CONTROLLER", "CAMERA_SENSOR", "FINAL_COLOR_OVERRIDE_RAYTRACING_AO_INTENSITY"});
    scene_capture_component->PostProcessSettings.RayTracingAOIntensity                 = Config::getValue<float>        ({"SIMULATION_CONTROLLER", "CAMERA_SENSOR", "FINAL_COLOR_RAYTRACING_AO_INTENSITY"});
    scene_capture_component->PostProcessSettings.bOverride_RayTracingAORadius          = Config::getValue<bool>         ({"SIMULATION_CONTROLLER", "CAMERA_SENSOR", "FINAL_COLOR_OVERRIDE_RAYTRACING_AO_RADIUS"});
    scene_capture_component->PostProcessSettings.RayTracingAORadius                    = Config::getValue<float>        ({"SIMULATION_CONTROLLER", "CAMERA_SENSOR", "FINAL_COLOR_RAYTRACING_AO_RADIUS"});

    // update raytracing reflections
    scene_capture_component->PostProcessSettings.bOverride_ReflectionsType = Config::getValue<bool>({"SIMULATION_CONTROLLER", "CAMERA_SENSOR", "FINAL_COLOR_OVERRIDE_REFLECTIONS_TYPE"});
    auto reflections_type = Config::getValue<std::string>({"SIMULATION_CONTROLLER", "CAMERA_SENSOR", "FINAL_COLOR_REFLECTIONS_TYPE"});
    if (reflections_type == "RayTracing") {
        scene_capture_component->PostProcessSettings.ReflectionsType = EReflectionsType::RayTracing; 
    } else if (reflections_type != "") {
        ASSERT(false); 
    }

    scene_capture_component->PostProcessSettings.bOverride_RayTracingReflectionsMaxBounces      = Config::getValue<bool>         ({"SIMULATION_CONTROLLER", "CAMERA_SENSOR", "FINAL_COLOR_OVERRIDE_RAYTRACING_REFLECTIONS_MAX_BOUNCES"});
    scene_capture_component->PostProcessSettings.RayTracingReflectionsMaxBounces                = Config::getValue<unsigned long>({"SIMULATION_CONTROLLER", "CAMERA_SENSOR", "FINAL_COLOR_RAYTRACING_REFLECTIONS_MAX_BOUNCES"});
    scene_capture_component->PostProcessSettings.bOverride_RayTracingReflectionsMaxRoughness    = Config::getValue<bool>         ({"SIMULATION_CONTROLLER", "CAMERA_SENSOR", "FINAL_COLOR_OVERRIDE_RAYTRACING_REFLECTIONS_MAX_ROUGHNESS"});
    scene_capture_component->PostProcessSettings.RayTracingReflectionsMaxRoughness              = Config::getValue<float>        ({"SIMULATION_CONTROLLER", "CAMERA_SENSOR", "FINAL_COLOR_RAYTRACING_REFLECTIONS_MAX_ROUGHNESS"});
    scene_capture_component->PostProcessSettings.bOverride_RayTracingReflectionsSamplesPerPixel = Config::getValue<bool>         ({"SIMULATION_CONTROLLER", "CAMERA_SENSOR", "FINAL_COLOR_OVERRIDE_RAYTRACING_REFLECTIONS_SAMPLES_PER_PIXEL"});
    scene_capture_component->PostProcessSettings.RayTracingReflectionsSamplesPerPixel           = Config::getValue<unsigned long>({"SIMULATION_CONTROLLER", "CAMERA_SENSOR", "FINAL_COLOR_RAYTRACING_REFLECTIONS_SAMPLES_PER_PIXEL"});
    scene_capture_component->PostProcessSettings.bOverride_RayTracingReflectionsTranslucency    = Config::getValue<bool>         ({"SIMULATION_CONTROLLER", "CAMERA_SENSOR", "FINAL_COLOR_OVERRIDE_RAYTRACING_REFLECTIONS_TRANSLUCENCY"});
    scene_capture_component->PostProcessSettings.RayTracingReflectionsTranslucency              = Config::getValue<unsigned long>({"SIMULATION_CONTROLLER", "CAMERA_SENSOR", "FINAL_COLOR_RAYTRACING_REFLECTIONS_TRANSLUCENCY"});

    // update show flags
    scene_capture_component->ShowFlags.SetAmbientOcclusion      (Config::getValue<bool>({"SIMULATION_CONTROLLER", "CAMERA_SENSOR", "FINAL_COLOR_SET_AMBIENT_OCCLUSION"}));        // enabled by FEngineShowFlags::EnableAdvancedFeatures()
    scene_capture_component->ShowFlags.SetCameraImperfections   (Config::getValue<bool>({"SIMULATION_CONTROLLER", "CAMERA_SENSOR", "FINAL_COLOR_SET_CAMERA_IMPERFECTIONS"}));     // enabled by FEngineShowFlags::EnableAdvancedFeatures()
    scene_capture_component->ShowFlags.SetColorGrading          (Config::getValue<bool>({"SIMULATION_CONTROLLER", "CAMERA_SENSOR", "FINAL_COLOR_SET_COLOR_GRADING"}));            // enabled by FEngineShowFlags::EnableAdvancedFeatures()
    scene_capture_component->ShowFlags.SetDepthOfField          (Config::getValue<bool>({"SIMULATION_CONTROLLER", "CAMERA_SENSOR", "FINAL_COLOR_SET_DEPTH_OF_FIELD"}));           // enabled by FEngineShowFlags::EnableAdvancedFeatures()
    scene_capture_component->ShowFlags.SetDistanceFieldAO       (Config::getValue<bool>({"SIMULATION_CONTROLLER", "CAMERA_SENSOR", "FINAL_COLOR_SET_DISTANCE_FIELD_AO"}));        // enabled by FEngineShowFlags::EnableAdvancedFeatures()
    scene_capture_component->ShowFlags.SetEyeAdaptation         (Config::getValue<bool>({"SIMULATION_CONTROLLER", "CAMERA_SENSOR", "FINAL_COLOR_SET_EYE_ADAPTATION"}));           // enabled by FEngineShowFlags::EnableAdvancedFeatures()
    scene_capture_component->ShowFlags.SetGrain                 (Config::getValue<bool>({"SIMULATION_CONTROLLER", "CAMERA_SENSOR", "FINAL_COLOR_SET_GRAIN"}));                    // enabled by FEngineShowFlags::EnableAdvancedFeatures()
    scene_capture_component->ShowFlags.SetIndirectLightingCache (Config::getValue<bool>({"SIMULATION_CONTROLLER", "CAMERA_SENSOR", "FINAL_COLOR_SET_INDIRECT_LIGHTING_CACHE"}));  // enabled by FEngineShowFlags::EnableAdvancedFeatures()
    scene_capture_component->ShowFlags.SetLensFlares            (Config::getValue<bool>({"SIMULATION_CONTROLLER", "CAMERA_SENSOR", "FINAL_COLOR_SET_LENS_FLARES"}));              // enabled by FEngineShowFlags::EnableAdvancedFeatures()
    scene_capture_component->ShowFlags.SetLightShafts           (Config::getValue<bool>({"SIMULATION_CONTROLLER", "CAMERA_SENSOR", "FINAL_COLOR_SET_LIGHT_SHAFTS"}));             // enabled by FEngineShowFlags::EnableAdvancedFeatures()
    scene_capture_component->ShowFlags.SetScreenSpaceReflections(Config::getValue<bool>({"SIMULATION_CONTROLLER", "CAMERA_SENSOR", "FINAL_COLOR_SET_SCREEN_SPACE_REFLECTIONS"})); // enabled by FEngineShowFlags::EnableAdvancedFeatures()
    scene_capture_component->ShowFlags.SetSeparateTranslucency  (Config::getValue<bool>({"SIMULATION_CONTROLLER", "CAMERA_SENSOR", "FINAL_COLOR_SET_SEPARATE_TRANSLUCENCY"}));    // enabled by FEngineShowFlags::EnableAdvancedFeatures()
    scene_capture_component->ShowFlags.SetTemporalAA            (Config::getValue<bool>({"SIMULATION_CONTROLLER", "CAMERA_SENSOR", "FINAL_COLOR_SET_TEMPORAL_AA"}));              // enabled by FEngineShowFlags::EnableAdvancedFeatures()
    scene_capture_component->ShowFlags.SetVignette              (Config::getValue<bool>({"SIMULATION_CONTROLLER", "CAMERA_SENSOR", "FINAL_COLOR_SET_VIGNETTE"}));                 // enabled by FEngineShowFlags::EnableAdvancedFeatures()

    scene_capture_component->ShowFlags.SetAntiAliasing                 (Config::getValue<bool>({"SIMULATION_CONTROLLER", "CAMERA_SENSOR", "FINAL_COLOR_SET_ANTI_ALIASING"}));
    scene_capture_component->ShowFlags.SetRayTracedDistanceFieldShadows(Config::getValue<bool>({"SIMULATION_CONTROLLER", "CAMERA_SENSOR", "FINAL_COLOR_SET_RAYTRACED_DISTANCE_FIELD_SHADOWS"}));
    scene_capture_component->ShowFlags.SetDynamicShadows               (Config::getValue<bool>({"SIMULATION_CONTROLLER", "CAMERA_SENSOR", "FINAL_COLOR_SET_DYNAMIC_SHADOWS"}));

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
        TArray<FColor> render_data_component_array;
        FReadSurfaceDataFlags read_surface_data_flags(RCM_UNorm, CubeFace_MAX);
        read_surface_data_flags.SetLinearToGamma(false);

        ENQUEUE_RENDER_COMMAND(ReadSurfaceCommand)([rhi_texture, rect, &render_data_component_array, read_surface_data_flags](FRHICommandListImmediate& RHICmdList) {
            RHICmdList.ReadSurfaceData(rhi_texture, rect, render_data_component_array, read_surface_data_flags);
        });

        FRenderCommandFence ReadPixelFence;
        ReadPixelFence.BeginFence(true);
        ReadPixelFence.Wait(true);

        render_data[render_pass.first] = std::move(render_data_component_array);
    }

    return render_data;
}

std::vector<float> CameraSensor::getFloatDepthFromColorDepth(TArray<FColor>& color_depth)
{
    std::vector<float> float_depth;
    for (int i = 0; i < color_depth.Num(); i++) {
        float depth = color_depth[i].R  + (color_depth[i].G * 256) + (color_depth[i].B * 256 * 256); 
        float normalized_depth = depth / ((256 * 256 * 256) - 1);
        float dist = normalized_depth * 10;
        float_depth.push_back(dist);
    }
    return float_depth;
}

