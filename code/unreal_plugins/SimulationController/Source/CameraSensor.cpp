#include "CameraSensor.h"

#include <map>
#include <string>
#include <utility>
#include <vector>

#include <Camera/CameraComponent.h>
#include <Components/SceneCaptureComponent2D.h>
#include <Engine/TextureRenderTarget2D.h>
#include <Engine/World.h>
#include <EngineUtils.h>
#include <GameFramework/Actor.h>
#include <Materials/MaterialInstanceDynamic.h>
#include <UObject/UObjectGlobals.h>

#include "Assert/Assert.h"
#include "Config.h"
#include "Serialize.h"
#include "TickEvent.h"

const std::string MATERIALS_PATH = "/SimulationController/PostProcessMaterials/";

CameraSensor::CameraSensor(UCameraComponent* component, std::vector<std::string> pass_names, unsigned long width, unsigned long height)
{
    ASSERT(component);

    new_object_parent_actor_ = component->GetWorld()->SpawnActor<AActor>();
    ASSERT(new_object_parent_actor_);

    for (const auto& pass_name : pass_names) {
        // create SceneCaptureComponent2D
        USceneCaptureComponent2D* scene_capture_component = NewObject<USceneCaptureComponent2D>(new_object_parent_actor_, *FString::Printf(TEXT("SceneCaptureComponent2D_%s"), pass_name.c_str()));
        ASSERT(scene_capture_component);

        scene_capture_component->AttachToComponent(component, FAttachmentTransformRules::SnapToTargetNotIncludingScale);
        scene_capture_component->SetVisibility(true);

        // create TextureRenderTarget2D
        UTextureRenderTarget2D* texture_render_target = NewObject<UTextureRenderTarget2D>(new_object_parent_actor_, *FString::Printf(TEXT("TextureRenderTarget2D_%s"), pass_name.c_str()));
        ASSERT(texture_render_target);
        
        // Set Camera Parameters
        setCameraParameters(scene_capture_component, texture_render_target, width, height);

        if (pass_name != "final_color") {
            // Load PostProcessMaterial
            FString path = (MATERIALS_PATH + pass_name + "." + pass_name).c_str();
            UMaterial* mat = LoadObject<UMaterial>(nullptr, *path);
            ASSERT(mat);

            // Set PostProcessMaterial
            scene_capture_component->PostProcessSettings.AddBlendable(UMaterialInstanceDynamic::Create(mat, scene_capture_component), 1.0f);
        }

        CameraPass pass;

        // Set camera pass
        pass.scene_capture_component_ = scene_capture_component;
        pass.texture_render_target_ = texture_render_target;

        // Insert into map
        camera_passes_[pass_name] = std::move(pass);
    }
}

CameraSensor::~CameraSensor()
{
    for (auto& pass: camera_passes_) {
        ASSERT(pass.second.texture_render_target_);
        pass.second.texture_render_target_->MarkPendingKill();
        pass.second.texture_render_target_ = nullptr;

        ASSERT(pass.second.scene_capture_component_);
        pass.second.scene_capture_component_->DestroyComponent();
        pass.second.scene_capture_component_ = nullptr;
    }
    camera_passes_.clear();

    ASSERT(new_object_parent_actor_);
    new_object_parent_actor_->Destroy();
    new_object_parent_actor_ = nullptr;
}

std::map<std::string, TArray<FColor>> CameraSensor::getRenderData()
{
    std::map<std::string, TArray<FColor>> data;

    // Get data from all passes
    for (const auto& pass: camera_passes_) {
        FTextureRenderTargetResource* target_resource = pass.second.scene_capture_component_->TextureTarget->GameThread_GetRenderTargetResource();
        ASSERT(target_resource);
        TArray<FColor> pixels;

        struct FReadSurfaceContext {
            FRenderTarget* src_render_target_;
            TArray<FColor>& out_data_;
            FIntRect rect_;
            FReadSurfaceDataFlags flags_;
        };

        FReadSurfaceContext context = {target_resource, pixels, FIntRect(0, 0, target_resource->GetSizeXY().X, 
                                       target_resource->GetSizeXY().Y), FReadSurfaceDataFlags(RCM_UNorm, CubeFace_MAX)};

        // Required for uint8 read mode
        context.flags_.SetLinearToGamma(false);

        ENQUEUE_RENDER_COMMAND(ReadSurfaceCommand)([context](FRHICommandListImmediate& RHICmdList) {
            RHICmdList.ReadSurfaceData(context.src_render_target_->GetRenderTargetTexture(), context.rect_, context.out_data_, context.flags_);
        });

        FRenderCommandFence ReadPixelFence;
        ReadPixelFence.BeginFence(true);
        ReadPixelFence.Wait(true);

        data[pass.first] = std::move(pixels);
    }

    return data;
}

// depth codification
// decode formula : depth = ((r) + (g * 256) + (b * 256 * 256)) / ((256 * 256 * 256) - 1) * f
std::vector<float> CameraSensor::getFloatDepthFromColorDepth(TArray<FColor> data)
{
    std::vector<float> out;
    for (uint32 i = 0; i < static_cast<uint32>(data.Num()); ++i) {
        float depth = data[i].R  + (data[i].G * 256) + (data[i].B * 256 * 256); 
        float normalized_depth = depth / ((256 * 256 * 256) - 1);
        float dist = normalized_depth * 10; 
        out.push_back(dist);
    }
    return out;
}

void CameraSensor::setCameraParameters(USceneCaptureComponent2D* scene_capture_component, UTextureRenderTarget2D* texture_render_target, unsigned long width, unsigned long height)
{
    // SET BASIC PARAMETERS
    scene_capture_component->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
    scene_capture_component->FOVAngle = 60.f;
    scene_capture_component->bAlwaysPersistRenderingState = true;

    // Initialize TextureRenderTarget2D format. 
    // Changing the dimensions of a TextureRenderTarget2D after they have been set initially can lead to unexpected behavior. 
    // So we only set the dimensions once at object creation time, and we require width and height be passed into the CameraSensor constructor.
    bool force_linear_gamma = true;
    texture_render_target->InitCustomFormat(width, height, PF_B8G8R8A8, force_linear_gamma); // PF_B8G8R8A8 disables HDR; 
    texture_render_target->RenderTargetFormat = ETextureRenderTargetFormat::RTF_RGBA8;
    texture_render_target->bGPUSharedFlag = true; // demand buffer on GPU - might improve performance?
    texture_render_target->TargetGamma = GEngine->GetDisplayGamma();
    texture_render_target->SRGB = true; // false for pixels to be stored in linear space
    texture_render_target->bAutoGenerateMips = false;
    texture_render_target->UpdateResourceImmediate(true);

    // Set TextureRenderTarget2D it into SceneCaptureComponent2D   
    scene_capture_component->TextureTarget = texture_render_target;
    scene_capture_component->RegisterComponent();

    // raytracing
    scene_capture_component->bUseRayTracingIfEnabled = true;

    // Set ShowFlags
    scene_capture_component->ShowFlags.SetAmbientOcclusion(true);               // enabled by EnableAdvancedFeatures();
    scene_capture_component->ShowFlags.SetAntiAliasing(true);
    scene_capture_component->ShowFlags.SetCameraImperfections(true);            // enabled by EnableAdvancedFeatures();
    scene_capture_component->ShowFlags.SetColorGrading(true);                   // enabled by EnableAdvancedFeatures();
    scene_capture_component->ShowFlags.SetDepthOfField(true);                   // enabled by EnableAdvancedFeatures();
    scene_capture_component->ShowFlags.SetDistanceFieldAO(true);                // enabled by EnableAdvancedFeatures();
    scene_capture_component->ShowFlags.SetRayTracedDistanceFieldShadows(true);
    scene_capture_component->ShowFlags.SetDynamicShadows(true);
    scene_capture_component->ShowFlags.SetEyeAdaptation(true);                  // enabled by EnableAdvancedFeatures();
    scene_capture_component->ShowFlags.SetGrain(true);                          // enabled by EnableAdvancedFeatures();
    scene_capture_component->ShowFlags.SetIndirectLightingCache(true);          // enabled by EnableAdvancedFeatures();
    scene_capture_component->ShowFlags.SetLensFlares(true);                     // enabled by EnableAdvancedFeatures();
    scene_capture_component->ShowFlags.SetLightShafts(true);                    // enabled by EnableAdvancedFeatures();
    scene_capture_component->ShowFlags.SetPostProcessMaterial(true);            // enabled by EnableAdvancedFeatures();
    scene_capture_component->ShowFlags.SetScreenSpaceReflections(true);         // enabled by EnableAdvancedFeatures();
    scene_capture_component->ShowFlags.SetSeparateTranslucency(true);           // enabled by EnableAdvancedFeatures();
    scene_capture_component->ShowFlags.SetTemporalAA(true);                     // enabled by EnableAdvancedFeatures();
    scene_capture_component->ShowFlags.SetVignette(true);                       // enabled by EnableAdvancedFeatures();
}
