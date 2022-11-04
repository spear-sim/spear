#include "CameraSensor.h"

#include <map>
#include <string>
#include <utility>
#include <vector>

#include <Components/SceneCaptureComponent2D.h>
#include <Components/SceneComponent.h>
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

CameraSensor::CameraSensor(AActor* actor, USceneComponent* component_to_attach_to, std::vector<std::string> pass_names, unsigned long width, unsigned long height)
{
    ASSERT(actor);

    for (const auto& pass_name : pass_names) {
        // create SceneCaptureComponent2D
        USceneCaptureComponent2D* scene_capture_component = NewObject<USceneCaptureComponent2D>(actor, *FString::Printf(TEXT("SceneCaptureComponent2D_%s"), pass_name.c_str()));
        ASSERT(scene_capture_component);

        scene_capture_component->AttachToComponent(component_to_attach_to, FAttachmentTransformRules::SnapToTargetNotIncludingScale);
        scene_capture_component->SetVisibility(true);

        // create TextureRenderTarget2D
        UTextureRenderTarget2D* texture_render_target = NewObject<UTextureRenderTarget2D>(actor, *FString::Printf(TEXT("TextureRenderTarget2D_%s"), pass_name.c_str()));
        ASSERT(texture_render_target);
        
        // Set Camera Parameters
        setCameraParameters(scene_capture_component, texture_render_target, width, height);

        if (pass_name != "final_color") {
            setCameraParametersNonFinalColor(scene_capture_component, texture_render_target, width, height);

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
    texture_render_target->InitCustomFormat(width, height, PF_B8G8R8A8, true ); // PF_B8G8R8A8 disables HDR; 
    texture_render_target->RenderTargetFormat = ETextureRenderTargetFormat::RTF_RGBA8;
    texture_render_target->bGPUSharedFlag = true; // demand buffer on GPU - might improve performance?
    texture_render_target->TargetGamma = GEngine->GetDisplayGamma();
    texture_render_target->SRGB = false; // false for pixels to be stored in linear space
    texture_render_target->bAutoGenerateMips = false;
    texture_render_target->UpdateResourceImmediate(true);

    // Set TextureRenderTarget2D it into SceneCaptureComponent2D   
    scene_capture_component->TextureTarget = texture_render_target;
    scene_capture_component->RegisterComponent();

    // SET OVERRIDES
    scene_capture_component->PostProcessSettings.bOverride_AutoExposureMethod = true;
    scene_capture_component->PostProcessSettings.AutoExposureMethod = EAutoExposureMethod::AEM_Histogram;

    scene_capture_component->PostProcessSettings.bOverride_AutoExposureBias = true;
    scene_capture_component->PostProcessSettings.bOverride_AutoExposureMinBrightness = true;
    scene_capture_component->PostProcessSettings.bOverride_AutoExposureMaxBrightness = true;
    scene_capture_component->PostProcessSettings.AutoExposureBias = 1.0f;
    scene_capture_component->PostProcessSettings.AutoExposureMaxBrightness = 5.0f; //-1
    scene_capture_component->PostProcessSettings.AutoExposureMinBrightness = -10.0f; //-1
    
    scene_capture_component->PostProcessSettings.bOverride_AutoExposureSpeedUp = true;
    scene_capture_component->PostProcessSettings.AutoExposureSpeedUp = 10.0f;
    scene_capture_component->PostProcessSettings.bOverride_AutoExposureSpeedDown = true;
    scene_capture_component->PostProcessSettings.AutoExposureSpeedDown = 10.0f;

    scene_capture_component->PostProcessSettings.bOverride_AutoExposureCalibrationConstant_DEPRECATED = true;
    scene_capture_component->PostProcessSettings.bOverride_HistogramLogMin = true;
    scene_capture_component->PostProcessSettings.HistogramLogMin = -10.f; // 1.0f;
    scene_capture_component->PostProcessSettings.bOverride_HistogramLogMax = true;
    scene_capture_component->PostProcessSettings.HistogramLogMax = 10.f; // 12.0f;

    // Camera
    scene_capture_component->PostProcessSettings.bOverride_CameraShutterSpeed = true;
    scene_capture_component->PostProcessSettings.bOverride_CameraISO = true;
    scene_capture_component->PostProcessSettings.bOverride_DepthOfFieldFstop = true;
    scene_capture_component->PostProcessSettings.bOverride_DepthOfFieldMinFstop = true;
    scene_capture_component->PostProcessSettings.bOverride_DepthOfFieldBladeCount = true;

    // Film (Tonemapper)
    //scene_capture_component->PostProcessSettings.bOverride_FilmSlope = true;
    //scene_capture_component->PostProcessSettings.bOverride_FilmToe = true;
    //scene_capture_component->PostProcessSettings.bOverride_FilmShoulder = true;
    //scene_capture_component->PostProcessSettings.bOverride_FilmWhiteClip = true;
    //scene_capture_component->PostProcessSettings.bOverride_FilmBlackClip = true;

    // Motion blur
    scene_capture_component->PostProcessSettings.bOverride_MotionBlurAmount = true;
    scene_capture_component->PostProcessSettings.MotionBlurAmount = 0.6f;//0.45f;
    scene_capture_component->PostProcessSettings.bOverride_MotionBlurMax = true;
    scene_capture_component->PostProcessSettings.MotionBlurMax = 60.f;//0.35f;
    scene_capture_component->PostProcessSettings.bOverride_MotionBlurPerObjectSize = true;
    scene_capture_component->PostProcessSettings.MotionBlurPerObjectSize = 4.0f;//0.1f;

    // Color Grading
//    scene_capture_component->PostProcessSettings.bOverride_WhiteTemp = true;
//    scene_capture_component->PostProcessSettings.bOverride_WhiteTint = true;
//    scene_capture_component->PostProcessSettings.bOverride_ColorContrast = true;
//#if PLATFORM_LINUX
//    // Looks like Windows and Linux have different outputs with the
//    // same exposure compensation, this fixes it.
//    scene_capture_component->PostProcessSettings.ColorContrast = FVector4(1.2f, 1.2f, 1.2f, 1.0f);
//#endif

    // Chromatic Aberration
    //scene_capture_component->PostProcessSettings.bOverride_SceneFringeIntensity = true;
    //scene_capture_component->PostProcessSettings.bOverride_ChromaticAberrationStartOffset = true;

    // Ambient Occlusion
    //scene_capture_component->PostProcessSettings.bOverride_AmbientOcclusionIntensity = true;
    //scene_capture_component->PostProcessSettings.AmbientOcclusionIntensity = 0.5f;
    //scene_capture_component->PostProcessSettings.bOverride_AmbientOcclusionRadius = true;
    //scene_capture_component->PostProcessSettings.AmbientOcclusionRadius = 100.0f;
    //scene_capture_component->PostProcessSettings.bOverride_AmbientOcclusionStaticFraction = true;
    //scene_capture_component->PostProcessSettings.AmbientOcclusionStaticFraction = 1.0f;
    //scene_capture_component->PostProcessSettings.bOverride_AmbientOcclusionFadeDistance = true;
    //scene_capture_component->PostProcessSettings.AmbientOcclusionFadeDistance = 50000.0f;
    //scene_capture_component->PostProcessSettings.bOverride_AmbientOcclusionPower = true;
    //scene_capture_component->PostProcessSettings.AmbientOcclusionPower = 2.0f;
    //scene_capture_component->PostProcessSettings.bOverride_AmbientOcclusionBias = true;
    //scene_capture_component->PostProcessSettings.AmbientOcclusionBias = 3.0f;
    //scene_capture_component->PostProcessSettings.bOverride_AmbientOcclusionQuality = true;
    //scene_capture_component->PostProcessSettings.AmbientOcclusionQuality = 100.0f;

    // Bloom
    scene_capture_component->PostProcessSettings.bOverride_BloomMethod = true;
    scene_capture_component->PostProcessSettings.BloomMethod = EBloomMethod::BM_SOG;
    scene_capture_component->PostProcessSettings.bOverride_BloomIntensity = true;
    scene_capture_component->PostProcessSettings.BloomIntensity = 0.4f;
    scene_capture_component->PostProcessSettings.bOverride_BloomThreshold = true;
    scene_capture_component->PostProcessSettings.BloomThreshold = -1.0f;

    // Lens
    //scene_capture_component->PostProcessSettings.bOverride_LensFlareIntensity = true;
    //scene_capture_component->PostProcessSettings.LensFlareIntensity = 0.1;

    // Raytracing
    scene_capture_component->bUseRayTracingIfEnabled = true;
    scene_capture_component->PostProcessSettings.bOverride_IndirectLightingIntensity = true;
    scene_capture_component->PostProcessSettings.IndirectLightingIntensity = 0.2f;

    scene_capture_component->PostProcessSettings.bOverride_RayTracingGI = true;
    scene_capture_component->PostProcessSettings.RayTracingGIType = ERayTracingGlobalIlluminationType::BruteForce;
    //scene_capture_component->PostProcessSettings.RayTracingGIType = ERayTracingGlobalIlluminationType::BruteForce;
    scene_capture_component->PostProcessSettings.bOverride_RayTracingGIMaxBounces = true;
    scene_capture_component->PostProcessSettings.bOverride_RayTracingGISamplesPerPixel = true;

    scene_capture_component->PostProcessSettings.RayTracingGIMaxBounces = 4;
    scene_capture_component->PostProcessSettings.RayTracingGISamplesPerPixel = 16;

    scene_capture_component->PostProcessSettings.bOverride_RayTracingAO = true; // Default value
    scene_capture_component->PostProcessSettings.bOverride_RayTracingAOSamplesPerPixel = true; // min 1 - max 64
    scene_capture_component->PostProcessSettings.bOverride_RayTracingAOIntensity = true; // min 0.0 - max 1.0
    scene_capture_component->PostProcessSettings.bOverride_RayTracingAORadius = true; // min 0.0 - max 10000.0

    //scene_capture_component->PostProcessSettings.RayTracingAO = 1; // Default value
    scene_capture_component->PostProcessSettings.RayTracingAOSamplesPerPixel = 8; // min 1 - max 64
    scene_capture_component->PostProcessSettings.RayTracingAOIntensity = 0.6f; // min 0.0 - max 1.0
    scene_capture_component->PostProcessSettings.RayTracingAORadius = 4000.0f; // min 0.0 - max 10000.0

    //scene_capture_component->PostProcessSettings.bOverride_TranslucencyType = true;
    //scene_capture_component->PostProcessSettings.TranslucencyType = ETranslucencyType::RayTracing;
    ////scene_capture_component->PostProcessSettings.RayTracingTranslucencyShadows = EReflectedAndRefractedRayTracedShadows::Hard_shadows;
    //scene_capture_component->PostProcessSettings.bOverride_RayTracingTranslucencyMaxRoughness = true; // min 0.01 - max 1.0
    //scene_capture_component->PostProcessSettings.bOverride_RayTracingTranslucencyRefractionRays = true; // min 0 - max 50
    //scene_capture_component->PostProcessSettings.bOverride_RayTracingTranslucencySamplesPerPixel = true; // min 1 - max 64
    //scene_capture_component->PostProcessSettings.bOverride_RayTracingTranslucencyRefraction = true; // default value

    //scene_capture_component->PostProcessSettings.RayTracingTranslucencyMaxRoughness = 0.6f; // min 0.01 - max 1.0
    //scene_capture_component->PostProcessSettings.RayTracingTranslucencyRefractionRays = 4; // min 0 - max 50
    //scene_capture_component->PostProcessSettings.RayTracingTranslucencySamplesPerPixel = 4; // min 1 - max 64
    //scene_capture_component->PostProcessSettings.RayTracingTranslucencyRefraction = 1; // default value

    scene_capture_component->PostProcessSettings.bOverride_ReflectionsType = true;
    scene_capture_component->PostProcessSettings.bOverride_RayTracingReflectionsMaxBounces = true; // min 0 - max 50
    scene_capture_component->PostProcessSettings.bOverride_RayTracingReflectionsSamplesPerPixel = true; // min 1 - max 64
    scene_capture_component->PostProcessSettings.bOverride_RayTracingReflectionsTranslucency = true; // Default value
    scene_capture_component->PostProcessSettings.bOverride_RayTracingReflectionsShadows = true;

    scene_capture_component->PostProcessSettings.ReflectionsType = EReflectionsType::ScreenSpace;
    //scene_capture_component->PostProcessSettings.RayTracingReflectionsMaxRoughness = 0.6f; // min 0.01 - max 1.0
    //scene_capture_component->PostProcessSettings.RayTracingReflectionsMaxBounces = 1; // min 0 - max 50
    //scene_capture_component->PostProcessSettings.RayTracingReflectionsSamplesPerPixel = 6; // min 1 - max 64
    //scene_capture_component->PostProcessSettings.RayTracingReflectionsShadows = EReflectedAndRefractedRayTracedShadows::Area_shadows;

    //// Pathtracing
    //scene_capture_component->PostProcessSettings.PathTracingMaxBounces = 2;      // min 0 - max 50
    //scene_capture_component->PostProcessSettings.PathTracingSamplesPerPixel = 32; // min 1 - max 64
    //scene_capture_component->PostProcessSettings.bOverride_PathTracingEnableEmissive = 1;      // min 0 - max 50
    //scene_capture_component->PostProcessSettings.PathTracingMaxPathExposure = 20.0f;      // min -10 - max 30
    //scene_capture_component->PostProcessSettings.bOverride_PathTracingEnableDenoiser = 1;


    //scene_capture_component->ShowFlags.SetAmbientOcclusion(false);
    scene_capture_component->ShowFlags.SetAntiAliasing(true);
    //scene_capture_component->ShowFlags.SetVolumetricFog(false);
    //scene_capture_component->ShowFlags.SetAtmosphericFog(false);
    //scene_capture_component->ShowFlags.SetAudioRadius(false);
    //scene_capture_component->ShowFlags.SetBillboardSprites(false);
    //scene_capture_component->ShowFlags.SetBloom(false);
    //scene_capture_component->ShowFlags.SetBounds(false);
    //scene_capture_component->ShowFlags.SetBrushes(false);
    //scene_capture_component->ShowFlags.SetBSP(false);
    //scene_capture_component->ShowFlags.SetBSPSplit(false);
    //scene_capture_component->ShowFlags.SetBSPTriangles(false);
    //scene_capture_component->ShowFlags.SetBuilderBrush(false);
    //scene_capture_component->ShowFlags.SetCameraAspectRatioBars(false);
    //scene_capture_component->ShowFlags.SetCameraFrustums(false);
    //scene_capture_component->ShowFlags.SetCameraImperfections(false);
    //scene_capture_component->ShowFlags.SetCameraInterpolation(false);
    //scene_capture_component->ShowFlags.SetCameraSafeFrames(false);
    //scene_capture_component->ShowFlags.SetCollision(false);
    //scene_capture_component->ShowFlags.SetCollisionPawn(false);
    //scene_capture_component->ShowFlags.SetCollisionVisibility(false);
    //scene_capture_component->ShowFlags.SetColorGrading(false);
    //scene_capture_component->ShowFlags.SetCompositeEditorPrimitives(false);
    //scene_capture_component->ShowFlags.SetConstraints(false);
    //scene_capture_component->ShowFlags.SetCover(false);
    //scene_capture_component->ShowFlags.SetDebugAI(false);
    //scene_capture_component->ShowFlags.SetDecals(false);
    //scene_capture_component->ShowFlags.SetDeferredLighting(false);
    //scene_capture_component->ShowFlags.SetDepthOfField(false);
    //scene_capture_component->ShowFlags.SetDiffuse(false);
    //scene_capture_component->ShowFlags.SetDirectionalLights(false);
    //scene_capture_component->ShowFlags.SetDirectLighting(false);
    //scene_capture_component->ShowFlags.SetDistanceCulledPrimitives(false);
    scene_capture_component->ShowFlags.SetDistanceFieldAO(false);
    //scene_capture_component->ShowFlags.SetDistanceFieldGI(false);
    scene_capture_component->ShowFlags.SetRayTracedDistanceFieldShadows(true);
    scene_capture_component->ShowFlags.SetDynamicShadows(true);
    //scene_capture_component->ShowFlags.SetEditor(false);
    scene_capture_component->ShowFlags.SetEyeAdaptation(true);
    //scene_capture_component->ShowFlags.SetFog(false);
    //scene_capture_component->ShowFlags.SetGame(false);
    //scene_capture_component->ShowFlags.SetGameplayDebug(false);
    //scene_capture_component->ShowFlags.SetGBufferHints(false);
    //scene_capture_component->ShowFlags.SetGlobalIllumination(false);
    //scene_capture_component->ShowFlags.SetGrain(false);
    //scene_capture_component->ShowFlags.SetGrid(false);
    //scene_capture_component->ShowFlags.SetHighResScreenshotMask(false);
    //scene_capture_component->ShowFlags.SetHitProxies(false);
    //scene_capture_component->ShowFlags.SetHLODColoration(false);
    //scene_capture_component->ShowFlags.SetHMDDistortion(false);
    //scene_capture_component->ShowFlags.SetIndirectLightingCache(false);
    //scene_capture_component->ShowFlags.SetInstancedFoliage(false);
    //scene_capture_component->ShowFlags.SetInstancedGrass(false);
    //scene_capture_component->ShowFlags.SetInstancedStaticMeshes(false);
    //scene_capture_component->ShowFlags.SetLandscape(false);
    //scene_capture_component->ShowFlags.SetLargeVertices(false);
    //scene_capture_component->ShowFlags.SetLensFlares(false);
    //scene_capture_component->ShowFlags.SetLevelColoration(false);
    //scene_capture_component->ShowFlags.SetLightComplexity(false);
    //scene_capture_component->ShowFlags.SetLightFunctions(false);
    //scene_capture_component->ShowFlags.SetLightInfluences(false);
    //scene_capture_component->ShowFlags.SetLighting(false);
    //scene_capture_component->ShowFlags.SetLightMapDensity(false);
    //scene_capture_component->ShowFlags.SetLightRadius(false);
    //scene_capture_component->ShowFlags.SetLightShafts(false);
    //scene_capture_component->ShowFlags.SetLOD(false);
    //scene_capture_component->ShowFlags.SetLODColoration(false);
    //scene_capture_component->ShowFlags.SetMaterials(false);
    //scene_capture_component->ShowFlags.SetMaterialTextureScaleAccuracy(false);
    //scene_capture_component->ShowFlags.SetMeshEdges(false);
    //scene_capture_component->ShowFlags.SetMeshUVDensityAccuracy(false);
    //scene_capture_component->ShowFlags.SetModeWidgets(false);
    //scene_capture_component->ShowFlags.SetMotionBlur(false);
    //scene_capture_component->ShowFlags.SetNavigation(false);
    //scene_capture_component->ShowFlags.SetOnScreenDebug(false);
    //scene_capture_component->ShowFlags.SetOutputMaterialTextureScales(false);
    //scene_capture_component->ShowFlags.SetOverrideDiffuseAndSpecular(false);
    //scene_capture_component->ShowFlags.SetPaper2DSprites(false);
    //scene_capture_component->ShowFlags.SetParticles(false);
    //scene_capture_component->ShowFlags.SetPivot(false);
    //scene_capture_component->ShowFlags.SetPointLights(false);
    //scene_capture_component->ShowFlags.SetPostProcessing(false);
    //scene_capture_component->ShowFlags.SetPostProcessMaterial(false);
    //scene_capture_component->ShowFlags.SetPrecomputedVisibility(false);
    //scene_capture_component->ShowFlags.SetPrecomputedVisibilityCells(false);
    //scene_capture_component->ShowFlags.SetPreviewShadowsIndicator(false);
    //scene_capture_component->ShowFlags.SetPrimitiveDistanceAccuracy(false);
    //scene_capture_component->ShowFlags.SetPropertyColoration(false);
    //scene_capture_component->ShowFlags.SetQuadOverdraw(false);
    //scene_capture_component->ShowFlags.SetReflectionEnvironment(false);
    //scene_capture_component->ShowFlags.SetReflectionOverride(false);
    //scene_capture_component->ShowFlags.SetRefraction(false);
    //scene_capture_component->ShowFlags.SetRendering(false);
    //scene_capture_component->ShowFlags.SetSceneColorFringe(false);
    //scene_capture_component->ShowFlags.SetScreenPercentage(false);
    //scene_capture_component->ShowFlags.SetScreenSpaceAO(false);
    //scene_capture_component->ShowFlags.SetScreenSpaceReflections(false);
    //scene_capture_component->ShowFlags.SetSelection(false);
    //scene_capture_component->ShowFlags.SetSelectionOutline(false);
    //scene_capture_component->ShowFlags.SetSeparateTranslucency(false);
    //scene_capture_component->ShowFlags.SetShaderComplexity(false);
    //scene_capture_component->ShowFlags.SetShaderComplexityWithQuadOverdraw(false);
    //scene_capture_component->ShowFlags.SetShadowFrustums(false);
    //scene_capture_component->ShowFlags.SetSkeletalMeshes(false);
    //scene_capture_component->ShowFlags.SetSkinCache(false);
    //scene_capture_component->ShowFlags.SetSkyLighting(false);
    //scene_capture_component->ShowFlags.SetSnap(false);
    //scene_capture_component->ShowFlags.SetSpecular(false);
    //scene_capture_component->ShowFlags.SetSplines(false);
    //scene_capture_component->ShowFlags.SetSpotLights(false);
    //scene_capture_component->ShowFlags.SetStaticMeshes(false);
    //scene_capture_component->ShowFlags.SetStationaryLightOverlap(false);
    //scene_capture_component->ShowFlags.SetStereoRendering(false);
    //scene_capture_component->ShowFlags.SetStreamingBounds(false);
    //scene_capture_component->ShowFlags.SetSubsurfaceScattering(false);
    scene_capture_component->ShowFlags.SetTemporalAA(false);
    //scene_capture_component->ShowFlags.SetTessellation(false);
    //scene_capture_component->ShowFlags.SetTestImage(false);
    //scene_capture_component->ShowFlags.SetTextRender(false);
    //scene_capture_component->ShowFlags.SetTexturedLightProfiles(false);
    //scene_capture_component->ShowFlags.SetTonemapper(false);
    //scene_capture_component->ShowFlags.SetTranslucency(false);
    //scene_capture_component->ShowFlags.SetVectorFields(false);
    //scene_capture_component->ShowFlags.SetVertexColors(false);
    //scene_capture_component->ShowFlags.SetVignette(false);
    //scene_capture_component->ShowFlags.SetVisLog(false);
    //scene_capture_component->ShowFlags.SetVisualizeAdaptiveDOF(false);
    //scene_capture_component->ShowFlags.SetVisualizeBloom(false);
    //scene_capture_component->ShowFlags.SetVisualizeBuffer(false);
    //scene_capture_component->ShowFlags.SetVisualizeDistanceFieldAO(false);
    //scene_capture_component->ShowFlags.SetVisualizeDOF(false);
    //scene_capture_component->ShowFlags.SetVisualizeHDR(false);
    //scene_capture_component->ShowFlags.SetVisualizeLightCulling(false);
    //scene_capture_component->ShowFlags.SetVisualizeLPV(false);
    //scene_capture_component->ShowFlags.SetVisualizeMeshDistanceFields(false);
    //scene_capture_component->ShowFlags.SetVisualizeMotionBlur(false);
    //scene_capture_component->ShowFlags.SetVisualizeOutOfBoundsPixels(false);
    //scene_capture_component->ShowFlags.SetVisualizeSenses(false);
    //scene_capture_component->ShowFlags.SetVisualizeShadingModels(false);
    //scene_capture_component->ShowFlags.SetVisualizeSSR(false);
    //scene_capture_component->ShowFlags.SetVisualizeSSS(false);
    //scene_capture_component->ShowFlags.SetVolumeLightingSamples(false);
    //scene_capture_component->ShowFlags.SetVolumes(false);
    //scene_capture_component->ShowFlags.SetWidgetComponents(false);
    //scene_capture_component->ShowFlags.SetWireframe(false);
}

void CameraSensor::setCameraParametersNonFinalColor(USceneCaptureComponent2D* scene_capture_component, UTextureRenderTarget2D* texture_render_target, unsigned long width, unsigned long height)
{
    scene_capture_component->ShowFlags.EnableAdvancedFeatures();
    scene_capture_component->ShowFlags.SetMotionBlur(true);
}
