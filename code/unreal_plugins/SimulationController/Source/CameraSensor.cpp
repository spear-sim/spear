#include "CameraSensor.h"

#include <string>
#include <utility>
#include <vector>
#include <map>
#include <thread>

#include <Components/SceneCaptureComponent2D.h>
#include <Engine/TextureRenderTarget2D.h>
#include "Kismet/KismetSystemLibrary.h"
#include "Materials/MaterialInstanceDynamic.h"
#include <Engine/World.h>
#include <EngineUtils.h>
#include <GameFramework/Actor.h>
#include <UObject/UObjectGlobals.h>

#include "Assert.h"
#include "Config.h"
#include "Serialize.h"
#include "TickEvent.h"

CameraSensor::CameraSensor(UWorld* world, AActor* actor_){
    camera_actor_ = actor_;
    ASSERT(camera_actor_);

    new_object_parent_actor_ = world->SpawnActor<AActor>();
    ASSERT(new_object_parent_actor_);

    // create SceneCaptureComponent2D and TextureRenderTarget2D
    this->scene_capture_component_ = NewObject<USceneCaptureComponent2D>(new_object_parent_actor_, TEXT("SceneCaptureComponent2D"));
    ASSERT(scene_capture_component_);       
    this->scene_capture_component_->AttachToComponent(camera_actor_->GetRootComponent(), FAttachmentTransformRules::SnapToTargetNotIncludingScale);
    this->scene_capture_component_->SetVisibility(true);
    this->scene_capture_component_->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
    this->scene_capture_component_->bAlwaysPersistRenderingState = true;

    SetCameraDefaultOverrides();    
    ConfigureShowFlags(this->enable_postprocessing_effects_); 

    UKismetSystemLibrary::ExecuteConsoleCommand(world, FString("g.TimeoutForBlockOnRenderFence 300000"));

    this->texture_render_target_ = NewObject<UTextureRenderTarget2D>(new_object_parent_actor_, TEXT("TextureRenderTarget2D"));
    ASSERT(texture_render_target_);  

    pre_render_tick_event_ = NewObject<UTickEvent>(new_object_parent_actor_, TEXT("PreRenderTickEvent"));
    ASSERT(pre_render_tick_event_);
    pre_render_tick_event_->RegisterComponent();
    pre_render_tick_event_->initialize(ETickingGroup::TG_PostPhysics);
    pre_render_tick_event_handle_ = pre_render_tick_event_->delegate_.AddRaw(this, &CameraSensor::PreRenderTickEventHandler); 
}

CameraSensor::~CameraSensor(){
    //check if we need it 
    ASSERT(pre_render_tick_event_);
    pre_render_tick_event_->delegate_.Remove(pre_render_tick_event_handle_);
    pre_render_tick_event_handle_.Reset();
    pre_render_tick_event_->DestroyComponent();
    pre_render_tick_event_ = nullptr;

    ASSERT(this->texture_render_target_);
    this->texture_render_target_->MarkPendingKill();
    this->texture_render_target_ = nullptr;

    ASSERT(this->scene_capture_component_);
    this->scene_capture_component_->DestroyComponent();
    this->scene_capture_component_ = nullptr;

    ASSERT(this->new_object_parent_actor_);
    this->new_object_parent_actor_->Destroy();
    this->new_object_parent_actor_ = nullptr; 

    ASSERT(this->camera_actor_);
    this->camera_actor_ = nullptr;
}

void CameraSensor::SetRenderTarget(unsigned long w, unsigned long h){
    this->texture_render_target_->InitCustomFormat(w ,h ,PF_B8G8R8A8 ,true ); // PF_B8G8R8A8 disables HDR;
    this->texture_render_target_->RenderTargetFormat = ETextureRenderTargetFormat::RTF_RGBA8;
    this->texture_render_target_->bGPUSharedFlag = true; // demand buffer on GPU - might improve performance?
    this->texture_render_target_->TargetGamma = GEngine->GetDisplayGamma();
    //this->texture_render_target_->TargetGamma = 1.2f;
    //this->texture_render_target_->SRGB = false; // false for pixels to be stored in linear space
    //this->texture_render_target_->bAutoGenerateMips = false;
    this->texture_render_target_->UpdateResourceImmediate(true);    
    this->scene_capture_component_->TextureTarget = texture_render_target_;
    this->scene_capture_component_->RegisterComponent();
}

void CameraSensor::SetPostProcessBlendables(std::vector<std::string> blendables){
    for(std::string pass_name_ : blendables){
        ASSERT(std::find(PASSES_.begin(), PASSES_.end(), pass_name_) != PASSES_.end());
    }

    //Set blendables
    unsigned long pass_index_ = 0;
    for(std::string pass_name_ : blendables){
        FString path_ = MATERIALS_PATH_ + pass_name_.c_str() + "." + pass_name_.c_str();
        UMaterial* mat = LoadObject<UMaterial>(nullptr, *path_);
        ASSERT(mat);
        this->scene_capture_component_->PostProcessSettings.AddBlendable(UMaterialInstanceDynamic::Create(mat, this->scene_capture_component_), 0.0f);
                
        passes_.insert( std::pair<std::string, unsigned long> (pass_name_, pass_index_));
        pass_index_++;
    }   
}

void CameraSensor::ActivateBlendablePass(std::string pass_name){
    ASSERT(pass_name != "");

    this->scene_capture_component_->PostProcessBlendWeight = 0.0f;
    for(int i=0;i < this->scene_capture_component_->PostProcessSettings.WeightedBlendables.Array.Num();i++){
        this->scene_capture_component_->PostProcessSettings.WeightedBlendables.Array[i].Weight = 0.0f;
    }

    if(pass_name != "finalColor"){
        this->scene_capture_component_->PostProcessBlendWeight = 1.0f;
        this->scene_capture_component_->PostProcessSettings.WeightedBlendables.Array[int(passes_.find(pass_name)->second)].Weight = 1.0f;
    }
    //pre_loaded_pass_ = pass_name;
}

void CameraSensor::PreRenderTickEventHandler(float delta_time, enum ELevelTick level_tick)
{
    printf("UE4 cameraSensor - tick event : executing tick pre render \n");
}

TArray<FColor> CameraSensor::GetRenderData(){
    FTextureRenderTargetResource* target_resource = this->scene_capture_component_->TextureTarget->GameThread_GetRenderTargetResource();
    ASSERT(target_resource);
    TArray<FColor> pixels;

    struct FReadSurfaceContext
    {
        FRenderTarget* src_render_target_;
        TArray<FColor>& out_data_;
        FIntRect rect_;
        FReadSurfaceDataFlags flags_;
    };

    FReadSurfaceContext context = {target_resource, pixels, FIntRect(0, 0, target_resource->GetSizeXY().X, target_resource->GetSizeXY().Y), FReadSurfaceDataFlags(RCM_UNorm, CubeFace_MAX)};

    // Required for uint8 read mode
    context.flags_.SetLinearToGamma(false);

    ENQUEUE_RENDER_COMMAND(ReadSurfaceCommand)([context](FRHICommandListImmediate& RHICmdList) {
        RHICmdList.ReadSurfaceData(context.src_render_target_->GetRenderTargetTexture(), context.rect_, context.out_data_, context.flags_);
    });

    FlushRenderingCommands();
    /*FRenderCommandFence ReadPixelFence;
    ReadPixelFence.BeginFence(true);
    ReadPixelFence.Wait(true);*/
        
    return pixels;
}

//depth codification
//decode formula : depth = ((r) + (g * 256) + (b * 256 * 256)) / ((256 * 256 * 256) - 1) * f
void CameraSensor::FcolorArrayToUintVector(std::vector<uint32_t>& out, TArray<FColor> data)
{
    ASSERT(out.size() == data.Num());
    for (uint32 i = 0; i < static_cast<uint32>(data.Num()); ++i) {
        float depth = ((data[i].R  + (data[i].G * 256) + (data[i].B * 256 * 256)) / ((256 * 256 * 256) - 1)) * 10; 
        out.at(i) = (uint32_t)(*(uint32_t*)&depth);
    }
}

void CameraSensor::SetCameraDefaultOverrides(){
	this->scene_capture_component_->PostProcessSettings.bOverride_AutoExposureMethod = true;
    this->scene_capture_component_->PostProcessSettings.AutoExposureMethod = EAutoExposureMethod::AEM_Histogram;

    this->scene_capture_component_->PostProcessSettings.bOverride_AutoExposureBias = true;
    this->scene_capture_component_->PostProcessSettings.bOverride_AutoExposureMinBrightness = true;
    this->scene_capture_component_->PostProcessSettings.bOverride_AutoExposureMaxBrightness = true;

    this->scene_capture_component_->PostProcessSettings.AutoExposureBias = 1.0f;
    this->scene_capture_component_->PostProcessSettings.AutoExposureMaxBrightness = 5.0f; //-1
    this->scene_capture_component_->PostProcessSettings.AutoExposureMinBrightness = -10.0f; //-1

    this->scene_capture_component_->PostProcessSettings.bOverride_AutoExposureSpeedUp = true;
    this->scene_capture_component_->PostProcessSettings.AutoExposureSpeedUp = 10.0f;
    this->scene_capture_component_->PostProcessSettings.bOverride_AutoExposureSpeedDown = true;
    this->scene_capture_component_->PostProcessSettings.AutoExposureSpeedDown = 10.0f;
    this->scene_capture_component_->PostProcessSettings.bOverride_AutoExposureCalibrationConstant_DEPRECATED = true;
    this->scene_capture_component_->PostProcessSettings.bOverride_HistogramLogMin = true;
    this->scene_capture_component_->PostProcessSettings.HistogramLogMin = -10.0f;
    this->scene_capture_component_->PostProcessSettings.bOverride_HistogramLogMax = true;
    this->scene_capture_component_->PostProcessSettings.HistogramLogMax = 10.0f;
//
//    // Camera
    this->scene_capture_component_->PostProcessSettings.bOverride_CameraShutterSpeed = true;
    this->scene_capture_component_->PostProcessSettings.bOverride_CameraISO = true;
    this->scene_capture_component_->PostProcessSettings.bOverride_DepthOfFieldFstop = true;
    this->scene_capture_component_->PostProcessSettings.bOverride_DepthOfFieldMinFstop = true;
    this->scene_capture_component_->PostProcessSettings.bOverride_DepthOfFieldBladeCount = true;
//
//    // Film (Tonemapper)
//    this->scene_capture_component_->PostProcessSettings.bOverride_FilmSlope = true;
//    this->scene_capture_component_->PostProcessSettings.bOverride_FilmToe = true;
//    this->scene_capture_component_->PostProcessSettings.bOverride_FilmShoulder = true;
//    this->scene_capture_component_->PostProcessSettings.bOverride_FilmWhiteClip = true;
//    this->scene_capture_component_->PostProcessSettings.bOverride_FilmBlackClip = true;
//
    // Motion blur
    this->scene_capture_component_->PostProcessSettings.bOverride_MotionBlurAmount = true;
    this->scene_capture_component_->PostProcessSettings.MotionBlurAmount = 0.6f; //0.45f
    this->scene_capture_component_->PostProcessSettings.bOverride_MotionBlurMax = true;
    this->scene_capture_component_->PostProcessSettings.MotionBlurMax = 60.0f; //0.35f
    this->scene_capture_component_->PostProcessSettings.bOverride_MotionBlurPerObjectSize = true;
    this->scene_capture_component_->PostProcessSettings.MotionBlurPerObjectSize = 4.0f;
//
//    // Color Grading
//    this->scene_capture_component_->PostProcessSettings.bOverride_WhiteTemp = true;
//    this->scene_capture_component_->PostProcessSettings.bOverride_WhiteTint = true;
//    this->scene_capture_component_->PostProcessSettings.bOverride_ColorContrast = true;
//#if PLATFORM_LINUX
//    // Looks like Windows and Linux have different outputs with the
//    // same exposure compensation, this fixes it.
//    this->scene_capture_component_->PostProcessSettings.ColorContrast = FVector4(1.2f, 1.2f, 1.2f, 1.0f);
//#endif
//
//    // Chromatic Aberration
//    this->scene_capture_component_->PostProcessSettings.bOverride_SceneFringeIntensity = true;
//    this->scene_capture_component_->PostProcessSettings.bOverride_ChromaticAberrationStartOffset = true;
//
    // Ambient Occlusion
    //this->scene_capture_component_->PostProcessSettings.bOverride_AmbientOcclusionIntensity = true;
    //this->scene_capture_component_->PostProcessSettings.AmbientOcclusionIntensity = 0.5f;
    //this->scene_capture_component_->PostProcessSettings.bOverride_AmbientOcclusionRadius = true;
    //this->scene_capture_component_->PostProcessSettings.AmbientOcclusionRadius = 100.0f;
    //this->scene_capture_component_->PostProcessSettings.bOverride_AmbientOcclusionStaticFraction = true;
    //this->scene_capture_component_->PostProcessSettings.AmbientOcclusionStaticFraction = 1.0f;
    //this->scene_capture_component_->PostProcessSettings.bOverride_AmbientOcclusionFadeDistance = true;
    //this->scene_capture_component_->PostProcessSettings.AmbientOcclusionFadeDistance = 50000.0f;
    //this->scene_capture_component_->PostProcessSettings.bOverride_AmbientOcclusionPower = true;
    //this->scene_capture_component_->PostProcessSettings.AmbientOcclusionPower = 2.0f;
    //this->scene_capture_component_->PostProcessSettings.bOverride_AmbientOcclusionBias = true;
    //this->scene_capture_component_->PostProcessSettings.AmbientOcclusionBias = 3.0f;
    //this->scene_capture_component_->PostProcessSettings.bOverride_AmbientOcclusionQuality = true;
    //this->scene_capture_component_->PostProcessSettings.AmbientOcclusionQuality = 100.0f;
//
//    // Bloom
    this->scene_capture_component_->PostProcessSettings.bOverride_BloomMethod = true;
    this->scene_capture_component_->PostProcessSettings.BloomMethod = EBloomMethod::BM_SOG;
    this->scene_capture_component_->PostProcessSettings.bOverride_BloomIntensity = true;
    this->scene_capture_component_->PostProcessSettings.BloomIntensity = 0.4f ; // 0.675f
    this->scene_capture_component_->PostProcessSettings.bOverride_BloomThreshold = true;
    this->scene_capture_component_->PostProcessSettings.BloomThreshold = -1.0f;
//
//    // Lens
    //this->scene_capture_component_->PostProcessSettings.bOverride_LensFlareIntensity = true;
    //this->scene_capture_component_->PostProcessSettings.LensFlareIntensity = 0.2;

    // Raytracing
    this->scene_capture_component_->bUseRayTracingIfEnabled = true;
    this->scene_capture_component_->PostProcessSettings.bOverride_IndirectLightingIntensity = true;
    this->scene_capture_component_->PostProcessSettings.IndirectLightingIntensity = 0.2f;

    this->scene_capture_component_->PostProcessSettings.bOverride_RayTracingGI = true;
    this->scene_capture_component_->PostProcessSettings.RayTracingGIType = ERayTracingGlobalIlluminationType::BruteForce;
    //this->scene_capture_component_->PostProcessSettings.RayTracingGIType = ERayTracingGlobalIlluminationType::BruteForce;
    this->scene_capture_component_->PostProcessSettings.bOverride_RayTracingGIMaxBounces = true;
    this->scene_capture_component_->PostProcessSettings.bOverride_RayTracingGISamplesPerPixel = true;

    this->scene_capture_component_->PostProcessSettings.RayTracingGIMaxBounces = 4;
    this->scene_capture_component_->PostProcessSettings.RayTracingGISamplesPerPixel = 16;

    this->scene_capture_component_->PostProcessSettings.bOverride_RayTracingAO = true; // Default value
    this->scene_capture_component_->PostProcessSettings.bOverride_RayTracingAOSamplesPerPixel = true; // min 1 - max 64
    this->scene_capture_component_->PostProcessSettings.bOverride_RayTracingAOIntensity = true; // min 0.0 - max 1.0
    this->scene_capture_component_->PostProcessSettings.bOverride_RayTracingAORadius = true; // min 0.0 - max 10000.0

    //this->scene_capture_component_->PostProcessSettings.RayTracingAO = 1; // Default value
    this->scene_capture_component_->PostProcessSettings.RayTracingAOSamplesPerPixel = 8; // min 1 - max 64
    this->scene_capture_component_->PostProcessSettings.RayTracingAOIntensity = 0.6f; // min 0.0 - max 1.0
    this->scene_capture_component_->PostProcessSettings.RayTracingAORadius = 4000.0f; // min 0.0 - max 10000.0

    //this->scene_capture_component_->PostProcessSettings.bOverride_TranslucencyType = true;
    //this->scene_capture_component_->PostProcessSettings.TranslucencyType = ETranslucencyType::RayTracing;
    ////this->scene_capture_component_->PostProcessSettings.RayTracingTranslucencyShadows = EReflectedAndRefractedRayTracedShadows::Hard_shadows;
    //this->scene_capture_component_->PostProcessSettings.bOverride_RayTracingTranslucencyMaxRoughness = true; // min 0.01 - max 1.0
    //this->scene_capture_component_->PostProcessSettings.bOverride_RayTracingTranslucencyRefractionRays = true; // min 0 - max 50
    //this->scene_capture_component_->PostProcessSettings.bOverride_RayTracingTranslucencySamplesPerPixel = true; // min 1 - max 64
    //this->scene_capture_component_->PostProcessSettings.bOverride_RayTracingTranslucencyRefraction = true; // default value

    //this->scene_capture_component_->PostProcessSettings.RayTracingTranslucencyMaxRoughness = 0.6f; // min 0.01 - max 1.0
    //this->scene_capture_component_->PostProcessSettings.RayTracingTranslucencyRefractionRays = 4; // min 0 - max 50
    //this->scene_capture_component_->PostProcessSettings.RayTracingTranslucencySamplesPerPixel = 4; // min 1 - max 64
    //this->scene_capture_component_->PostProcessSettings.RayTracingTranslucencyRefraction = 1; // default value

    this->scene_capture_component_->PostProcessSettings.bOverride_ReflectionsType = true;
    this->scene_capture_component_->PostProcessSettings.bOverride_RayTracingReflectionsMaxBounces = true; // min 0 - max 50
    this->scene_capture_component_->PostProcessSettings.bOverride_RayTracingReflectionsSamplesPerPixel = true; // min 1 - max 64
    this->scene_capture_component_->PostProcessSettings.bOverride_RayTracingReflectionsTranslucency = true; // Default value
    this->scene_capture_component_->PostProcessSettings.bOverride_RayTracingReflectionsShadows = true;

    this->scene_capture_component_->PostProcessSettings.ReflectionsType = EReflectionsType::RayTracing;
    this->scene_capture_component_->PostProcessSettings.RayTracingReflectionsMaxRoughness = 0.4f; // min 0.01 - max 1.0
    this->scene_capture_component_->PostProcessSettings.RayTracingReflectionsMaxBounces = 2; // min 0 - max 50
    this->scene_capture_component_->PostProcessSettings.RayTracingReflectionsSamplesPerPixel = 4; // min 1 - max 64
    this->scene_capture_component_->PostProcessSettings.RayTracingReflectionsShadows = EReflectedAndRefractedRayTracedShadows::Area_shadows;

    //// Pathtracing
    //this->scene_capture_component_->PostProcessSettings.PathTracingMaxBounces = 2;      // min 0 - max 50
    //this->scene_capture_component_->PostProcessSettings.PathTracingSamplesPerPixel = 32; // min 1 - max 64
    //this->scene_capture_component_->PostProcessSettings.bOverride_PathTracingEnableEmissive = 1;      // min 0 - max 50
    //this->scene_capture_component_->PostProcessSettings.PathTracingMaxPathExposure = 20.0f;      // min -10 - max 30
    //this->scene_capture_component_->PostProcessSettings.bOverride_PathTracingEnableDenoiser = 1;

}

void CameraSensor::ConfigureShowFlags(bool bPostProcessing){
	if (bPostProcessing)
        {
            this->scene_capture_component_->ShowFlags.EnableAdvancedFeatures();
            this->scene_capture_component_->ShowFlags.SetMotionBlur(true);
            return;
        }

    this->scene_capture_component_->ShowFlags.SetTemporalAA(false);
    this->scene_capture_component_->ShowFlags.SetAntiAliasing(true);

    //this->scene_capture_component_->ShowFlags.SetAmbientOcclusion(false);
    //this->scene_capture_component_->ShowFlags.SetAntiAliasing(false);
    //this->scene_capture_component_->ShowFlags.SetVolumetricFog(false);
    //// ShowFlags.SetAtmosphericFog(false);
    //// ShowFlags.SetAudioRadius(false);
    //// ShowFlags.SetBillboardSprites(false);
    //this->scene_capture_component_->ShowFlags.SetBloom(false);
    //// ShowFlags.SetBounds(false);
    //// ShowFlags.SetBrushes(false);
    //// ShowFlags.SetBSP(false);
    //// ShowFlags.SetBSPSplit(false);
    //// ShowFlags.SetBSPTriangles(false);
    //// ShowFlags.SetBuilderBrush(false);
    //// ShowFlags.SetCameraAspectRatioBars(false);
    //// ShowFlags.SetCameraFrustums(false);
    //this->scene_capture_component_->ShowFlags.SetCameraImperfections(false);
    //this->scene_capture_component_->ShowFlags.SetCameraInterpolation(false);
    //// ShowFlags.SetCameraSafeFrames(false);
    //// ShowFlags.SetCollision(false);
    //// ShowFlags.SetCollisionPawn(false);
    //// ShowFlags.SetCollisionVisibility(false);
    //this->scene_capture_component_->ShowFlags.SetColorGrading(false);
    //// ShowFlags.SetCompositeEditorPrimitives(false);
    //// ShowFlags.SetConstraints(false);
    //// ShowFlags.SetCover(false);
    //// ShowFlags.SetDebugAI(false);
    //// ShowFlags.SetDecals(false);
    //// ShowFlags.SetDeferredLighting(false);
    //this->scene_capture_component_->ShowFlags.SetDepthOfField(false);
    //this->scene_capture_component_->ShowFlags.SetDiffuse(false);
    //this->scene_capture_component_->ShowFlags.SetDirectionalLights(false);
    //this->scene_capture_component_->ShowFlags.SetDirectLighting(false);
    //// ShowFlags.SetDistanceCulledPrimitives(false);
    scene_capture_component_->ShowFlags.SetDistanceFieldAO(false);
    scene_capture_component_->ShowFlags.SetRayTracedDistanceFieldShadows(true);
    scene_capture_component_->ShowFlags.SetDynamicShadows(true);
    //// ShowFlags.SetEditor(false);
    scene_capture_component_->ShowFlags.SetEyeAdaptation(true);
    //this->scene_capture_component_->ShowFlags.SetFog(false);
    //// ShowFlags.SetGame(false);
    //// ShowFlags.SetGameplayDebug(false);
    //// ShowFlags.SetGBufferHints(false);
    //scene_capture_component_->ShowFlags.SetGlobalIllumination(false);
    //this->scene_capture_component_->ShowFlags.SetGrain(false);
    //// ShowFlags.SetGrid(false);
    //// ShowFlags.SetHighResScreenshotMask(false);
    //// ShowFlags.SetHitProxies(false);
    //this->scene_capture_component_->ShowFlags.SetHLODColoration(false);
    //this->scene_capture_component_->ShowFlags.SetHMDDistortion(false);
    //// ShowFlags.SetIndirectLightingCache(false);
    //// ShowFlags.SetInstancedFoliage(false);
    //// ShowFlags.SetInstancedGrass(false);
    //// ShowFlags.SetInstancedStaticMeshes(false);
    //// ShowFlags.SetLandscape(false);
    //// ShowFlags.SetLargeVertices(false);
    //this->scene_capture_component_->ShowFlags.SetLensFlares(false);
    //this->scene_capture_component_->ShowFlags.SetLevelColoration(false);
    //this->scene_capture_component_->ShowFlags.SetLightComplexity(false);
    //this->scene_capture_component_->ShowFlags.SetLightFunctions(false);
    //this->scene_capture_component_->ShowFlags.SetLightInfluences(false);
    //this->scene_capture_component_->ShowFlags.SetLighting(false);
    //this->scene_capture_component_->ShowFlags.SetLightMapDensity(false);
    //this->scene_capture_component_->ShowFlags.SetLightRadius(false);
    //this->scene_capture_component_->ShowFlags.SetLightShafts(false);
    //// ShowFlags.SetLOD(false);
    //this->scene_capture_component_->ShowFlags.SetLODColoration(false);
    //// ShowFlags.SetMaterials(false);
    //// ShowFlags.SetMaterialTextureScaleAccuracy(false);
    //// ShowFlags.SetMeshEdges(false);
    //// ShowFlags.SetMeshUVDensityAccuracy(false);
    //// ShowFlags.SetModeWidgets(false);
    //this->scene_capture_component_->ShowFlags.SetMotionBlur(false);
    //// ShowFlags.SetNavigation(false);
    //this->scene_capture_component_->ShowFlags.SetOnScreenDebug(false);
    //// ShowFlags.SetOutputMaterialTextureScales(false);
    //// ShowFlags.SetOverrideDiffuseAndSpecular(false);
    //// ShowFlags.SetPaper2DSprites(false);
    //this->scene_capture_component_->ShowFlags.SetParticles(false);
    //// ShowFlags.SetPivot(false);
    //this->scene_capture_component_->ShowFlags.SetPointLights(false);
    //// ShowFlags.SetPostProcessing(false);
    //// ShowFlags.SetPostProcessMaterial(false);
    //// ShowFlags.SetPrecomputedVisibility(false);
    //// ShowFlags.SetPrecomputedVisibilityCells(false);
    //// ShowFlags.SetPreviewShadowsIndicator(false);
    //// ShowFlags.SetPrimitiveDistanceAccuracy(false);
    //this->scene_capture_component_->ShowFlags.SetPropertyColoration(false);
    //// ShowFlags.SetQuadOverdraw(false);
    //// ShowFlags.SetReflectionEnvironment(false);
    //// ShowFlags.SetReflectionOverride(false);
    //this->scene_capture_component_->ShowFlags.SetRefraction(false);
    //// ShowFlags.SetRendering(false);
    //this->scene_capture_component_->ShowFlags.SetSceneColorFringe(false);
    //// ShowFlags.SetScreenPercentage(false);
    //this->scene_capture_component_->ShowFlags.SetScreenSpaceAO(false);
    //this->scene_capture_component_->ShowFlags.SetScreenSpaceReflections(false);
    //// ShowFlags.SetSelection(false);
    //// ShowFlags.SetSelectionOutline(false);
    //// ShowFlags.SetSeparateTranslucency(false);
    //// ShowFlags.SetShaderComplexity(false);
    //// ShowFlags.SetShaderComplexityWithQuadOverdraw(false);
    //// ShowFlags.SetShadowFrustums(false);
    //// ShowFlags.SetSkeletalMeshes(false);
    //// ShowFlags.SetSkinCache(false);
    //this->scene_capture_component_->ShowFlags.SetSkyLighting(false);
    //// ShowFlags.SetSnap(false);
    //// ShowFlags.SetSpecular(false);
    //// ShowFlags.SetSplines(false);
    //this->scene_capture_component_->ShowFlags.SetSpotLights(false);
    //// ShowFlags.SetStaticMeshes(false);
    //this->scene_capture_component_->ShowFlags.SetStationaryLightOverlap(false);
    //// ShowFlags.SetStereoRendering(false);
    //// ShowFlags.SetStreamingBounds(false);
    //this->scene_capture_component_->ShowFlags.SetSubsurfaceScattering(false);
    //// ShowFlags.SetTemporalAA(false);
    //// ShowFlags.SetTessellation(false);
    //// ShowFlags.SetTestImage(false);
    //// ShowFlags.SetTextRender(false);
    //// ShowFlags.SetTexturedLightProfiles(false);
    //this->scene_capture_component_->ShowFlags.SetTonemapper(false);
    //// ShowFlags.SetTranslucency(false);
    //// ShowFlags.SetVectorFields(false);
    //// ShowFlags.SetVertexColors(false);
    //// ShowFlags.SetVignette(false);
    //// ShowFlags.SetVisLog(false);
    //// ShowFlags.SetVisualizeAdaptiveDOF(false);
    //// ShowFlags.SetVisualizeBloom(false);
    //this->scene_capture_component_->ShowFlags.SetVisualizeBuffer(false);
    //this->scene_capture_component_->ShowFlags.SetVisualizeDistanceFieldAO(false);
    //this->scene_capture_component_->ShowFlags.SetVisualizeDOF(false);
    //this->scene_capture_component_->ShowFlags.SetVisualizeHDR(false);
    //this->scene_capture_component_->ShowFlags.SetVisualizeLightCulling(false);
    //this->scene_capture_component_->ShowFlags.SetVisualizeLPV(false);
    //this->scene_capture_component_->ShowFlags.SetVisualizeMeshDistanceFields(false);
    //this->scene_capture_component_->ShowFlags.SetVisualizeMotionBlur(false);
    //this->scene_capture_component_->ShowFlags.SetVisualizeOutOfBoundsPixels(false);
    //this->scene_capture_component_->ShowFlags.SetVisualizeSenses(false);
    //this->scene_capture_component_->ShowFlags.SetVisualizeShadingModels(false);
    //this->scene_capture_component_->ShowFlags.SetVisualizeSSR(false);
    //this->scene_capture_component_->ShowFlags.SetVisualizeSSS(false);
    //// ShowFlags.SetVolumeLightingSamples(false);
    //// ShowFlags.SetVolumes(false);
    //// ShowFlags.SetWidgetComponents(false);
    //// ShowFlags.SetWireframe(false);

}
