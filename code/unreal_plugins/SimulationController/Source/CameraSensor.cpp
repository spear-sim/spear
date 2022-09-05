#include "CameraSensor.h"

#include <string>
#include <utility>
#include <vector>
#include <map>

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
        ASSERT(actor_);

        // create SceneCaptureComponent2D and TextureRenderTarget2D
        scene_capture_component_ = NewObject<USceneCaptureComponent2D>(this, TEXT("SceneCaptureComponent2D"));
        ASSERT(scene_capture_component_);       
        scene_capture_component_->AttachToComponent(actor_->GetRootComponent(), FAttachmentTransformRules::SnapToTargetNotIncludingScale);
        scene_capture_component_->SetVisibility(true);
        scene_capture_component_->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
        scene_capture_component_->FOVAngle = 60.f;
        scene_capture_component_->bAlwaysPersistRenderingState = true;

        SetCameraDefaultOverrides();    
        ConfigureShowFlags(enable_postprocessing_effects_); 

        UKismetSystemLibrary::ExecuteConsoleCommand(world, FString("g.TimeoutForBlockOnRenderFence 300000"));

        texture_render_target_ = NewObject<UTextureRenderTarget2D>(this, TEXT("TextureRenderTarget2D"));
        ASSERT(texture_render_target_);
}

CameraSensor::~CameraSensor(){
        ASSERT(texture_render_target_);
        texture_render_target_->MarkPendingKill();
        texture_render_target_ = nullptr;

        ASSERT(scene_capture_component_);
        scene_capture_component_->DestroyComponent();
        scene_capture_component_ = nullptr; 
}

void CameraSensor::SetRenderTarget(unsigned long w, unsigned long h){
        texture_render_target_->InitCustomFormat(w ,h ,PF_B8G8R8A8 ,true ); // PF_B8G8R8A8 disables HDR;
        texture_render_target_->RenderTargetFormat = ETextureRenderTargetFormat::RTF_RGBA8;
        texture_render_target_->bGPUSharedFlag = true; // demand buffer on GPU - might improve performance?
        texture_render_target_->TargetGamma = GEngine->GetDisplayGamma();
        texture_render_target_->SRGB = false; // false for pixels to be stored in linear space
        texture_render_target_->bAutoGenerateMips = false;
        texture_render_target_->UpdateResourceImmediate(true);    
        scene_capture_component_->TextureTarget = texture_render_target_;
        scene_capture_component_->RegisterComponent();
}

void CameraSensor::SetPostProcessBlendables(std::vector<std::string> blendables){
        for(std::string pass_name_ : blendables){
            ASSERT(std::find(PASSES.begin(), PASSES.end(), pass_name_) != PASSES.end());
        }

        //Set blendables
        unsigned long pass_index_ = 0;
        for(std::string pass_name_ : blendables){
                FString path_ = MATERIALS_PATH.c_str() + pass_name_.c_str() + "." + pass_name_.c_str();
                UMaterial* mat = LoadObject<UMaterial>(nullptr, *path_);
                ASSERT(mat);
                scene_capture_component_->PostProcessSettings.AddBlendable(UMaterialInstanceDynamic::Create(mat, this->scene_capture_component_), 0.0f);
                
                passes_.insert( std::pair<std::string, unsigned long> (pass_name_, pass_index_));
                pass_index_++;
        }   
}

void CameraSensor::ActivateBlendablePass(std::string pass_name){
        ASSERT(pass_name != "");

        scene_capture_component_->PostProcessBlendWeight = 0.0f;
        for(int i=0;i < scene_capture_component_->PostProcessSettings.WeightedBlendables.Array.Num();i++){
                scene_capture_component_->PostProcessSettings.WeightedBlendables.Array[i].Weight = 0.0f;
        }

        if(pass_name != "finalColor"){
                scene_capture_component_->PostProcessBlendWeight = 1.0f;
                scene_capture_component_->PostProcessSettings.WeightedBlendables.Array[int(passes_.find(pass_name)->second)].Weight = 1.0f;
        }
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

        FRenderCommandFence ReadPixelFence;
        ReadPixelFence.BeginFence(true);
        ReadPixelFence.Wait(true);
        
        return pixels;
}

//depth codification
//decode formula : depth = ((r) + (g * 256) + (b * 256 * 256)) / ((256 * 256 * 256) - 1) * f
void CameraSensor::FColorToFloatImage(std::vector<float>& out, TArray<FColor> data)
{
        ASSERT(out.size() == data.Num());
        for (uint32 i = 0; i < static_cast<uint32>(data.Num()); ++i) {
                float depth = data[i].R  + (data[i].G * 256) + (data[i].B * 256 * 256); 
                float normalized_depth = depth / ((256 * 256 * 256) - 1);
                float dist = normalized_depth * 10; 
                out.at(i) = dist;
        }
}

void CameraSensor::SetCameraDefaultOverrides(){
	scene_capture_component_->PostProcessSettings.bOverride_AutoExposureMethod = true;
        scene_capture_component_->PostProcessSettings.AutoExposureMethod = EAutoExposureMethod::AEM_Histogram;
        scene_capture_component_->PostProcessSettings.bOverride_AutoExposureBias = true;
        scene_capture_component_->PostProcessSettings.bOverride_AutoExposureMinBrightness = true;
        scene_capture_component_->PostProcessSettings.bOverride_AutoExposureMaxBrightness = true;
        scene_capture_component_->PostProcessSettings.bOverride_AutoExposureSpeedUp = true;
        scene_capture_component_->PostProcessSettings.bOverride_AutoExposureSpeedDown = true;
        scene_capture_component_->PostProcessSettings.bOverride_AutoExposureCalibrationConstant_DEPRECATED = true;
        scene_capture_component_->PostProcessSettings.bOverride_HistogramLogMin = true;
        scene_capture_component_->PostProcessSettings.HistogramLogMin = 1.0f;
        scene_capture_component_->PostProcessSettings.bOverride_HistogramLogMax = true;
        scene_capture_component_->PostProcessSettings.HistogramLogMax = 12.0f;

        // Camera
        scene_capture_component_->PostProcessSettings.bOverride_CameraShutterSpeed = true;
        scene_capture_component_->PostProcessSettings.bOverride_CameraISO = true;
        scene_capture_component_->PostProcessSettings.bOverride_DepthOfFieldFstop = true;
        scene_capture_component_->PostProcessSettings.bOverride_DepthOfFieldMinFstop = true;
        scene_capture_component_->PostProcessSettings.bOverride_DepthOfFieldBladeCount = true;

        // Film (Tonemapper)
        scene_capture_component_->PostProcessSettings.bOverride_FilmSlope = true;
        scene_capture_component_->PostProcessSettings.bOverride_FilmToe = true;
        scene_capture_component_->PostProcessSettings.bOverride_FilmShoulder = true;
        scene_capture_component_->PostProcessSettings.bOverride_FilmWhiteClip = true;
        scene_capture_component_->PostProcessSettings.bOverride_FilmBlackClip = true;

        // Motion blur
        scene_capture_component_->PostProcessSettings.bOverride_MotionBlurAmount = true;
        scene_capture_component_->PostProcessSettings.MotionBlurAmount = 0.45f;
        scene_capture_component_->PostProcessSettings.bOverride_MotionBlurMax = true;
        scene_capture_component_->PostProcessSettings.MotionBlurMax = 0.35f;
        scene_capture_component_->PostProcessSettings.bOverride_MotionBlurPerObjectSize = true;
        scene_capture_component_->PostProcessSettings.MotionBlurPerObjectSize = 0.1f;

        // Color Grading
        scene_capture_component_->PostProcessSettings.bOverride_WhiteTemp = true;
        scene_capture_component_->PostProcessSettings.bOverride_WhiteTint = true;
        scene_capture_component_->PostProcessSettings.bOverride_ColorContrast = true;
#if PLATFORM_LINUX
        // Looks like Windows and Linux have different outputs with the
        // same exposure compensation, this fixes it.
        scene_capture_component_->PostProcessSettings.ColorContrast = FVector4(1.2f, 1.2f, 1.2f, 1.0f);
#endif

        // Chromatic Aberration
        scene_capture_component_->PostProcessSettings.bOverride_SceneFringeIntensity = true;
        scene_capture_component_->PostProcessSettings.bOverride_ChromaticAberrationStartOffset = true;

        // Ambient Occlusion
        scene_capture_component_->PostProcessSettings.bOverride_AmbientOcclusionIntensity = true;
        scene_capture_component_->PostProcessSettings.AmbientOcclusionIntensity = 0.5f;
        scene_capture_component_->PostProcessSettings.bOverride_AmbientOcclusionRadius = true;
        scene_capture_component_->PostProcessSettings.AmbientOcclusionRadius = 100.0f;
        scene_capture_component_->PostProcessSettings.bOverride_AmbientOcclusionStaticFraction = true;
        scene_capture_component_->PostProcessSettings.AmbientOcclusionStaticFraction = 1.0f;
        scene_capture_component_->PostProcessSettings.bOverride_AmbientOcclusionFadeDistance = true;
        scene_capture_component_->PostProcessSettings.AmbientOcclusionFadeDistance = 50000.0f;
        scene_capture_component_->PostProcessSettings.bOverride_AmbientOcclusionPower = true;
        scene_capture_component_->PostProcessSettings.AmbientOcclusionPower = 2.0f;
        scene_capture_component_->PostProcessSettings.bOverride_AmbientOcclusionBias = true;
        scene_capture_component_->PostProcessSettings.AmbientOcclusionBias = 3.0f;
        scene_capture_component_->PostProcessSettings.bOverride_AmbientOcclusionQuality = true;
        scene_capture_component_->PostProcessSettings.AmbientOcclusionQuality = 100.0f;

        // Bloom
        scene_capture_component_->PostProcessSettings.bOverride_BloomMethod = true;
        scene_capture_component_->PostProcessSettings.BloomMethod = EBloomMethod::BM_SOG;
        scene_capture_component_->PostProcessSettings.bOverride_BloomIntensity = true;
        scene_capture_component_->PostProcessSettings.BloomIntensity = 0.675f;
        scene_capture_component_->PostProcessSettings.bOverride_BloomThreshold = true;
        scene_capture_component_->PostProcessSettings.BloomThreshold = -1.0f;

        // Lens
        scene_capture_component_->PostProcessSettings.bOverride_LensFlareIntensity = true;
        scene_capture_component_->PostProcessSettings.LensFlareIntensity = 0.1;

}

void CameraSensor::ConfigureShowFlags(bool bPostProcessing){
	if (bPostProcessing)
        {
            scene_capture_component_->ShowFlags.EnableAdvancedFeatures();
            scene_capture_component_->ShowFlags.SetMotionBlur(true);
            return;
        }

        //scene_capture_component_->ShowFlags.SetAmbientOcclusion(false);
        //scene_capture_component_->ShowFlags.SetAntiAliasing(false);
        //scene_capture_component_->ShowFlags.SetVolumetricFog(false);
        //// ShowFlags.SetAtmosphericFog(false);
        //// ShowFlags.SetAudioRadius(false);
        //// ShowFlags.SetBillboardSprites(false);
        //scene_capture_component_->ShowFlags.SetBloom(false);
        //// ShowFlags.SetBounds(false);
        //// ShowFlags.SetBrushes(false);
        //// ShowFlags.SetBSP(false);
        //// ShowFlags.SetBSPSplit(false);
        //// ShowFlags.SetBSPTriangles(false);
        //// ShowFlags.SetBuilderBrush(false);
        //// ShowFlags.SetCameraAspectRatioBars(false);
        //// ShowFlags.SetCameraFrustums(false);
        //scene_capture_component_->ShowFlags.SetCameraImperfections(false);
        //scene_capture_component_->ShowFlags.SetCameraInterpolation(false);
        //// ShowFlags.SetCameraSafeFrames(false);
        //// ShowFlags.SetCollision(false);
        //// ShowFlags.SetCollisionPawn(false);
        //// ShowFlags.SetCollisionVisibility(false);
        //scene_capture_component_->ShowFlags.SetColorGrading(false);
        //// ShowFlags.SetCompositeEditorPrimitives(false);
        //// ShowFlags.SetConstraints(false);
        //// ShowFlags.SetCover(false);
        //// ShowFlags.SetDebugAI(false);
        //// ShowFlags.SetDecals(false);
        //// ShowFlags.SetDeferredLighting(false);
        //scene_capture_component_->ShowFlags.SetDepthOfField(false);
        //scene_capture_component_->ShowFlags.SetDiffuse(false);
        //scene_capture_component_->ShowFlags.SetDirectionalLights(false);
        //scene_capture_component_->ShowFlags.SetDirectLighting(false);
        //// ShowFlags.SetDistanceCulledPrimitives(false);
        //// ShowFlags.SetDistanceFieldAO(false);
        //// ShowFlags.SetDistanceFieldGI(false);
        //scene_capture_component_->ShowFlags.SetDynamicShadows(false);
        //// ShowFlags.SetEditor(false);
        //scene_capture_component_->ShowFlags.SetEyeAdaptation(false);
        //scene_capture_component_->ShowFlags.SetFog(false);
        //// ShowFlags.SetGame(false);
        //// ShowFlags.SetGameplayDebug(false);
        //// ShowFlags.SetGBufferHints(false);
        //scene_capture_component_->ShowFlags.SetGlobalIllumination(false);
        //scene_capture_component_->ShowFlags.SetGrain(false);
        //// ShowFlags.SetGrid(false);
        //// ShowFlags.SetHighResScreenshotMask(false);
        //// ShowFlags.SetHitProxies(false);
        //scene_capture_component_->ShowFlags.SetHLODColoration(false);
        //scene_capture_component_->ShowFlags.SetHMDDistortion(false);
        //// ShowFlags.SetIndirectLightingCache(false);
        //// ShowFlags.SetInstancedFoliage(false);
        //// ShowFlags.SetInstancedGrass(false);
        //// ShowFlags.SetInstancedStaticMeshes(false);
        //// ShowFlags.SetLandscape(false);
        //// ShowFlags.SetLargeVertices(false);
        //scene_capture_component_->ShowFlags.SetLensFlares(false);
        //scene_capture_component_->ShowFlags.SetLevelColoration(false);
        //scene_capture_component_->ShowFlags.SetLightComplexity(false);
        //scene_capture_component_->ShowFlags.SetLightFunctions(false);
        //scene_capture_component_->ShowFlags.SetLightInfluences(false);
        //scene_capture_component_->ShowFlags.SetLighting(false);
        //scene_capture_component_->ShowFlags.SetLightMapDensity(false);
        //scene_capture_component_->ShowFlags.SetLightRadius(false);
        //scene_capture_component_->ShowFlags.SetLightShafts(false);
        //// ShowFlags.SetLOD(false);
        //scene_capture_component_->ShowFlags.SetLODColoration(false);
        //// ShowFlags.SetMaterials(false);
        //// ShowFlags.SetMaterialTextureScaleAccuracy(false);
        //// ShowFlags.SetMeshEdges(false);
        //// ShowFlags.SetMeshUVDensityAccuracy(false);
        //// ShowFlags.SetModeWidgets(false);
        //scene_capture_component_->ShowFlags.SetMotionBlur(false);
        //// ShowFlags.SetNavigation(false);
        //scene_capture_component_->ShowFlags.SetOnScreenDebug(false);
        //// ShowFlags.SetOutputMaterialTextureScales(false);
        //// ShowFlags.SetOverrideDiffuseAndSpecular(false);
        //// ShowFlags.SetPaper2DSprites(false);
        //scene_capture_component_->ShowFlags.SetParticles(false);
        //// ShowFlags.SetPivot(false);
        //scene_capture_component_->ShowFlags.SetPointLights(false);
        //// ShowFlags.SetPostProcessing(false);
        //// ShowFlags.SetPostProcessMaterial(false);
        //// ShowFlags.SetPrecomputedVisibility(false);
        //// ShowFlags.SetPrecomputedVisibilityCells(false);
        //// ShowFlags.SetPreviewShadowsIndicator(false);
        //// ShowFlags.SetPrimitiveDistanceAccuracy(false);
        //scene_capture_component_->ShowFlags.SetPropertyColoration(false);
        //// ShowFlags.SetQuadOverdraw(false);
        //// ShowFlags.SetReflectionEnvironment(false);
        //// ShowFlags.SetReflectionOverride(false);
        //scene_capture_component_->ShowFlags.SetRefraction(false);
        //// ShowFlags.SetRendering(false);
        //scene_capture_component_->ShowFlags.SetSceneColorFringe(false);
        //// ShowFlags.SetScreenPercentage(false);
        //scene_capture_component_->ShowFlags.SetScreenSpaceAO(false);
        //scene_capture_component_->ShowFlags.SetScreenSpaceReflections(false);
        //// ShowFlags.SetSelection(false);
        //// ShowFlags.SetSelectionOutline(false);
        //// ShowFlags.SetSeparateTranslucency(false);
        //// ShowFlags.SetShaderComplexity(false);
        //// ShowFlags.SetShaderComplexityWithQuadOverdraw(false);
        //// ShowFlags.SetShadowFrustums(false);
        //// ShowFlags.SetSkeletalMeshes(false);
        //// ShowFlags.SetSkinCache(false);
        //scene_capture_component_->ShowFlags.SetSkyLighting(false);
        //// ShowFlags.SetSnap(false);
        //// ShowFlags.SetSpecular(false);
        //// ShowFlags.SetSplines(false);
        //scene_capture_component_->ShowFlags.SetSpotLights(false);
        //// ShowFlags.SetStaticMeshes(false);
        //scene_capture_component_->ShowFlags.SetStationaryLightOverlap(false);
        //// ShowFlags.SetStereoRendering(false);
        //// ShowFlags.SetStreamingBounds(false);
        //scene_capture_component_->ShowFlags.SetSubsurfaceScattering(false);
        //// ShowFlags.SetTemporalAA(false);
        //// ShowFlags.SetTessellation(false);
        //// ShowFlags.SetTestImage(false);
        //// ShowFlags.SetTextRender(false);
        //// ShowFlags.SetTexturedLightProfiles(false);
        //scene_capture_component_->ShowFlags.SetTonemapper(false);
        //// ShowFlags.SetTranslucency(false);
        //// ShowFlags.SetVectorFields(false);
        //// ShowFlags.SetVertexColors(false);
        //// ShowFlags.SetVignette(false);
        //// ShowFlags.SetVisLog(false);
        //// ShowFlags.SetVisualizeAdaptiveDOF(false);
        //// ShowFlags.SetVisualizeBloom(false);
        //scene_capture_component_->ShowFlags.SetVisualizeBuffer(false);
        //scene_capture_component_->ShowFlags.SetVisualizeDistanceFieldAO(false);
        //scene_capture_component_->ShowFlags.SetVisualizeDOF(false);
        //scene_capture_component_->ShowFlags.SetVisualizeHDR(false);
        //scene_capture_component_->ShowFlags.SetVisualizeLightCulling(false);
        //scene_capture_component_->ShowFlags.SetVisualizeLPV(false);
        //scene_capture_component_->ShowFlags.SetVisualizeMeshDistanceFields(false);
        //scene_capture_component_->ShowFlags.SetVisualizeMotionBlur(false);
        //scene_capture_component_->ShowFlags.SetVisualizeOutOfBoundsPixels(false);
        //scene_capture_component_->ShowFlags.SetVisualizeSenses(false);
        //scene_capture_component_->ShowFlags.SetVisualizeShadingModels(false);
        //scene_capture_component_->ShowFlags.SetVisualizeSSR(false);
        //scene_capture_component_->ShowFlags.SetVisualizeSSS(false);
        //// ShowFlags.SetVolumeLightingSamples(false);
        //// ShowFlags.SetVolumes(false);
        //// ShowFlags.SetWidgetComponents(false);
        //// ShowFlags.SetWireframe(false);

}
