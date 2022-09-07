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

CameraSensor::CameraSensor(UWorld* world, AActor* actor , std::vector<std::string> pass_names, unsigned long w, unsigned long h){
        ASSERT(actor);

        for(std::string pass_name : pass_names){
                CameraPass new_pass;

                // create SceneCaptureComponent2D
                USceneCaptureComponent2D* new_scene_capture_component = NewObject<USceneCaptureComponent2D>(actor, TEXT("SceneCaptureComponent2D"));
                ASSERT(new_scene_capture_component);

                new_scene_capture_component->AttachToComponent(actor->GetRootComponent(), FAttachmentTransformRules::SnapToTargetNotIncludingScale);
                new_scene_capture_component->SetVisibility(true);
                new_scene_capture_component->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
                new_scene_capture_component->FOVAngle = 60.f;
                new_scene_capture_component->bAlwaysPersistRenderingState = true;

                // Set Camera Parameters
                SetCameraParameters(new_scene_capture_component, (pass_name != "finalColor") ? true : false);

                UKismetSystemLibrary::ExecuteConsoleCommand(world, FString("g.TimeoutForBlockOnRenderFence 300000"));

                // create TextureRenderTarget2D
                UTextureRenderTarget2D* new_texture_render_target = NewObject<UTextureRenderTarget2D>(actor, TEXT("TextureRenderTarget2D"));
                ASSERT(new_texture_render_target);

                new_texture_render_target->InitCustomFormat(w ,h ,PF_B8G8R8A8 ,true ); // PF_B8G8R8A8 disables HDR;
                new_texture_render_target->RenderTargetFormat = ETextureRenderTargetFormat::RTF_RGBA8;
                new_texture_render_target->bGPUSharedFlag = true; // demand buffer on GPU - might improve performance?
                new_texture_render_target->TargetGamma = GEngine->GetDisplayGamma();
                new_texture_render_target->SRGB = false; // false for pixels to be stored in linear space
                new_texture_render_target->bAutoGenerateMips = false;
                new_texture_render_target->UpdateResourceImmediate(true);    
                new_scene_capture_component->TextureTarget = new_texture_render_target;
                new_scene_capture_component->RegisterComponent();

                if(pass_name != "finalColor"){
                        // Load PostProcessMaterial
                        FString path_ = (MATERIALS_PATH + pass_name + "." + pass_name).c_str();
                        UMaterial* mat = LoadObject<UMaterial>(nullptr, *path_);
                        ASSERT(mat);

                        // Set PostProcessMaterial
                        new_scene_capture_component->PostProcessSettings.AddBlendable(
                                UMaterialInstanceDynamic::Create(mat, new_scene_capture_component), 1.0f);
                }
                
                // Set camera pass
                new_pass.scene_capture_component_ = new_scene_capture_component;
                new_pass.texture_render_target_ = new_texture_render_target;

                //Insert into map
                camera_passes_.insert(std::pair<std::string, CameraPass> (pass_name, new_pass));

        }
}

CameraSensor::~CameraSensor(){
        std::map<std::string, CameraPass>::iterator pass;

        for (pass = camera_passes_.begin(); pass != camera_passes_.end(); pass++)
        {
                ASSERT(pass->second.scene_capture_component_);
                pass->second.texture_render_target_->MarkPendingKill();
                pass->second.texture_render_target_ = nullptr;

                ASSERT(pass->second.texture_render_target_);
                pass->second.scene_capture_component_->DestroyComponent();
                pass->second.scene_capture_component_ = nullptr;
        }
}

std::map<std::string, TArray<FColor>> CameraSensor::GetRenderData(){
        std::map<std::string, TArray<FColor>> data;
        std::map<std::string, CameraPass>::iterator pass;

        //Get data from all passes
        for (pass = camera_passes_.begin(); pass != camera_passes_.end(); pass++){
                FTextureRenderTargetResource* target_resource = pass->second.scene_capture_component_->TextureTarget->GameThread_GetRenderTargetResource();
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

                data.insert(std::pair<std::string, TArray<FColor>> (pass->first, pixels));
        }

        return data;
}

//depth codification
//decode formula : depth = ((r) + (g * 256) + (b * 256 * 256)) / ((256 * 256 * 256) - 1) * f
std::vector<float> CameraSensor::FColorToFloatImage(TArray<FColor> data)
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

void CameraSensor::SetCameraParameters(USceneCaptureComponent2D* camera, bool bPostProcessing){
        //SET OVERRIDES
        camera->PostProcessSettings.bOverride_AutoExposureMethod = true;
        camera->PostProcessSettings.AutoExposureMethod = EAutoExposureMethod::AEM_Histogram;
        camera->PostProcessSettings.bOverride_AutoExposureBias = true;
        camera->PostProcessSettings.bOverride_AutoExposureMinBrightness = true;
        camera->PostProcessSettings.bOverride_AutoExposureMaxBrightness = true;
        camera->PostProcessSettings.bOverride_AutoExposureSpeedUp = true;
        camera->PostProcessSettings.bOverride_AutoExposureSpeedDown = true;
        camera->PostProcessSettings.bOverride_AutoExposureCalibrationConstant_DEPRECATED = true;
        camera->PostProcessSettings.bOverride_HistogramLogMin = true;
        camera->PostProcessSettings.HistogramLogMin = 1.0f;
        camera->PostProcessSettings.bOverride_HistogramLogMax = true;
        camera->PostProcessSettings.HistogramLogMax = 12.0f;

        // Camera
        camera->PostProcessSettings.bOverride_CameraShutterSpeed = true;
        camera->PostProcessSettings.bOverride_CameraISO = true;
        camera->PostProcessSettings.bOverride_DepthOfFieldFstop = true;
        camera->PostProcessSettings.bOverride_DepthOfFieldMinFstop = true;
        camera->PostProcessSettings.bOverride_DepthOfFieldBladeCount = true;

        // Film (Tonemapper)
        camera->PostProcessSettings.bOverride_FilmSlope = true;
        camera->PostProcessSettings.bOverride_FilmToe = true;
        camera->PostProcessSettings.bOverride_FilmShoulder = true;
        camera->PostProcessSettings.bOverride_FilmWhiteClip = true;
        camera->PostProcessSettings.bOverride_FilmBlackClip = true;

        // Motion blur
        camera->PostProcessSettings.bOverride_MotionBlurAmount = true;
        camera->PostProcessSettings.MotionBlurAmount = 0.45f;
        camera->PostProcessSettings.bOverride_MotionBlurMax = true;
        camera->PostProcessSettings.MotionBlurMax = 0.35f;
        camera->PostProcessSettings.bOverride_MotionBlurPerObjectSize = true;
        camera->PostProcessSettings.MotionBlurPerObjectSize = 0.1f;

        // Color Grading
        camera->PostProcessSettings.bOverride_WhiteTemp = true;
        camera->PostProcessSettings.bOverride_WhiteTint = true;
        camera->PostProcessSettings.bOverride_ColorContrast = true;
#if PLATFORM_LINUX
        // Looks like Windows and Linux have different outputs with the
        // same exposure compensation, this fixes it.
        camera->PostProcessSettings.ColorContrast = FVector4(1.2f, 1.2f, 1.2f, 1.0f);
#endif

        // Chromatic Aberration
        camera->PostProcessSettings.bOverride_SceneFringeIntensity = true;
        camera->PostProcessSettings.bOverride_ChromaticAberrationStartOffset = true;

        // Ambient Occlusion
        camera->PostProcessSettings.bOverride_AmbientOcclusionIntensity = true;
        camera->PostProcessSettings.AmbientOcclusionIntensity = 0.5f;
        camera->PostProcessSettings.bOverride_AmbientOcclusionRadius = true;
        camera->PostProcessSettings.AmbientOcclusionRadius = 100.0f;
        camera->PostProcessSettings.bOverride_AmbientOcclusionStaticFraction = true;
        camera->PostProcessSettings.AmbientOcclusionStaticFraction = 1.0f;
        camera->PostProcessSettings.bOverride_AmbientOcclusionFadeDistance = true;
        camera->PostProcessSettings.AmbientOcclusionFadeDistance = 50000.0f;
        camera->PostProcessSettings.bOverride_AmbientOcclusionPower = true;
        camera->PostProcessSettings.AmbientOcclusionPower = 2.0f;
        camera->PostProcessSettings.bOverride_AmbientOcclusionBias = true;
        camera->PostProcessSettings.AmbientOcclusionBias = 3.0f;
        camera->PostProcessSettings.bOverride_AmbientOcclusionQuality = true;
        camera->PostProcessSettings.AmbientOcclusionQuality = 100.0f;

        // Bloom
        camera->PostProcessSettings.bOverride_BloomMethod = true;
        camera->PostProcessSettings.BloomMethod = EBloomMethod::BM_SOG;
        camera->PostProcessSettings.bOverride_BloomIntensity = true;
        camera->PostProcessSettings.BloomIntensity = 0.675f;
        camera->PostProcessSettings.bOverride_BloomThreshold = true;
        camera->PostProcessSettings.BloomThreshold = -1.0f;

        // Lens
        camera->PostProcessSettings.bOverride_LensFlareIntensity = true;
        camera->PostProcessSettings.LensFlareIntensity = 0.1;

        if (bPostProcessing)
        {
            camera->ShowFlags.EnableAdvancedFeatures();
            camera->ShowFlags.SetMotionBlur(true);
            return;
        }
        //camera->ShowFlags.SetAmbientOcclusion(false);
        //camera->ShowFlags.SetAntiAliasing(false);
        //camera->ShowFlags.SetVolumetricFog(false);
        //camera->ShowFlags.SetAtmosphericFog(false);
        //camera->ShowFlags.SetAudioRadius(false);
        //camera->ShowFlags.SetBillboardSprites(false);
        //camera->ShowFlags.SetBloom(false);
        //camera->ShowFlags.SetBounds(false);
        //camera->ShowFlags.SetBrushes(false);
        //camera->ShowFlags.SetBSP(false);
        //camera->ShowFlags.SetBSPSplit(false);
        //camera->ShowFlags.SetBSPTriangles(false);
        //camera->ShowFlags.SetBuilderBrush(false);
        //camera->ShowFlags.SetCameraAspectRatioBars(false);
        //camera->ShowFlags.SetCameraFrustums(false);
        //camera->ShowFlags.SetCameraImperfections(false);
        //camera->ShowFlags.SetCameraInterpolation(false);
        //camera->ShowFlags.SetCameraSafeFrames(false);
        //camera->ShowFlags.SetCollision(false);
        //camera->ShowFlags.SetCollisionPawn(false);
        //camera->ShowFlags.SetCollisionVisibility(false);
        //camera->ShowFlags.SetColorGrading(false);
        //camera->ShowFlags.SetCompositeEditorPrimitives(false);
        //camera->ShowFlags.SetConstraints(false);
        //camera->ShowFlags.SetCover(false);
        //camera->ShowFlags.SetDebugAI(false);
        //camera->ShowFlags.SetDecals(false);
        //camera->ShowFlags.SetDeferredLighting(false);
        //camera->ShowFlags.SetDepthOfField(false);
        //camera->ShowFlags.SetDiffuse(false);
        //camera->ShowFlags.SetDirectionalLights(false);
        //camera->ShowFlags.SetDirectLighting(false);
        //camera->ShowFlags.SetDistanceCulledPrimitives(false);
        //camera->ShowFlags.SetDistanceFieldAO(false);
        //camera->ShowFlags.SetDistanceFieldGI(false);
        //camera->ShowFlags.SetDynamicShadows(false);
        //camera->ShowFlags.SetEditor(false);
        //camera->ShowFlags.SetEyeAdaptation(false);
        //camera->ShowFlags.SetFog(false);
        //camera->ShowFlags.SetGame(false);
        //camera->ShowFlags.SetGameplayDebug(false);
        //camera->ShowFlags.SetGBufferHints(false);
        //camera->ShowFlags.SetGlobalIllumination(false);
        //camera->ShowFlags.SetGrain(false);
        //camera->ShowFlags.SetGrid(false);
        //camera->ShowFlags.SetHighResScreenshotMask(false);
        //camera->ShowFlags.SetHitProxies(false);
        //camera->ShowFlags.SetHLODColoration(false);
        //camera->ShowFlags.SetHMDDistortion(false);
        //camera->ShowFlags.SetIndirectLightingCache(false);
        //camera->ShowFlags.SetInstancedFoliage(false);
        //camera->ShowFlags.SetInstancedGrass(false);
        //camera->ShowFlags.SetInstancedStaticMeshes(false);
        //camera->ShowFlags.SetLandscape(false);
        //camera->ShowFlags.SetLargeVertices(false);
        //camera->ShowFlags.SetLensFlares(false);
        //camera->ShowFlags.SetLevelColoration(false);
        //camera->ShowFlags.SetLightComplexity(false);
        //camera->ShowFlags.SetLightFunctions(false);
        //camera->ShowFlags.SetLightInfluences(false);
        //camera->ShowFlags.SetLighting(false);
        //camera->ShowFlags.SetLightMapDensity(false);
        //camera->ShowFlags.SetLightRadius(false);
        //camera->ShowFlags.SetLightShafts(false);
        //camera->ShowFlags.SetLOD(false);
        //camera->ShowFlags.SetLODColoration(false);
        //camera->ShowFlags.SetMaterials(false);
        //camera->ShowFlags.SetMaterialTextureScaleAccuracy(false);
        //camera->ShowFlags.SetMeshEdges(false);
        //camera->ShowFlags.SetMeshUVDensityAccuracy(false);
        //camera->ShowFlags.SetModeWidgets(false);
        //camera->ShowFlags.SetMotionBlur(false);
        //camera->ShowFlags.SetNavigation(false);
        //camera->ShowFlags.SetOnScreenDebug(false);
        //camera->ShowFlags.SetOutputMaterialTextureScales(false);
        //camera->ShowFlags.SetOverrideDiffuseAndSpecular(false);
        //camera->ShowFlags.SetPaper2DSprites(false);
        //camera->ShowFlags.SetParticles(false);
        //camera->ShowFlags.SetPivot(false);
        //camera->ShowFlags.SetPointLights(false);
        //camera->ShowFlags.SetPostProcessing(false);
        //camera->ShowFlags.SetPostProcessMaterial(false);
        //camera->ShowFlags.SetPrecomputedVisibility(false);
        //camera->ShowFlags.SetPrecomputedVisibilityCells(false);
        //camera->ShowFlags.SetPreviewShadowsIndicator(false);
        //camera->ShowFlags.SetPrimitiveDistanceAccuracy(false);
        //camera->ShowFlags.SetPropertyColoration(false);
        //camera->ShowFlags.SetQuadOverdraw(false);
        //camera->ShowFlags.SetReflectionEnvironment(false);
        //camera->ShowFlags.SetReflectionOverride(false);
        //camera->ShowFlags.SetRefraction(false);
        //camera->ShowFlags.SetRendering(false);
        //camera->ShowFlags.SetSceneColorFringe(false);
        //camera->ShowFlags.SetScreenPercentage(false);
        //camera->ShowFlags.SetScreenSpaceAO(false);
        //camera->ShowFlags.SetScreenSpaceReflections(false);
        //camera->ShowFlags.SetSelection(false);
        //camera->ShowFlags.SetSelectionOutline(false);
        //camera->ShowFlags.SetSeparateTranslucency(false);
        //camera->ShowFlags.SetShaderComplexity(false);
        //camera->ShowFlags.SetShaderComplexityWithQuadOverdraw(false);
        //camera->ShowFlags.SetShadowFrustums(false);
        //camera->ShowFlags.SetSkeletalMeshes(false);
        //camera->ShowFlags.SetSkinCache(false);
        //camera->ShowFlags.SetSkyLighting(false);
        //camera->ShowFlags.SetSnap(false);
        //camera->ShowFlags.SetSpecular(false);
        //camera->ShowFlags.SetSplines(false);
        //camera->ShowFlags.SetSpotLights(false);
        //camera->ShowFlags.SetStaticMeshes(false);
        //camera->ShowFlags.SetStationaryLightOverlap(false);
        //camera->ShowFlags.SetStereoRendering(false);
        //camera->ShowFlags.SetStreamingBounds(false);
        //camera->ShowFlags.SetSubsurfaceScattering(false);
        //camera->ShowFlags.SetTemporalAA(false);
        //camera->ShowFlags.SetTessellation(false);
        //camera->ShowFlags.SetTestImage(false);
        //camera->ShowFlags.SetTextRender(false);
        //camera->ShowFlags.SetTexturedLightProfiles(false);
        //camera->ShowFlags.SetTonemapper(false);
        //camera->ShowFlags.SetTranslucency(false);
        //camera->ShowFlags.SetVectorFields(false);
        //camera->ShowFlags.SetVertexColors(false);
        //camera->ShowFlags.SetVignette(false);
        //camera->ShowFlags.SetVisLog(false);
        //camera->ShowFlags.SetVisualizeAdaptiveDOF(false);
        //camera->ShowFlags.SetVisualizeBloom(false);
        //camera->ShowFlags.SetVisualizeBuffer(false);
        //camera->ShowFlags.SetVisualizeDistanceFieldAO(false);
        //camera->ShowFlags.SetVisualizeDOF(false);
        //camera->ShowFlags.SetVisualizeHDR(false);
        //camera->ShowFlags.SetVisualizeLightCulling(false);
        //camera->ShowFlags.SetVisualizeLPV(false);
        //camera->ShowFlags.SetVisualizeMeshDistanceFields(false);
        //camera->ShowFlags.SetVisualizeMotionBlur(false);
        //camera->ShowFlags.SetVisualizeOutOfBoundsPixels(false);
        //camera->ShowFlags.SetVisualizeSenses(false);
        //camera->ShowFlags.SetVisualizeShadingModels(false);
        //camera->ShowFlags.SetVisualizeSSR(false);
        //camera->ShowFlags.SetVisualizeSSS(false);
        //camera->ShowFlags.SetVolumeLightingSamples(false);
        //camera->ShowFlags.SetVolumes(false);
        //camera->ShowFlags.SetWidgetComponents(false);
        //camera->ShowFlags.SetWireframe(false);
}
