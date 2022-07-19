#include "PostProcessCaptureComponent2D.h"


UPostProcessCaptureComponent2D::UPostProcessCaptureComponent2D(const FObjectInitializer &ObjectInitializer)
: Super(ObjectInitializer){
        //Load PostProcess Materials
        //AddPostProcessingMaterial(TEXT("Material'/SimulationController/PostProcessMaterials/PostProcess_Segmentation.PostProcess_Segmentation'"));
        //AddPostProcessingMaterial(TEXT("Material'/SimulationController/PostProcessMaterials/PostProcessBlendable.PostProcessBlendable'"));
        AddPostProcessingMaterial(TEXT("/SimulationController/PostProcessMaterials/PostProcessBlendable.PostProcessBlendable"));

        //Set Default Camera Parameters
        SetCameraDefaultOverrides();

        this->bCaptureEveryFrame = true;
}

bool UPostProcessCaptureComponent2D::AddPostProcessingMaterial(const FString &Path){
        UMaterial* mat = LoadObject<UMaterial>(nullptr, *Path);
        if(mat != nullptr){
                this->materialsFound.Add(mat);
                return true;
        }else{
                return false;
        }
	//ConstructorHelpers::FObjectFinder<UMaterial> Loader(*Path);
  	//if (Loader.Succeeded()){
    	//        this->materialsFound.Add(Loader.Object);
  	//}
  	//return Loader.Succeeded();
}

void UPostProcessCaptureComponent2D::SetPostProcessBlendables(){
        if(this->materialsFound.Num() == 0){
                UE_LOG(LogTemp, Warning, TEXT("no materials found"));
                return;
        }
        //UE_LOG(LogTemp, Warning, TEXT("Materials in array,%s"), this->materialsFound.Num());

        for(const auto &m : this->materialsFound){
                //ASSERT(m);
                UE_LOG(LogTemp, Warning, TEXT("material found"));
                AddPostProcessBlendable(m);  
        }
}

void UPostProcessCaptureComponent2D::AddPostProcessBlendable(UMaterial* mat){
	ASSERT(mat);
	this->PostProcessSettings.AddBlendable(UMaterialInstanceDynamic::Create(mat, this), 1.0f);
}

bool UPostProcessCaptureComponent2D::ActivateBlendablePass(uint8 pass_id){
        if(pass_id > this->materialsFound.Num()){
                return false;
        }
        //for(uint8 pass = 0;pass < this->PostProcessSettings.WeightedBlendables.Array.Num(); pass++){
        //        if(pass == pass_id){
        //                this->PostProcessSettings.WeightedBlendables.Array[pass].Weight = 1.0f;
        //       }else{
        //                this->PostProcessSettings.WeightedBlendables.Array[pass].Weight = 0.0f;
        //        } 
        //}
        for(auto passes : this->PostProcessSettings.WeightedBlendables.Array){
                passes.Weight = .0f;
        }
        this->PostProcessSettings.WeightedBlendables.Array[pass_id].Weight = 1.0f;
        return true;
}

bool UPostProcessCaptureComponent2D::ActivateBlendablePass(std::string pass_name){
        //SEARCH THE BEST WAY TO PARSE DIFERENT PASSES FFROM PYTHON AND GIVE THE HAB TO GIVE CUSTOM PASSES FROM CLIENT
        return true;
}

void UPostProcessCaptureComponent2D::SetCameraComponent(){
        //Configure Camera Flags
        ConfigureShowFlags(this->bEnablePostProcessingEffects);

        //Set Blendable Materials
        SetPostProcessBlendables(); 

        //Set Scene capture source
        this->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;

        UKismetSystemLibrary::ExecuteConsoleCommand(
		GetWorld(),
		FString("g.TimeoutForBlockOnRenderFence 300000"));
}

void UPostProcessCaptureComponent2D::OnComponentDestroyed(bool bDestroyingHierarchy){
	Super::OnComponentDestroyed(bDestroyingHierarchy);
        //for(const auto &m : this->materialsFound){
        //        m = nullptr;
        //}
}

void UPostProcessCaptureComponent2D::OnRegister(){
	Super::OnRegister();
	SetCameraComponent();
}

void UPostProcessCaptureComponent2D::SendRenderTransform_Concurrent(){	
	Super::SendRenderTransform_Concurrent();
}

void UPostProcessCaptureComponent2D::TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction *ThisTickFunction){
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
        //
}

void UPostProcessCaptureComponent2D::SetCameraDefaultOverrides(){
	this->PostProcessSettings.bOverride_AutoExposureMethod = true;
        this->PostProcessSettings.AutoExposureMethod = EAutoExposureMethod::AEM_Histogram;
        this->PostProcessSettings.bOverride_AutoExposureBias = true;
        this->PostProcessSettings.bOverride_AutoExposureMinBrightness = true;
        this->PostProcessSettings.bOverride_AutoExposureMaxBrightness = true;
        this->PostProcessSettings.bOverride_AutoExposureSpeedUp = true;
        this->PostProcessSettings.bOverride_AutoExposureSpeedDown = true;
        this->PostProcessSettings.bOverride_AutoExposureCalibrationConstant_DEPRECATED = true;
        this->PostProcessSettings.bOverride_HistogramLogMin = true;
        this->PostProcessSettings.HistogramLogMin = 1.0f;
        this->PostProcessSettings.bOverride_HistogramLogMax = true;
        this->PostProcessSettings.HistogramLogMax = 12.0f;

        // Camera
        this->PostProcessSettings.bOverride_CameraShutterSpeed = true;
        this->PostProcessSettings.bOverride_CameraISO = true;
        this->PostProcessSettings.bOverride_DepthOfFieldFstop = true;
        this->PostProcessSettings.bOverride_DepthOfFieldMinFstop = true;
        this->PostProcessSettings.bOverride_DepthOfFieldBladeCount = true;

        // Film (Tonemapper)
        this->PostProcessSettings.bOverride_FilmSlope = true;
        this->PostProcessSettings.bOverride_FilmToe = true;
        this->PostProcessSettings.bOverride_FilmShoulder = true;
        this->PostProcessSettings.bOverride_FilmWhiteClip = true;
        this->PostProcessSettings.bOverride_FilmBlackClip = true;

        // Motion blur
        this->PostProcessSettings.bOverride_MotionBlurAmount = true;
        this->PostProcessSettings.MotionBlurAmount = 0.45f;
        this->PostProcessSettings.bOverride_MotionBlurMax = true;
        this->PostProcessSettings.MotionBlurMax = 0.35f;
        this->PostProcessSettings.bOverride_MotionBlurPerObjectSize = true;
        this->PostProcessSettings.MotionBlurPerObjectSize = 0.1f;

        // Color Grading
        this->PostProcessSettings.bOverride_WhiteTemp = true;
        this->PostProcessSettings.bOverride_WhiteTint = true;
        this->PostProcessSettings.bOverride_ColorContrast = true;
#if PLATFORM_LINUX
        // Looks like Windows and Linux have different outputs with the
        // same exposure compensation, this fixes it.
        this->PostProcessSettings.ColorContrast = FVector4(1.2f, 1.2f, 1.2f, 1.0f);
#endif

        // Chromatic Aberration
        this->PostProcessSettings.bOverride_SceneFringeIntensity = true;
        this->PostProcessSettings.bOverride_ChromaticAberrationStartOffset = true;

        // Ambient Occlusion
        this->PostProcessSettings.bOverride_AmbientOcclusionIntensity = true;
        this->PostProcessSettings.AmbientOcclusionIntensity = 0.5f;
        this->PostProcessSettings.bOverride_AmbientOcclusionRadius = true;
        this->PostProcessSettings.AmbientOcclusionRadius = 100.0f;
        this->PostProcessSettings.bOverride_AmbientOcclusionStaticFraction = true;
        this->PostProcessSettings.AmbientOcclusionStaticFraction = 1.0f;
        this->PostProcessSettings.bOverride_AmbientOcclusionFadeDistance = true;
        this->PostProcessSettings.AmbientOcclusionFadeDistance = 50000.0f;
        this->PostProcessSettings.bOverride_AmbientOcclusionPower = true;
        this->PostProcessSettings.AmbientOcclusionPower = 2.0f;
        this->PostProcessSettings.bOverride_AmbientOcclusionBias = true;
        this->PostProcessSettings.AmbientOcclusionBias = 3.0f;
        this->PostProcessSettings.bOverride_AmbientOcclusionQuality = true;
        this->PostProcessSettings.AmbientOcclusionQuality = 100.0f;

        // Bloom
        this->PostProcessSettings.bOverride_BloomMethod = true;
        this->PostProcessSettings.BloomMethod = EBloomMethod::BM_SOG;
        this->PostProcessSettings.bOverride_BloomIntensity = true;
        this->PostProcessSettings.BloomIntensity = 0.675f;
        this->PostProcessSettings.bOverride_BloomThreshold = true;
        this->PostProcessSettings.BloomThreshold = -1.0f;

        // Lens
        this->PostProcessSettings.bOverride_LensFlareIntensity = true;
        this->PostProcessSettings.LensFlareIntensity = 0.1;

}

void UPostProcessCaptureComponent2D::ConfigureShowFlags(bool bPostProcessing){
	if (bPostProcessing)
        {
            this->ShowFlags.EnableAdvancedFeatures();
            this->ShowFlags.SetMotionBlur(true);
            return;
        }

        this->ShowFlags.SetAmbientOcclusion(false);
        this->ShowFlags.SetAntiAliasing(false);
        this->ShowFlags.SetVolumetricFog(false);
        // ShowFlags.SetAtmosphericFog(false);
        // ShowFlags.SetAudioRadius(false);
        // ShowFlags.SetBillboardSprites(false);
        this->ShowFlags.SetBloom(false);
        // ShowFlags.SetBounds(false);
        // ShowFlags.SetBrushes(false);
        // ShowFlags.SetBSP(false);
        // ShowFlags.SetBSPSplit(false);
        // ShowFlags.SetBSPTriangles(false);
        // ShowFlags.SetBuilderBrush(false);
        // ShowFlags.SetCameraAspectRatioBars(false);
        // ShowFlags.SetCameraFrustums(false);
        this->ShowFlags.SetCameraImperfections(false);
        this->ShowFlags.SetCameraInterpolation(false);
        // ShowFlags.SetCameraSafeFrames(false);
        // ShowFlags.SetCollision(false);
        // ShowFlags.SetCollisionPawn(false);
        // ShowFlags.SetCollisionVisibility(false);
        this->ShowFlags.SetColorGrading(false);
        // ShowFlags.SetCompositeEditorPrimitives(false);
        // ShowFlags.SetConstraints(false);
        // ShowFlags.SetCover(false);
        // ShowFlags.SetDebugAI(false);
        // ShowFlags.SetDecals(false);
        // ShowFlags.SetDeferredLighting(false);
        this->ShowFlags.SetDepthOfField(false);
        this->ShowFlags.SetDiffuse(false);
        this->ShowFlags.SetDirectionalLights(false);
        this->ShowFlags.SetDirectLighting(false);
        // ShowFlags.SetDistanceCulledPrimitives(false);
        // ShowFlags.SetDistanceFieldAO(false);
        // ShowFlags.SetDistanceFieldGI(false);
        this->ShowFlags.SetDynamicShadows(false);
        // ShowFlags.SetEditor(false);
        this->ShowFlags.SetEyeAdaptation(false);
        this->ShowFlags.SetFog(false);
        // ShowFlags.SetGame(false);
        // ShowFlags.SetGameplayDebug(false);
        // ShowFlags.SetGBufferHints(false);
        this->ShowFlags.SetGlobalIllumination(false);
        this->ShowFlags.SetGrain(false);
        // ShowFlags.SetGrid(false);
        // ShowFlags.SetHighResScreenshotMask(false);
        // ShowFlags.SetHitProxies(false);
        this->ShowFlags.SetHLODColoration(false);
        this->ShowFlags.SetHMDDistortion(false);
        // ShowFlags.SetIndirectLightingCache(false);
        // ShowFlags.SetInstancedFoliage(false);
        // ShowFlags.SetInstancedGrass(false);
        // ShowFlags.SetInstancedStaticMeshes(false);
        // ShowFlags.SetLandscape(false);
        // ShowFlags.SetLargeVertices(false);
        this->ShowFlags.SetLensFlares(false);
        this->ShowFlags.SetLevelColoration(false);
        this->ShowFlags.SetLightComplexity(false);
        this->ShowFlags.SetLightFunctions(false);
        this->ShowFlags.SetLightInfluences(false);
        this->ShowFlags.SetLighting(false);
        this->ShowFlags.SetLightMapDensity(false);
        this->ShowFlags.SetLightRadius(false);
        this->ShowFlags.SetLightShafts(false);
        // ShowFlags.SetLOD(false);
        this->ShowFlags.SetLODColoration(false);
        // ShowFlags.SetMaterials(false);
        // ShowFlags.SetMaterialTextureScaleAccuracy(false);
        // ShowFlags.SetMeshEdges(false);
        // ShowFlags.SetMeshUVDensityAccuracy(false);
        // ShowFlags.SetModeWidgets(false);
        this->ShowFlags.SetMotionBlur(false);
        // ShowFlags.SetNavigation(false);
        this->ShowFlags.SetOnScreenDebug(false);
        // ShowFlags.SetOutputMaterialTextureScales(false);
        // ShowFlags.SetOverrideDiffuseAndSpecular(false);
        // ShowFlags.SetPaper2DSprites(false);
        this->ShowFlags.SetParticles(false);
        // ShowFlags.SetPivot(false);
        this->ShowFlags.SetPointLights(false);
        // ShowFlags.SetPostProcessing(false);
        // ShowFlags.SetPostProcessMaterial(false);
        // ShowFlags.SetPrecomputedVisibility(false);
        // ShowFlags.SetPrecomputedVisibilityCells(false);
        // ShowFlags.SetPreviewShadowsIndicator(false);
        // ShowFlags.SetPrimitiveDistanceAccuracy(false);
        this->ShowFlags.SetPropertyColoration(false);
        // ShowFlags.SetQuadOverdraw(false);
        // ShowFlags.SetReflectionEnvironment(false);
        // ShowFlags.SetReflectionOverride(false);
        this->ShowFlags.SetRefraction(false);
        // ShowFlags.SetRendering(false);
        this->ShowFlags.SetSceneColorFringe(false);
        // ShowFlags.SetScreenPercentage(false);
        this->ShowFlags.SetScreenSpaceAO(false);
        this->ShowFlags.SetScreenSpaceReflections(false);
        // ShowFlags.SetSelection(false);
        // ShowFlags.SetSelectionOutline(false);
        // ShowFlags.SetSeparateTranslucency(false);
        // ShowFlags.SetShaderComplexity(false);
        // ShowFlags.SetShaderComplexityWithQuadOverdraw(false);
        // ShowFlags.SetShadowFrustums(false);
        // ShowFlags.SetSkeletalMeshes(false);
        // ShowFlags.SetSkinCache(false);
        this->ShowFlags.SetSkyLighting(false);
        // ShowFlags.SetSnap(false);
        // ShowFlags.SetSpecular(false);
        // ShowFlags.SetSplines(false);
        this->ShowFlags.SetSpotLights(false);
        // ShowFlags.SetStaticMeshes(false);
        this->ShowFlags.SetStationaryLightOverlap(false);
        // ShowFlags.SetStereoRendering(false);
        // ShowFlags.SetStreamingBounds(false);
        this->ShowFlags.SetSubsurfaceScattering(false);
        // ShowFlags.SetTemporalAA(false);
        // ShowFlags.SetTessellation(false);
        // ShowFlags.SetTestImage(false);
        // ShowFlags.SetTextRender(false);
        // ShowFlags.SetTexturedLightProfiles(false);
        this->ShowFlags.SetTonemapper(false);
        // ShowFlags.SetTranslucency(false);
        // ShowFlags.SetVectorFields(false);
        // ShowFlags.SetVertexColors(false);
        // ShowFlags.SetVignette(false);
        // ShowFlags.SetVisLog(false);
        // ShowFlags.SetVisualizeAdaptiveDOF(false);
        // ShowFlags.SetVisualizeBloom(false);
        this->ShowFlags.SetVisualizeBuffer(false);
        this->ShowFlags.SetVisualizeDistanceFieldAO(false);
        this->ShowFlags.SetVisualizeDOF(false);
        this->ShowFlags.SetVisualizeHDR(false);
        this->ShowFlags.SetVisualizeLightCulling(false);
        this->ShowFlags.SetVisualizeLPV(false);
        this->ShowFlags.SetVisualizeMeshDistanceFields(false);
        this->ShowFlags.SetVisualizeMotionBlur(false);
        this->ShowFlags.SetVisualizeOutOfBoundsPixels(false);
        this->ShowFlags.SetVisualizeSenses(false);
        this->ShowFlags.SetVisualizeShadingModels(false);
        this->ShowFlags.SetVisualizeSSR(false);
        this->ShowFlags.SetVisualizeSSS(false);
        // ShowFlags.SetVolumeLightingSamples(false);
        // ShowFlags.SetVolumes(false);
        // ShowFlags.SetWidgetComponents(false);
        // ShowFlags.SetWireframe(false);

}