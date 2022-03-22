
#include "PIPCamera.h"

#include "Engine/SceneCapture2D.h"
#include "Engine/TextureRenderTarget2D.h"
#include "EngineUtils.h"
#include "Camera/CameraComponent.h"

APIPCamera::APIPCamera()
{
    ConstructorHelpers::FObjectFinder<UMaterial> material(
        TEXT("Material'/Game/Koolab/Materials/Segmentation/"
             "SegmentationMaterial.SegmentationMaterial'"));
    if (material.Succeeded())
    {
        this->PostProcessMaterial = material.Object;
    }
}

void APIPCamera::BeginPlay()
{
    Super::BeginPlay();
}

void APIPCamera::ActivateCamera()
{
    // set main camera
    UCameraComponent* camera = this->GetCameraComponent();
    camera->Deactivate();
    camera->SetVisibility(false);
    camera->AddOrUpdateBlendable(this->PostProcessMaterial, 0.0f);
    APlayerController* controller =
        this->GetWorld()->GetFirstPlayerController();
    if (controller)
    {
        controller->SetViewTarget(this);
    }
}

void APIPCamera::DeactivateCamera()
{
    UCameraComponent* camera = this->GetCameraComponent();
    camera->Deactivate();
    camera->SetVisibility(false);
    APlayerController* controller =
        this->GetWorld()->GetFirstPlayerController();
    if (controller && controller->GetViewTarget() == this)
    {
        controller->SetViewTarget(nullptr);
    }
}

void APIPCamera::SetIndex(int UpdateIndex)
{
    this->index = UpdateIndex;
}

int APIPCamera::GetIndex()
{
    return this->index;
}

void APIPCamera::SetupCameraFromSettings(
    const RobotSim::RobotSimSettings::CameraSetting& camera_setting)
{
    FActorSpawnParameters spawnParameteres;
    this->CaptureComponent = this->GetWorld()->SpawnActor<ASceneCapture2D>(
        FVector::ZeroVector, FRotator::ZeroRotator, spawnParameteres);
    CaptureComponent->AttachToActor(
        this, FAttachmentTransformRules::SnapToTargetNotIncludingScale);
    SetupCaptureComponent();
}

USceneCaptureComponent2D* APIPCamera::GetSceneCaptureComponent()
{
    return CaptureComponent->GetCaptureComponent2D();
}

void APIPCamera::SetupCaptureComponent()
{
    CaptureComponent->GetCaptureComponent2D()->bAlwaysPersistRenderingState = 1;
    CaptureComponent->GetCaptureComponent2D()->bCaptureEveryFrame = 0;

    // Create RenderTargets
    UTextureRenderTarget2D* renderTarget2D =
        NewObject<UTextureRenderTarget2D>();

    // Set FrameWidth and FrameHeight
    renderTarget2D->TargetGamma =
        GEngine
            ->GetDisplayGamma(); // 1.2f; // for Vulkan
                                 // //GEngine->GetDisplayGamma(); // for DX11/12

    // Setup the RenderTarget capture format
    renderTarget2D->InitAutoFormat(
        160,120); // some random format, got crashing otherwise
    // int32 frameWidht = 2048;
    // int32 frameHeight = 2048;
    renderTarget2D->InitCustomFormat(
        160,120, PF_B8G8R8A8,
        true); // PF_B8G8R8A8 disables HDR which will boost storing to disk due
               // to less image information
    renderTarget2D->RenderTargetFormat = ETextureRenderTargetFormat::RTF_RGBA8;
    renderTarget2D->bGPUSharedFlag = true; // demand buffer on GPU
    
    CaptureComponent->GetCaptureComponent2D()->FOVAngle = 72.0f; // Smartphone FOV

    // Assign RenderTarget
    CaptureComponent->GetCaptureComponent2D()->TextureTarget = renderTarget2D;

    // Set Camera Properties
    CaptureComponent->GetCaptureComponent2D()->CaptureSource =
        ESceneCaptureSource::SCS_FinalColorLDR;
    CaptureComponent->GetCaptureComponent2D()->ShowFlags.SetTemporalAA(false);
    CaptureComponent->GetCaptureComponent2D()->ShowFlags.SetAntiAliasing(true);
    // lookup more showflags in the UE4 documentation..

    // Set post processing parameters:
    FPostProcessSettings postProcSet;
    postProcSet.MotionBlurAmount =
        10.f; // Strength of motion blur, 0:off, should be renamed to intensity
    postProcSet.MotionBlurMax = 10.f; // Max distortion caused by motion blur,
                                      // in percent of the screen width, 0:off
    CaptureComponent->GetCaptureComponent2D()->PostProcessSettings =
        postProcSet;
    CaptureComponent->GetCaptureComponent2D()->PostProcessBlendWeight =
        1.f; // Range (0.0, 1.0) where 0 indicates no effect, 1 indicates full
             // effect.

    // Assign PostProcess Material if assigned
    if (PostProcessMaterial)
    { // check nullptr
      // CaptureComponent->GetCaptureComponent2D()->AddOrUpdateBlendable(PostProcessMaterial);
    }
    else
    {
        UE_LOG(LogTemp, Log, TEXT("No PostProcessMaterial is assigend"));
    }
}
