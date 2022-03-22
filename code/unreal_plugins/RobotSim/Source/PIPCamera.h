#pragma once
class ASceneCapture2D;

#include "CoreMinimal.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/SceneCapture.h"
#include "Camera/CameraActor.h"
#include "Materials/Material.h"

#include "common_utils/RobotSimSettings.hpp"

#include "PIPCamera.generated.h"

UCLASS()
class ROBOTSIM_API APIPCamera : public ACameraActor
{
    GENERATED_BODY()
public:
    APIPCamera();
    virtual void BeginPlay() override;

    void SetupCameraFromSettings(
        const RobotSim::RobotSimSettings::CameraSetting& camera_setting);
    void SetupCaptureComponent();
    USceneCaptureComponent2D* GetSceneCaptureComponent();
    int GetIndex();
    void SetIndex(int UpdateIndex);

    void DeactivateCamera();
    void ActivateCamera();

    UPROPERTY(EditAnywhere, Category = "Capture")
    UMaterial* PostProcessMaterial = nullptr;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Capture")
    ASceneCapture2D* CaptureComponent;

private:
    int index = 0;
};
