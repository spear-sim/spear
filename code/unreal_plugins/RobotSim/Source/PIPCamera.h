#pragma once

#include "CoreMinimal.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Camera/CameraActor.h"
#include "Materials/Material.h"

#include "common_utils/ImageCaptureBase.hpp"
#include "common_utils/Utils.hpp"
#include "common_utils/RobotSimSettings.hpp"
#include "NedTransform.h"

#include "PIPCamera.generated.h"


UCLASS()
class ROBOTSIM_API APIPCamera : public ACameraActor
{
    GENERATED_BODY()
    

public:
    typedef RobotSim::ImageCaptureBase::ImageType ImageType;
    typedef RobotSim::RobotSimSettings RobotSimSettings;
    typedef RobotSimSettings::CameraSetting CameraSetting;


    APIPCamera();

    virtual void PostInitializeComponents() override;
    virtual void BeginPlay() override;
    virtual void Tick(float DeltaSeconds) override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

    void showToScreen();
    void disableAll();
    void disableAllPIP();
    void disableMain();

    void setCameraTypeEnabled(ImageType type, bool enabled);
    bool getCameraTypeEnabled(ImageType type) const;
    void setupCameraFromSettings(const APIPCamera::CameraSetting& camera_setting, const NedTransform& ned_transform);
    void setCameraOrientation(const FRotator& rotator);

    RobotSim::ProjectionMatrix getProjectionMatrix(const APIPCamera::ImageType image_type) const;


    USceneCaptureComponent2D* getCaptureComponent(const ImageType type, bool if_active);
    UTextureRenderTarget2D* getRenderTarget(const ImageType type, bool if_active);

    RobotSim::Pose getPose() const;

    void setIndex(int index) { this->index_ = index; }
    int getIndex() { return this->index_; }
    
private: //members
    UPROPERTY() TArray<USceneCaptureComponent2D*> captures_;
    UPROPERTY() TArray<UTextureRenderTarget2D*> render_targets_;

    UPROPERTY() UCameraComponent*  camera_;
    //TMap<int, UMaterialInstanceDynamic*> noise_materials_;
    //below is needed because TMap doesn't work with UPROPERTY, but we do have -ve index
    UPROPERTY() TArray<UMaterialInstanceDynamic*> noise_materials_;
    UPROPERTY() UMaterial* noise_material_static_;

    std::vector<bool> camera_type_enabled_;
    FRotator gimbald_rotator_;
    float gimbal_stabilization_;
    const NedTransform* ned_transform_;

    int index_ = 0; // for URDF bot camera cycling

private: //methods
    typedef common_utils::Utils Utils;
    typedef RobotSimSettings::CaptureSetting CaptureSetting;
    typedef RobotSimSettings::NoiseSetting NoiseSetting;

    static unsigned int imageTypeCount();
    void enableCaptureComponent(const ImageType type, bool is_enabled);
    static void updateCaptureComponentSetting(USceneCaptureComponent2D* capture, UTextureRenderTarget2D* render_target, const CaptureSetting& setting, 
        const NedTransform& ned_transform);
    void setNoiseMaterial(int image_type, UObject* outer, FPostProcessSettings& obj, const NoiseSetting& settings);
    static void updateCameraPostProcessingSetting(FPostProcessSettings& obj, const CaptureSetting& setting);
    static void updateCameraSetting(UCameraComponent* camera, const CaptureSetting& setting, const NedTransform& ned_transform);
};
