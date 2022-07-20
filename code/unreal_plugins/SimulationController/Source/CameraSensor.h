#pragma once

#include <string>
#include <vector>

class AActor;
class USceneCaptureComponent2D;
class UTextureRenderTarget2D;
class UWorld;

class CameraSensor
{
public:
    CameraSensor(UWorld* world);
    ~CameraSensor();

    bool bEnablePostProcessingEffects = true;

    void AddPostProcessingMaterial(const FString &Path);

	void SetPostProcessBlendables();
	void AddPostProcessBlendable(UMaterial* mat);

	bool ActivateBlendablePass(uint8 pass_id);
	bool ActivateBlendablePass(std::string pass_name);

    FTextureRenderTargetResource* GetRenderResource();

    FRotator GetCameraRotation(){return this->camera_actor_->GetActorRotation();}
    void SetCameraRotation(FRotator r){this->camera_actor_->SetActorRotation(r);}

    void SetCameraLocation(FVector l){this->camera_actor_->SetActorLocation(l);}

private:

    AActor* camera_actor_ = nullptr;
    AActor* new_object_parent_actor_ = nullptr;

    USceneCaptureComponent2D* scene_capture_component_ = nullptr;
    UTextureRenderTarget2D* texture_render_target_ = nullptr;

    std::vector<UMaterial*> materialsFound;

    void SetCameraDefaultOverrides();
	void ConfigureShowFlags(bool bPostProcessing);
};