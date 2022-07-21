#pragma once

#include <string>
#include <vector>

class AActor;
class USceneCaptureComponent2D;
class UTextureRenderTarget2D;
class UWorld;

const TArray<FString> PASS_PATHS_ = 
{
    TEXT("/SimulationController/PostProcessMaterials/Depth.Depth"),
    TEXT("/SimulationController/PostProcessMaterials/Segmentation.Segmentation")
};

enum passes{
    Depth,
    Segmentation
};

class CameraSensor
{
public:
    CameraSensor(UWorld* world, AActor* actor_);
    ~CameraSensor();

    bool enable_postprocessing_effects_ = true;

    //DEPRECATED
    void SetPostProcessingMaterial(const FString &Path);
    //DEPRECATED
    void SetPostProcessBlendable(UMaterial* mat);

	//void SetPostProcessBlendables();
    void SetPostProcessBlendables(std::vector<passes> blendables);

	void ActivateBlendablePass(passes pass_id);
	void ActivateBlendablePass(std::string pass_name);

    TArray<FColor> GetRenderData();

    AActor* camera_actor_ = nullptr;

private:

    AActor* new_object_parent_actor_ = nullptr;

    USceneCaptureComponent2D* scene_capture_component_ = nullptr;
    UTextureRenderTarget2D* texture_render_target_ = nullptr;

    void SetCameraDefaultOverrides();
	void ConfigureShowFlags(bool bPostProcessing);
};
// pass basic material transformation into the CameraSensor class
//find directly all the materials in the PostProcessMaterials folder
