#pragma once

#include <string>
#include <vector>
#include <map>

class AActor;
class USceneCaptureComponent2D;
class UTextureRenderTarget2D;
class UWorld;

class UTickEvent;

const std::string MATERIALS_PATH = "/SimulationController/PostProcessMaterials/";
//Find a way to find assets in path
const std::vector<std::string> PASSES =
{
    "Depth",
    "Depth_GLSL",
    "Segmentation",
    "LensDistortion",
    "Normals",
    "PixelVelocity"
};

class CameraSensor
{
public:
    CameraSensor(UWorld* world, AActor* actor_);
    ~CameraSensor();

    void SetRenderTarget(unsigned long w, unsigned long h);

    void SetPostProcessBlendables(std::vector<std::string> blendables);

	void ActivateBlendablePass(std::string pass_name);

    TArray<FColor> GetRenderData();

    void FColorToFloatImage(std::vector<float>& out, TArray<FColor> data);

    bool enable_postprocessing_effects_ = true;

private:
    //used on pre_load functionality
    std::string pre_loaded_pass_ = "";
    std::map<std::string, unsigned long> passes_;

    USceneCaptureComponent2D* scene_capture_component_ = nullptr;
    UTextureRenderTarget2D* texture_render_target_ = nullptr;

    void SetCameraDefaultOverrides();
	void ConfigureShowFlags(bool bPostProcessing);
};
