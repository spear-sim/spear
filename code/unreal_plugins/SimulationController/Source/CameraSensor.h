#pragma once

#include <string>
#include <vector>
#include <map>

class AActor;
class USceneCaptureComponent2D;
class UTextureRenderTarget2D;
class UWorld;

class UTickEvent;

const FString MATERIALS_PATH_ = "/SimulationController/PostProcessMaterials/";
//Find a way to find assets in path
const std::vector<std::string> PASSES_ =
{
    "Depth",
    "Depth_GLSL"
};

class CameraSensor
{
public:
    CameraSensor(UWorld* world, AActor* actor_);
    ~CameraSensor();

    void SetRenderTarget(unsigned long w, unsigned long h);

    void SetPostProcessBlendables(std::vector<std::string> blendables);

	void ActivateBlendablePass(std::string pass_name);

    void PreRenderTickEventHandler(float delta_time, enum ELevelTick level_tick);

    TArray<FColor> GetRenderData();

    void FcolorArrayToUintVector(std::vector<uint32_t>& out, TArray<FColor> data);

    bool enable_postprocessing_effects_ = true;

    AActor* camera_actor_ = nullptr;
    USceneCaptureComponent2D* scene_capture_component_ = nullptr;

private:
    //used on pre_load functionality
    std::string pre_loaded_pass_ = "";
    std::map<std::string, unsigned long> passes_;

    AActor* new_object_parent_actor_ = nullptr;

    UTextureRenderTarget2D* texture_render_target_ = nullptr;

    //tick event update component
    UTickEvent* pre_render_tick_event_ = nullptr;
    FDelegateHandle pre_render_tick_event_handle_;

    void SetCameraDefaultOverrides();
	void ConfigureShowFlags(bool bPostProcessing);
};
