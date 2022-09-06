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

struct CameraPass
{
    USceneCaptureComponent2D* scene_capture_component_ = nullptr;
    UTextureRenderTarget2D* texture_render_target_ = nullptr;
};

class CameraSensor
{
public:
    CameraSensor(UWorld* world, AActor* actor, std::vector<std::string> passes. unsigned long w, unsigned long h);
    ~CameraSensor();

    std::map<std::string, TArray<FColor>> GetRenderData();

    //Remove this function from here
    std::vector<float> FColorToFloatImage(TArray<FColor> data);

private:
    std::map<std::string, CameraPass> passes_;

    void SetCameraDefaultOverrides(USceneCaptureComponent2D* camera);
	void ConfigureShowFlags(USceneCaptureComponent2D* camera, bool bPostProcessing);
};
