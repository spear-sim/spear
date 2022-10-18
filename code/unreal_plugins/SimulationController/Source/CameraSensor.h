#pragma once

#include <map>
#include <string>
#include <vector>

class AActor;
class USceneCaptureComponent2D;
class UTextureRenderTarget2D;

struct CameraPass
{
    USceneCaptureComponent2D* scene_capture_component_ = nullptr;
    UTextureRenderTarget2D* texture_render_target_ = nullptr;
};

class CameraSensor
{
public:
    CameraSensor(AActor* actor, std::vector<std::string> passes, unsigned long width, unsigned long height);
    ~CameraSensor();

    std::map<std::string, TArray<FColor>> getRenderData();

    static std::vector<float> getFloatDepthFromColorDepth(TArray<FColor> data);

    std::map<std::string, CameraPass> camera_passes_;

private:
    void setCameraParameters(USceneCaptureComponent2D* scene_capture_component, UTextureRenderTarget2D* texture_render_target, unsigned long width, unsigned long height);
    void setCameraParametersNonFinalColor(USceneCaptureComponent2D* scene_capture_component, UTextureRenderTarget2D* texture_render_target, unsigned long width, unsigned long height);
};
