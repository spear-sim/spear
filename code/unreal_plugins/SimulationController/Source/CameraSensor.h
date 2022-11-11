#pragma once

#include <map>
#include <string>
#include <vector>

#include <Math/Color.h>

class AActor;
class UCameraComponent;
class USceneCaptureComponent2D;
class UTextureRenderTarget2D;

struct RenderPass
{
    USceneCaptureComponent2D* scene_capture_component_ = nullptr;
    UTextureRenderTarget2D* texture_render_target_ = nullptr;
};

class CameraSensor
{
public:
    CameraSensor(UCameraComponent* component, const std::vector<std::string>& render_pass_names, unsigned int width, unsigned int height);
    ~CameraSensor();

    std::map<std::string, TArray<FColor>> getRenderData();

    static std::vector<float> getFloatDepthFromColorDepth(TArray<FColor> data);

    std::map<std::string, RenderPass> render_passes_;

private:
    void initializeSceneCaptureComponentFinalColor(USceneCaptureComponent2D* scene_capture_component);

    AActor* new_object_parent_actor_ = nullptr;
};
