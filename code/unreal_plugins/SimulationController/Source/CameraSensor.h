#pragma once

#include <map>
#include <string>
#include <vector>

#include <Math/Color.h>

class AActor;
class UCameraComponent;
class USceneCaptureComponent2D;
class UTextureRenderTarget2D;

struct Box;

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

    // High-level interface for getting render data as an observation that conforms to our Agent interface
    std::map<std::string, Box> getObservationSpace() const;
    std::map<std::string, std::vector<uint8_t>> getObservation() const;

    // Unreal resources for each render pass are public in case they need to be modified by user code
    std::map<std::string, RenderPass> render_passes_;

private:

    // Initialize scene capture component for the "final_color" render pass
    void initializeSceneCaptureComponentFinalColor(USceneCaptureComponent2D* scene_capture_component);

    // Low-level interface for getting render data as directly as possible from Unreal
    std::map<std::string, TArray<FColor>> getRenderData() const;

    // Static function for decoding depth data encoded as a color image
    static std::vector<float> getFloatDepthFromColorDepth(TArray<FColor>& data);

    AActor* new_object_parent_actor_ = nullptr;

    int width_ = -1;
    int height_ = -1;
};
