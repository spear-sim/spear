#pragma once

#include <map>
#include <string>
#include <vector>

#include "AgentController.h"

struct Box;
class AActor;
class UWorld;
class USceneCaptureComponent2D;
class UTextureRenderTarget2D;

class SphereAgentController : public AgentController
{
public:

    // CAUTION: This UWorld pointer points to the only running game world, so do not clear the value stored at this address.
    // We are using it here only to iterate through all actors in the world.
    SphereAgentController(UWorld* world);
    ~SphereAgentController() = default;
    
    std::map<std::string, Box> getActionSpace() const override;
    std::map<std::string, Box> getObservationSpace() const override;
    void applyAction(const std::map<std::string, std::vector<float>>& action) override;
    std::map<std::string, std::vector<uint8_t>> getObservation() const override;

private:

    AActor* sphere_actor_ = nullptr;
    AActor* observation_camera_ = nullptr;
    USceneCaptureComponent2D* scene_capture_component_ = nullptr;
    UTextureRenderTarget2D* texture_render_target_ = nullptr;
};
