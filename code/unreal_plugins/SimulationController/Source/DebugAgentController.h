// This is useful only when this Plugin is used with interiorsim/code/experiments/debug_synchronization_model/DebugProject as the actors referred to in this file are available only in DebugProject.
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

class DebugAgentController : public AgentController
{
public:

    // CAUTION: This UWorld pointer points to the only running game world, so do not clear the value stored at this address.
    // We are using it here only to iterate through all actors in the world.
    DebugAgentController(UWorld* world);
    ~DebugAgentController() = default;
    
    std::map<std::string, Box> getActionSpace() const override;
    std::map<std::string, Box> getObservationSpace() const override;
    void applyAction(const std::map<std::string, std::vector<float>>& action) override;
    std::map<std::string, std::vector<uint8_t>> getObservation() const override;

private:

    AActor* sphere_actor_ = nullptr;
    AActor* first_observation_camera_ = nullptr;
    AActor* second_observation_camera_ = nullptr;
    USceneCaptureComponent2D* first_scene_capture_component_ = nullptr;
    USceneCaptureComponent2D* second_scene_capture_component_ = nullptr;
    UTextureRenderTarget2D* texture_render_target_ = nullptr;
};
