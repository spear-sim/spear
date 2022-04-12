// This is useful only when this Plugin is used with interiorsim/code/experiments/debug_synchronization_model/DebugProject,
// as the actors referred to in this file are available only in DebugProject.
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

    // This UWorld pointer passed here points to the only running game world.
    DebugAgentController(UWorld* world);
    ~DebugAgentController();
    
    std::map<std::string, Box> getActionSpace() const override;
    std::map<std::string, Box> getObservationSpace() const override;
    void applyAction(const std::map<std::string, std::vector<float>>& action) override;
    std::map<std::string, std::vector<uint8_t>> getObservation() const override;

private:

    AActor* agent_actor_ = nullptr;
    AActor* first_observation_camera_ = nullptr;
    AActor* second_observation_camera_ = nullptr;
    USceneCaptureComponent2D* first_scene_capture_component_ = nullptr;
    USceneCaptureComponent2D* second_scene_capture_component_ = nullptr;
};
