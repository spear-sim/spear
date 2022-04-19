#pragma once

#include <map>
#include <string>
#include <vector>

#include "AgentController.h"

struct Box;
class AActor;
class USceneCaptureComponent2D;
class UTextureRenderTarget2D;
class UWorld;

class OpenBotAgentController : public AgentController
{
public:

    // This UWorld pointer passed here points to the only running game world.
    OpenBotAgentController(UWorld* world);
    ~OpenBotAgentController();
    
    std::map<std::string, Box> getActionSpace() const override;
    std::map<std::string, Box> getObservationSpace() const override;
    void applyAction(const std::map<std::string, std::vector<float>>& action) override;
    std::map<std::string, std::vector<uint8_t>> getObservation() const override;
    void reset() override;

private:

    AActor* agent_actor_ = nullptr;
    AActor* goal_actor_ = nullptr;
    AActor* observation_camera_actor_ = nullptr;
    AActor* new_object_parent_actor_ = nullptr;

    UTextureRenderTarget2D* texture_render_target_ = nullptr;
    USceneCaptureComponent2D* scene_capture_component_ = nullptr;
};
