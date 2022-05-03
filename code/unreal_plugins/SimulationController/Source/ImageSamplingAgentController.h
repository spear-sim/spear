    #pragma once

#include <map>
#include <string>
#include <vector>

#include "AgentController.h"

class AActor;
class USceneCaptureComponent2D;
class UTextureRenderTarget2D;
class UWorld;

struct FNavAgentProperties;
struct FVector;

struct Box;

class ImageSamplingAgentController : public AgentController
{
public:

    // This UWorld pointer passed here points to the only running game world.
    ImageSamplingAgentController(UWorld* world);
    ~ImageSamplingAgentController();
    
    std::map<std::string, Box> getActionSpace() const override;
    std::map<std::string, Box> getObservationSpace() const override;
    void applyAction(const std::map<std::string, std::vector<float>>& action) override;
    std::map<std::string, std::vector<uint8_t>> getObservation() const override;
    void reset() override;
    bool isReady() const override;

private:

    FVector getRandomPointOnNavMesh();
    
    FNavAgentProperties agent_properties_;
    UWorld* world_;
    std::vector<std::string> scenes_;

    AActor* camera_actor_ = nullptr;
    AActor* new_object_parent_actor_ = nullptr;
    AActor* virtual_world_level_manager_ = nullptr;

    UTextureRenderTarget2D* texture_render_target_ = nullptr;
    USceneCaptureComponent2D* scene_capture_component_ = nullptr;
};
