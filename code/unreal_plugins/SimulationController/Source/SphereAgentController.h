#pragma once

#include <map>
#include <string>
#include <vector>

#include "AgentController.h"

// forward declarations
struct Box;
class AActor;
class UWorld;
class USceneCaptureComponent2D;

class SphereAgentController : public AgentController
{
public:

    // CAUTION: This UWorld pointer points to the only running game world, so be careful in how you use it.
    // When you pass around UWorld*, try to use only get() functions and not do any write operations with it.
    SphereAgentController(UWorld* world);
    ~SphereAgentController() = default;
    
    void applyAction(const std::map<std::string, std::vector<float>>& action) override;
    std::map<std::string, std::vector<uint8_t>> getObservation() const override;
    std::map<std::string, Box> getObservationSpace() const override;
    std::map<std::string, Box> getActionSpace() const override;

private:

    AActor* sphere_actor_ = nullptr;
    AActor* observation_camera_ = nullptr;
    USceneCaptureComponent2D* scene_capture_component_ = nullptr;
};
