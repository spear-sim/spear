#pragma once

#include <map>
#include <string>
#include <vector>

#include "AgentController.h"

struct Box;
class AActor;
class UWorld;
class USceneCaptureComponent2D;

class SphereAgentController : public AgentController
{
public:

    // This UWorld pointer passed here points to the only running game world.
    SphereAgentController(UWorld* world);
    ~SphereAgentController() = default;
    
    std::map<std::string, Box> getActionSpace() const override;
    std::map<std::string, Box> getObservationSpace() const override;
    void applyAction(const std::map<std::string, std::vector<float>>& action) override;
    std::map<std::string, std::vector<uint8_t>> getObservation() const override;

private:

    AActor* sphere_actor_ = nullptr;
    AActor* cone_actor_ = nullptr;
    AActor* debug_camera_actor_ = nullptr;
    AActor* observation_camera_actor_ = nullptr;

    UStaticMeshComponent* sphere_static_mesh_component_ = nullptr;
    UStaticMeshComponent* cone_static_mesh_component_ = nullptr;
    USceneCaptureComponent2D* scene_capture_component_ = nullptr;
};
