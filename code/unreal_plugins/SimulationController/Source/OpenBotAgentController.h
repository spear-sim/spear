#pragma once

#include <map>
#include <string>
#include <vector>

#include "AgentController.h"
#include "NavMesh/NavMeshBoundsVolume.h"
#include "NavMesh/RecastNavMesh.h"
#include "NavigationSystem.h"
#include "NavigationSystemTypes.h"

class AActor;
class USceneCaptureComponent2D;
class UTextureRenderTarget2D;
class UWorld;

struct Box;

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
    bool isReady() const override;

private:

    AActor* agent_actor_ = nullptr;
    AActor* goal_actor_ = nullptr;
    AActor* observation_camera_actor_ = nullptr;
    AActor* new_object_parent_actor_ = nullptr;

    UTextureRenderTarget2D* texture_render_target_ = nullptr;
    USceneCaptureComponent2D* scene_capture_component_ = nullptr;

    // An array containing the different waypoints to be followed by the agent:
    mutable TArray<FNavPathPoint> pathPoints_;

    // The path point begin considered by the PID controller:
    mutable FVector2D currentPathPoint_;
    mutable FNavLocation targetLocation_;

    // Index of the considered path point (starts at one since 0 is the initial position):
    mutable unsigned int indexPath_ = 1;
};
