#pragma once

#include <map>
#include <string>
#include <vector>

#include <NavigationSystem.h>
#include <NavMesh/NavMeshBoundsVolume.h>
#include <NavMesh/RecastNavMesh.h>

#include "AgentController.h"

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

    // Generate a collision-free trajectory between an initial and a target location.
    void generateTrajectoryToTarget();

    AActor* agent_actor_ = nullptr;
    AActor* goal_actor_ = nullptr;
    AActor* observation_camera_actor_ = nullptr;
    AActor* new_object_parent_actor_ = nullptr;

    UTextureRenderTarget2D* texture_render_target_ = nullptr;
    USceneCaptureComponent2D* scene_capture_component_ = nullptr;

    // Navigation
    UNavigationSystemV1* nav_sys_;
    ANavigationData* nav_data_;
    ARecastNavMesh* nav_mesh_;
    FVector agent_initial_position_;    // Initial position of the learning agent
    FVector agent_goal_position_;       // Goal position of the learning agent (should be the position of the goal agent)
    TArray<FNavPathPoint> path_points_; // An array containing the different waypoints to be followed by the agent
    unsigned int index_path_point_ = 1; // Because index 0 is the initial position of the agent...
};
