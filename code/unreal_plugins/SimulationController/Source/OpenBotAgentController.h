#pragma once

#include <map>
#include <string>
#include <vector>

#include <NavMesh/NavMeshBoundsVolume.h>
#include <NavMesh/RecastNavMesh.h>
#include <NavigationSystem.h>

#include <AgentController.h>

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

    // Get an initial position for the agent from a parameter file 
    void updateInitialPositionFromParameterFile();

    // Get an initial position for the agent from a parameter file 
    void updateTargetPositionFromParameterFile();

    // Generate a random point in the agent's navigable space
    FVector generateRandomNavigablePoint();

    // Generate a random target point reachable by the agent from its initial position
    void generateRandomReachableTargetPoint();

    // Generate a collision-free trajectory between an initial and a target location.
    // Returns true if successful.
    bool generateTrajectoryToTarget();

    // From a given initial position, generate a random reachable target point as well as a collision-free trajectory between them.
    // Returns true if successful.
    bool generateTrajectoryToRandomTarget();

    // Generate a pair of random (initial point - reachable target point) as well as a collision-free trajectory between them.
    // Returns true if successful.
    bool generateRandomTrajectory();

    // From a given initial position, generate a random reachable target point as well as a collision-free trajectory between them.
    // Multiple target points as well as the trajectories between the initial and target points are generated and evaluated. 
    // Only the best pair of target point / trajectory is kept.
    // Returns true if successful.
    bool sampleTrajectoryToRandomTarget();

    // Generate a pair of random (initial point - reachable target point) as well as a collision-free trajectory between them.
    // Multiple pairs of (initial point - reachable target point) as well as trajectories between them are generated and evaluated. 
    // Only the best pair is kept.
    // Returns true if successful.
    bool sampleRandomTrajectory();


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
    float trajectory_length_;           // Approximate length of the trajctory between agent_initial_position_ and agent_goal_position_
    float world_to_meters_;             // Scaling factor of the environment
    bool initial_point_generated_ = false; 
    bool target_point_generated_ = false; 
};
