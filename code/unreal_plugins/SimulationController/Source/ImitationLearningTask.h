#pragma once

#include <Engine/EngineTypes.h>
#include <Math/RandomStream.h>
#include <NavMesh/NavMeshBoundsVolume.h>
#include <NavMesh/RecastNavMesh.h>
#include <NavigationSystem.h>

#include <Task.h>

class AActor;
class UWorld;

class UActorHitEvent;

struct Box;

class ImitationLearningTask : public Task {
public:
    ImitationLearningTask(UWorld* world);
    ~ImitationLearningTask();

    // Task overrides
    void beginFrame() override;
    void endFrame() override;
    float getReward() const override;
    bool isEpisodeDone() const override;
    std::map<std::string, Box> getStepInfoSpace() const override;
    std::map<std::string, std::vector<uint8_t>> getStepInfo() const override;
    void reset() override;
    bool isReady() const override;

    // Handles collision-related logic
    void actorHitEventHandler(AActor* self_actor, AActor* other_actor, FVector normal_impulse, const FHitResult& hit);

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

    mutable bool hit_goal_ = false;
    bool hit_obstacle_ = false;

    FRandomStream random_stream_;

    AActor* agent_actor_ = nullptr;
    AActor* goal_actor_ = nullptr;
    AActor* new_object_parent_actor_ = nullptr;

    UActorHitEvent* actor_hit_event_ = nullptr;
    FDelegateHandle actor_hit_event_delegate_handle_;

    std::vector<AActor*> obstacle_ignore_actors_;

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
