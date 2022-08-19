#pragma once

#include <iterator>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

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

    // Rebuild the navigation mesh of the agent
    void buildNavMesh();

    // Get an initial and target positions from a parameter file 
    void getPositionsFromParameterFile();

    // Generate a collision-free trajectory between an initial and a target location.
    // Returns true if successful.
    bool generateTrajectoryToTarget();

    // Generate a pair of random (initial point - reachable target point) as well as a collision-free trajectory between them.
    // Multiple pairs of (initial point - reachable target point) as well as trajectories between them are generated and evaluated. 
    // Only the best pair is kept.
    // Returns true if successful.
    bool getPositionsFromSamplingCandidateTrajectories();

    // Get the World bounding box dimensions
    FBox getWorldBoundingBox(bool scale_ceiling = true);

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
    std::vector<FVector> agent_initial_position_;    // Initial position of the learning agent
    std::vector<FVector> agent_goal_position_;       // Goal position of the learning agent (should be the position of the goal agent)
    TArray<FNavPathPoint> path_points_; // An array containing the different waypoints to be followed by the agent
    float trajectory_length_;           // Approximate length of the trajctory between agent_initial_position_ and agent_goal_position_
    float world_to_meters_;             // Scaling factor of the environment
    bool initial_point_generated_ = false; 
    bool target_point_generated_ = false; 
    unsigned int trajectory_index_ = 0;          // Index of the pair 
    unsigned int number_start_goal_pairs_ = 0;  // Number of pairs of start/stop points
};
