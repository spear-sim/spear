#pragma once

#include <Engine/EngineTypes.h>
#include <Math/RandomStream.h>
#include <NavMesh/NavMeshBoundsVolume.h>
#include <NavMesh/RecastNavMesh.h>
#include <NavigationSystem.h>

#include "Task.h"

class AActor;
class UWorld;

class UActorHitEvent;

struct Box;

class ImitationLearningTask : public Task {
public:

    ImitationLearningTask() = default;
    ~ImitationLearningTask() = default;

    void findObjectReferences(UWorld* world) override;
    void cleanUpObjectReferences() override;

    void beginFrame() override;
    void endFrame() override;
    float getReward() const override;
    bool isEpisodeDone() const override;
    std::map<std::string, Box> getStepInfoSpace() const override;
    std::map<std::string, std::vector<uint8_t>> getStepInfo() const override;
    void reset() override;
    bool isReady() const override;

    void actorHitEventHandler(AActor* self_actor, AActor* other_actor, FVector normal_impulse, const FHitResult& hit);

private:
    // Get a list of the different initial and final positions the agent is expected to navigate to.
    // Positions are in the format "init.X, init.Y, init.Z, goal.X, goal.Y, goal.Z".
    void getPositionsFromFile();

    // Generate a pair of random (initial point - reachable target point) as well as a collision-free trajectory between them.
    // Multiple pairs of (initial point - reachable target point) as well as trajectories between them are generated and evaluated.
    // Only the best pair is kept.
    void getPositionsFromTrajectorySampling();

    bool hit_goal_ = false;
    bool hit_obstacle_ = false;

    FRandomStream random_stream_;

    AActor* agent_actor_ = nullptr;
    AActor* goal_actor_ = nullptr;
    AActor* new_object_parent_actor_ = nullptr;

    UActorHitEvent* actor_hit_event_ = nullptr;
    FDelegateHandle actor_hit_event_delegate_handle_;

    std::vector<AActor*> obstacle_ignore_actors_;

    // Navigation
    UNavigationSystemV1* nav_sys_ = nullptr;
    ANavigationData* nav_data_ = nullptr;
    ARecastNavMesh* nav_mesh_ = nullptr;
    std::vector<FVector> agent_initial_position_; // Initial position of the learning agent
    std::vector<FVector> agent_goal_position_;    // Goal position of the learning agent (should be the position of the goal agent)
    unsigned int position_index_ = 0;             // Index of the trajectory pair
};
