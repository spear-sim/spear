#pragma once

#include <map>
#include <string>
#include <vector>

#include <Delegates/IDelegateInstance.h>
#include <Engine/EngineTypes.h>
#include <Math/Vector.h>

#include "Task.h"

class AActor;
class ARecastNavMesh;
class UActorHitEvent;
class UNavigationSystemV1;
class UWorld;

struct Box;

class ImitationLearningTask : public Task {
public:

    ImitationLearningTask(UWorld* world);
    ~ImitationLearningTask();

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
    // Get a list of the different initial and goal positions the agent is expected to navigate to.
    // Positions are in the format "init.X, init.Y, init.Z, goal.X, goal.Y, goal.Z".
    void getPositionsFromFile();

    // Generate a pair of random (initial point, reachable goal point) as well as a collision-free trajectory between them.
    // Multiple pairs of (initial point, reachable target point) as well as trajectories between them are generated and evaluated.
    // Only the best pair is kept.
    void getPositionsFromTrajectorySampling();

    // Clear our lists of initial and goal positions.
    void clearPositions();
    
    AActor* agent_actor_ = nullptr;
    AActor* goal_actor_ = nullptr;
    AActor* new_object_parent_actor_ = nullptr;

    std::vector<AActor*> obstacle_ignore_actors_;

    UActorHitEvent* actor_hit_event_ = nullptr;
    FDelegateHandle actor_hit_event_delegate_handle_;

    UNavigationSystemV1* nav_sys_ = nullptr;
    ARecastNavMesh* nav_mesh_ = nullptr;

    // Task state
    std::vector<FVector> agent_initial_positions_; // Initial position of the learning agent
    std::vector<FVector> agent_goal_positions_;    // Goal position of the learning agent
    int position_index_ = -1;                      // Index of the trajectory pair
    bool hit_goal_ = false;                        // Was the goal hit?
    bool hit_obstacle_ = false;                    // Was an obstacle hit?
};
