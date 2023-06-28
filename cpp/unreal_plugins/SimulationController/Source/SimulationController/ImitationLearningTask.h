//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <map>
#include <string>
#include <vector>

#include <Delegates/IDelegateInstance.h>
#include <Math/Vector.h>

#include "CoreUtils/ArrayDesc.h"
#include "SimulationController/Task.h"

class AActor;
class UWorld;
struct FHitResult;

class UActorHitEventComponent;

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
    std::map<std::string, ArrayDesc> getStepInfoSpace() const override;
    std::map<std::string, std::vector<uint8_t>> getStepInfo() const override;
    void reset() override;
    bool isReady() const override;

private:
    void actorHitEventHandler(AActor* self_actor, AActor* other_actor, FVector normal_impulse, const FHitResult& hit_result);

    // Get a list of the different initial and goal positions the agent is expected to navigate to.
    // Positions are in the format "start_location.X, start_location.Y, start_location.Z, end_location.X, end_location.Y, end_location.Z".
    void getPositionsFromFile();

    // Clear our lists of initial and goal positions.
    void clearPositions();

    AActor* agent_actor_ = nullptr;
    AActor* goal_actor_ = nullptr;
    AActor* parent_actor_ = nullptr;

    std::vector<AActor*> obstacle_ignore_actors_;

    UActorHitEventComponent* actor_hit_event_component_ = nullptr;
    FDelegateHandle actor_hit_event_handle_;

    // Task state
    std::vector<FVector> agent_initial_positions_; // Initial position of the learning agent
    std::vector<FVector> agent_goal_positions_;    // Goal position of the learning agent
    int position_index_ = -1;                      // Index of the trajectory pair
    bool hit_goal_ = false;                        // Was the goal hit?
    bool hit_obstacle_ = false;                    // Was an obstacle hit?
};
