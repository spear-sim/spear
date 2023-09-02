//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <Delegates/IDelegateInstance.h>
#include <Math/Vector.h>

#include "CoreUtils/ArrayDesc.h"
#include "SimulationController/Component.h"
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

    AActor* agent_actor_ = nullptr;
    AActor* goal_actor_ = nullptr;
    std::vector<AActor*> obstacle_ignore_actors_;

    std::unique_ptr<Component<UActorHitEventComponent>> actor_hit_event_component_ = nullptr;
    FDelegateHandle actor_hit_event_delegate_handle_;

    // Task state
    std::vector<FVector> agent_initial_locations_;
    std::vector<FVector> agent_goal_locations_;
    int episode_index_ = -1;
    bool hit_goal_ = false;
    bool hit_obstacle_ = false;
};
