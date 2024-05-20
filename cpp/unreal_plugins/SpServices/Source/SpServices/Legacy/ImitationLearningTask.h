//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // uint8_t

#include <map>
#include <memory> // std::unique_ptr
#include <string>
#include <vector>

#include <Math/Vector.h>

#include "SpCore/ArrayDesc.h"

#include "SpServices/Legacy/ActorHitComponent.h"
#include "SpServices/Legacy/StandaloneComponent.h"
#include "SpServices/Legacy/Task.h"

class AActor;
class UWorld;

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
    AActor* agent_actor_ = nullptr;
    AActor* goal_actor_ = nullptr;
    std::vector<AActor*> obstacle_ignore_actors_;

    std::unique_ptr<StandaloneComponent<UActorHitComponent>> actor_hit_component_ = nullptr;

    std::vector<FVector> agent_initial_locations_;
    std::vector<FVector> agent_goal_locations_;
    std::string previous_scene_id_ = "";
    int episode_index_ = -1;
    bool hit_goal_ = false;
    bool hit_obstacle_ = false;
};
