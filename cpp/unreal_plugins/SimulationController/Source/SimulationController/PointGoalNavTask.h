//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <map>
#include <random>
#include <string>
#include <vector>

#include <Delegates/IDelegateInstance.h>
#include <Math/Vector.h>

#include "CoreUtils/ArrayDesc.h"
#include "SimulationController/Task.h"

class AActor;
class AStaticMeshActor;
class UWorld;
struct FHitResult;

class UActorHitEventComponent;
struct ArrayDesc;

class PointGoalNavTask: public Task
{
public:
    PointGoalNavTask(UWorld* world);
    ~PointGoalNavTask();

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

    void actorHitEventHandler(AActor* self_actor, AActor* other_actor, FVector normal_impulse, const FHitResult& hit_result);

private:
    AStaticMeshActor* goal_actor_ = nullptr;
    AActor* agent_actor_ = nullptr;
    AActor* parent_actor_ = nullptr;

    std::vector<AActor*> obstacle_ignore_actors_;

    UActorHitEventComponent* actor_hit_event_component_ = nullptr;
    FDelegateHandle actor_hit_event_handle_;

    std::minstd_rand minstd_rand_;

    bool hit_goal_ = false;
    bool hit_obstacle_ = false;
};
