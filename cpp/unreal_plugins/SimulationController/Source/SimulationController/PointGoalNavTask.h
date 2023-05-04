//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <map>
#include <string>
#include <vector>

#include <Delegates/IDelegateInstance.h>
#include <Math/RandomStream.h>
#include <Math/Vector.h>

#include "SimulationController/Task.h"

class AActor;
class AStaticMeshActor;
class UWorld;
struct FHitResult;

class UActorHitEvent;
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

    UActorHitEvent* actor_hit_event_ = nullptr;
    FDelegateHandle actor_hit_event_delegate_handle_;

    FRandomStream random_stream_;    

    bool hit_goal_ = false;
    bool hit_obstacle_ = false;
};
