#pragma once

#include <Engine/EngineTypes.h>
#include <Math/RandomStream.h>

#include "Task.h"

class AActor;
class UWorld;
class UActorHitEvent;

class PointGoalNavTask: public Task
{
public:

    PointGoalNavTask(UWorld* world);
    ~PointGoalNavTask();

    // Task overrides
    void beginFrame();
    float getReward() override;
    bool isEpisodeDone() const override;
    void reset() override;

    // handles collision related logic
    void ActorHitEventHandler(AActor* self_actor, AActor* other_actor, FVector normal_impulse, const FHitResult& hit);

private:
    bool hit_goal_ = false;
    bool hit_obstacle_ = false;
    
    FRandomStream random_stream_;

    AActor* agent_actor_ = nullptr;
    AActor* goal_actor_ = nullptr;

    UActorHitEvent* actor_hit_event_ = nullptr;
    FDelegateHandle actor_hit_event_delegate_handle_;

    std::vector<AActor*> obstacle_ignore_actors_;
};
