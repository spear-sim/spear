#pragma once

#include <Engine/EngineTypes.h>
#include <Math/RandomStream.h>

#include "Task.h"

class AActor;
class UWorld;
class UActorHitDummyHandler;

class PointGoalNavTask: public Task
{
public:

    PointGoalNavTask(UWorld* world);
    ~PointGoalNavTask();

    // Task overrides
    float getReward() override;
    void reset() override;
    bool isEpisodeDone() const override;
    void ActorHitEventHandler(AActor* self_actor, AActor* other_actor, FVector normal_impulse, const FHitResult& hit) override; // handles collision related logic

private:
    bool hit_goal_ = false;
    bool hit_other_ = false;
    bool end_episode_ = false;
    float reward_ = -1.f;
    FRandomStream random_stream_;

    AActor* agent_actor_ = nullptr;
    AActor* goal_actor_ = nullptr;
    AActor* observation_camera_actor_ = nullptr;

    UActorHitDummyHandler* actor_hit_dummy_handler_ = nullptr;
};
