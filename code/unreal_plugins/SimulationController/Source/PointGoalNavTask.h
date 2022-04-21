#pragma once

#include <Engine/EngineTypes.h>
#include <Math/RandomStream.h>

#include "Task.h"

class AActor;
struct StepInfo;
class UWorld;
class UActorHitEvent;

struct Box;

class PointGoalNavTask: public Task
{
public:

    PointGoalNavTask(UWorld* world);
    ~PointGoalNavTask();

    // Task overrides
    void beginFrame() override;
    void endFrame() override;
    float getReward() const override;
    bool isEpisodeDone() const override;
    std::map<std::string, Box> getStepInfoSpace() const override;
    std::map<std::string, std::vector<uint8_t>> getStepInfo() const override;
    void reset() override;
    bool isReady() const override;
    StepInfo getStepInfo() override;

    // Handles collision-related logic
    void actorHitEventHandler(AActor* self_actor, AActor* other_actor, FVector normal_impulse, const FHitResult& hit);

private:
    bool hit_goal_ = false;
    bool hit_obstacle_ = false;
    
    FRandomStream random_stream_;

    AActor* agent_actor_ = nullptr;
    AActor* goal_actor_ = nullptr;
    AActor* new_object_parent_actor_ = nullptr;

    UActorHitEvent* actor_hit_event_ = nullptr;
    FDelegateHandle actor_hit_event_delegate_handle_;

    std::vector<AActor*> obstacle_ignore_actors_;
};
