#pragma once

#include <map>
#include <string>
#include <vector>

#include <Engine/EngineTypes.h>
#include <Math/RandomStream.h>

#include "Task.h"

class AActor;
class SphereAgentController;

class PointGoalNavTask: public Task
{
public:

    PointGoalNavTask(SphereAgentController*);
    ~PointGoalNavTask() = default;
    
    float getReward() override;
    void reset() override;

private:

    void ActorHitEventHandler(AActor* self_actor, AActor* other_actor, FVector normal_impulse, const FHitResult& hit);

    SphereAgentController* agent_controller_ = nullptr;
    bool hit_goal_ = false;
    bool hit_other_ = false;
    bool end_episode_ = false;
    float reward_ = -1.f;
    FRandomStream random_stream_;
};
