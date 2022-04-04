#pragma once

#include <CoreMinimal.h>
#include <Engine/EngineTypes.h>
#include <Math/RandomStream.h>

#include "Task.h"

#include "PointGoalNavTask.generated.h"

class AActor;
class SphereAgentController;

UCLASS()
class UPointGoalNavTask: public UObject, public Task
{
    GENERATED_BODY()
public:

    UPointGoalNavTask(const FObjectInitializer& ObjectInitializer);
    ~UPointGoalNavTask() = default;
    
    void initializeAgentController(SphereAgentController* agent_controller);

    float getReward() override;
    void reset() override;
    bool isEpisodeDone() const override;

private:

    UFUNCTION()
    void ActorHitEventHandler(AActor* self_actor, AActor* other_actor, FVector normal_impulse, const FHitResult& hit);

    SphereAgentController* agent_controller_ = nullptr;
    bool hit_goal_ = false;
    bool hit_other_ = false;
    bool end_episode_ = false;
    float reward_ = -1.f;
    FRandomStream random_stream_;
};
