#pragma once

#include <functional>

#include <CoreMinimal.h>

#include "Assert.h"
#include "Task.h"

#include "ActorHitDummyHandler.generated.h"

UCLASS()
class UActorHitDummyHandler : public UObject
{
    GENERATED_BODY()
public:
    UActorHitDummyHandler(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer) {}

    void initialize(AActor* actor, Task* task)
    {
        actor->OnActorHit.AddDynamic(this, &UActorHitDummyHandler::ActorHitEventHandler);
        ASSERT(task);
        task_ = task;
    }

    UFUNCTION()
    void ActorHitEventHandler(AActor* self_actor, AActor* other_actor, FVector normal_impulse, const FHitResult& hit)
    {
        task_->ActorHitEventHandler(self_actor, other_actor, normal_impulse, hit);
    }

    void terminate(AActor* actor)
    {
        actor->OnActorHit.RemoveDynamic(this, &UActorHitDummyHandler::ActorHitEventHandler);
    }

private:
    Task* task_ = nullptr;
};
