#pragma once

class Task
{
public:
    Task() = default;
    virtual ~Task() = default;

    virtual float getReward() = 0;
    virtual bool isEpisodeDone() const = 0;
    virtual void reset() = 0;
    virtual void ActorHitEventHandler(AActor* self_actor, AActor* other_actor, FVector normal_impulse, const FHitResult& hit) = 0;
};
