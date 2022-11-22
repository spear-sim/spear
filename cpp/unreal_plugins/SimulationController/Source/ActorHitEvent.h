#pragma once

#include <CoreMinimal.h>
#include <Components/ActorComponent.h>
#include <GameFramework/Actor.h>

#include "ActorHitEvent.generated.h"

DECLARE_MULTICAST_DELEGATE_FourParams(OnActorHitEvent, AActor*, AActor*, FVector, const FHitResult&);

UCLASS()
class UActorHitEvent : public UActorComponent
{
    GENERATED_BODY()
public:
    UActorHitEvent(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
    {
        PrimaryComponentTick.bCanEverTick = false;
    }

    void subscribeToActor(AActor* actor)
    {
        actor->OnActorHit.AddDynamic(this, &UActorHitEvent::ActorHitEventHandler);
    }

    void unsubscribeFromActor(AActor* actor)
    {
        actor->OnActorHit.RemoveDynamic(this, &UActorHitEvent::ActorHitEventHandler);
    }

    UFUNCTION()
    void ActorHitEventHandler(AActor* self_actor, AActor* other_actor, FVector normal_impulse, const FHitResult& hit_result)
    {
        delegate_.Broadcast(self_actor, other_actor, normal_impulse, hit_result);
    }

    OnActorHitEvent delegate_;
};
