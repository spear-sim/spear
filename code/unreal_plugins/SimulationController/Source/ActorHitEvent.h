#pragma once

#include <CoreMinimal.h>
#include <GameFramework/Actor.h>

#include "ActorHitEvent.generated.h"

DECLARE_MULTICAST_DELEGATE_FourParams(OnActorHitEvent, AActor*, AActor*, FVector, const FHitResult&);

UCLASS()
class UActorHitEvent : public UObject
{
    GENERATED_BODY()
public:
    UActorHitEvent(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
    {
        UE_LOG(LogTemp, Warning, TEXT("UActorHitEvent is created..."));
    }

    ~UActorHitEvent()
    {
        UE_LOG(LogTemp, Warning, TEXT("~UActorHitEvent is destroyed..."));
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
    void ActorHitEventHandler(AActor* self_actor, AActor* other_actor, FVector normal_impulse, const FHitResult& hit)
    {
        delegate_.Broadcast(self_actor, other_actor, normal_impulse, hit);
    }

    OnActorHitEvent delegate_;
};
