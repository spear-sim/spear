//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <iostream>

#include <CoreMinimal.h>
#include <Components/ActorComponent.h>
#include <GameFramework/Actor.h>

#include "ActorHitEvent.generated.h"

DECLARE_MULTICAST_DELEGATE_FourParams(FActorHitDelegate, AActor*, AActor*, FVector, const FHitResult&);

UCLASS()
class UActorHitEvent : public UActorComponent
{
    GENERATED_BODY()
public:
    UActorHitEvent(const FObjectInitializer& object_initializer) : UActorComponent(object_initializer)
    {
        std::cout << "[SPEAR | ActorHitEvent.h] UActorHitEvent::UActorHitEvent" << std::endl;
    }

    ~UActorHitEvent()
    {
        std::cout << "[SPEAR | ActorHitEvent.h] UActorHitEvent::~UActorHitEvent" << std::endl;
    }

    void subscribeToActor(AActor* actor)
    {
        actor->OnActorHit.AddDynamic(this, &UActorHitEvent::actorHitEventHandler);
    }

    void unsubscribeFromActor(AActor* actor)
    {
        actor->OnActorHit.RemoveDynamic(this, &UActorHitEvent::actorHitEventHandler);
    }

    FActorHitDelegate delegate_;

private:
    UFUNCTION()
    void actorHitEventHandler(AActor* self_actor, AActor* other_actor, FVector normal_impulse, const FHitResult& hit_result)
    {
        delegate_.Broadcast(self_actor, other_actor, normal_impulse, hit_result);
    }
};
