//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <CoreMinimal.h>
#include <Components/ActorComponent.h>
#include <GameFramework/Actor.h>
#include <Math/Vector.h>

#include "CoreUtils/Log.h"

#include "ActorHitEventComponent.generated.h"

class FObjectInitializer;
struct FHitResult;

DECLARE_MULTICAST_DELEGATE_FourParams(FActorHitDelegate, AActor*, AActor*, FVector, const FHitResult&);

UCLASS()
class UActorHitEventComponent : public UActorComponent
{
    GENERATED_BODY()
public:
    UActorHitEventComponent(const FObjectInitializer& object_initializer) : UActorComponent(object_initializer)
    {
        SP_LOG_CURRENT_FUNCTION();
    }

    ~UActorHitEventComponent()
    {
        SP_LOG_CURRENT_FUNCTION();
    }

    void subscribeToActor(AActor* actor)
    {
        actor->OnActorHit.AddDynamic(this, &UActorHitEventComponent::actorHitEventHandler);
    }

    void unsubscribeFromActor(AActor* actor)
    {
        actor->OnActorHit.RemoveDynamic(this, &UActorHitEventComponent::actorHitEventHandler);
    }

    FActorHitDelegate delegate_;

private:
    UFUNCTION()
    void actorHitEventHandler(AActor* self_actor, AActor* other_actor, FVector normal_impulse, const FHitResult& hit_result)
    {
        delegate_.Broadcast(self_actor, other_actor, normal_impulse, hit_result);
    }
};
