//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <functional> // std::function

#include <Components/ActorComponent.h>
#include <GameFramework/Actor.h>
#include <Math/Vector.h>
#include <UObject/ObjectMacros.h> // GENERATED_BODY, UCLASS, UFUNCTION

#include "CoreUtils/Log.h"

#include "ActorHitEventComponent.generated.h"

struct FHitResult;

UCLASS()
class UActorHitEventComponent : public UActorComponent
{
    GENERATED_BODY()
public:
    UActorHitEventComponent()
    {
        SP_LOG_CURRENT_FUNCTION();
    }

    ~UActorHitEventComponent()
    {
        SP_LOG_CURRENT_FUNCTION();
    }

    void subscribe(AActor* actor)
    {
        actor->OnActorHit.AddDynamic(this, &UActorHitEventComponent::actorHitEventHandler);
    }

    void unsubscribe(AActor* actor)
    {
        actor->OnActorHit.RemoveDynamic(this, &UActorHitEventComponent::actorHitEventHandler);
    }

    std::function<void(AActor*, AActor*, FVector, const FHitResult&)> actor_hit_func_;

private:
    UFUNCTION()
    void actorHitEventHandler(AActor* self_actor, AActor* other_actor, FVector normal_impulse, const FHitResult& hit_result)
    {
        if (actor_hit_func_) {
            actor_hit_func_(self_actor, other_actor, normal_impulse, hit_result);
        }
    }
};
