#pragma once

#include <CoreMinimal.h>
#include <Components/ActorComponent.h>

#include "TickEvent.generated.h"

DECLARE_MULTICAST_DELEGATE_TwoParams(OnTickEvent, float, enum ELevelTick);

UCLASS()
class UTickEvent : public UActorComponent
{
    GENERATED_BODY()
public:
    UTickEvent(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
    {
        PrimaryComponentTick.bCanEverTick = true;
        PrimaryComponentTick.bTickEvenWhenPaused = false;
    }
    
    void initialize(const ETickingGroup& tick_group)
    {
        PrimaryComponentTick.TickGroup = tick_group;
    }

    void TickComponent(float delta_time, enum ELevelTick level_tick, FActorComponentTickFunction* this_tick_function) override
    {
        delegate_.Broadcast(delta_time, level_tick);
    }

    OnTickEvent delegate_;
};
