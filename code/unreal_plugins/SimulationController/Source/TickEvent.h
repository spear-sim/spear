#pragma once

#include <CoreMinimal.h>
#include <Components/ActorComponent.h>

#include "TickEvent.generated.h"

DECLARE_MULTICAST_DELEGATE(OnTickEvent);

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
        RegisterComponent();
    }

    void TickComponent(float delta_time, enum ELevelTick tick_type, FActorComponentTickFunction* this_tick_function) override
    {
        delegate_.Broadcast();
    }

    OnTickEvent delegate_;
};
