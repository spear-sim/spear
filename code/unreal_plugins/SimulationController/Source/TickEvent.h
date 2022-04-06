#pragma once

#include <CoreMinimal.h>
#include <Components/ActorComponent.h>

#include "TickEvent.generated.h"

DECLARE_MULTICAST_DELEGATE_ThreeParams(OnTickEvent, float, enum ELevelTick, FActorComponentTickFunction*);

UCLASS()
class UTickEvent : public UActorComponent
{
    GENERATED_BODY()
public:
    UTickEvent(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
    {
        UE_LOG(LogTemp, Warning, TEXT("UTickEvent is created..."));
        PrimaryComponentTick.bCanEverTick = true;
        PrimaryComponentTick.bTickEvenWhenPaused = false;
    }

    ~UTickEvent()
    {
        UE_LOG(LogTemp, Warning, TEXT("~UTickEvent is destroyed..."));   
    }

    void initialize(const ETickingGroup& tick_group)
    {
        PrimaryComponentTick.TickGroup = tick_group;
        RegisterComponent();
    }

    void TickComponent(float delta_time, enum ELevelTick tick_type, FActorComponentTickFunction *this_tick_function) override
    {
        delegate_.Broadcast(delta_time, tick_type, this_tick_function);
    }

    OnTickEvent delegate_;
};
