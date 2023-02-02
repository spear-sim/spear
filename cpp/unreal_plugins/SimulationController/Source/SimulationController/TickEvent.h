//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <iostream>

#include <CoreMinimal.h>
#include <Components/ActorComponent.h>

#include "TickEvent.generated.h"

DECLARE_MULTICAST_DELEGATE_TwoParams(OnTickEvent, float, enum ELevelTick);

UCLASS()
class UTickEvent : public UActorComponent
{
    GENERATED_BODY()
public:
    UTickEvent(const FObjectInitializer& object_initializer) : UActorComponent(object_initializer)
    {
        std::cout << "[SPEAR | TickEvent.h] UTickEvent::UTickEvent" << std::endl;

        PrimaryComponentTick.bCanEverTick = true;
        PrimaryComponentTick.bTickEvenWhenPaused = false;
    }

    ~UTickEvent()
    {
        std::cout << "[SPEAR | TickEvent.h] UTickEvent::~UTickEvent" << std::endl;
    }
    
    // UActorComponent interface
    void TickComponent(float delta_time, enum ELevelTick level_tick, FActorComponentTickFunction* this_tick_function) override
    {
        delegate_.Broadcast(delta_time, level_tick);
    }

    OnTickEvent delegate_;
};
