//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <functional> // std::function

#include <Components/ActorComponent.h>
#include <Engine/EngineBaseTypes.h> // ELevelTick
#include <UObject/ObjectMacros.h>   // GENERATED_BODY, UCLASS

#include "SpCore/Log.h"

#include "TickComponent.generated.h"

struct FActorComponentTickFunction;

UCLASS()
class UTickComponent : public UActorComponent
{
    GENERATED_BODY()
public:
    UTickComponent()
    {
        SP_LOG_CURRENT_FUNCTION();

        PrimaryComponentTick.bCanEverTick = true;
        PrimaryComponentTick.bTickEvenWhenPaused = false;
        PrimaryComponentTick.TickGroup = ETickingGroup::TG_PrePhysics;
    }

    ~UTickComponent()
    {
        SP_LOG_CURRENT_FUNCTION();
    }

    // UActorComponent interface
    void TickComponent(float delta_time, ELevelTick level_tick, FActorComponentTickFunction* this_tick_function) override
    {
        UActorComponent::TickComponent(delta_time, level_tick, this_tick_function);

        if (tick_func_) {
            tick_func_(delta_time, level_tick, this_tick_function);
        }
    }

    void setTickFunc(const std::function<void(float, ELevelTick, FActorComponentTickFunction*)>& tick_func)
    {
        tick_func_ = tick_func_;
    }

private:
    std::function<void(float, ELevelTick, FActorComponentTickFunction*)> tick_func_;
};
