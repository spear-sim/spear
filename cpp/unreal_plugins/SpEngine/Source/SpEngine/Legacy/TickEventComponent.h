//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <functional> // std::function

#include <Components/ActorComponent.h>
#include <Engine/EngineBaseTypes.h> // ELevelTick
#include <UObject/ObjectMacros.h>   // GENERATED_BODY, UCLASS

#include "SpCore/Log.h"

#include "TickEventComponent.generated.h"

struct FActorComponentTickFunction;

UCLASS()
class UTickEventComponent : public UActorComponent
{
    GENERATED_BODY()
public:
    UTickEventComponent()
    {
        SP_LOG_CURRENT_FUNCTION();

        PrimaryComponentTick.bCanEverTick = true;
        PrimaryComponentTick.bTickEvenWhenPaused = false;
    }

    ~UTickEventComponent()
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

    std::function<void(float, ELevelTick, FActorComponentTickFunction*)> tick_func_;
};
