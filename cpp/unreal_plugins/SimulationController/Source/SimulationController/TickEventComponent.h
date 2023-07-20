//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <CoreMinimal.h>
#include <Components/ActorComponent.h>
#include <Engine/EngineBaseTypes.h>

#include "CoreUtils/Log.h"

#include "TickEventComponent.generated.h"

class FObjectInitializer;
struct FActorComponentTickFunction;

DECLARE_MULTICAST_DELEGATE_TwoParams(FTickDelegate, float, enum ELevelTick);

UCLASS()
class UTickEventComponent : public UActorComponent
{
    GENERATED_BODY()
public:
    UTickEventComponent(const FObjectInitializer& object_initializer) : UActorComponent(object_initializer)
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
    void TickComponent(float delta_time, enum ELevelTick level_tick, FActorComponentTickFunction* this_tick_function) override
    {
        delegate_.Broadcast(delta_time, level_tick);
    }

    FTickDelegate delegate_;
};
