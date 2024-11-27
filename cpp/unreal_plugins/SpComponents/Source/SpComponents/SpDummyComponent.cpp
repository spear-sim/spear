//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpComponents/SpDummyComponent.h"

#include <string>

#include <Components/SceneComponent.h>
#include <Engine/EngineBaseTypes.h> // ELevelTick
#include <Engine/EngineTypes.h>     // EEndPlayReason, ETickingGroup

#include "SpCore/Log.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"

#include "SpComponents/SpFuncComponent.h"
#include "SpComponents/SpUserInputComponent.h"

struct FActorComponentTickFunction;

USpDummyComponent::USpDummyComponent()
{
    SP_LOG_CURRENT_FUNCTION();

    PrimaryComponentTick.bCanEverTick = true;
    PrimaryComponentTick.bTickEvenWhenPaused = false;
    PrimaryComponentTick.TickGroup = ETickingGroup::TG_PrePhysics;

    SpFuncComponent = Unreal::createComponentInsideOwnerConstructor<USpFuncComponent>(this, "sp_func_component");
    SP_ASSERT(SpFuncComponent);

    SpUserInputComponent = Unreal::createComponentInsideOwnerConstructor<USpUserInputComponent>(this, "sp_user_input_component");
    SP_ASSERT(SpUserInputComponent);
}

USpDummyComponent::~USpDummyComponent()
{
    SP_LOG_CURRENT_FUNCTION();
}

void USpDummyComponent::BeginPlay()
{
    SP_LOG_CURRENT_FUNCTION();

    UActorComponent::BeginPlay();

    SpUserInputComponent->subscribeToUserInputs({"One"});
    SpUserInputComponent->setHandleUserInputFunc([this](const std::string& key, float axis_value) -> void {
        SP_LOG(key, axis_value);
    });

    SpFuncComponent->registerFunc("hello_world", [this](SpFuncDataBundle& args) -> SpFuncDataBundle {
        SP_LOG_CURRENT_FUNCTION();
        SpFuncDataBundle return_values;
        return return_values;
    });
}

void USpDummyComponent::EndPlay(const EEndPlayReason::Type end_play_reason)
{
    SP_LOG_CURRENT_FUNCTION();

    SpFuncComponent->unregisterFunc("hello_world");

    SpUserInputComponent->setHandleUserInputFunc(nullptr);
    SpUserInputComponent->unsubscribeFromUserInputs({"One"});

    UActorComponent::EndPlay(end_play_reason);
}

void USpDummyComponent::TickComponent(float delta_time, ELevelTick level_tick, FActorComponentTickFunction* this_tick_function)
{
    SP_LOG_CURRENT_FUNCTION();

    USceneComponent::TickComponent(delta_time, level_tick, this_tick_function);
}
