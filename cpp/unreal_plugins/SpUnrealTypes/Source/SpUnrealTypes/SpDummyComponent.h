//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <string>

#include <Components/SceneComponent.h>
#include <Engine/EngineBaseTypes.h>  // ELevelTick
#include <Engine/EngineTypes.h>      // EEndPlayReason, ETickingGroup
#include <UObject/ObjectMacros.h>    // GENERATED_BODY, UCLASS, UFUNCTION, UPROPERTY

#include "SpCore/Log.h"
#include "SpCore/SpFuncComponent.h"
#include "SpCore/Unreal.h"
#include "SpCore/UnrealUtils.h"

#include "SpUnrealTypes/SpUserInputComponent.h"

#include "SpDummyComponent.generated.h"

struct FActorComponentTickFunction;

//
// The purpose of USpDummyComponent is to provide a minimal component with all of the boilerplate required to
// implement custom debug keyboard commands and SpFuncs. If you want to create a new component for use with
// the SPEAR Python API, USpDummyComponent is a good starting point.
//

// We need meta=(BlueprintSpawnableComponent) for the component to show up when using the "+Add" button in the editor.
UCLASS(ClassGroup="SPEAR", HideCategories=(Activation, AssetUserData, Collision, Cooking, LOD, Navigation, Physics, Rendering, Tags), meta=(BlueprintSpawnableComponent))
class USpDummyComponent : public USceneComponent
{
    GENERATED_BODY()
public:
    USpDummyComponent()
    {
        SP_LOG_CURRENT_FUNCTION();

        PrimaryComponentTick.bCanEverTick = true;
        PrimaryComponentTick.bTickEvenWhenPaused = false;
        PrimaryComponentTick.TickGroup = ETickingGroup::TG_PrePhysics;

        SpFuncComponent = UnrealUtils::createSceneComponentInsideOwnerConstructor<USpFuncComponent>(this, "sp_func_component");
        SP_ASSERT(SpFuncComponent);

        SpUserInputComponent = UnrealUtils::createSceneComponentInsideOwnerConstructor<USpUserInputComponent>(this, "sp_user_input_component");
        SP_ASSERT(SpUserInputComponent);
    }

    ~USpDummyComponent() override
    {
        SP_LOG_CURRENT_FUNCTION();
    }

    // UActorComponent interface
    void BeginPlay() override
    {
        SP_LOG_CURRENT_FUNCTION();

        USceneComponent::BeginPlay();

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

    void EndPlay(const EEndPlayReason::Type end_play_reason) override
    {
        SP_LOG_CURRENT_FUNCTION();

        SpFuncComponent->unregisterFunc("hello_world");

        SpUserInputComponent->setHandleUserInputFunc(nullptr);
        SpUserInputComponent->unsubscribeFromUserInputs({"One"});

        USceneComponent::EndPlay(end_play_reason);
    }

    void TickComponent(float delta_time, ELevelTick level_tick, FActorComponentTickFunction* this_tick_function) override
    {
        SP_LOG_CURRENT_FUNCTION();

        USceneComponent::TickComponent(delta_time, level_tick, this_tick_function);        
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    void HelloWorld() const
    {
        SP_LOG_CURRENT_FUNCTION();
    }

    // Required for custom debug keyboard commands
    UPROPERTY(VisibleAnywhere, Category="SPEAR")
    USpUserInputComponent* SpUserInputComponent = nullptr;

    // Required for custom SpFuncs
    UPROPERTY(VisibleAnywhere, Category="SPEAR")
    USpFuncComponent* SpFuncComponent = nullptr;
};
