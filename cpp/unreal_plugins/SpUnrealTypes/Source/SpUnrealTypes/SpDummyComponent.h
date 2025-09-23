//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Components/SceneComponent.h>
#include <Engine/EngineBaseTypes.h>  // ELevelTick
#include <Engine/EngineTypes.h>      // EEndPlayReason
#include <HAL/Platform.h>            // SPUNREALTYPES_API

#include "SpCore/SpFuncComponent.h"

#include "SpUnrealTypes/SpUserInputComponent.h"

#include "SpDummyComponent.generated.h"

struct FActorComponentTickFunction;

//
// The purpose of USpDummyComponent is to provide a minimal component with all of the boilerplate required to
// implement custom debug keyboard commands and SpFuncs. If you want to create a new component for use with
// the SPEAR Python API, USpDummyComponent is a good starting point.
//

// We need meta=(BlueprintSpawnableComponent) for the component to show up when using the "+Add" button in the editor.
UCLASS(ClassGroup="SPEAR", HideCategories=(Rendering, Tags, Activation, Cooking, Physics, LOD, AssetUserData, Collision), meta=(BlueprintSpawnableComponent))
class SPUNREALTYPES_API USpDummyComponent : public USceneComponent
{
    GENERATED_BODY()
public:
    USpDummyComponent();
    ~USpDummyComponent() override;

    // UActorComponent interface
    void BeginPlay() override;
    void EndPlay(const EEndPlayReason::Type end_play_reason) override;
    void TickComponent(float delta_time, ELevelTick level_tick, FActorComponentTickFunction* this_tick_function) override;

    UFUNCTION()
    void HelloWorld();

    // Required for custom debug keyboard commands
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="SPEAR")
    USpUserInputComponent* SpUserInputComponent = nullptr;

    // Required for custom SpFuncs
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="SPEAR")
    USpFuncComponent* SpFuncComponent = nullptr;
};
