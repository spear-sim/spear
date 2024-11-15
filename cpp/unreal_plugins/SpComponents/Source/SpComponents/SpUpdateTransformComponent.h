//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Components/ActorComponent.h>
#include <Containers/UnrealString.h> // FString
#include <Engine/EngineBaseTypes.h>  // ELevelTick
#include <Engine/EngineTypes.h>      // ETeleportType

#include "SpUpdateTransformComponent.generated.h"

class USceneComponent;
struct FActorComponentTickFunction;

// We need meta=(BlueprintSpawnableComponent) for the component to show up when using the "+Add" button in the editor.
UCLASS(ClassGroup="SPEAR", HideCategories=(Rendering, Tags, Activation, Cooking, Physics, LOD, AssetUserData, Collision), meta=(BlueprintSpawnableComponent))
class SPCOMPONENTS_API USpUpdateTransformComponent : public UActorComponent
{
    GENERATED_BODY()
public:
    USpUpdateTransformComponent();
    ~USpUpdateTransformComponent();

    // UActorComponent interface
    void BeginPlay() override;
    void TickComponent(float delta_time, ELevelTick level_tick, FActorComponentTickFunction* this_tick_function) override;

private:
    UPROPERTY(EditAnywhere, Category="SPEAR")
    FString SourceComponentPath;
    UPROPERTY(VisibleAnywhere, Category="SPEAR")
    FString SourceComponent;

    UPROPERTY(EditAnywhere, Category="SPEAR")
    FString DestinationComponentPath;
    UPROPERTY(VisibleAnywhere, Category="SPEAR")
    FString DestinationComponent;

    UPROPERTY(EditAnywhere, Category="SPEAR")
    bool bSetWorldLocation = false;
    UPROPERTY(EditAnywhere, Category="SPEAR")
    bool bSetWorldLocationSweep = false;
    UPROPERTY(EditAnywhere, Category="SPEAR")
    FHitResult SetWorldLocationHitResult;
    UPROPERTY(EditAnywhere, Category="SPEAR")
    ETeleportType SetWorldLocationTeleport = ETeleportType::None;

    UPROPERTY(EditAnywhere, Category="SPEAR")
    bool bSetWorldRotation = false;
    UPROPERTY(EditAnywhere, Category="SPEAR")
    bool bSetWorldRotationSweep = false;
    UPROPERTY(EditAnywhere, Category="SPEAR")
    FHitResult SetWorldRotationHitResult;
    UPROPERTY(EditAnywhere, Category="SPEAR")
    ETeleportType SetWorldRotationTeleport = ETeleportType::None;

    UPROPERTY(EditAnywhere, Category="SPEAR")
    bool bSetWorldScale3D = false;

    USceneComponent* src_component_ = nullptr;
    USceneComponent* dest_component_ = nullptr;
};
