//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Components/ActorComponent.h>
#include <Containers/UnrealString.h> // FString
#include <Engine/EngineBaseTypes.h>  // ELevelTick
#include <Engine/EngineTypes.h>      // ETeleportType
#include <Engine/HitResult.h>
#include <HAL/Platform.h>            // SPCOMPONENTS_API

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

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    FString SourceComponentPath;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="SPEAR")
    FString SourceComponent;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    FString DestinationComponentPath;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="SPEAR")
    FString DestinationComponent;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    bool bSetWorldLocation = false;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    bool bSetWorldLocationSweep = false;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    FHitResult SetWorldLocationHitResult;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    ETeleportType SetWorldLocationTeleport = ETeleportType::None;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    bool bSetWorldRotation = false;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    bool bSetWorldRotationSweep = false;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    FHitResult SetWorldRotationHitResult;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    ETeleportType SetWorldRotationTeleport = ETeleportType::None;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    bool bSetWorldScale3D = false;

private:
    USceneComponent* src_component_ = nullptr;
    USceneComponent* dest_component_ = nullptr;
};