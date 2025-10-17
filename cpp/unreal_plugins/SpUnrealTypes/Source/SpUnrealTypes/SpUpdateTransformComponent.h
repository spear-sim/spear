//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Components/ActorComponent.h>
#include <Components/SceneComponent.h>
#include <Containers/UnrealString.h> // FString
#include <Engine/EngineBaseTypes.h>  // ELevelTick, ETickingGroup
#include <Engine/EngineTypes.h>      // ETeleportType
#include <Engine/HitResult.h>

#include "SpCore/Log.h"
#include "SpCore/Unreal.h"

#include "SpUpdateTransformComponent.generated.h"

struct FActorComponentTickFunction;

// We need meta=(BlueprintSpawnableComponent) for the component to show up when using the "+Add" button in the editor.
UCLASS(ClassGroup="SPEAR", HideCategories=(Activation, AssetUserData, Collision, Cooking, LOD, Navigation, Physics, Rendering, Tags), meta=(BlueprintSpawnableComponent))
class USpUpdateTransformComponent : public UActorComponent
{
    GENERATED_BODY()
public:
    USpUpdateTransformComponent()
    {
        SP_LOG_CURRENT_FUNCTION();

        PrimaryComponentTick.bCanEverTick = true;
        PrimaryComponentTick.bTickEvenWhenPaused = false;
        PrimaryComponentTick.TickGroup = ETickingGroup::TG_PostPhysics;
    }

    ~USpUpdateTransformComponent() override
    {
        SP_LOG_CURRENT_FUNCTION();
    }

    // UActorComponent interface
    void BeginPlay() override
    {
        SP_LOG_CURRENT_FUNCTION();

        UActorComponent::BeginPlay();

        if (!SourceComponentPath.IsEmpty()) {
            auto [_, src_component] = Unreal::findActorAndComponentByPath<AActor, USceneComponent>(GetWorld(), GetOwner(), Unreal::toStdString(SourceComponentPath));
            src_component_ = src_component;
            SourceComponent = Unreal::toFString(Unreal::getStableName(src_component_));
        }

        if (!DestinationComponentPath.IsEmpty()) {
            auto [_, dest_component] = Unreal::findActorAndComponentByPath<AActor, USceneComponent>(GetWorld(), GetOwner(), Unreal::toStdString(DestinationComponentPath));
            dest_component_ = dest_component;
            DestinationComponent = Unreal::toFString(Unreal::getStableName(dest_component_));
        }
    }

    void TickComponent(float delta_time, ELevelTick level_tick, FActorComponentTickFunction* this_tick_function) override
    {
        UActorComponent::TickComponent(delta_time, level_tick, this_tick_function);

        SetWorldLocationHitResult = FHitResult();
        SetWorldRotationHitResult = FHitResult();

        if (!src_component_ || !dest_component_) {
            return;
        }

        if (bSetWorldLocation) {
            dest_component_->SetWorldLocation(src_component_->K2_GetComponentLocation(), bSetWorldLocationSweep, &SetWorldLocationHitResult, SetWorldLocationTeleport);
        }

        if (bSetWorldRotation) {
            dest_component_->SetWorldRotation(src_component_->K2_GetComponentRotation(), bSetWorldRotationSweep, &SetWorldRotationHitResult, SetWorldRotationTeleport);
        }

        if (bSetWorldScale3D) {
            dest_component_->SetWorldScale3D(src_component_->K2_GetComponentScale());
        }
    }

    // Specifies a component (whose path is SourceComponentPath) to use as the source componennt.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    FString SourceComponentPath;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="SPEAR")
    FString SourceComponent;

    // Specifies a component (whose path is DestinationComponentPath) to use as the destination componennt.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    FString DestinationComponentPath;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="SPEAR")
    FString DestinationComponent;

    // Set location of the source component using the location of the destination component.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    bool bSetWorldLocation = false;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    bool bSetWorldLocationSweep = false;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    FHitResult SetWorldLocationHitResult;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    ETeleportType SetWorldLocationTeleport = ETeleportType::None;

    // Set rotation of the source component using the rotation of the destination component.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    bool bSetWorldRotation = false;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    bool bSetWorldRotationSweep = false;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    FHitResult SetWorldRotationHitResult;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    ETeleportType SetWorldRotationTeleport = ETeleportType::None;

    // Set scale of the source component using the scale of the destination component.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    bool bSetWorldScale3D = false;

private:
    USceneComponent* src_component_ = nullptr;
    USceneComponent* dest_component_ = nullptr;
};
