//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpComponents/SpUpdateTransformComponent.h"

#include <string>
#include <vector>

#include <Components/ActorComponent.h>
#include <Components/SceneComponent.h>
#include <Engine/EngineBaseTypes.h> // ELevelTick, ETickingGroup
#include <Engine/HitResult.h>

#include "SpCore/Std.h"
#include "SpCore/Log.h"
#include "SpCore/Unreal.h"

struct FActorComponentTickFunction;

USpUpdateTransformComponent::USpUpdateTransformComponent()
{
    SP_LOG_CURRENT_FUNCTION();

    PrimaryComponentTick.bCanEverTick = true;
    PrimaryComponentTick.bTickEvenWhenPaused = false;
    PrimaryComponentTick.TickGroup = ETickingGroup::TG_PostPhysics;
}

USpUpdateTransformComponent::~USpUpdateTransformComponent()
{
    SP_LOG_CURRENT_FUNCTION();
}

void USpUpdateTransformComponent::BeginPlay()
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

void USpUpdateTransformComponent::TickComponent(float delta_time, ELevelTick level_tick, FActorComponentTickFunction* this_tick_function)
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
