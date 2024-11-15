//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpComponents/SpDebugActionComponent.h"

#include <string>
#include <vector>

#include <Components/SceneComponent.h>
#include <Engine/EngineBaseTypes.h> // ELevelTick
#include <HAL/Platform.h>           // uint64
#include <Math/Rotator.h>
#include <Math/Vector.h>

#include "SpCore/Std.h"
#include "SpCore/Log.h"
#include "SpCore/Unreal.h"

#include "SpComponents/SpUserInputComponent.h"

struct FActorComponentTickFunction;

USpDebugActionComponent::USpDebugActionComponent()
{
    SP_LOG_CURRENT_FUNCTION();

    PrimaryComponentTick.bCanEverTick = true;
    PrimaryComponentTick.bTickEvenWhenPaused = false;
    PrimaryComponentTick.TickGroup = ETickingGroup::TG_PrePhysics;

    SpUserInputComponent = Unreal::createComponentInsideOwnerConstructor<USpUserInputComponent>(this, "sp_user_input_component");
    SP_ASSERT(SpUserInputComponent);
}

USpDebugActionComponent::~USpDebugActionComponent()
{
    SP_LOG_CURRENT_FUNCTION();
}

void USpDebugActionComponent::BeginPlay()
{
    SP_LOG_CURRENT_FUNCTION();

    UActorComponent::BeginPlay();

    if (!AddRotationComponentPath.IsEmpty()) {
        auto [_, add_rotation_component] = Unreal::findActorAndComponentByPath<AActor, USceneComponent>(GetWorld(), GetOwner(), Unreal::toStdString(AddRotationComponentPath));
        add_rotation_component_ = add_rotation_component;
        AddRotationComponent = Unreal::toFString(Unreal::getStableName(add_rotation_component_));
    }

    if (!AddForceTargetComponentPath.IsEmpty()) {
        auto [_, add_force_target_component] = Unreal::findActorAndComponentByPath<AActor, UPrimitiveComponent>(GetWorld(), GetOwner(), Unreal::toStdString(AddForceTargetComponentPath));
        add_force_target_component_ = add_force_target_component;
        AddForceTargetComponent = Unreal::toFString(Unreal::getStableName(add_force_target_component_));
    }

    if (!AddForceRotationComponentPath.IsEmpty()) {
        auto [_, add_force_rotation_component] = Unreal::findActorAndComponentByPath<AActor, USceneComponent>(GetWorld(), GetOwner(), Unreal::toStdString(AddForceRotationComponentPath));
        add_force_rotation_component_ = add_force_rotation_component;
        AddForceRotationComponent = Unreal::toFString(Unreal::getStableName(add_force_rotation_component_));
    }

    SpUserInputComponent->subscribeToUserInputs({"One", "Two", "Three", "Four"});
    SpUserInputComponent->setHandleUserInputFunc([this](const std::string& key, float axis_value) -> void {

        if (key == "One" && add_rotation_component_) {
            add_rotation_component_->AddRelativeRotation(FRotator(0.0, -2.0, 0.0));
        }

        if (key == "Two" && add_rotation_component_) {
            add_rotation_component_->AddRelativeRotation(FRotator(0.0, 2.0, 0.0));
        }

        if (key == "Three" && add_force_target_component_) {
            FVector force = FVector(-1000.0, 0.0, 0.0);
            if (add_force_rotation_component_) {
                force = add_force_rotation_component_->K2_GetComponentRotation().RotateVector(force);
            }
            add_force_target_component_->AddForce(force);
        }

        if (key == "Four" && add_force_target_component_) {
            FVector force = FVector(1000.0, 0.0, 0.0);
            if (add_force_rotation_component_) {
                force = add_force_rotation_component_->K2_GetComponentRotation().RotateVector(force);
            }
            add_force_target_component_->AddForce(force);
        }
    });
}
