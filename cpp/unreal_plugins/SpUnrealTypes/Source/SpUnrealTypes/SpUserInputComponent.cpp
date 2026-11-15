//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpUnrealTypes/SpUserInputComponent.h"

#include <functional> // std::function
#include <string>
#include <vector>

#include <Components/InputComponent.h>
#include <Components/SceneComponent.h>
#include <Engine/EngineBaseTypes.h> // ELevelTick, ETickingGroup
#include <Engine/World.h>
#include <GameFramework/PlayerController.h>
#include <GameFramework/PlayerInput.h>

#include "SpCore/Assert.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"
#include "SpCore/UnrealUtils.h"

USpUserInputComponent::USpUserInputComponent()
{
    PrimaryComponentTick.bCanEverTick = true;
    PrimaryComponentTick.bTickEvenWhenPaused = false;
    PrimaryComponentTick.TickGroup = ETickingGroup::TG_PrePhysics;
}

void USpUserInputComponent::TickComponent(float delta_time, ELevelTick level_tick, FActorComponentTickFunction* this_tick_function)
{
    USceneComponent::TickComponent(delta_time, level_tick, this_tick_function);

    if (!bHandleUserInput || !handle_user_input_func_) {
        return;
    }

    UWorld* world = GetWorld();
    if (!world) {
        return;
    }

    APlayerController* player_controller = world->GetFirstPlayerController();
    if (!player_controller || !player_controller->InputComponent) {
        return;
    }

    for (auto& [user_input_name, user_input_desc] : user_input_descs_) {
        float axis_value = player_controller->InputComponent->GetAxisValue(user_input_desc.axis_);
        if (axis_value >= user_input_desc.threshold_) {
            handle_user_input_func_(user_input_name, axis_value);
        }
    }
}

void USpUserInputComponent::subscribeToUserInputs(const std::vector<std::string>& user_input_names)
{
    SP_ASSERT(GetWorld());
    APlayerController* player_controller = GetWorld()->GetFirstPlayerController();
    SP_ASSERT(player_controller);
    SP_ASSERT(player_controller->InputComponent);
    SP_ASSERT(player_controller->PlayerInput);

    for (auto& user_input_name : user_input_names) {
        UserInputDesc user_input_desc;

        user_input_desc.scale_ = 1.0f;
        user_input_desc.threshold_ = 1.0f;
        user_input_desc.axis_ = Unreal::toFName(getUniqueAxisNameFromUserInputName(user_input_name));
        user_input_desc.input_axis_key_mapping_ = FInputAxisKeyMapping(user_input_desc.axis_, FKey(Unreal::toFName(user_input_name)), user_input_desc.scale_);

        player_controller->PlayerInput->AddAxisMapping(user_input_desc.input_axis_key_mapping_);
        player_controller->InputComponent->BindAxis(user_input_desc.axis_);

        Std::insert(user_input_descs_, user_input_name, user_input_desc);
        SubscribedToUserInputs.Add(Unreal::toFString(user_input_name));
    }
}

void USpUserInputComponent::unsubscribeFromUserInputs(const std::vector<std::string>& user_input_names)
{
    // The player controller and its input objects might already be gone (e.g., if the player controller was
    // explicitly destroyed), so we only remove axis bindings and mappings from objects that are still valid.
    UWorld* world = GetWorld();
    APlayerController* player_controller = nullptr;
    if (world) {
        player_controller = world->GetFirstPlayerController();
    }

    UInputComponent* input_component = nullptr;
    UPlayerInput* player_input = nullptr;
    if (player_controller) {
        input_component = player_controller->InputComponent;
        player_input = player_controller->PlayerInput;
    }

    for (auto& user_input_name : user_input_names) {
        SP_ASSERT(Std::containsKey(user_input_descs_, user_input_name));
        const UserInputDesc& user_input_desc = user_input_descs_.at(user_input_name);

        if (input_component) {
            input_component->RemoveAxisBinding(user_input_desc.axis_);
        }
        if (player_input) {
            player_input->RemoveAxisMapping(user_input_desc.input_axis_key_mapping_);
        }

        Std::remove(user_input_descs_, user_input_name);
        SubscribedToUserInputs.Remove(Unreal::toFString(user_input_name));
    }
}

void USpUserInputComponent::setHandleUserInputFunc(const std::function<void(const std::string&, float)>& handle_user_input_func)
{
    handle_user_input_func_ = handle_user_input_func;
}

std::string USpUserInputComponent::getUniqueAxisNameFromUserInputName(const std::string& user_input_name) const
{
    bool include_unreal_name = true;
    return UnrealUtils::getStableName(this, include_unreal_name) + ":" + user_input_name;
}
