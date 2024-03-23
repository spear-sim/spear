//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include "SpCore/UserInputComponent.h"

#include <functional> // std::function
#include <string>
#include <vector>

#include <Components/InputComponent.h>
#include <Components/SceneComponent.h>
#include <Engine/EngineBaseTypes.h> // ELevelTick, ETickingGroup
#include <GameFramework/PlayerController.h>
#include <GameFramework/PlayerInput.h>

#include "SpCore/Log.h"
#include "SpCore/Unreal.h"

UUserInputComponent::UUserInputComponent()
{
    SP_LOG_CURRENT_FUNCTION();

    PrimaryComponentTick.TickGroup = ETickingGroup::TG_PrePhysics;
    PrimaryComponentTick.bCanEverTick = true;
    PrimaryComponentTick.bTickEvenWhenPaused = false;
}

UUserInputComponent::~UUserInputComponent()
{
    SP_LOG_CURRENT_FUNCTION();

    handle_user_input_func_ = nullptr;
    user_input_descs_.clear();
    input_component_ = nullptr;
}

void UUserInputComponent::TickComponent(float delta_time, ELevelTick level_tick, FActorComponentTickFunction* this_tick_function)
{
    USceneComponent::TickComponent(delta_time, level_tick, this_tick_function);

    if (bHandleUserInput && input_component_ && handle_user_input_func_) {
        for (auto& user_input_desc : user_input_descs_) {
            float axis_value = input_component_->GetAxisValue(Unreal::toFName(user_input_desc.axis_));
            if (axis_value >= user_input_desc.threshold_) {
                handle_user_input_func_(user_input_desc.key_, axis_value);
            }
        }
    }
}

void UUserInputComponent::subscribeToUserInputs(const std::vector<std::string>& user_inputs)
{
    SP_ASSERT(GetWorld());
    SP_ASSERT(GetWorld()->GetFirstPlayerController());
    SP_ASSERT(GetOwner());

    input_component_ = GetWorld()->GetFirstPlayerController()->InputComponent;
    SP_ASSERT(input_component_);

    UPlayerInput* player_input = GetWorld()->GetFirstPlayerController()->PlayerInput;
    SP_ASSERT(player_input);

    for (auto& user_input : user_inputs) {
        UserInputDesc user_input_desc;

        // The only requirement when setting axis_ is that it is a globally unique string. We do not need to use the
        // actor's stable name specifically. So we avoid using the actor's stable name here, because this will enable
        // the use of UUserInputComponents on actors that don't have a UStableNameComponent.
        user_input_desc.key_ = user_input;
        user_input_desc.axis_ = Unreal::toStdString(GetOwner()->GetFullName()) + ":" + Unreal::getStableComponentName(this) + ":axis:" + user_input;
        user_input_desc.threshold_ = 1.0f;
        user_input_desc.scale_ = 1.0f;

        player_input->AddAxisMapping(FInputAxisKeyMapping(Unreal::toFName(user_input_desc.axis_), FKey(Unreal::toFName(user_input)), user_input_desc.scale_));
        input_component_->BindAxis(Unreal::toFName(user_input_desc.axis_));

        user_input_descs_.push_back(user_input_desc);
    }
}

void UUserInputComponent::setHandleUserInputFunc(const std::function<void(const std::string&, float)>& handle_user_input_func)
{
    handle_user_input_func_ = handle_user_input_func;
}
