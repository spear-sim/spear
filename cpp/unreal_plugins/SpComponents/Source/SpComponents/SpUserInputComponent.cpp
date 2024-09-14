//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpComponents/SpUserInputComponent.h"

#include <functional> // std::function
#include <string>
#include <vector>

#include <Components/InputComponent.h>
#include <Components/SceneComponent.h>
#include <Engine/EngineBaseTypes.h> // ELevelTick, ETickingGroup
#include <GameFramework/PlayerController.h>
#include <GameFramework/PlayerInput.h>

#include "SpCore/Assert.h"
#include "SpCore/Log.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"

USpUserInputComponent::USpUserInputComponent()
{
    SP_LOG_CURRENT_FUNCTION();

    PrimaryComponentTick.bCanEverTick = true;
    PrimaryComponentTick.bTickEvenWhenPaused = false;
    PrimaryComponentTick.TickGroup = ETickingGroup::TG_PrePhysics;
}

USpUserInputComponent::~USpUserInputComponent()
{
    SP_LOG_CURRENT_FUNCTION();
}

void USpUserInputComponent::TickComponent(float delta_time, ELevelTick level_tick, FActorComponentTickFunction* this_tick_function)
{
    USceneComponent::TickComponent(delta_time, level_tick, this_tick_function);

    if (bHandleUserInput && input_component_ && handle_user_input_func_) {
        for (auto& [user_input_name, user_input_desc] : user_input_descs_) {
            float axis_value = input_component_->GetAxisValue(user_input_desc.axis_);
            if (axis_value >= user_input_desc.threshold_) {
                handle_user_input_func_(user_input_name, axis_value);
            }
        }
    }
}

void USpUserInputComponent::subscribeToUserInputs(const std::vector<std::string>& user_input_names)
{
    SP_ASSERT(GetWorld());
    SP_ASSERT(GetWorld()->GetFirstPlayerController());
    SP_ASSERT(GetOwner());

    input_component_ = GetWorld()->GetFirstPlayerController()->InputComponent;
    SP_ASSERT(input_component_);

    player_input_ = GetWorld()->GetFirstPlayerController()->PlayerInput;
    SP_ASSERT(player_input_);

    for (auto& user_input_name : user_input_names) {
        UserInputDesc user_input_desc;

        user_input_desc.scale_ = 1.0f;
        user_input_desc.threshold_ = 1.0f;
        user_input_desc.axis_ = getUniqueAxisNameFromUserInputName(user_input_name);
        user_input_desc.input_axis_key_mapping_ = FInputAxisKeyMapping(user_input_desc.axis_, FKey(Unreal::toFName(user_input_name)), user_input_desc.scale_);

        player_input_->AddAxisMapping(user_input_desc.input_axis_key_mapping_);
        input_component_->BindAxis(user_input_desc.axis_);

        Std::insert(user_input_descs_, user_input_name, user_input_desc);
    }
}

void USpUserInputComponent::unsubscribeFromUserInputs(const std::vector<std::string>& user_input_names)
{
    SP_ASSERT(input_component_);
    SP_ASSERT(player_input_);

    for (auto& user_input_name : user_input_names) {
        SP_ASSERT(Std::containsKey(user_input_descs_, user_input_name));
        UserInputDesc user_input_desc = user_input_descs_.at(user_input_name);

        input_component_->RemoveAxisBinding(user_input_desc.axis_);
        player_input_->RemoveAxisMapping(user_input_desc.input_axis_key_mapping_);

        Std::remove(user_input_descs_, user_input_name);
    }
}

void USpUserInputComponent::setHandleUserInputFunc(const std::function<void(const std::string&, float)>& handle_user_input_func)
{
    handle_user_input_func_ = handle_user_input_func;
}

FName USpUserInputComponent::getUniqueAxisNameFromUserInputName(const std::string& user_input_name) const
{
    // The only requirement when setting creating an axis name is that it is a globally unique string. We do not
    // need to use the actor's stable name specifically. So we avoid using the actor's stable name here, because
    // this will enable the use of USpUserInputComponent on actors that don't have an USpStableNameComponent.
    return Unreal::toFName(Unreal::toStdString(GetOwner()->GetFullName()) + ":" + Unreal::getStableName(this) + ":axis:" + user_input_name);
}
