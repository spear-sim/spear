//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <functional>
#include <map>
#include <string>
#include <vector>

#include <Components/InputComponent.h>
#include <Engine/EngineBaseTypes.h> // ELevelTick
#include <GameFramework/PlayerController.h>
#include <GameFramework/PlayerInput.h>
#include <UObject/ObjectMacros.h>   // GENERATED_BODY, UCLASS

#include "CoreUtils/Log.h"
#include "CoreUtils/Unreal.h"

#include "PlayerInputComponent.generated.h"

struct FActorComponentTickFunction;

struct PlayerInputActionDesc
{
    std::string key_;
    std::string axis_;
    float value_ = 1.0f;
    float threshold_ = 1.0f;
};

UCLASS()
class COREUTILS_API UPlayerInputComponent : public UActorComponent
{
    GENERATED_BODY()
public:
    UPlayerInputComponent()
    {
        SP_LOG_CURRENT_FUNCTION();

        PrimaryComponentTick.bCanEverTick = true;
        PrimaryComponentTick.bTickEvenWhenPaused = false;
    }

    virtual ~UPlayerInputComponent()
    {
        SP_LOG_CURRENT_FUNCTION();

        input_component_ = nullptr;
        player_input_action_descs_.clear();
    }

    // UActorComponent interface
    void TickComponent(float delta_time, ELevelTick level_tick, FActorComponentTickFunction* this_tick_function) override
    {
        UActorComponent::TickComponent(delta_time, level_tick, this_tick_function);

        if (input_component_ && apply_action_func_) {
            for (auto& player_input_action_desc : player_input_action_descs_) {
                float axis_value = input_component_->GetAxisValue(Unreal::toFName(player_input_action_desc.axis_));
                if (axis_value >= player_input_action_desc.threshold_) {
                    apply_action_func_(player_input_action_desc, axis_value);
                }   
            }
        }
    }

    template <typename T>
    void setPlayerInputActions(const std::map<std::string, T>& player_input_actions)
    {
        for (auto& player_input_action : player_input_actions) {
            PlayerInputActionDesc player_input_action_desc;
            player_input_action_desc.key_ = player_input_action.first;
            player_input_action_descs_.push_back(player_input_action_desc);
        }
    }

    void addAxisMappingsAndBindAxes()
    {
        SP_ASSERT(GetWorld());
        SP_ASSERT(GetWorld()->GetFirstPlayerController());
        SP_ASSERT(GetOwner());

        UPlayerInput* player_input = GetWorld()->GetFirstPlayerController()->PlayerInput;
        SP_ASSERT(player_input);

        for (auto& player_input_action_desc : player_input_action_descs_) {
            player_input_action_desc.axis_ =
                Unreal::toStdString(GetOwner()->GetName()) + "." + Unreal::toStdString(GetName()) + "." + player_input_action_desc.key_;
            player_input->AddAxisMapping(FInputAxisKeyMapping(
                Unreal::toFName(player_input_action_desc.axis_),
                FKey(Unreal::toFName(player_input_action_desc.key_)),
                player_input_action_desc.value_));
            if (input_component_) {
                input_component_->BindAxis(Unreal::toFName(player_input_action_desc.axis_));
            }
        }
    }

    // Set in setPlayerInputActions(...), but we expose the underlying variable as public in case user code has an alternative way of setting it.
    std::vector<PlayerInputActionDesc> player_input_action_descs_;

    // Set by any system that will receive player input, e.g., SpearSimSpectatorPawn::SetupPlayerInputComponent(...).
    UInputComponent* input_component_ = nullptr;

    // Set by user code to specify what happens when an action is triggered, e.g., UUrdfRobotComponent and UUrdfJointComponent.
    std::function<void(const PlayerInputActionDesc&, float)> apply_action_func_;
};
