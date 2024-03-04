//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <functional> // std::function
#include <map>
#include <string>
#include <vector>

#include <Components/InputComponent.h>
#include <Components/SceneComponent.h>
#include <Engine/EngineBaseTypes.h> // ELevelTick
#include <GameFramework/PlayerController.h>
#include <GameFramework/PlayerInput.h>
#include <UObject/ObjectMacros.h>   // GENERATED_BODY, UCLASS

#include "SpCore/Log.h"
#include "SpCore/Unreal.h"

#include "InputActionComponent.generated.h"

struct FActorComponentTickFunction;

UCLASS()
class SPCORE_API UInputActionComponent : public USceneComponent
{
    GENERATED_BODY()
public:
    UInputActionComponent()
    {
        SP_LOG_CURRENT_FUNCTION();

        PrimaryComponentTick.bCanEverTick = true;
        PrimaryComponentTick.bTickEvenWhenPaused = false;
    }

    ~UInputActionComponent()
    {
        SP_LOG_CURRENT_FUNCTION();

        input_component_ = nullptr;
        input_action_descs_.clear();
    }

    // USceneComponent interface
    void TickComponent(float delta_time, ELevelTick level_tick, FActorComponentTickFunction* this_tick_function) override
    {
        USceneComponent::TickComponent(delta_time, level_tick, this_tick_function);

        if (input_component_ && apply_input_action_func_) {
            for (auto& input_action_desc : input_action_descs_) {
                float axis_value = input_component_->GetAxisValue(Unreal::toFName(input_action_desc.axis_));
                if (axis_value >= input_action_desc.threshold_) {
                    apply_input_action_func_(input_action_desc.key_);
                }   
            }
        }
    }

    // Must be called in BeginPlay() or later as GetWorld() needs to be valid.
    void bindInputActions(const std::vector<std::string>& input_action_keys)
    {
        SP_ASSERT(GetWorld());
        SP_ASSERT(GetWorld()->GetFirstPlayerController());

        input_component_ = GetWorld()->GetFirstPlayerController()->InputComponent;
        SP_ASSERT(input_component_);

        UPlayerInput* player_input = GetWorld()->GetFirstPlayerController()->PlayerInput;
        SP_ASSERT(player_input);

        for (auto& input_action_key : input_action_keys) {
            InputActionDesc input_action_desc;
            input_action_desc.key_ = input_action_key;

            bool include_actor_name = true;
            std::string separator = ".";
            input_action_desc.axis_ =
                Unreal::getFullyQualifiedComponentName(this, separator, include_actor_name) + separator + input_action_desc.key_;

            player_input->AddAxisMapping(FInputAxisKeyMapping(
                Unreal::toFName(input_action_desc.axis_),
                FKey(Unreal::toFName(input_action_desc.key_)),
                input_action_desc.scale_));
            input_component_->BindAxis(Unreal::toFName(input_action_desc.axis_));

            input_action_descs_.push_back(input_action_desc);
        }
    }

    // Set by user code to specify what happens when an action is triggered, e.g., UUrdfRobotComponent and UUrdfJointComponent.
    std::function<void(const std::string&)> apply_input_action_func_;

private:
    struct InputActionDesc
    {
        std::string key_;
        std::string axis_;
        float scale_ = 1.0f;
        float threshold_ = 1.0f;
    };

    std::vector<InputActionDesc> input_action_descs_;
    UInputComponent* input_component_ = nullptr;
};
