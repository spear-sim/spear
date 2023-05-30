//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "UrdfBot/UrdfBotPawn.h"

#include <iostream>

#include <Camera/CameraComponent.h>
#include <Components/InputComponent.h>
#include <GameFramework/PlayerInput.h>

#include "CoreUtils/Config.h"
#include "CoreUtils/Std.h"
#include "CoreUtils/Unreal.h"
#include "UrdfBot/UrdfParser.h"
#include "UrdfBot/UrdfLinkComponent.h"
#include "UrdfBot/UrdfRobotComponent.h"

AUrdfBotPawn::AUrdfBotPawn(const FObjectInitializer& object_initializer) : APawn(object_initializer)
{
    SP_LOG_CURRENT_FUNCTION();

    urdf_robot_component_ = CreateDefaultSubobject<UUrdfRobotComponent>(Unreal::toFName("AUrdfBotPawn::urdf_robot_component_"));

    RootComponent = urdf_robot_component_;

    // setup UCameraComponent
    camera_component_ = CreateDefaultSubobject<UCameraComponent>(Unreal::toFName("AUrdfBotPawn::camera_component_"));
    SP_ASSERT(camera_component_);

    FVector camera_location(
        Config::get<float>("URDFBOT.URDFBOT_PAWN.CAMERA_COMPONENT.POSITION_X"),
        Config::get<float>("URDFBOT.URDFBOT_PAWN.CAMERA_COMPONENT.POSITION_Y"),
        Config::get<float>("URDFBOT.URDFBOT_PAWN.CAMERA_COMPONENT.POSITION_Z"));

    FRotator camera_orientation(
        Config::get<float>("URDFBOT.URDFBOT_PAWN.CAMERA_COMPONENT.PITCH"),
        Config::get<float>("URDFBOT.URDFBOT_PAWN.CAMERA_COMPONENT.YAW"),
        Config::get<float>("URDFBOT.URDFBOT_PAWN.CAMERA_COMPONENT.ROLL"));

    camera_component_->SetRelativeLocationAndRotation(camera_location, camera_orientation);
    camera_component_->bUsePawnControlRotation = false;
    camera_component_->FieldOfView = Config::get<float>("URDFBOT.URDFBOT_PAWN.CAMERA_COMPONENT.FOV");
    camera_component_->SetupAttachment(urdf_robot_component_->root_link_component_);
}

AUrdfBotPawn::~AUrdfBotPawn()
{
    SP_LOG_CURRENT_FUNCTION();
}

void AUrdfBotPawn::SetupPlayerInputComponent(UInputComponent* input_component)
{
    APawn::SetupPlayerInputComponent(input_component);

    UPlayerInput* player_input = GetWorld()->GetFirstPlayerController()->PlayerInput;
    auto keyboard_actions = Config::get<std::map<std::string, std::map<std::string, std::map<std::string, float>>>>("URDFBOT.URDFBOT_PAWN.KEYBOARD_ACTIONS");
    for (auto& keyboard_action_config : keyboard_actions) {
        KeyboardAction keyboard_action;
        keyboard_action.axis_ = Unreal::toStdString(GetName()) + "::" + keyboard_action_config.first;

        if (Std::containsKey(keyboard_action_config.second, "APPLY_ACTION")) {
            keyboard_action.apply_action_ = keyboard_action_config.second.at("APPLY_ACTION");
        }

        if (Std::containsKey(keyboard_action_config.second, "ADD_ACTION")) {
            keyboard_action.add_action_ = keyboard_action_config.second.at("ADD_ACTION");
        }

        player_input->AddAxisMapping(FInputAxisKeyMapping(Unreal::toFName(keyboard_action.axis_), FKey(Unreal::toFName(keyboard_action_config.first)), 1.0f));
        input_component->BindAxis(Unreal::toFName(keyboard_action.axis_));

        keyboard_actions_.push_back(keyboard_action);
    }
}

void AUrdfBotPawn::Tick(float delta_time)
{
    APawn::Tick(delta_time);

    for (auto& keyboard_action : keyboard_actions_) {
        float axis_value = InputComponent->GetAxisValue(Unreal::toFName(keyboard_action.axis_));
        if (axis_value > 0.0f) {
            urdf_robot_component_->applyAction(keyboard_action.apply_action_);
            urdf_robot_component_->addAction(keyboard_action.add_action_);
        }
    }
}
