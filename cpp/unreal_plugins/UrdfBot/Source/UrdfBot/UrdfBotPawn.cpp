//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "UrdfBot/UrdfBotPawn.h"

#include <Camera/CameraComponent.h>
#include <Components/InputComponent.h>
#include <GameFramework/PlayerInput.h>

#include "CoreUtils/Config.h"
#include "CoreUtils/Unreal.h"
#include "UrdfBot/UrdfParser.h"
#include "UrdfBot/UrdfLinkComponent.h"
#include "UrdfBot/UrdfRobotComponent.h"

AUrdfBotPawn::AUrdfBotPawn(const FObjectInitializer& object_initializer) : APawn(object_initializer)
{
    UrdfRobotDesc robot_desc = UrdfParser::parse(Unreal::toStdString(FPaths::Combine(
        Unreal::toFString(Config::get<std::string>("URDFBOT.URDFBOT_PAWN.URDF_DIR")),
        Unreal::toFString(Config::get<std::string>("URDFBOT.URDFBOT_PAWN.URDF_FILE")))));

    robot_component_ = CreateDefaultSubobject<UUrdfRobotComponent>(Unreal::toFName("AUrdfBotPawn::robot_component_::" + robot_desc.name_));
    robot_component_->createChildComponents(&robot_desc);

    RootComponent = robot_component_;

    // setup camera
    FVector camera_location(
        Config::get<float>("URDFBOT.URDFBOT_PAWN.CAMERA_COMPONENT.POSITION_X"),
        Config::get<float>("URDFBOT.URDFBOT_PAWN.CAMERA_COMPONENT.POSITION_Y"),
        Config::get<float>("URDFBOT.URDFBOT_PAWN.CAMERA_COMPONENT.POSITION_Z"));

    FRotator camera_orientation(
        Config::get<float>("URDFBOT.URDFBOT_PAWN.CAMERA_COMPONENT.PITCH"),
        Config::get<float>("URDFBOT.URDFBOT_PAWN.CAMERA_COMPONENT.YAW"),
        Config::get<float>("URDFBOT.URDFBOT_PAWN.CAMERA_COMPONENT.ROLL"));

    camera_component_ = CreateDefaultSubobject<UCameraComponent>(Unreal::toFName("AOpenBotPawn::camera_component_"));
    ASSERT(camera_component_);

    camera_component_->SetRelativeLocationAndRotation(camera_location, camera_orientation);
    camera_component_->SetupAttachment(robot_component_->root_link_component_);
    camera_component_->bUsePawnControlRotation = false;
    camera_component_->FieldOfView = Config::get<float>("URDFBOT.URDFBOT_PAWN.CAMERA_COMPONENT.FOV");
}

void AUrdfBotPawn::SetupPlayerInputComponent(class UInputComponent* input_component)
{
    Super::SetupPlayerInputComponent(input_component);

    UPlayerInput* player_input = GetWorld()->GetFirstPlayerController()->PlayerInput;
    auto keyboard_actions = Config::get<std::map<std::string, std::map<std::string, std::map<std::string, float>>>>("URDFBOT.URDFBOT_PAWN.KEYBOARD_ACTIONS");
    for (auto& keyboard_acton : keyboard_actions) {
        InputAxisBinding input_axis_binding;
        input_axis_binding.axis_ = FName(GetName() + "::" + keyboard_acton.first.c_str());
        input_axis_binding.apply_action_ = keyboard_acton.second["APPLY_ACTION"];
        input_axis_binding.add_action_ = keyboard_acton.second["ADD_ACTION"];

        player_input->AddAxisMapping(FInputAxisKeyMapping(input_axis_binding.axis_, FKey(FName(keyboard_acton.first.c_str())), 1));
        input_component->BindAxis(input_axis_binding.axis_);

        input_axis_bindings_.push_back(input_axis_binding);
    }
}

void AUrdfBotPawn::Tick(float delta_time)
{
    Super::Tick(delta_time);

    for (auto& input_axis_binding : input_axis_bindings_) {
        float axis_value = InputComponent->GetAxisValue(input_axis_binding.axis_);
        if (axis_value > 0.0f) {
            robot_component_->applyAction(input_axis_binding.apply_action_);
            robot_component_->addAction(input_axis_binding.add_action_);
        }
    }
}
