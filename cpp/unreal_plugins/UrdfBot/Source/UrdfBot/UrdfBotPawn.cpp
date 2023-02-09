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
#include "UrdfBot/UrdfRobotComponent.h"

AUrdfBotPawn::AUrdfBotPawn(const FObjectInitializer& object_initializer)
    : APawn(object_initializer)
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

    camera_component_ = CreateDefaultSubobject<UCameraComponent>(TEXT("AOpenBotPawn::camera_component_"));
    ASSERT(camera_component_);

    camera_component_->SetRelativeLocationAndRotation(camera_location, camera_orientation);
    camera_component_->SetupAttachment(robot_component_->root_link_component_);
    camera_component_->bUsePawnControlRotation = false;
    camera_component_->FieldOfView = Config::get<float>("URDFBOT.URDFBOT_PAWN.CAMERA_COMPONENT.FOV");
}

void AUrdfBotPawn::SetupPlayerInputComponent(class UInputComponent* input_component)
{
    Super::SetupPlayerInputComponent(input_component);

    int axis_binding_index = 0;
    UPlayerInput* player_input = GetWorld()->GetFirstPlayerController()->PlayerInput;
    std::vector<YAML::Node> keyboard_controls_ = Config::get<std::vector<YAML::Node>>("URDFBOT.URDFBOT_PAWN.ROBOT_COMPONENT.KEYBOARD_CONTROL");
    for (auto& control : keyboard_controls_) {
        InputAxisBinding input_axis_binding;
        input_axis_binding.axis_name_ = FName(GetName() + "_" + FString::FromInt(axis_binding_index));

        input_axis_binding.key_ = control["KEY"].as<std::string>();
        std::string type = control["TYPE"].as<std::string>();
        if (type == "set") {
            input_axis_binding.type_ = AxisBindingType::Set;
        } else if (type == "add") {
            input_axis_binding.type_ = AxisBindingType::Add;
        } else {
            ASSERT(false);
        }
        input_axis_binding.component_names_ = control["COMPONENT"].as<std::vector<std::string>>();
        input_axis_binding.values_ = control["VALUE"].as<std::vector<float>>();
        ASSERT(input_axis_binding.component_names_.size() == input_axis_binding.values_.size());

        player_input->AddAxisMapping(FInputAxisKeyMapping(input_axis_binding.axis_name_, FKey(FName(input_axis_binding.key_.c_str())), 1));
        input_component->BindAxis(input_axis_binding.axis_name_);

        robot_component_->input_axis_bindings_.push_back(input_axis_binding);

        axis_binding_index++;
    }
}

void AUrdfBotPawn::Tick(float delta_time)
{
    Super::Tick(delta_time);

    robot_component_->applyKeyInputs();
}
