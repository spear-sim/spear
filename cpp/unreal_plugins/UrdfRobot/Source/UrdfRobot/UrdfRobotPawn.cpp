//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "UrdfRobot/UrdfRobotPawn.h"

#include <filesystem>
#include <iostream>

#include <Camera/CameraComponent.h>
#include <Components/InputComponent.h>
#include <CoreMinimal.h>
#include <GameFramework/Pawn.h>
#include <GameFramework/PlayerInput.h>
#include <Math/Rotator.h>
#include <Math/Vector.h>

#include "CoreUtils/Config.h"
#include "CoreUtils/Log.h"
#include "CoreUtils/Std.h"
#include "CoreUtils/Unreal.h"
#include "UrdfRobot/UrdfParser.h"
#include "UrdfRobot/UrdfRobotComponent.h"

AUrdfRobotPawn::AUrdfRobotPawn()
{
    SP_LOG_CURRENT_FUNCTION();

    if (!Config::s_initialized_) {
        return;
    }

    // setup UUrdfRobotComponent
    std::string urdf_file = 
        Config::get<std::string>("URDF_ROBOT.URDF_ROBOT_PAWN.URDF_DIR") +
        std::string(1, std::filesystem::path::preferred_separator) +
        Config::get<std::string>("URDF_ROBOT.URDF_ROBOT_PAWN.URDF_FILE");

    UrdfRobotDesc robot_desc = UrdfParser::parse(urdf_file);

    urdf_robot_component_ = CreateDefaultSubobject<UUrdfRobotComponent>(Unreal::toFName(robot_desc.name_));
    SP_ASSERT(urdf_robot_component_);
    urdf_robot_component_->initialize(&robot_desc);
    // no need to call RegisterComponent() in the constructor

    RootComponent = urdf_robot_component_;

    // setup UCameraComponent
    camera_component_ = CreateDefaultSubobject<UCameraComponent>(Unreal::toFName("camera_component"));
    SP_ASSERT(camera_component_);

    FVector camera_location(
        Config::get<double>("URDF_ROBOT.URDF_ROBOT_PAWN.CAMERA_COMPONENT.LOCATION_X"),
        Config::get<double>("URDF_ROBOT.URDF_ROBOT_PAWN.CAMERA_COMPONENT.LOCATION_Y"),
        Config::get<double>("URDF_ROBOT.URDF_ROBOT_PAWN.CAMERA_COMPONENT.LOCATION_Z"));

    FRotator camera_orientation(
        Config::get<double>("URDF_ROBOT.URDF_ROBOT_PAWN.CAMERA_COMPONENT.ROTATION_PITCH"),
        Config::get<double>("URDF_ROBOT.URDF_ROBOT_PAWN.CAMERA_COMPONENT.ROTATION_YAW"),
        Config::get<double>("URDF_ROBOT.URDF_ROBOT_PAWN.CAMERA_COMPONENT.ROTATION_ROLL"));

    camera_component_->SetRelativeLocationAndRotation(camera_location, camera_orientation);
    camera_component_->SetupAttachment(urdf_robot_component_);    
    camera_component_->bUsePawnControlRotation = false;
    camera_component_->FieldOfView = Config::get<float>("URDF_ROBOT.URDF_ROBOT_PAWN.CAMERA_COMPONENT.FOV");
    camera_component_->AspectRatio = Config::get<float>("URDF_ROBOT.URDF_ROBOT_PAWN.CAMERA_COMPONENT.ASPECT_RATIO");
    // no need to call RegisterComponent() in the constructor
}

AUrdfRobotPawn::~AUrdfRobotPawn()
{
    SP_LOG_CURRENT_FUNCTION();

    // Pawns don't need to be cleaned up explicitly.
}

void AUrdfRobotPawn::SetupPlayerInputComponent(UInputComponent* input_component)
{
    APawn::SetupPlayerInputComponent(input_component);

    UPlayerInput* player_input = GetWorld()->GetFirstPlayerController()->PlayerInput;
    auto keyboard_actions = Config::get<std::map<std::string, std::map<std::string, std::vector<double>>>>("URDF_ROBOT.URDF_ROBOT_PAWN.KEYBOARD_ACTIONS");
    for (auto& keyboard_action : keyboard_actions) {
        KeyboardActionDesc keyboard_action_desc;
        keyboard_action_desc.axis_ = Unreal::toStdString(GetName()) + "." + keyboard_action.first;
        keyboard_action_desc.action_ = keyboard_action.second;
        player_input->AddAxisMapping(FInputAxisKeyMapping(Unreal::toFName(keyboard_action_desc.axis_), FKey(Unreal::toFName(keyboard_action.first)), 1.0f));
        input_component->BindAxis(Unreal::toFName(keyboard_action_desc.axis_));
        keyboard_action_descs_.push_back(keyboard_action_desc);
    }
}

void AUrdfRobotPawn::Tick(float delta_time)
{
    APawn::Tick(delta_time);

    for (auto& keyboard_action_desc : keyboard_action_descs_) {
        float axis_value = InputComponent->GetAxisValue(Unreal::toFName(keyboard_action_desc.axis_));
        if (axis_value > 0.0f) {
            urdf_robot_component_->applyAction(keyboard_action_desc.action_);
        }
    }
}

void AUrdfRobotPawn::initialize(const std::string& urdf_file)
{
    UrdfRobotDesc robot_desc = UrdfParser::parse(urdf_file);

    urdf_robot_component_ = NewObject<UUrdfRobotComponent>(this, Unreal::toFName(robot_desc.name_));
    SP_ASSERT(urdf_robot_component_);
    urdf_robot_component_->initialize(&robot_desc);
    urdf_robot_component_->RegisterComponent();

    RootComponent = urdf_robot_component_;

    // Since this function is intended for debugging within the Unreal Editor, there is no need to
    // initialize the camera component.
}
