//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "UrdfBotPawn.h"

#include <Camera/CameraComponent.h>
#include <Components/InputComponent.h>

#include "CoreUtils/Config.h"
#include "CoreUtils/Unreal.h"
#include "UrdfRobotComponent.h"
#include "UrdfParser.h"

AUrdfBotPawn::AUrdfBotPawn(const FObjectInitializer& object_initializer): APawn(object_initializer)
{

    // setup robot
    std::string urdf_file = Unreal::toString(FPaths::Combine(
        Unreal::toFString(Config::get<std::string>("URDFBOT.URDFBOT_PAWN.URDF_DIR")),
        Unreal::toFString(Config::get<std::string>("URDFBOT.URDFBOT_PAWN.URDF_FILE"))));
    UrdfRobotDesc robot_desc = UrdfParser::parse(urdf_file);

    robot_component_ = CreateDefaultSubobject<UUrdfRobotComponent>(TEXT("RobotComponent"));
    robot_component_->initialize(&(robot_desc));
    RootComponent = robot_component_->root_link_component_;

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
    camera_component_->SetupAttachment(RootComponent);
    camera_component_->bUsePawnControlRotation = false;
    camera_component_->FieldOfView = Config::get<float>("URDFBOT.URDFBOT_PAWN.CAMERA_COMPONENT.FOV");

    // debug only
    robot_component_->test(this);
    this->AddInstanceComponent(camera_component_);
}

void AUrdfBotPawn::SetupPlayerInputComponent(class UInputComponent* input_component)
{
    Super::SetupPlayerInputComponent(input_component);

    // debug only
    input_component->BindKey(EKeys::SpaceBar, IE_Pressed, this, &AUrdfBotPawn::test);
    input_component->BindKey(EKeys::One, IE_Pressed, this, &AUrdfBotPawn::test1);
    input_component->BindKey(EKeys::Two, IE_Pressed, this, &AUrdfBotPawn::test2);
}

void AUrdfBotPawn::Tick(float delta_time)
{
    Super::Tick(delta_time);
}

void AUrdfBotPawn::test()
{
    robot_component_->action(signal);
    signal++;
}

void AUrdfBotPawn::test1()
{
    robot_component_->actionJoint("shoulder_lift_joint", 10);
}

void AUrdfBotPawn::test2()
{
    robot_component_->actionJoint("shoulder_lift_joint", -10);
}
