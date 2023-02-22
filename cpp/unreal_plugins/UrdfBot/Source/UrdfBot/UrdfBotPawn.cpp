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
#include "UrdfBot/UrdfJointComponent.h"

AUrdfBotPawn::AUrdfBotPawn(const FObjectInitializer& object_initializer) : APawn(object_initializer)
{
    std::cout << "[SPEAR | UrdfBotPawn.cpp] AUrdfBotPawn::AUrdfBotPawn" << std::endl;

    // setup UUrdfRobotComponent
    UrdfRobotDesc robot_desc = UrdfParser::parse(Unreal::toStdString(
        FPaths::Combine(Unreal::toFString(Config::get<std::string>("URDFBOT.URDFBOT_PAWN.URDF_DIR")), Unreal::toFString(Config::get<std::string>("URDFBOT.URDFBOT_PAWN.URDF_FILE")))));

    urdf_robot_component_ = NewObject<UUrdfRobotComponent>(this, Unreal::toFName("AUrdfBotPawn::urdf_robot_component_"));
    urdf_robot_component_->createChildComponents(&robot_desc);
    RootComponent = urdf_robot_component_;

    // setup UCameraComponent
    camera_component_ = CreateDefaultSubobject<UCameraComponent>(Unreal::toFName("AUrdfBotPawn::camera_component_"));
    ASSERT(camera_component_);

    FVector camera_location(Config::get<float>("URDFBOT.URDFBOT_PAWN.CAMERA_COMPONENT.POSITION_X"), Config::get<float>("URDFBOT.URDFBOT_PAWN.CAMERA_COMPONENT.POSITION_Y"),
                            Config::get<float>("URDFBOT.URDFBOT_PAWN.CAMERA_COMPONENT.POSITION_Z"));

    FRotator camera_orientation(Config::get<float>("URDFBOT.URDFBOT_PAWN.CAMERA_COMPONENT.PITCH"), Config::get<float>("URDFBOT.URDFBOT_PAWN.CAMERA_COMPONENT.YAW"),
                                Config::get<float>("URDFBOT.URDFBOT_PAWN.CAMERA_COMPONENT.ROLL"));

    camera_component_->SetRelativeLocationAndRotation(camera_location, camera_orientation);
    camera_component_->bUsePawnControlRotation = false;
    camera_component_->FieldOfView = Config::get<float>("URDFBOT.URDFBOT_PAWN.CAMERA_COMPONENT.FOV");
    camera_component_->SetupAttachment(urdf_robot_component_->root_link_component_);
    camera_component_->SetupAttachment(urdf_robot_component_);

    // setup InverseDynamics
    joint_names_ = robot_desc.joint_names_;
    // debug
    for (auto& pair : urdf_robot_component_->link_components_) {
        AddInstanceComponent(pair.second);
    }
    for (auto& pair : urdf_robot_component_->joint_components_) {
        AddInstanceComponent(pair.second);
    }
}

AUrdfBotPawn::~AUrdfBotPawn()
{
    std::cout << "[SPEAR | UrdfBotPawn.cpp] AUrdfBotPawn::~AUrdfBotPawn" << std::endl;
    if (mujoco_control_) {
        delete mujoco_control_;
    }
}

void AUrdfBotPawn::BeginPlay()
{
    Super::BeginPlay();

    std::string urdf_filename = Unreal::toStdString(
        FPaths::Combine(Unreal::toFString(Config::get<std::string>("URDFBOT.URDFBOT_PAWN.URDF_DIR")), Unreal::toFString(Config::get<std::string>("URDFBOT.URDFBOT_PAWN.URDF_FILE"))));
    mujoco_control_ = new UrdfMujocoControl(urdf_filename);
}

void AUrdfBotPawn::SetupPlayerInputComponent(class UInputComponent* input_component)
{
    Super::SetupPlayerInputComponent(input_component);

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

        player_input->AddAxisMapping(FInputAxisKeyMapping(Unreal::toFName(keyboard_action.axis_), FKey(Unreal::toFName(keyboard_action_config.first)), 1));
        input_component->BindAxis(Unreal::toFName(keyboard_action.axis_));

        keyboard_actions_.push_back(keyboard_action);
    }

    // debug
    input_component->BindKey(EKeys::SpaceBar, EInputEvent::IE_Pressed, this, &AUrdfBotPawn::testKey);
    input_component->BindKey(EKeys::C, EInputEvent::IE_Pressed, this, &AUrdfBotPawn::testKey2);
}

void AUrdfBotPawn::Tick(float delta_time)
{
    Super::Tick(delta_time);

    for (auto& keyboard_action : keyboard_actions_) {
        float axis_value = InputComponent->GetAxisValue(Unreal::toFName(keyboard_action.axis_));
        if (axis_value > 0.0f) {
            urdf_robot_component_->applyAction(keyboard_action.apply_action_);
            urdf_robot_component_->addAction(keyboard_action.add_action_);
        }
    }

    // InverseDynamics
    if (flag % 2 == 0) {
        addGravityCompensationAction();
    }
}

void AUrdfBotPawn::addGravityCompensationAction()
{
    int dof = joint_names_.size();

    Eigen::VectorXf qpos;
    qpos.resize(dof);
    for (int i = 0; i < dof; i++) {
        UUrdfJointComponent* joint = urdf_robot_component_->joint_components_.at(joint_names_[i]);
        // TODO why negative sign???
        qpos[i] = -joint->ConstraintInstance.GetCurrentTwist();
    }
    Eigen::VectorXf qfrc_applied = mujoco_control_->inverseDynamics(qpos);
    std::map<std::string, float> actions;
    for (int i = 0; i < dof; i++) {
        actions[joint_names_[i]] = qfrc_applied[i];
    }
    urdf_robot_component_->addAction(actions);
}

void AUrdfBotPawn::resetConfig()
{
    Config::terminate();
    Config::initialize();
}

void AUrdfBotPawn::testKey()
{
    int size = joint_names_.size();
    for (int i = 0; i < size; i++) {
        UUrdfJointComponent* joint = urdf_robot_component_->joint_components_.at(joint_names_[i]);
        joint->child_link_component_->SetPhysicsLinearVelocity(FVector::ZeroVector);
        joint->child_link_component_->SetPhysicsAngularVelocityInDegrees(FVector::ZeroVector);
    }
    GEngine->AddOnScreenDebugMessage(1, 200, FColor::Blue, FString::Printf(TEXT("[AUrdfBotPawn::testKey] zero velocity")));
}

void AUrdfBotPawn::testKey2()
{
    flag++;
}
