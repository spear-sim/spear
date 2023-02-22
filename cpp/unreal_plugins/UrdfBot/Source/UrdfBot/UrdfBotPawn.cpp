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

    urdf_robot_component_ = CreateDefaultSubobject<UUrdfRobotComponent>(Unreal::toFName("AUrdfBotPawn::urdf_robot_component_::" + robot_desc.name_));
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
    // camera_component_->SetupAttachment(urdf_robot_component_->root_link_component_);
    camera_component_->SetupAttachment(urdf_robot_component_);

    for (auto& pair : urdf_robot_component_->link_components_) {
        AddInstanceComponent(pair.second);
    }
    for (auto& pair : urdf_robot_component_->joint_components_) {
        AddInstanceComponent(pair.second);
    }
    control = new UrdfSimpleControl();

    joint_names.push_back("joint_0");
    if (urdf_robot_component_->joint_components_.size() == 2) {
        joint_names.push_back("joint_1");
    }
    control->initialize(&robot_desc, joint_names);
}

AUrdfBotPawn::~AUrdfBotPawn()
{
    std::cout << "[SPEAR | UrdfBotPawn.cpp] AUrdfBotPawn::~AUrdfBotPawn" << std::endl;
    if (control) {
        delete control;
    }
    if (mujoco_control_) {
        delete mujoco_control_;
    }
}
void AUrdfBotPawn::BeginPlay()
{
    Super::BeginPlay();
    UE_LOG(LogTemp, Log, TEXT("AUrdfBotPawn::BeginPlay"));

    mujoco_control_ = new UrdfMujocoControl();
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

    input_component->BindKey(EKeys::SpaceBar, EInputEvent::IE_Pressed, this, &AUrdfBotPawn::testKey);
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
    addGravityCompensationAction();
}

void AUrdfBotPawn::addGravityCompensationAction()
{
    float g = 9.81f;
    float mass = 1.0f;
    float length = 1.0f;
#if 0
    if (urdf_robot_component_->joint_components_.size() == 1) {
        std::string joint_name = "joint_0";
        UUrdfJointComponent* joint = urdf_robot_component_->joint_components_.at(joint_name);
        float angle = joint->ConstraintInstance.GetCurrentTwist();

        if (angle > PI / 2) {
            angle = PI - angle;
        }

        if (angle < -PI / 2) {
            angle = -PI - angle;
        }

        float force0 = -g * mass * length * FMath::Sin(angle);

        std::map<std::string, float> compensation_actions;
        compensation_actions[joint_name] = force0;
        // urdf_robot_component_->addAction(compensation_actions);
        UE_LOG(LogTemp, Log, TEXT("[AUrdfBotPawn::Tick] analytical angle=%f force0=%f"), angle, force0);
    }
#endif
    int size = joint_names.size();

    std::vector<float> qpos;
    qpos.resize(size);
    for (int i = 0; i < size; i++) {
        UUrdfJointComponent* joint = urdf_robot_component_->joint_components_.at(joint_names[i]);
        float angle = joint->ConstraintInstance.GetCurrentTwist();

        angle = -angle;
        qpos[i] = angle;
    }
    std::vector<float> result = mujoco_control_->get_qfrc_inverse(qpos);
    std::map<std::string, float> actions;
    actions["joint_0"] = result[0];
    if (joint_names.size() == 2) {
        actions["joint_1"] = result[1];
    }
    urdf_robot_component_->addAction(actions);
    UE_LOG(LogTemp, Log, TEXT("[AUrdfBotPawn::Tick] mujoco     angle0=%f force0=%f "), qpos[0], actions[joint_names[0]]);
}

void AUrdfBotPawn::resetConfig()
{
    Config::terminate();
    Config::initialize();
}

void AUrdfBotPawn::testKey()
{
    int size = joint_names.size();
    for (int i = 0; i < size; i++) {
        UUrdfJointComponent* joint = urdf_robot_component_->joint_components_.at(joint_names[i]);
        joint->child_link_component_->SetPhysicsLinearVelocity(FVector::ZeroVector);
        joint->child_link_component_->SetPhysicsAngularVelocityInDegrees(FVector::ZeroVector);
    }
    GEngine->AddOnScreenDebugMessage(1, 200, FColor::Blue, FString::Printf(TEXT("[AUrdfBotPawn::testKey] zero velocity")));
    UE_LOG(LogTemp, Log, TEXT("[AUrdfBotPawn::testKey] zero velocity"));
}
