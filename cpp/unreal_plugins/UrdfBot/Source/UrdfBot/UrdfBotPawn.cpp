//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "UrdfBot/UrdfBotPawn.h"

#include <iostream>

#include <Camera/CameraComponent.h>
#include <Components/InputComponent.h>
#include <GameFramework/PlayerInput.h>
#include <DrawDebugHelpers.h>

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

    // setup InverseDynamics
    joint_names_ = robot_desc.joint_names_;
    // debug
    for (auto& pair : urdf_robot_component_->link_components_) {
        AddInstanceComponent(pair.second);
    }
    for (auto& pair : urdf_robot_component_->joint_components_) {
        AddInstanceComponent(pair.second);
    }

    // create track_ball_ if required
    eef_target_ = CreateDefaultSubobject<UStaticMeshComponent>(FName("TrackBall"));

    // FVector track_ball_position(Config::getValue<float>({"URDFBOT", "TRACK_BALL_COMPONENT", "POSITION_X"}), Config::getValue<float>({"URDFBOT", "TRACK_BALL_COMPONENT", "POSITION_Y"}),
    //                            Config::getValue<float>({"URDFBOT", "TRACK_BALL_COMPONENT", "POSITION_Z"}));

    // FRotator track_ball_orientation(Config::getValue<float>({"URDFBOT", "TRACK_BALL_COMPONENT", "PITCH"}), Config::getValue<float>({"URDFBOT", "TRACK_BALL_COMPONENT", "YAW"}),
    //                                Config::getValue<float>({"URDFBOT", "TRACK_BALL_COMPONENT", "ROLL"}));
    eef_target_->SetRelativeLocation(FVector(200,200,200));
    eef_target_->SetRelativeRotation(FRotator::ZeroRotator);
    eef_target_->SetRelativeScale3D(FVector::OneVector * 1);
    UStaticMesh* sphere_static_mesh = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Sphere.Sphere"));
    eef_target_->SetStaticMesh(sphere_static_mesh);
    eef_target_->SetMobility(EComponentMobility::Movable);
    eef_target_->SetSimulatePhysics(false);
    eef_target_->SetCollisionEnabled(ECollisionEnabled::Type::NoCollision);
    eef_target_->SetCollisionObjectType(ECollisionChannel::ECC_Visibility);

    eef_target_->SetupAttachment(RootComponent);
    this->AddInstanceComponent(eef_target_);

    camera_component_->SetupAttachment(eef_target_);
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

    UMaterialInterface* base_material = LoadObject<UMaterialInterface>(nullptr, *FString("Material'/UrdfBot/Common/M_PureColor.M_PureColor'"));
    UMaterialInstanceDynamic* material = UMaterialInstanceDynamic::Create(base_material, this);
    material->SetVectorParameterValue("BaseColor_Color", FLinearColor(1,0,0,1));

    eef_target_->SetMaterial(0, material);
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
        // addGravityCompensationAction();
        taskSpaceControl();
    }
    FTransform eef_transform = eef_target_->GetRelativeTransform();
    DrawDebugCoordinateSystem(GetWorld(), eef_transform.GetLocation(), eef_transform.GetRotation().Rotator(), 100, false, -1.0F, 0U, 2.f);
}

void AUrdfBotPawn::addGravityCompensationAction()
{
    int dof = joint_names_.size();

    Eigen::VectorXf qpos;
    Eigen::VectorXf qvel;
    qpos.resize(dof);
    qvel.resize(dof);
    for (int i = 0; i < dof; i++) {
        UUrdfJointComponent* joint = urdf_robot_component_->joint_components_.at(joint_names_[i]);
        // negative sign as unreal consider joint direction differently
        qpos[i] = joint->getQPos();
        qvel[i] = joint->getQVel();
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

void AUrdfBotPawn::taskSpaceControl()
{
    int dof = joint_names_.size();

    UUrdfLinkComponent* eef_link_component = urdf_robot_component_->joint_components_.at(joint_names_[dof - 1])->child_link_component_;

    Eigen::VectorXf qpos(dof);
    Eigen::VectorXf qvel(dof);
    for (int i = 0; i < dof; i++) {
        UUrdfJointComponent* joint = urdf_robot_component_->joint_components_.at(joint_names_[i]);
        qpos[i] = joint->getQPos();
        // -joint->ConstraintInstance.GetCurrentTwist();
        qvel[i] = joint->getQVel();
    }
    Eigen::VectorXf qfrc_applied = mujoco_control_->task_space_control(eef_target_->GetRelativeTransform(), eef_link_component->GetRelativeTransform(), eef_link_component->GetComponentVelocity(),
                                                                       FMath::DegreesToRadians(eef_link_component->GetPhysicsAngularVelocity()), qpos, qvel);

    std::map<std::string, float> actions;
    for (int i = 0; i < dof; i++) {
        actions[joint_names_[i]] = qfrc_applied[i];
    }
    urdf_robot_component_->addAction(actions);
}
