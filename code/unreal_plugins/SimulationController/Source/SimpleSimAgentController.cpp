#include "SimpleSimAgentController.h"

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Camera/CameraActor.h>
#include <Components/StaticMeshComponent.h>
#include <Engine/StaticMeshActor.h>
#include <Engine/World.h>
#include <EngineUtils.h>
#include <GameFramework/Actor.h>
#include <UObject/UObjectGlobals.h>
#include <UObject/Object.h>

#include "Assert/Assert.h"
#include "Box.h"
#include "Config.h"
#include "CameraSensor.h"
#include "OpenBotPawn.h"
#include "Serialize.h"
#include "TickEvent.h"

SimpleSimAgentController::SimpleSimAgentController(UWorld* world)
{
    // store ref to world
    world_ = world;

    if (Config::getValue<std::string>({"SIMULATION_CONTROLLER", "SIMPLE_SIM_AGENT_CONTROLLER", "OBSERVATION_MODE"}) == "mixed") {
        new_object_parent_actor_ = world->SpawnActor<AActor>();
        ASSERT(new_object_parent_actor_);

        post_physics_pre_render_tick_event_ = NewObject<UTickEvent>(new_object_parent_actor_, TEXT("PostPhysicsPreRenderTickEvent"));
        ASSERT(post_physics_pre_render_tick_event_);
        post_physics_pre_render_tick_event_->RegisterComponent();
        post_physics_pre_render_tick_event_->initialize(ETickingGroup::TG_PostPhysics);
        post_physics_pre_render_tick_event_handle_ = post_physics_pre_render_tick_event_->delegate_.AddRaw(this, &SimpleSimAgentController::postPhysicsPreRenderTickEventHandler);
    }

    // spawn agent actor
    FActorSpawnParameters spawn_params;
    spawn_params.Name = FName(Config::getValue<std::string>({ "SIMULATION_CONTROLLER", "SIMPLE_SIM_AGENT_CONTROLLER", "AGENT_ACTOR_NAME" }).c_str());
    spawn_params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

    if (Config::getValue<std::string>({ "SIMULATION_CONTROLLER", "SIMPLE_SIM_AGENT_CONTROLLER", "AGENT_ACTOR_MESH" }) == "sphere") {
        // load agent mesh and agent material
        agent_actor_ = world_->SpawnActor<AStaticMeshActor>(AStaticMeshActor::StaticClass(),
            FVector(0, 0, Config::getValue<float>({ "SIMULATION_CONTROLLER", "SIMPLE_SIM_AGENT_CONTROLLER", "MIXED_MODE", "OBSERVATION_CAMERA_ACTOR_HEIGHT" })),
            FRotator(0, 0, 0), spawn_params);
        ASSERT(dynamic_cast<AStaticMeshActor*>(agent_actor_));

        dynamic_cast<AStaticMeshActor*>(agent_actor_)->SetMobility(EComponentMobility::Type::Movable);

        agent_static_mesh_component_ = dynamic_cast<AStaticMeshActor*>(agent_actor_)->GetStaticMeshComponent();
        ASSERT(agent_static_mesh_component_);

        UStaticMesh* agent_mesh = LoadObject<UStaticMesh>(world_, TEXT("StaticMesh'/Engine/BasicShapes/Sphere.Sphere'"));
        UMaterial* agent_mat = LoadObject<UMaterial>(nullptr, TEXT("Material'/Game/Materials/Agent_MAT.Agent_MAT'"));
        ASSERT(agent_mesh);
        ASSERT(agent_mat);

        dynamic_cast<UStaticMeshComponent*>(agent_static_mesh_component_)->SetStaticMesh(agent_mesh);
        agent_static_mesh_component_->SetMaterial(0, agent_mat);
    }
    else if (Config::getValue<std::string>({ "SIMULATION_CONTROLLER", "SIMPLE_SIM_AGENT_CONTROLLER", "AGENT_ACTOR_MESH" }) == "openBot") {
        agent_actor_ = world_->SpawnActor<AOpenBotPawn>(AOpenBotPawn::StaticClass(),
            FVector(0, 0, Config::getValue<float>({ "SIMULATION_CONTROLLER", "SIMPLE_SIM_AGENT_CONTROLLER", "MIXED_MODE", "OBSERVATION_CAMERA_ACTOR_HEIGHT" })),
            FRotator(0, 0, 0), spawn_params);
        ASSERT(dynamic_cast<AOpenBotPawn*>(agent_actor_));

        agent_static_mesh_component_ = dynamic_cast<AOpenBotPawn*>(agent_actor_)->skeletal_mesh_component_;
    }
    else {
        ASSERT(false);
    }

    // spawn goal actor
    spawn_params.Name = FName(Config::getValue<std::string>({ "SIMULATION_CONTROLLER", "SIMPLE_SIM_AGENT_CONTROLLER", "GOAL_ACTOR_NAME" }).c_str());
    spawn_params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

    goal_actor_ = world_->SpawnActor<AStaticMeshActor>(AStaticMeshActor::StaticClass(), 
        FVector(0, 0, Config::getValue<float>({ "SIMULATION_CONTROLLER", "SIMPLE_SIM_AGENT_CONTROLLER", "MIXED_MODE", "OBSERVATION_CAMERA_ACTOR_HEIGHT" })), 
        FRotator(0, 0, 0), spawn_params);
    goal_actor_->SetMobility(EComponentMobility::Type::Movable);
    ASSERT(goal_actor_);

    goal_static_mesh_component_ = goal_actor_->GetStaticMeshComponent();
    ASSERT(goal_static_mesh_component_);

    // load goal mesh and goal material
    UStaticMesh* goal_mesh = LoadObject<UStaticMesh>(world_, TEXT("StaticMesh'/Engine/BasicShapes/Cylinder.Cylinder'"));
    UMaterial* goal_mat = LoadObject<UMaterial>(world_, TEXT("Material'/Game/Materials/Goal_MAT.Goal_MAT'"));
    ASSERT(goal_mesh);
    ASSERT(goal_mat);

    goal_static_mesh_component_->SetStaticMesh(goal_mesh);
    goal_static_mesh_component_->SetMaterial(0, goal_mat);

    // set physics state
    agent_static_mesh_component_->BodyInstance.SetCollisionProfileName(UCollisionProfile::PhysicsActor_ProfileName);
    agent_static_mesh_component_->SetSimulatePhysics(true);
    agent_static_mesh_component_->SetAngularDamping(Config::getValue<float>({ "SIMULATION_CONTROLLER", "SIMPLE_SIM_AGENT_CONTROLLER", "SPHERE", "ANGULAR_DAMPING" }));
    agent_static_mesh_component_->SetLinearDamping(Config::getValue<float>({ "SIMULATION_CONTROLLER", "SIMPLE_SIM_AGENT_CONTROLLER", "SPHERE", "LINEAR_DAMPING" }));
    agent_static_mesh_component_->BodyInstance.MaxAngularVelocity = FMath::RadiansToDegrees(Config::getValue<float>({ "SIMULATION_CONTROLLER", "SIMPLE_SIM_AGENT_CONTROLLER", "SPHERE", "MAX_ANGULAR_VELOCITY" }));
    agent_static_mesh_component_->BodyInstance.MassScale = FMath::RadiansToDegrees(Config::getValue<float>({ "SIMULATION_CONTROLLER", "SIMPLE_SIM_AGENT_CONTROLLER", "SPHERE", "MASS_SCALE" }));
    agent_static_mesh_component_->SetNotifyRigidBodyCollision(true);

    // setup observation camera
    if (Config::getValue<std::string>({ "SIMULATION_CONTROLLER", "SIMPLE_SIM_AGENT_CONTROLLER", "OBSERVATION_MODE" }) == "mixed") {

        spawn_params.Name = FName(Config::getValue<std::string>({ "SIMULATION_CONTROLLER", "SIMPLE_SIM_AGENT_CONTROLLER", "MIXED_MODE", "OBSERVATION_CAMERA_ACTOR_NAME" }).c_str());
        spawn_params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

        camera_actor_ = world_->SpawnActor<ACameraActor>(agent_actor_->GetActorLocation(), FRotator(0, 0, 0), spawn_params);
        ASSERT(camera_actor_);
        ASSERT(dynamic_cast<ACameraActor*>(camera_actor_));

        observation_camera_sensor_ = std::make_unique<CameraSensor>(
            dynamic_cast<ACameraActor*>(camera_actor_)->GetCameraComponent(),
            Config::getValue<std::vector<std::string>>({ "SIMULATION_CONTROLLER", "SIMPLE_SIM_AGENT_CONTROLLER", "MIXED_MODE", "RENDER_PASSES" }),
            Config::getValue<unsigned long>({ "SIMULATION_CONTROLLER", "SIMPLE_SIM_AGENT_CONTROLLER", "MIXED_MODE", "IMAGE_WIDTH" }),
            Config::getValue<unsigned long>({ "SIMULATION_CONTROLLER", "SIMPLE_SIM_AGENT_CONTROLLER", "MIXED_MODE", "IMAGE_HEIGHT" }));
        ASSERT(observation_camera_sensor_);
    }
}

SimpleSimAgentController::~SimpleSimAgentController()
{
    if (Config::getValue<std::string>({ "SIMULATION_CONTROLLER", "SIMPLE_SIM_AGENT_CONTROLLER", "OBSERVATION_MODE" }) == "mixed") {
        ASSERT(observation_camera_sensor_);
        observation_camera_sensor_ = nullptr;

        ASSERT(camera_actor_);
        camera_actor_ = nullptr;
    }

    ASSERT(goal_static_mesh_component_);
    goal_static_mesh_component_ = nullptr;

    ASSERT(goal_actor_);
    goal_actor_ = nullptr;

    ASSERT(agent_static_mesh_component_);
    agent_static_mesh_component_ = nullptr;

    ASSERT(agent_actor_);
    agent_actor_ = nullptr;

    if (Config::getValue<std::string>({"SIMULATION_CONTROLLER", "SIMPLE_SIM_AGENT_CONTROLLER", "OBSERVATION_MODE"}) == "mixed") {
        ASSERT(post_physics_pre_render_tick_event_);
        post_physics_pre_render_tick_event_->delegate_.Remove(post_physics_pre_render_tick_event_handle_);
        post_physics_pre_render_tick_event_handle_.Reset();
        post_physics_pre_render_tick_event_->DestroyComponent();
        post_physics_pre_render_tick_event_ = nullptr;

        ASSERT(new_object_parent_actor_);
        new_object_parent_actor_->Destroy();
        new_object_parent_actor_ = nullptr;

        ASSERT(world_);
        world_ = nullptr;
    }
}

void SimpleSimAgentController::findObjectReferences(UWorld* world)
{ 
}

void SimpleSimAgentController::cleanUpObjectReferences()
{  
}

std::map<std::string, Box> SimpleSimAgentController::getActionSpace() const
{
    std::map<std::string, Box> action_space;
    Box box;
    
    box.low = -1.f;
    box.high = 1.f;
    box.shape = {2};
    box.dtype = DataType::Float32;
    action_space["apply_force"] = std::move(box);

    return action_space;
}

std::map<std::string, Box> SimpleSimAgentController::getObservationSpace() const
{
    std::map<std::string, Box> observation_space;
    Box box;
    
    if (Config::getValue<std::string>({"SIMULATION_CONTROLLER", "SIMPLE_SIM_AGENT_CONTROLLER", "OBSERVATION_MODE"}) == "mixed") {
        box.low = std::numeric_limits<float>::lowest();
        box.high = std::numeric_limits<float>::max();
        box.shape = {5};
        box.dtype = DataType::Float32;
        observation_space["physical_observation"] = std::move(box);

        std::vector<std::string> passes = Config::getValue<std::vector<std::string>>(
            {"SIMULATION_CONTROLLER", "SIMPLE_SIM_AGENT_CONTROLLER", "MIXED_MODE", "RENDER_PASSES" });
        for (const auto& pass : passes) {
            box.low = 0;
            box.high = 255;
            box.shape = {Config::getValue<int64_t>({"SIMULATION_CONTROLLER", "SIMPLE_SIM_AGENT_CONTROLLER", "MIXED_MODE", "IMAGE_HEIGHT"}),
                         Config::getValue<int64_t>({"SIMULATION_CONTROLLER", "SIMPLE_SIM_AGENT_CONTROLLER", "MIXED_MODE", "IMAGE_WIDTH"}),
                         3};
            box.dtype = DataType::UInteger8;
            observation_space["visual_observation_" + pass] = std::move(box);
        }
        
    } else if (Config::getValue<std::string>({"SIMULATION_CONTROLLER", "SIMPLE_SIM_AGENT_CONTROLLER", "OBSERVATION_MODE"}) == "physical") {
        box.low = std::numeric_limits<float>::lowest();
        box.high = std::numeric_limits<float>::max();
        box.shape = {4};
        box.dtype = DataType::Float32;
        observation_space["physical_observation"] = std::move(box);
    } else {
        ASSERT(false);
    }

    return observation_space;
}

std::map<std::string, Box> SimpleSimAgentController::getStepInfoSpace() const
{
    std::map<std::string, Box> step_info_space;
    Box box;

    box.low = std::numeric_limits<float>::lowest();
    box.high = std::numeric_limits<float>::max();
    box.shape = {-1,3};
    box.dtype = DataType::Float32;
    step_info_space["debug_info"] = std::move(box);

    return step_info_space;
}

void SimpleSimAgentController::applyAction(const std::map<std::string, std::vector<float>>& action)
{
    if (Config::getValue<std::string>({"SIMULATION_CONTROLLER", "SIMPLE_SIM_AGENT_CONTROLLER", "OBSERVATION_MODE"}) == "mixed") {
        // Get yaw from the observation camera, apply force to the sphere in that direction
        FVector force = camera_actor_->GetActorRotation().RotateVector(FVector(action.at("apply_force").at(0), 0.0f, 0.0f)) * Config::getValue<float>({"SIMULATION_CONTROLLER", "SIMPLE_SIM_AGENT_CONTROLLER", "MIXED_MODE", "ACTION_APPLY_FORCE_SCALE"});

        ASSERT(isfinite(force.X));
        ASSERT(isfinite(force.Y));
        ASSERT(isfinite(force.Z));

        agent_static_mesh_component_->AddForce(force);

        // Set observation camera yaw by adding to the current observation camera yaw
        FRotator rotation = camera_actor_->GetActorRotation().Add(0.0f, action.at("apply_force").at(1) * Config::getValue<float>({"SIMULATION_CONTROLLER", "SIMPLE_SIM_AGENT_CONTROLLER", "MIXED_MODE", "ACTION_ROTATE_OBSERVATION_CAMERA_SCALE"}), 0.0f);

        ASSERT(isfinite(rotation.Pitch));
        ASSERT(isfinite(rotation.Yaw));
        ASSERT(isfinite(rotation.Roll));
        ASSERT(rotation.Pitch >= -360.0 && rotation.Pitch <= 360.0, "%f", rotation.Pitch);
        ASSERT(rotation.Yaw   >= -360.0 && rotation.Yaw   <= 360.0, "%f", rotation.Yaw);
        ASSERT(rotation.Roll  >= -360.0 && rotation.Roll  <= 360.0, "%f", rotation.Roll);

        camera_actor_->SetActorRotation(rotation);
    } else if (Config::getValue<std::string>({"SIMULATION_CONTROLLER", "SIMPLE_SIM_AGENT_CONTROLLER", "OBSERVATION_MODE"}) == "physical") {
        FVector force = FVector(action.at("apply_force").at(0), action.at("apply_force").at(1), 0.0f) * Config::getValue<float>({"SIMULATION_CONTROLLER", "SIMPLE_SIM_AGENT_CONTROLLER", "PHYSICAL_MODE", "ACTION_APPLY_FORCE_SCALE"});

        ASSERT(isfinite(force.X));
        ASSERT(isfinite(force.Y));
        ASSERT(isfinite(force.Z));

        agent_static_mesh_component_->AddForce(force);
    } else {
        ASSERT(false);
    }
}

std::map<std::string, std::vector<uint8_t>> SimpleSimAgentController::getObservation() const
{
    std::map<std::string, std::vector<uint8_t>> observation;

    // get game state
    const FVector sphere_to_goal = goal_actor_->GetActorLocation() - agent_actor_->GetActorLocation();
    const FVector linear_velocity = agent_static_mesh_component_->GetPhysicsLinearVelocity();

    // get observations
    if (Config::getValue<std::string>({"SIMULATION_CONTROLLER", "SIMPLE_SIM_AGENT_CONTROLLER", "OBSERVATION_MODE"}) == "mixed") {
        const float observation_camera_yaw = camera_actor_->GetActorRotation().Yaw;
        observation["physical_observation"] = Serialize::toUint8(std::vector<float>{
            Config::getValue<float>({"SIMULATION_CONTROLLER", "SIMPLE_SIM_AGENT_CONTROLLER", "MIXED_MODE", "OFFSET_TO_GOAL_SCALE"}) * sphere_to_goal.X,
            Config::getValue<float>({"SIMULATION_CONTROLLER", "SIMPLE_SIM_AGENT_CONTROLLER", "MIXED_MODE", "OFFSET_TO_GOAL_SCALE"}) * sphere_to_goal.Y,
            Config::getValue<float>({"SIMULATION_CONTROLLER", "SIMPLE_SIM_AGENT_CONTROLLER", "MIXED_MODE", "LINEAR_VELOCITY_SCALE"}) * linear_velocity.X,
            Config::getValue<float>({"SIMULATION_CONTROLLER", "SIMPLE_SIM_AGENT_CONTROLLER", "MIXED_MODE", "LINEAR_VELOCITY_SCALE"}) * linear_velocity.Y,
            Config::getValue<float>({"SIMULATION_CONTROLLER", "SIMPLE_SIM_AGENT_CONTROLLER", "MIXED_MODE", "YAW_SCALE"}) * observation_camera_yaw});

        ASSERT(IsInGameThread());

        // get render data
        std::map<std::string, TArray<FColor>> render_data = observation_camera_sensor_->getRenderData();
        
        for (const auto& data: render_data) {
            std::vector<uint8_t> image(Config::getValue<int>({"SIMULATION_CONTROLLER", "SIMPLE_SIM_AGENT_CONTROLLER", "MIXED_MODE", "IMAGE_HEIGHT"}) *
                                       Config::getValue<int>({"SIMULATION_CONTROLLER", "SIMPLE_SIM_AGENT_CONTROLLER", "MIXED_MODE", "IMAGE_WIDTH"}) *
                                       3);

            for (uint32 i = 0; i < static_cast<uint32>(data.second.Num()); ++i) {
                image.at(3 * i + 0) = data.second[i].R;
                image.at(3 * i + 1) = data.second[i].G;
                image.at(3 * i + 2) = data.second[i].B;
            }

            observation["visual_observation_" + data.first] = std::move(image);
        }

    } else if (Config::getValue<std::string>({"SIMULATION_CONTROLLER", "SIMPLE_SIM_AGENT_CONTROLLER", "OBSERVATION_MODE"}) == "physical") {
        observation["physical_observation"] = Serialize::toUint8(std::vector<float>{
            Config::getValue<float>({"SIMULATION_CONTROLLER", "SIMPLE_SIM_AGENT_CONTROLLER", "PHYSICAL_MODE", "OFFSET_TO_GOAL_SCALE"}) * sphere_to_goal.X,
            Config::getValue<float>({"SIMULATION_CONTROLLER", "SIMPLE_SIM_AGENT_CONTROLLER", "PHYSICAL_MODE", "OFFSET_TO_GOAL_SCALE"}) * sphere_to_goal.Y,
            Config::getValue<float>({"SIMULATION_CONTROLLER", "SIMPLE_SIM_AGENT_CONTROLLER", "PHYSICAL_MODE", "LINEAR_VELOCITY_SCALE"}) * linear_velocity.X,
            Config::getValue<float>({"SIMULATION_CONTROLLER", "SIMPLE_SIM_AGENT_CONTROLLER", "PHYSICAL_MODE", "LINEAR_VELOCITY_SCALE"}) * linear_velocity.Y});
    } else {
        ASSERT(false);
    }

    return observation;
}

std::map<std::string, std::vector<uint8_t>> SimpleSimAgentController::getStepInfo() const
{
    return { {"debug_info", {} } };
}

void SimpleSimAgentController::reset()
{
    agent_static_mesh_component_->SetPhysicsLinearVelocity(FVector(0), false);
    agent_static_mesh_component_->SetPhysicsAngularVelocityInRadians(FVector(0), false);
    agent_static_mesh_component_->GetBodyInstance()->ClearTorques();
    agent_static_mesh_component_->GetBodyInstance()->ClearForces();
}

bool SimpleSimAgentController::isReady() const
{
    return true;
}

void SimpleSimAgentController::postPhysicsPreRenderTickEventHandler(float delta_time, enum ELevelTick level_tick)
{    
    if (Config::getValue<std::string>({"SIMULATION_CONTROLLER", "SIMPLE_SIM_AGENT_CONTROLLER", "OBSERVATION_MODE"}) == "mixed") {
        if (Config::getValue<bool>({"SIMULATION_CONTROLLER", "SIMPLE_SIM_AGENT_CONTROLLER", "MIXED_MODE", "SET_OBSERVATION_CAMERA_POSE_EGOCENTRIC"})) {
            const FVector observation_camera_pose(
                agent_actor_->GetActorLocation() +
                FVector(Config::getValue<float>({"SIMULATION_CONTROLLER", "SIMPLE_SIM_AGENT_CONTROLLER", "MIXED_MODE", "OBSERVATION_CAMERA_POSITION_OFFSET_X"}),
                        Config::getValue<float>({"SIMULATION_CONTROLLER", "SIMPLE_SIM_AGENT_CONTROLLER", "MIXED_MODE", "OBSERVATION_CAMERA_POSITION_OFFSET_Y"}),
                        Config::getValue<float>({"SIMULATION_CONTROLLER", "SIMPLE_SIM_AGENT_CONTROLLER", "MIXED_MODE", "OBSERVATION_CAMERA_POSITION_OFFSET_Z"})));
            camera_actor_->SetActorLocation(observation_camera_pose);
        }
    }
}
