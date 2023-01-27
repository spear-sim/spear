//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SimulationController/SphereAgent.h"

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <Camera/CameraActor.h>
#include <Components/SceneCaptureComponent2D.h>
#include <Components/StaticMeshComponent.h>
#include <Delegates/IDelegateInstance.h>
#include <Engine/CollisionProfile.h>
#include <Engine/EngineBaseTypes.h>
#include <Engine/StaticMesh.h>
#include <Engine/StaticMeshActor.h>
#include <Engine/World.h>
#include <EngineUtils.h>
#include <GameFramework/Actor.h>
#include <Materials/Material.h>
#include <Math/Rotator.h>
#include <UObject/UObjectGlobals.h>

#include "CoreUtils/Assert.h"
#include "CoreUtils/Config.h"
#include "CoreUtils/Std.h"
#include "CoreUtils/Unreal.h"
#include "SimulationController/Box.h"
#include "SimulationController/CameraSensor.h"
#include "SimulationController/TickEvent.h"

SphereAgent::SphereAgent(UWorld* world)
{
    // spawn sphere_actor
    FActorSpawnParameters actor_spawn_params;
    actor_spawn_params.Name = Unreal::toFName(Config::get<std::string>("SIMULATION_CONTROLLER.SPHERE_AGENT.SPHERE.ACTOR_NAME"));
    actor_spawn_params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    sphere_actor_ = world->SpawnActor<AStaticMeshActor>(FVector::ZeroVector, FRotator::ZeroRotator, actor_spawn_params);
    ASSERT(sphere_actor_);

    sphere_actor_->SetMobility(EComponentMobility::Type::Movable);

    static_mesh_component_ = sphere_actor_->GetStaticMeshComponent();
    ASSERT(static_mesh_component_);

    // load agent mesh and material
    UStaticMesh* sphere_mesh = LoadObject<UStaticMesh>(
        nullptr,
        *Unreal::toFString(Config::get<std::string>("SIMULATION_CONTROLLER.SPHERE_AGENT.SPHERE.STATIC_MESH")));
    ASSERT(sphere_mesh);
    UMaterial* sphere_material = LoadObject<UMaterial>(
        nullptr,
        *Unreal::toFString(Config::get<std::string>("SIMULATION_CONTROLLER.SPHERE_AGENT.SPHERE.MATERIAL")));
    ASSERT(sphere_material);
    
    static_mesh_component_->SetStaticMesh(sphere_mesh);
    static_mesh_component_->SetMaterial(0, sphere_material);
    sphere_actor_->SetActorScale3D(FVector(
        Config::get<float>("SIMULATION_CONTROLLER.SPHERE_AGENT.SPHERE.MESH_SCALE"),
        Config::get<float>("SIMULATION_CONTROLLER.SPHERE_AGENT.SPHERE.MESH_SCALE"),
        Config::get<float>("SIMULATION_CONTROLLER.SPHERE_AGENT.SPHERE.MESH_SCALE")));

    // set physics state
    static_mesh_component_->SetMobility(EComponentMobility::Type::Movable);
    static_mesh_component_->BodyInstance.SetCollisionProfileName(UCollisionProfile::PhysicsActor_ProfileName);
    static_mesh_component_->SetSimulatePhysics(true);
    static_mesh_component_->SetAngularDamping(Config::get<float>("SIMULATION_CONTROLLER.SPHERE_AGENT.SPHERE.ANGULAR_DAMPING"));
    static_mesh_component_->SetLinearDamping(Config::get<float>("SIMULATION_CONTROLLER.SPHERE_AGENT.SPHERE.LINEAR_DAMPING"));
    static_mesh_component_->BodyInstance.MaxAngularVelocity = Config::get<float>("SIMULATION_CONTROLLER.SPHERE_AGENT.SPHERE.MAX_ANGULAR_VELOCITY");
    static_mesh_component_->BodyInstance.MassScale = Config::get<float>("SIMULATION_CONTROLLER.SPHERE_AGENT.SPHERE.MASS_SCALE");
    static_mesh_component_->SetNotifyRigidBodyCollision(true);

    auto observation_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.SPHERE_AGENT.OBSERVATION_COMPONENTS");

    if (Std::contains(observation_components, "camera")) {
        FActorSpawnParameters camera_spawn_params;
        camera_spawn_params.Name = Unreal::toFName(Config::get<std::string>("SIMULATION_CONTROLLER.SPHERE_AGENT.CAMERA.ACTOR_NAME"));
        camera_spawn_params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
        camera_actor_ = world->SpawnActor<ACameraActor>(FVector::ZeroVector, FRotator::ZeroRotator, camera_spawn_params);
        ASSERT(camera_actor_);

        camera_sensor_ = std::make_unique<CameraSensor>(
            camera_actor_->GetCameraComponent(),
            Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.SPHERE_AGENT.CAMERA.RENDER_PASSES"),
            Config::get<unsigned int>("SIMULATION_CONTROLLER.SPHERE_AGENT.CAMERA.IMAGE_WIDTH"),
            Config::get<unsigned int>("SIMULATION_CONTROLLER.SPHERE_AGENT.CAMERA.IMAGE_HEIGHT"));
        ASSERT(camera_sensor_);

        // update FOV
        for (auto& pass : Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.SPHERE_AGENT.CAMERA.RENDER_PASSES")) {
            camera_sensor_->render_passes_.at(pass).scene_capture_component_->FOVAngle =
                Config::get<float>("SIMULATION_CONTROLLER.SPHERE_AGENT.CAMERA.FOV");
        }

        parent_actor_ = world->SpawnActor<AActor>();
        ASSERT(parent_actor_);

        tick_event_ = NewObject<UTickEvent>(parent_actor_);
        ASSERT(tick_event_);
        tick_event_->RegisterComponent();
        tick_event_->PrimaryComponentTick.TickGroup = ETickingGroup::TG_PostPhysics;
        tick_event_handle_ = tick_event_->delegate_.AddRaw(this, &SphereAgent::postPhysicsPreRenderTickEventHandler);
    }
}

SphereAgent::~SphereAgent()
{
    auto observation_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.SPHERE_AGENT.OBSERVATION_COMPONENTS");

    if (Std::contains(observation_components, "camera")) {        
        ASSERT(tick_event_);
        tick_event_->delegate_.Remove(tick_event_handle_);
        tick_event_handle_.Reset();
        tick_event_->DestroyComponent();
        tick_event_ = nullptr;

        ASSERT(parent_actor_);
        parent_actor_->Destroy();
        parent_actor_ = nullptr;

        ASSERT(camera_sensor_);
        camera_sensor_ = nullptr;

        ASSERT(camera_actor_);
        camera_actor_->Destroy();
        camera_actor_ = nullptr;
    }

    ASSERT(static_mesh_component_);
    static_mesh_component_ = nullptr;

    ASSERT(sphere_actor_);
    sphere_actor_->Destroy();
    sphere_actor_ = nullptr;
}

void SphereAgent::findObjectReferences(UWorld* world)
{
    auto observation_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.SPHERE_AGENT.OBSERVATION_COMPONENTS");

    if (Std::contains(observation_components, "compass")) {
        goal_actor_ = Unreal::findActorByName(world, Config::get<std::string>("SIMULATION_CONTROLLER.SPHERE_AGENT.COMPASS.GOAL_ACTOR_NAME"));
        ASSERT(goal_actor_);
    }
}

void SphereAgent::cleanUpObjectReferences()
{
    auto observation_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.SPHERE_AGENT.OBSERVATION_COMPONENTS");

    if (Std::contains(observation_components, "compass")) {
        ASSERT(goal_actor_);
        goal_actor_ = nullptr;
    }
}

std::map<std::string, Box> SphereAgent::getActionSpace() const
{
    std::map<std::string, Box> action_space;
    auto action_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.SPHERE_AGENT.ACTION_COMPONENTS");

    if (Std::contains(action_components, "apply_force")) {
        Box box;
        box.low_ = -1.f;
        box.high_ = 1.f;
        box.shape_ = {2};
        box.datatype_ = DataType::Float32;
        action_space["apply_force"] = std::move(box);
    }

    return action_space;
}

std::map<std::string, Box> SphereAgent::getObservationSpace() const
{
    std::map<std::string, Box> observation_space;
    auto observation_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.SPHERE_AGENT.OBSERVATION_COMPONENTS");

    if (Std::contains(observation_components, "compass")) {
        Box box;
        box.low_ = std::numeric_limits<float>::lowest();
        box.high_ = std::numeric_limits<float>::max();
        box.shape_ = {5};
        box.datatype_ = DataType::Float32;
        observation_space["compass"] = std::move(box);
    }

    std::map<std::string, Box> camera_sensor_observation_space = camera_sensor_->getObservationSpace(observation_components);
    for (auto& camera_sensor_observation_space_component : camera_sensor_observation_space) {
        observation_space[camera_sensor_observation_space_component.first] = std::move(camera_sensor_observation_space_component.second);
    }

    return observation_space;
}

std::map<std::string, Box> SphereAgent::getStepInfoSpace() const
{
    std::map<std::string, Box> step_info_space;
    auto step_info_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.SPHERE_AGENT.STEP_INFO_COMPONENTS");

    if (Std::contains(step_info_components, "debug_info")) {
        Box box;
        box.low_ = std::numeric_limits<float>::lowest();
        box.high_ = std::numeric_limits<float>::max();
        box.shape_ = {-1,3};
        box.datatype_ = DataType::Float32;
        step_info_space["debug_info"] = std::move(box);
    }

    return step_info_space;
}

void SphereAgent::applyAction(const std::map<std::string, std::vector<uint8_t>>& action)
{
    auto action_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.SPHERE_AGENT.ACTION_COMPONENTS");

    if (Std::contains(action_components, "apply_force")) {

        // get component data
        std::vector<float> component_data = Std::reinterpret_as<float>(action.at("apply_force"));

        // apply force to the sphere in the current yaw direction
        FVector force = rotation_.RotateVector(
            FVector(component_data.at(0), 0.0f, 0.0f)) * Config::get<float>("SIMULATION_CONTROLLER.SPHERE_AGENT.APPLY_FORCE.FORCE_SCALE");

        ASSERT(isfinite(force.X));
        ASSERT(isfinite(force.Y));
        ASSERT(isfinite(force.Z));

        static_mesh_component_->AddForce(force);

        // increment the current yaw direction
        rotation_.Add(0.0f, component_data.at(1) * Config::get<float>("SIMULATION_CONTROLLER.SPHERE_AGENT.APPLY_FORCE.ROTATE_SCALE"), 0.0f);

        ASSERT(isfinite(rotation_.Pitch));
        ASSERT(isfinite(rotation_.Yaw));
        ASSERT(isfinite(rotation_.Roll));
        ASSERT(rotation_.Pitch >= -360.0f && rotation_.Pitch <= 360.0f, "%f", rotation_.Pitch);
        ASSERT(rotation_.Yaw   >= -360.0f && rotation_.Yaw   <= 360.0f, "%f", rotation_.Yaw);
        ASSERT(rotation_.Roll  >= -360.0f && rotation_.Roll  <= 360.0f, "%f", rotation_.Roll);
    }
}

std::map<std::string, std::vector<uint8_t>> SphereAgent::getObservation() const
{
    std::map<std::string, std::vector<uint8_t>> observation;

    auto observation_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.SPHERE_AGENT.OBSERVATION_COMPONENTS");

    if (Std::contains(observation_components, "compass")) {
        FVector sphere_to_goal = goal_actor_->GetActorLocation() - sphere_actor_->GetActorLocation();
        FVector linear_velocity = static_mesh_component_->GetPhysicsLinearVelocity();
        float yaw = rotation_.Yaw;

        observation["compass"] = Std::reinterpret_as<uint8_t>(std::vector<float>{
            Config::get<float>("SIMULATION_CONTROLLER.SPHERE_AGENT.COMPASS.OFFSET_TO_GOAL_SCALE") * sphere_to_goal.X,
            Config::get<float>("SIMULATION_CONTROLLER.SPHERE_AGENT.COMPASS.OFFSET_TO_GOAL_SCALE") * sphere_to_goal.Y,
            Config::get<float>("SIMULATION_CONTROLLER.SPHERE_AGENT.COMPASS.LINEAR_VELOCITY_SCALE") * linear_velocity.X,
            Config::get<float>("SIMULATION_CONTROLLER.SPHERE_AGENT.COMPASS.LINEAR_VELOCITY_SCALE") * linear_velocity.Y,
            Config::get<float>("SIMULATION_CONTROLLER.SPHERE_AGENT.COMPASS.YAW_SCALE") * yaw});
    }

    std::map<std::string, std::vector<uint8_t>> camera_sensor_observation = camera_sensor_->getObservation(observation_components);
    for (auto& camera_sensor_observation_component : camera_sensor_observation) {
        observation[camera_sensor_observation_component.first] = std::move(camera_sensor_observation_component.second);
    }

    return observation;
}

std::map<std::string, std::vector<uint8_t>> SphereAgent::getStepInfo() const
{
    std::map<std::string, std::vector<uint8_t>> step_info;

    auto step_info_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.SPHERE_AGENT.STEP_INFO_COMPONENTS");

    if (Std::contains(step_info_components, "debug_info")) {
        step_info["debug_info"] = {};
    }

    return step_info;
}

void SphereAgent::reset()
{
    static_mesh_component_->SetPhysicsLinearVelocity(FVector::ZeroVector, false);
    static_mesh_component_->SetPhysicsAngularVelocityInRadians(FVector::ZeroVector, false);
    static_mesh_component_->GetBodyInstance()->ClearTorques();
    static_mesh_component_->GetBodyInstance()->ClearForces();

    rotation_ = FRotator::ZeroRotator;
}

bool SphereAgent::isReady() const
{
    return true;
}

void SphereAgent::postPhysicsPreRenderTickEventHandler(float delta_time, ELevelTick level_tick)
{
    std::map<std::string, std::vector<uint8_t>> observation;

    auto observation_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.SPHERE_AGENT.OBSERVATION_COMPONENTS");

    if (Std::contains(observation_components, "camera")) {
        camera_actor_->SetActorLocationAndRotation(
            sphere_actor_->GetActorLocation() +
            FVector(Config::get<float>("SIMULATION_CONTROLLER.SPHERE_AGENT.CAMERA.POSITION_OFFSET_X"),
                    Config::get<float>("SIMULATION_CONTROLLER.SPHERE_AGENT.CAMERA.POSITION_OFFSET_Y"),
                    Config::get<float>("SIMULATION_CONTROLLER.SPHERE_AGENT.CAMERA.POSITION_OFFSET_Z")),
            rotation_);
    }
}
