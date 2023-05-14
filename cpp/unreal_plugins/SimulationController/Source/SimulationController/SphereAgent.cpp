//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SimulationController/SphereAgent.h"

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <Camera/CameraActor.h>
#include <Components/StaticMeshComponent.h>
#include <Delegates/IDelegateInstance.h>
#include <Engine/CollisionProfile.h>
#include <Engine/EngineBaseTypes.h>
#include <Engine/StaticMesh.h>
#include <Engine/StaticMeshActor.h>
#include <Engine/World.h>
#include <GameFramework/Actor.h>
#include <Materials/Material.h>
#include <Math/Rotator.h>
#include <Math/Vector.h>

#include "CoreUtils/ArrayDesc.h"
#include "CoreUtils/Assert.h"
#include "CoreUtils/Config.h"
#include "CoreUtils/Std.h"
#include "CoreUtils/Unreal.h"
#include "SimulationController/CameraSensor.h"
#include "SimulationController/TickEventComponent.h"

SphereAgent::SphereAgent(UWorld* world)
{
    // spawn actor
    FVector spawn_location = FVector::ZeroVector;
    FRotator spawn_rotation = FRotator::ZeroRotator;
    std::string spawn_mode = Config::get<std::string>("SIMULATION_CONTROLLER.SPHERE_AGENT.SPAWN_MODE");
    if (spawn_mode == "specify_existing_actor") {
        AActor* spawn_actor = Unreal::findActorByName(world, Config::get<std::string>("SIMULATION_CONTROLLER.SPHERE_AGENT.SPAWN_ACTOR_NAME"));
        SP_ASSERT(spawn_actor);
        spawn_location = spawn_actor->GetActorLocation();
        spawn_rotation = spawn_actor->GetActorRotation();
    } else if (spawn_mode == "specify_pose") {
        spawn_location = FVector(
            Config::get<double>("SIMULATION_CONTROLLER.SPHERE_AGENT.SPAWN_POSITION_X"),
            Config::get<double>("SIMULATION_CONTROLLER.SPHERE_AGENT.SPAWN_POSITION_Y"),
            Config::get<double>("SIMULATION_CONTROLLER.SPHERE_AGENT.SPAWN_POSITION_Z"));
        spawn_rotation = FRotator(
            Config::get<double>("SIMULATION_CONTROLLER.SPHERE_AGENT.SPAWN_PITCH"),
            Config::get<double>("SIMULATION_CONTROLLER.SPHERE_AGENT.SPAWN_YAW"),
            Config::get<double>("SIMULATION_CONTROLLER.SPHERE_AGENT.SPAWN_ROLL"));
    } else {
        SP_ASSERT(false);
    }
    FActorSpawnParameters actor_spawn_params;
    actor_spawn_params.Name = Unreal::toFName(Config::get<std::string>("SIMULATION_CONTROLLER.SPHERE_AGENT.SPHERE_ACTOR_NAME"));
    actor_spawn_params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    static_mesh_actor_ = world->SpawnActor<AStaticMeshActor>(spawn_location, spawn_rotation, actor_spawn_params);
    SP_ASSERT(static_mesh_actor_);

    static_mesh_actor_->SetMobility(EComponentMobility::Type::Movable);

    FVector scale =
        FVector(
            Config::get<double>("SIMULATION_CONTROLLER.SPHERE_AGENT.SPHERE.MESH_SCALE"),
            Config::get<double>("SIMULATION_CONTROLLER.SPHERE_AGENT.SPHERE.MESH_SCALE"),
            Config::get<double>("SIMULATION_CONTROLLER.SPHERE_AGENT.SPHERE.MESH_SCALE"));
    static_mesh_actor_->SetActorScale3D(scale);

    // load mesh and material
    UStaticMesh* sphere_mesh = LoadObject<UStaticMesh>(
        nullptr,
        *Unreal::toFString(Config::get<std::string>("SIMULATION_CONTROLLER.SPHERE_AGENT.SPHERE.STATIC_MESH")));
    SP_ASSERT(sphere_mesh);
    UMaterial* sphere_material = LoadObject<UMaterial>(
        nullptr,
        *Unreal::toFString(Config::get<std::string>("SIMULATION_CONTROLLER.SPHERE_AGENT.SPHERE.MATERIAL")));
    SP_ASSERT(sphere_material);

    // configure static mesh component
    static_mesh_component_ = static_mesh_actor_->GetStaticMeshComponent();
    SP_ASSERT(static_mesh_component_);
    
    static_mesh_component_->SetStaticMesh(sphere_mesh);
    static_mesh_component_->SetMaterial(0, sphere_material);
    static_mesh_component_->SetMobility(EComponentMobility::Type::Movable);
    static_mesh_component_->BodyInstance.SetCollisionProfileName(UCollisionProfile::PhysicsActor_ProfileName);
    static_mesh_component_->SetSimulatePhysics(true);
    static_mesh_component_->SetNotifyRigidBodyCollision(true);

    auto observation_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.SPHERE_AGENT.OBSERVATION_COMPONENTS");

    // set up camera
    if (Std::contains(observation_components, "camera")) {
        FActorSpawnParameters camera_spawn_params;
        camera_spawn_params.Name = Unreal::toFName(Config::get<std::string>("SIMULATION_CONTROLLER.SPHERE_AGENT.CAMERA_ACTOR_NAME"));
        camera_spawn_params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
        camera_actor_ = world->SpawnActor<ACameraActor>(FVector::ZeroVector, FRotator::ZeroRotator, camera_spawn_params);
        SP_ASSERT(camera_actor_);

        camera_sensor_ = std::make_unique<CameraSensor>(
            camera_actor_->GetCameraComponent(),
            Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.SPHERE_AGENT.CAMERA.RENDER_PASSES"),
            Config::get<unsigned int>("SIMULATION_CONTROLLER.SPHERE_AGENT.CAMERA.IMAGE_WIDTH"),
            Config::get<unsigned int>("SIMULATION_CONTROLLER.SPHERE_AGENT.CAMERA.IMAGE_HEIGHT"),
            Config::get<float>("SIMULATION_CONTROLLER.SPHERE_AGENT.CAMERA.FOV"));
        SP_ASSERT(camera_sensor_);

        parent_actor_ = world->SpawnActor<AActor>();
        SP_ASSERT(parent_actor_);

        tick_event_component_ = NewObject<UTickEventComponent>(parent_actor_);
        SP_ASSERT(tick_event_component_);
        tick_event_component_->RegisterComponent();
        tick_event_component_->PrimaryComponentTick.TickGroup = ETickingGroup::TG_PostPhysics;
        tick_event_handle_ = tick_event_component_->delegate_.AddRaw(this, &SphereAgent::postPhysicsPreRenderTickEventHandler);
    }
}

SphereAgent::~SphereAgent()
{
    auto observation_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.SPHERE_AGENT.OBSERVATION_COMPONENTS");

    if (Std::contains(observation_components, "camera")) {        
        SP_ASSERT(tick_event_component_);
        tick_event_component_->delegate_.Remove(tick_event_handle_);
        tick_event_handle_.Reset();
        tick_event_component_->DestroyComponent();
        tick_event_component_ = nullptr;

        SP_ASSERT(parent_actor_);
        parent_actor_->Destroy();
        parent_actor_ = nullptr;

        SP_ASSERT(camera_sensor_);
        camera_sensor_ = nullptr;

        SP_ASSERT(camera_actor_);
        camera_actor_->Destroy();
        camera_actor_ = nullptr;
    }

    SP_ASSERT(static_mesh_component_);
    static_mesh_component_ = nullptr;

    SP_ASSERT(static_mesh_actor_);
    static_mesh_actor_->Destroy();
    static_mesh_actor_ = nullptr;
}

void SphereAgent::findObjectReferences(UWorld* world)
{
    auto observation_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.SPHERE_AGENT.OBSERVATION_COMPONENTS");
}

void SphereAgent::cleanUpObjectReferences()
{
    auto observation_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.SPHERE_AGENT.OBSERVATION_COMPONENTS");
}

std::map<std::string, ArrayDesc> SphereAgent::getActionSpace() const
{
    std::map<std::string, ArrayDesc> action_space;
    auto action_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.SPHERE_AGENT.ACTION_COMPONENTS");

    if (Std::contains(action_components, "add_rotation")) {
        ArrayDesc array_desc;
        array_desc.low_ = std::numeric_limits<double>::lowest();
        array_desc.high_ = std::numeric_limits<double>::max();
        array_desc.shape_ = {3};
        array_desc.datatype_ = DataType::Float64;
        action_space["add_rotation"] = std::move(array_desc);
    }

    if (Std::contains(action_components, "add_force")) {
        ArrayDesc array_desc;
        array_desc.low_ = std::numeric_limits<double>::lowest();
        array_desc.high_ = std::numeric_limits<double>::max();
        array_desc.shape_ = {3};
        array_desc.datatype_ = DataType::Float64;
        action_space["add_force"] = std::move(array_desc);
    }

    return action_space;
}

std::map<std::string, ArrayDesc> SphereAgent::getObservationSpace() const
{
    std::map<std::string, ArrayDesc> observation_space;
    auto observation_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.SPHERE_AGENT.OBSERVATION_COMPONENTS");

    if (Std::contains(observation_components, "position")) {
        ArrayDesc array_desc;
        array_desc.low_ = std::numeric_limits<double>::lowest();
        array_desc.high_ = std::numeric_limits<double>::max();
        array_desc.shape_ = {3};
        array_desc.datatype_ = DataType::Float64;
        observation_space["position"] = std::move(array_desc);
    }

    if (Std::contains(observation_components, "rotation")) {
        ArrayDesc array_desc;
        array_desc.low_ = std::numeric_limits<double>::lowest();
        array_desc.high_ = std::numeric_limits<double>::max();
        array_desc.shape_ = {3};
        array_desc.datatype_ = DataType::Float64;
        observation_space["rotation"] = std::move(array_desc);
    }

    std::map<std::string, ArrayDesc> camera_sensor_observation_space = camera_sensor_->getObservationSpace(observation_components);
    for (auto& camera_sensor_observation_space_component : camera_sensor_observation_space) {
        observation_space[camera_sensor_observation_space_component.first] = std::move(camera_sensor_observation_space_component.second);
    }

    return observation_space;
}

std::map<std::string, ArrayDesc> SphereAgent::getStepInfoSpace() const
{
    std::map<std::string, ArrayDesc> step_info_space;
    auto step_info_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.SPHERE_AGENT.STEP_INFO_COMPONENTS");

    if (Std::contains(step_info_components, "debug")) {
        ArrayDesc array_desc;
        array_desc.low_ = std::numeric_limits<double>::lowest();
        array_desc.high_ = std::numeric_limits<double>::max();
        array_desc.shape_ = {-1, 3};
        array_desc.datatype_ = DataType::Float64;
        step_info_space["debug"] = std::move(array_desc);
    }

    return step_info_space;
}

void SphereAgent::applyAction(const std::map<std::string, std::vector<uint8_t>>& action)
{
    auto action_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.SPHERE_AGENT.ACTION_COMPONENTS");

    if (Std::contains(action_components, "add_rotation")) {
        std::vector<double> component_data = Std::reinterpretAs<double>(action.at("add_rotation"));
        rotation_.Add(component_data.at(0), component_data.at(1), component_data.at(2));
    }

    if (Std::contains(action_components, "add_force")) {
        std::vector<double> component_data = Std::reinterpretAs<double>(action.at("add_force"));
        FVector force = rotation_.RotateVector(FVector(component_data.at(0), component_data.at(1), component_data.at(2)));
        static_mesh_component_->AddForce(force);
    }
}

std::map<std::string, std::vector<uint8_t>> SphereAgent::getObservation() const
{
    std::map<std::string, std::vector<uint8_t>> observation;
    auto observation_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.SPHERE_AGENT.OBSERVATION_COMPONENTS");

    if (Std::contains(observation_components, "position")) {
        FVector location = static_mesh_actor_->GetActorLocation();
        observation["position"] = Std::reinterpretAs<uint8_t>(std::vector<double>{location.X, location.Y, location.Z});
    }

    if (Std::contains(observation_components, "rotation")) {
        observation["rotation"] = Std::reinterpretAs<uint8_t>(std::vector<double>{rotation_.Pitch, rotation_.Yaw, rotation_.Roll}); 
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

    if (Std::contains(step_info_components, "debug")) {
        step_info["debug"] = Std::reinterpretAs<uint8_t>(std::vector<double>{0.0, 1.0, 2.0, 3.0, 4.0, 5.0});
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
        camera_actor_->SetActorLocationAndRotation(static_mesh_actor_->GetActorLocation(), rotation_);
    }
}
