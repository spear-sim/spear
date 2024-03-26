//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpEngine/Legacy/SphereAgent.h"

#include <stdint.h> // uint8_t

#include <map>
#include <memory> // std::make_unique
#include <string>
#include <utility> // std::move
#include <vector>

#include <Camera/CameraActor.h>
#include <Camera/CameraComponent.h> // UCameraComponent::AspectRatio, UCameraComponent::FieldOfView
#include <Components/StaticMeshComponent.h>
#include <Engine/CollisionProfile.h>
#include <Engine/EngineBaseTypes.h> // ELevelTick, ETickingGroup
#include <Engine/StaticMesh.h>
#include <Engine/StaticMeshActor.h>
#include <Engine/World.h>           // FActorSpawnParameters
#include <GameFramework/Actor.h>
#include <Materials/Material.h>
#include <Math/Rotator.h>
#include <Math/Vector.h>
#include <UObject/UObjectGlobals.h> // LoadObject

#include "SpCore/ArrayDesc.h" // DataType
#include "SpCore/Assert.h"
#include "SpCore/Config.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"
#include "SpEngine/Legacy/CameraSensor.h"
#include "SpEngine/Legacy/StandaloneComponent.h"
#include "SpEngine/Legacy/TickEventComponent.h"

struct FActorComponentTickFunction;

SphereAgent::SphereAgent(UWorld* world)
{
    // Spawn static mesh actor
    FVector spawn_location = FVector::ZeroVector;
    FRotator spawn_rotation = FRotator::ZeroRotator;
    std::string spawn_mode = Config::get<std::string>("SP_ENGINE.LEGACY.SPHERE_AGENT.SPAWN_MODE");
    if (spawn_mode == "specify_existing_actor") {
        AActor* spawn_actor = Unreal::findActorByName(world, Config::get<std::string>("SP_ENGINE.LEGACY.SPHERE_AGENT.SPAWN_ACTOR_NAME"));
        SP_ASSERT(spawn_actor);
        spawn_location = spawn_actor->GetActorLocation();
        spawn_rotation = spawn_actor->GetActorRotation();
    } else if (spawn_mode == "specify_pose") {
        spawn_location = FVector(
            Config::get<double>("SP_ENGINE.LEGACY.SPHERE_AGENT.SPAWN_LOCATION_X"),
            Config::get<double>("SP_ENGINE.LEGACY.SPHERE_AGENT.SPAWN_LOCATION_Y"),
            Config::get<double>("SP_ENGINE.LEGACY.SPHERE_AGENT.SPAWN_LOCATION_Z"));
        spawn_rotation = FRotator(
            Config::get<double>("SP_ENGINE.LEGACY.SPHERE_AGENT.SPAWN_ROTATION_PITCH"),
            Config::get<double>("SP_ENGINE.LEGACY.SPHERE_AGENT.SPAWN_ROTATION_YAW"),
            Config::get<double>("SP_ENGINE.LEGACY.SPHERE_AGENT.SPAWN_ROTATION_ROLL"));
    } else {
        SP_ASSERT(false);
    }
    FActorSpawnParameters static_mesh_actor_spawn_parameters;
    static_mesh_actor_spawn_parameters.Name = Unreal::toFName(Config::get<std::string>("SP_ENGINE.LEGACY.SPHERE_AGENT.SPHERE_ACTOR_NAME"));
    static_mesh_actor_spawn_parameters.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    static_mesh_actor_ = world->SpawnActor<AStaticMeshActor>(spawn_location, spawn_rotation, static_mesh_actor_spawn_parameters);
    SP_ASSERT(static_mesh_actor_);

    static_mesh_actor_->SetMobility(EComponentMobility::Type::Movable);

    FVector scale =
        FVector(
            Config::get<double>("SP_ENGINE.LEGACY.SPHERE_AGENT.SPHERE.MESH_SCALE_X"),
            Config::get<double>("SP_ENGINE.LEGACY.SPHERE_AGENT.SPHERE.MESH_SCALE_Y"),
            Config::get<double>("SP_ENGINE.LEGACY.SPHERE_AGENT.SPHERE.MESH_SCALE_Z"));
    static_mesh_actor_->SetActorScale3D(scale);

    // Load mesh and material
    auto sphere_mesh = LoadObject<UStaticMesh>(nullptr, *Unreal::toFString(Config::get<std::string>("SP_ENGINE.LEGACY.SPHERE_AGENT.SPHERE.STATIC_MESH")));
    SP_ASSERT(sphere_mesh);
    auto sphere_material = LoadObject<UMaterial>(nullptr, *Unreal::toFString(Config::get<std::string>("SP_ENGINE.LEGACY.SPHERE_AGENT.SPHERE.MATERIAL")));
    SP_ASSERT(sphere_material);

    // Configure static mesh component
    static_mesh_component_ = static_mesh_actor_->GetStaticMeshComponent();
    SP_ASSERT(static_mesh_component_);
    
    static_mesh_component_->SetStaticMesh(sphere_mesh);
    static_mesh_component_->SetMaterial(0, sphere_material);
    static_mesh_component_->SetMobility(EComponentMobility::Type::Movable);
    static_mesh_component_->BodyInstance.SetCollisionProfileName(UCollisionProfile::PhysicsActor_ProfileName);
    static_mesh_component_->SetSimulatePhysics(true);
    static_mesh_component_->SetNotifyRigidBodyCollision(true);

    // Spawn camera actor
    FActorSpawnParameters camera_actor_spawn_parameters;
    camera_actor_spawn_parameters.Name = Unreal::toFName(Config::get<std::string>("SP_ENGINE.LEGACY.SPHERE_AGENT.CAMERA_ACTOR_NAME"));
    camera_actor_spawn_parameters.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    camera_actor_ = world->SpawnActor<ACameraActor>(FVector::ZeroVector, FRotator::ZeroRotator, camera_actor_spawn_parameters);
    SP_ASSERT(camera_actor_);

    camera_actor_->GetCameraComponent()->FieldOfView =
        Config::get<float>("SP_ENGINE.LEGACY.SPHERE_AGENT.CAMERA.FOV");
    camera_actor_->GetCameraComponent()->AspectRatio =
        Config::get<float>("SP_ENGINE.LEGACY.SPHERE_AGENT.CAMERA.IMAGE_WIDTH") /
        Config::get<float>("SP_ENGINE.LEGACY.SPHERE_AGENT.CAMERA.IMAGE_HEIGHT");

    // Create UTickEventComponent
    tick_event_component_ = std::make_unique<StandaloneComponent<UTickEventComponent>>(world, "tick_event_component");
    SP_ASSERT(tick_event_component_);
    SP_ASSERT(tick_event_component_->component_);
    tick_event_component_->component_->PrimaryComponentTick.bCanEverTick = true;
    tick_event_component_->component_->PrimaryComponentTick.bTickEvenWhenPaused = false;
    tick_event_component_->component_->PrimaryComponentTick.TickGroup = ETickingGroup::TG_PostPhysics;
    tick_event_component_->component_->tick_func_ = [this](float delta_time, ELevelTick level_tick, FActorComponentTickFunction* this_tick_function) -> void {
        camera_actor_->SetActorLocationAndRotation(static_mesh_actor_->GetActorLocation(), rotation_);
    };

    auto observation_components = Config::get<std::vector<std::string>>("SP_ENGINE.LEGACY.SPHERE_AGENT.OBSERVATION_COMPONENTS");

    if (Std::contains(observation_components, "camera")) {
        camera_sensor_ = std::make_unique<CameraSensor>(
            camera_actor_->GetCameraComponent(),
            Config::get<std::vector<std::string>>("SP_ENGINE.LEGACY.SPHERE_AGENT.CAMERA.RENDER_PASSES"),
            Config::get<unsigned int>("SP_ENGINE.LEGACY.SPHERE_AGENT.CAMERA.IMAGE_WIDTH"),
            Config::get<unsigned int>("SP_ENGINE.LEGACY.SPHERE_AGENT.CAMERA.IMAGE_HEIGHT"),
            Config::get<float>("SP_ENGINE.LEGACY.SPHERE_AGENT.CAMERA.FOV"));
        SP_ASSERT(camera_sensor_);
    }
}

SphereAgent::~SphereAgent()
{
    auto observation_components = Config::get<std::vector<std::string>>("SP_ENGINE.LEGACY.SPHERE_AGENT.OBSERVATION_COMPONENTS");

    if (Std::contains(observation_components, "camera")) {        
        SP_ASSERT(camera_sensor_);
        camera_sensor_ = nullptr;
    }

    SP_ASSERT(tick_event_component_);
    SP_ASSERT(tick_event_component_->component_);
    tick_event_component_->component_->tick_func_ = nullptr;
    tick_event_component_ = nullptr;

    SP_ASSERT(camera_actor_);
    camera_actor_->Destroy();
    camera_actor_ = nullptr;

    SP_ASSERT(static_mesh_component_);
    static_mesh_component_ = nullptr;

    SP_ASSERT(static_mesh_actor_);
    static_mesh_actor_->Destroy();
    static_mesh_actor_ = nullptr;
}

void SphereAgent::findObjectReferences(UWorld* world) {}
void SphereAgent::cleanUpObjectReferences() {}

std::map<std::string, ArrayDesc> SphereAgent::getActionSpace() const
{
    std::map<std::string, ArrayDesc> action_space;
    auto action_components = Config::get<std::vector<std::string>>("SP_ENGINE.LEGACY.SPHERE_AGENT.ACTION_COMPONENTS");

    if (Std::contains(action_components, "add_force")) {
        ArrayDesc array_desc;
        array_desc.low_ = std::numeric_limits<double>::lowest();
        array_desc.high_ = std::numeric_limits<double>::max();
        array_desc.shape_ = {3};
        array_desc.datatype_ = DataType::Float64;
        Std::insert(action_space, "add_force", std::move(array_desc));
    }

    if (Std::contains(action_components, "add_to_rotation")) {
        ArrayDesc array_desc;
        array_desc.low_ = std::numeric_limits<double>::lowest();
        array_desc.high_ = std::numeric_limits<double>::max();
        array_desc.shape_ = {3};
        array_desc.datatype_ = DataType::Float64;
        Std::insert(action_space, "add_to_rotation", std::move(array_desc));
    }

    return action_space;
}

std::map<std::string, ArrayDesc> SphereAgent::getObservationSpace() const
{
    std::map<std::string, ArrayDesc> observation_space;
    auto observation_components = Config::get<std::vector<std::string>>("SP_ENGINE.LEGACY.SPHERE_AGENT.OBSERVATION_COMPONENTS");

    if (Std::contains(observation_components, "location")) {
        ArrayDesc array_desc;
        array_desc.low_ = std::numeric_limits<double>::lowest();
        array_desc.high_ = std::numeric_limits<double>::max();
        array_desc.shape_ = {3};
        array_desc.datatype_ = DataType::Float64;
        Std::insert(observation_space, "location", std::move(array_desc));
    }

    if (Std::contains(observation_components, "rotation")) {
        ArrayDesc array_desc;
        array_desc.low_ = std::numeric_limits<double>::lowest();
        array_desc.high_ = std::numeric_limits<double>::max();
        array_desc.shape_ = {3};
        array_desc.datatype_ = DataType::Float64;
        Std::insert(observation_space, "rotation", std::move(array_desc));
    }

    if (Std::contains(observation_components, "camera")) {
        Std::insert(observation_space, camera_sensor_->getObservationSpace());
    }

    return observation_space;
}

std::map<std::string, ArrayDesc> SphereAgent::getStepInfoSpace() const
{
    std::map<std::string, ArrayDesc> step_info_space;
    auto step_info_components = Config::get<std::vector<std::string>>("SP_ENGINE.LEGACY.SPHERE_AGENT.STEP_INFO_COMPONENTS");

    if (Std::contains(step_info_components, "debug")) {
        ArrayDesc array_desc;
        array_desc.low_ = std::numeric_limits<double>::lowest();
        array_desc.high_ = std::numeric_limits<double>::max();
        array_desc.shape_ = {-1, 3};
        array_desc.datatype_ = DataType::Float64;
        Std::insert(step_info_space, "debug", std::move(array_desc));
    }

    return step_info_space;
}

void SphereAgent::applyAction(const std::map<std::string, std::vector<uint8_t>>& action)
{
    auto action_components = Config::get<std::vector<std::string>>("SP_ENGINE.LEGACY.SPHERE_AGENT.ACTION_COMPONENTS");

    if (Std::contains(action_components, "add_force")) {
        std::span<const double> action_component_data = Std::reinterpretAsSpanOf<const double>(action.at("add_force"));
        FVector force = rotation_.RotateVector({Std::at(action_component_data, 0), Std::at(action_component_data, 1), Std::at(action_component_data, 2)});
        static_mesh_component_->AddForce(force);
    }

    if (Std::contains(action_components, "add_to_rotation")) {
        std::span<const double> action_component_data = Std::reinterpretAsSpanOf<const double>(action.at("add_to_rotation"));
        rotation_.Add(Std::at(action_component_data, 0), Std::at(action_component_data, 1), Std::at(action_component_data, 2));
    }
}

std::map<std::string, std::vector<uint8_t>> SphereAgent::getObservation() const
{
    std::map<std::string, std::vector<uint8_t>> observation;
    auto observation_components = Config::get<std::vector<std::string>>("SP_ENGINE.LEGACY.SPHERE_AGENT.OBSERVATION_COMPONENTS");

    if (Std::contains(observation_components, "location")) {
        FVector location = static_mesh_actor_->GetActorLocation();
        Std::insert(observation, "location", Std::reinterpretAsVector<uint8_t, double>({location.X, location.Y, location.Z}));
    }

    if (Std::contains(observation_components, "rotation")) {
        Std::insert(observation, "rotation", Std::reinterpretAsVector<uint8_t, double>({rotation_.Pitch, rotation_.Yaw, rotation_.Roll})); 
    }

    if (Std::contains(observation_components, "camera")) {
        Std::insert(observation, camera_sensor_->getObservation());
    }

    return observation;
}

std::map<std::string, std::vector<uint8_t>> SphereAgent::getStepInfo() const
{
    std::map<std::string, std::vector<uint8_t>> step_info;
    auto step_info_components = Config::get<std::vector<std::string>>("SP_ENGINE.LEGACY.SPHERE_AGENT.STEP_INFO_COMPONENTS");

    if (Std::contains(step_info_components, "debug")) {
        Std::insert(step_info, "debug", Std::reinterpretAsVector<uint8_t, double>({0.0, 1.0, 2.0, 3.0, 4.0, 5.0}));
    }

    return step_info;
}

void SphereAgent::reset()
{
    static_mesh_component_->SetPhysicsLinearVelocity(FVector::ZeroVector);
    static_mesh_component_->SetPhysicsAngularVelocityInRadians(FVector::ZeroVector);
    static_mesh_component_->GetBodyInstance()->ClearTorques();
    static_mesh_component_->GetBodyInstance()->ClearForces();

    rotation_ = FRotator::ZeroRotator;
}

bool SphereAgent::isReady() const
{
    return true;
}
