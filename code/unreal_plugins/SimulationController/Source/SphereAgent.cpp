#include "SphereAgent.h"

#include <algorithm>

#include <Camera/CameraActor.h>
#include <Components/SceneCaptureComponent2D.h>
#include <Components/StaticMeshComponent.h>
#include <Engine/CollisionProfile.h>
#include <Engine/StaticMesh.h>
#include <Engine/StaticMeshActor.h>
#include <Engine/World.h>
#include <EngineUtils.h>
#include <GameFramework/Actor.h>
#include <Materials/Material.h>
#include <UObject/UObjectGlobals.h>

#include "Assert/Assert.h"
#include "Box.h"
#include "CameraSensor.h"
#include "Config.h"
#include "Serialize.h"
#include "TickEvent.h"

SphereAgent::SphereAgent(UWorld* world)
{
    // spawn sphere_actor
    FActorSpawnParameters sphere_spawn_params;
    sphere_spawn_params.Name = FName(Config::getValue<std::string>({"SIMULATION_CONTROLLER", "SPHERE_AGENT", "SPHERE", "ACTOR_NAME"}).c_str());
    sphere_spawn_params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

    sphere_actor_ = world->SpawnActor<AStaticMeshActor>(AStaticMeshActor::StaticClass(), FVector::ZeroVector, FRotator::ZeroRotator, sphere_spawn_params);
    ASSERT(sphere_actor_);

    sphere_actor_->SetMobility(EComponentMobility::Type::Movable);

    sphere_static_mesh_component_ = sphere_actor_->GetStaticMeshComponent();
    ASSERT(sphere_static_mesh_component_);

    // load agent mesh and material
    UStaticMesh* sphere_mesh   = LoadObject<UStaticMesh>(nullptr, UTF8_TO_TCHAR(Config::getValue<std::string>({"SIMULATION_CONTROLLER", "SPHERE_AGENT", "SPHERE", "STATIC_MESH"}).c_str()));
    ASSERT(sphere_mesh);
    UMaterial* sphere_material = LoadObject<UMaterial>  (nullptr, UTF8_TO_TCHAR(Config::getValue<std::string>({"SIMULATION_CONTROLLER", "SPHERE_AGENT", "SPHERE", "MATERIAL"}).c_str()));
    ASSERT(sphere_material);
    
    sphere_static_mesh_component_->SetStaticMesh(sphere_mesh);
    sphere_static_mesh_component_->SetMaterial(0, sphere_material);
    sphere_actor_->SetActorScale3D(FVector(Config::getValue<float>({"SIMULATION_CONTROLLER", "SPHERE_AGENT", "SPHERE", "MESH_SCALE"}),
                                           Config::getValue<float>({"SIMULATION_CONTROLLER", "SPHERE_AGENT", "SPHERE", "MESH_SCALE"}),
                                           Config::getValue<float>({"SIMULATION_CONTROLLER", "SPHERE_AGENT", "SPHERE", "MESH_SCALE"})));

    // set physics state
    sphere_static_mesh_component_->SetMobility(EComponentMobility::Type::Movable);
    sphere_static_mesh_component_->BodyInstance.SetCollisionProfileName(UCollisionProfile::PhysicsActor_ProfileName);
    sphere_static_mesh_component_->SetSimulatePhysics(true);
    sphere_static_mesh_component_->SetAngularDamping(Config::getValue<float>({"SIMULATION_CONTROLLER", "SPHERE_AGENT", "SPHERE", "ANGULAR_DAMPING"}));
    sphere_static_mesh_component_->SetLinearDamping(Config::getValue<float>({"SIMULATION_CONTROLLER", "SPHERE_AGENT", "SPHERE", "LINEAR_DAMPING"}));
    sphere_static_mesh_component_->BodyInstance.MaxAngularVelocity = Config::getValue<float>({"SIMULATION_CONTROLLER", "SPHERE_AGENT", "SPHERE", "MAX_ANGULAR_VELOCITY"});
    sphere_static_mesh_component_->BodyInstance.MassScale = Config::getValue<float>({"SIMULATION_CONTROLLER", "SPHERE_AGENT", "SPHERE", "MASS_SCALE"});
    sphere_static_mesh_component_->SetNotifyRigidBodyCollision(true);

    auto observation_components = Config::getValue<std::vector<std::string>>({"SIMULATION_CONTROLLER", "SPHERE_AGENT", "OBSERVATION_COMPONENTS"});

    //
    // observation["camera"]
    //
    if (std::find(observation_components.begin(), observation_components.end(), "camera") != observation_components.end()) {
        FActorSpawnParameters camera_spawn_params;
        camera_spawn_params.Name = FName(Config::getValue<std::string>({"SIMULATION_CONTROLLER", "SPHERE_AGENT", "CAMERA", "ACTOR_NAME"}).c_str());
        camera_spawn_params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

        camera_actor_ = world->SpawnActor<ACameraActor>(FVector::ZeroVector, FRotator::ZeroRotator, camera_spawn_params);
        ASSERT(camera_actor_);

        camera_sensor_ = std::make_unique<CameraSensor>(
            camera_actor_->GetCameraComponent(),
            Config::getValue<std::vector<std::string>>({"SIMULATION_CONTROLLER", "SPHERE_AGENT", "CAMERA", "RENDER_PASSES"}),
            Config::getValue<unsigned int>({"SIMULATION_CONTROLLER", "SPHERE_AGENT", "CAMERA", "IMAGE_WIDTH"}),
            Config::getValue<unsigned int>({"SIMULATION_CONTROLLER", "SPHERE_AGENT", "CAMERA", "IMAGE_HEIGHT"}));
        ASSERT(camera_sensor_);

        // update FOV
        for (auto& pass : Config::getValue<std::vector<std::string>>({"SIMULATION_CONTROLLER", "SPHERE_AGENT", "CAMERA", "RENDER_PASSES"})) {
            camera_sensor_->render_passes_.at(pass).scene_capture_component_->FOVAngle = Config::getValue<float>({"SIMULATION_CONTROLLER", "SPHERE_AGENT", "CAMERA", "FOV"});
        }

        new_object_parent_actor_ = world->SpawnActor<AActor>();
        ASSERT(new_object_parent_actor_);

        tick_event_ = NewObject<UTickEvent>(new_object_parent_actor_, TEXT("TickEvent"));
        ASSERT(tick_event_);
        tick_event_->RegisterComponent();
        tick_event_->initialize(ETickingGroup::TG_PostPhysics);
        tick_event_handle_ = tick_event_->delegate_.AddRaw(this, &SphereAgent::postPhysicsPreRenderTickEventHandler);
    }
}

SphereAgent::~SphereAgent()
{
    auto observation_components = Config::getValue<std::vector<std::string>>({"SIMULATION_CONTROLLER", "SPHERE_AGENT", "OBSERVATION_COMPONENTS"});

    //
    // observation["camera"]
    //
    if (std::find(observation_components.begin(), observation_components.end(), "camera") != observation_components.end()) {        
        ASSERT(tick_event_);
        tick_event_->delegate_.Remove(tick_event_handle_);
        tick_event_handle_.Reset();
        tick_event_->DestroyComponent();
        tick_event_ = nullptr;

        ASSERT(new_object_parent_actor_);
        new_object_parent_actor_->Destroy();
        new_object_parent_actor_ = nullptr;

        ASSERT(camera_sensor_);
        camera_sensor_ = nullptr;

        ASSERT(camera_actor_);
        camera_actor_->Destroy();
        camera_actor_ = nullptr;
    }

    ASSERT(sphere_actor_);
    sphere_actor_->Destroy();
    sphere_actor_ = nullptr;
}

void SphereAgent::findObjectReferences(UWorld* world)
{
    auto observation_components = Config::getValue<std::vector<std::string>>({"SIMULATION_CONTROLLER", "SPHERE_AGENT", "OBSERVATION_COMPONENTS"});

    //
    // observation["compass"]
    //
    if (std::find(observation_components.begin(), observation_components.end(), "compass") != observation_components.end()) {
        for (TActorIterator<AActor> actor_itr(world); actor_itr; ++actor_itr) {
            std::string actor_name = TCHAR_TO_UTF8(*((*actor_itr)->GetName()));
            if (actor_name == Config::getValue<std::string>({"SIMULATION_CONTROLLER", "SPHERE_AGENT", "COMPASS", "GOAL_ACTOR_NAME"})) {
                ASSERT(!goal_actor_);
                goal_actor_ = *actor_itr;
                break;
            }
        }
        ASSERT(goal_actor_);
    }
}

void SphereAgent::cleanUpObjectReferences()
{
    auto observation_components = Config::getValue<std::vector<std::string>>({"SIMULATION_CONTROLLER", "SPHERE_AGENT", "OBSERVATION_COMPONENTS"});

    //
    // observation["compass"]
    //
    if (std::find(observation_components.begin(), observation_components.end(), "compass") != observation_components.end()) {
        ASSERT(goal_actor_);
        goal_actor_ = nullptr;
    }
}

std::map<std::string, Box> SphereAgent::getActionSpace() const
{
    std::map<std::string, Box> action_space;
    Box box;

    auto action_components = Config::getValue<std::vector<std::string>>({"SIMULATION_CONTROLLER", "SPHERE_AGENT", "ACTION_COMPONENTS"});

    //
    // action["apply_force"]
    //
    if (std::find(action_components.begin(), action_components.end(), "apply_force") != action_components.end()) {
        box.low = -1.f;
        box.high = 1.f;
        box.shape = {2};
        box.dtype = DataType::Float32;
        action_space["apply_force"] = std::move(box);
    }

    return action_space;
}

std::map<std::string, Box> SphereAgent::getObservationSpace() const
{
    std::map<std::string, Box> observation_space;
    Box box;

    auto observation_components = Config::getValue<std::vector<std::string>>({"SIMULATION_CONTROLLER", "SPHERE_AGENT", "OBSERVATION_COMPONENTS"});

    //
    // observation["compass"]
    //
    if (std::find(observation_components.begin(), observation_components.end(), "compass") != observation_components.end()) {
        box.low = std::numeric_limits<float>::lowest();
        box.high = std::numeric_limits<float>::max();
        box.shape = {5};
        box.dtype = DataType::Float32;
        observation_space["compass"] = std::move(box);
    }

    //
    // observation["camera"]
    //
    if (std::find(observation_components.begin(), observation_components.end(), "camera") != observation_components.end()) {

        // get an observation space from the CameraSensor and add it to our Agent's observation space
        std::map<std::string, Box> camera_sensor_observation_space = camera_sensor_->getObservationSpace();
        for (auto& camera_sensor_observation_space_component : camera_sensor_observation_space) {
            observation_space["camera_" + camera_sensor_observation_space_component.first] = std::move(camera_sensor_observation_space_component.second);
        }
    }

    return observation_space;
}

std::map<std::string, Box> SphereAgent::getStepInfoSpace() const
{
    std::map<std::string, Box> step_info_space;
    Box box;

    auto step_info_components = Config::getValue<std::vector<std::string>>({"SIMULATION_CONTROLLER", "SPHERE_AGENT", "STEP_INFO_COMPONENTS"});

    //
    // step_info["debug_info"]
    //
    if (std::find(step_info_components.begin(), step_info_components.end(), "debug_info") != step_info_components.end()) {
        box.low = std::numeric_limits<float>::lowest();
        box.high = std::numeric_limits<float>::max();
        box.shape = {-1,3};
        box.dtype = DataType::Float32;
        step_info_space["debug_info"] = std::move(box);
    }

    return step_info_space;
}

void SphereAgent::applyAction(const std::map<std::string, std::vector<float>>& action)
{
    auto action_components = Config::getValue<std::vector<std::string>>({"SIMULATION_CONTROLLER", "SPHERE_AGENT", "ACTION_COMPONENTS"});

    //
    // action["apply_force"]
    //
    if (std::find(action_components.begin(), action_components.end(), "apply_force") != action_components.end()) {

        // Get yaw from the camera, apply force to the sphere in that direction
        FVector force = camera_actor_->GetActorRotation().RotateVector(FVector(action.at("apply_force").at(0), 0.0f, 0.0f)) * Config::getValue<float>({"SIMULATION_CONTROLLER", "SPHERE_AGENT", "APPLY_FORCE", "FORCE_SCALE"});

        ASSERT(isfinite(force.X));
        ASSERT(isfinite(force.Y));
        ASSERT(isfinite(force.Z));

        sphere_static_mesh_component_->AddForce(force);

        // Set camera yaw by adding to the current camera yaw
        FRotator rotation = camera_actor_->GetActorRotation().Add(0.0f, action.at("apply_force").at(1) * Config::getValue<float>({"SIMULATION_CONTROLLER", "SPHERE_AGENT", "APPLY_FORCE", "ROTATE_SCALE"}), 0.0f);

        ASSERT(isfinite(rotation.Pitch));
        ASSERT(isfinite(rotation.Yaw));
        ASSERT(isfinite(rotation.Roll));
        ASSERT(rotation.Pitch >= -360.0 && rotation.Pitch <= 360.0, "%f", rotation.Pitch);
        ASSERT(rotation.Yaw   >= -360.0 && rotation.Yaw   <= 360.0, "%f", rotation.Yaw);
        ASSERT(rotation.Roll  >= -360.0 && rotation.Roll  <= 360.0, "%f", rotation.Roll);

        camera_actor_->SetActorRotation(rotation);
    }
}

std::map<std::string, std::vector<uint8_t>> SphereAgent::getObservation() const
{
    std::map<std::string, std::vector<uint8_t>> observation;

    auto observation_components = Config::getValue<std::vector<std::string>>({"SIMULATION_CONTROLLER", "SPHERE_AGENT", "OBSERVATION_COMPONENTS"});

    //
    // observation["compass"]
    //
    if (std::find(observation_components.begin(), observation_components.end(), "compass") != observation_components.end()) {

        FVector sphere_to_goal = goal_actor_->GetActorLocation() - sphere_actor_->GetActorLocation();
        FVector linear_velocity = sphere_static_mesh_component_->GetPhysicsLinearVelocity();
        float observation_camera_yaw = camera_actor_->GetActorRotation().Yaw;

        observation["compass"] = Serialize::toUint8(std::vector<float>{
            Config::getValue<float>({"SIMULATION_CONTROLLER", "SPHERE_AGENT", "COMPASS", "OFFSET_TO_GOAL_SCALE"}) * sphere_to_goal.X,
            Config::getValue<float>({"SIMULATION_CONTROLLER", "SPHERE_AGENT", "COMPASS", "OFFSET_TO_GOAL_SCALE"}) * sphere_to_goal.Y,
            Config::getValue<float>({"SIMULATION_CONTROLLER", "SPHERE_AGENT", "COMPASS", "LINEAR_VELOCITY_SCALE"}) * linear_velocity.X,
            Config::getValue<float>({"SIMULATION_CONTROLLER", "SPHERE_AGENT", "COMPASS", "LINEAR_VELOCITY_SCALE"}) * linear_velocity.Y,
            Config::getValue<float>({"SIMULATION_CONTROLLER", "SPHERE_AGENT", "COMPASS", "YAW_SCALE"}) * observation_camera_yaw});
    }

    //
    // observation["camera"]
    //
    if (std::find(observation_components.begin(), observation_components.end(), "camera") != observation_components.end()) {

        // get an observation from the CameraSensor and add it to our Agent's observation
        std::map<std::string, std::vector<uint8_t>> camera_sensor_observation = camera_sensor_->getObservation();
        for (auto& camera_sensor_observation_component : camera_sensor_observation) {
            observation["camera_" + camera_sensor_observation_component.first] = std::move(camera_sensor_observation_component.second);
        }
    }

    return observation;
}

std::map<std::string, std::vector<uint8_t>> SphereAgent::getStepInfo() const
{
    std::map<std::string, std::vector<uint8_t>> step_info;

    auto step_info_components = Config::getValue<std::vector<std::string>>({"SIMULATION_CONTROLLER", "SPHERE_AGENT", "STEP_INFO_COMPONENTS"});

    if (std::find(step_info_components.begin(), step_info_components.end(), "debug_info") != step_info_components.end()) {
        step_info["debug_info"] = {};
    }

    return step_info;
}

void SphereAgent::reset()
{
    sphere_static_mesh_component_->SetPhysicsLinearVelocity(FVector::ZeroVector, false);
    sphere_static_mesh_component_->SetPhysicsAngularVelocityInRadians(FVector::ZeroVector, false);
    sphere_static_mesh_component_->GetBodyInstance()->ClearTorques();
    sphere_static_mesh_component_->GetBodyInstance()->ClearForces();
}

bool SphereAgent::isReady() const
{
    return true;
}

void SphereAgent::postPhysicsPreRenderTickEventHandler(float delta_time, ELevelTick level_tick)
{
    camera_actor_->SetActorLocation(
        sphere_actor_->GetActorLocation() +
        FVector(Config::getValue<float>({"SIMULATION_CONTROLLER", "SPHERE_AGENT", "CAMERA", "POSITION_OFFSET_X"}),
                Config::getValue<float>({"SIMULATION_CONTROLLER", "SPHERE_AGENT", "CAMERA", "POSITION_OFFSET_Y"}),
                Config::getValue<float>({"SIMULATION_CONTROLLER", "SPHERE_AGENT", "CAMERA", "POSITION_OFFSET_Z"})));
}
