//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SimulationController/CameraAgent.h"

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <AI/NavDataGenerator.h>
#include <Camera/CameraActor.h>
#include <Components/SceneCaptureComponent2D.h>
#include <Engine/World.h>
#include <EngineUtils.h>
#include <GameFramework/Actor.h>
#include <NavigationSystem.h>
#include <NavMesh/NavMeshBoundsVolume.h>
#include <NavMesh/RecastNavMesh.h>
#include <NavModifierVolume.h>

#include "CoreUtils/Assert.h"
#include "CoreUtils/Box.h"
#include "CoreUtils/Config.h"
#include "CoreUtils/Std.h"
#include "CoreUtils/Unreal.h"
#include "SimulationController/CameraSensor.h"

CameraAgent::CameraAgent(UWorld* world)
{
    FVector spawn_location = FVector::ZeroVector;
    FRotator spawn_rotation = FRotator::ZeroRotator;
    std::string spawn_mode = Config::get<std::string>("SIMULATION_CONTROLLER.CAMERA_AGENT.SPAWN_MODE");
    if (spawn_mode == "specify_existing_actor") {
        AActor* spawn_actor = Unreal::findActorByName(world, Config::get<std::string>("SIMULATION_CONTROLLER.CAMERA_AGENT.SPAWN_ACTOR_NAME"));
        ASSERT(spawn_actor);
        spawn_location = spawn_actor->GetActorLocation();
        spawn_rotation = spawn_actor->GetActorRotation();
    } else if (spawn_mode == "specify_pose") {
        spawn_location = FVector(
            Config::get<float>("SIMULATION_CONTROLLER.CAMERA_AGENT.SPAWN_POSITION_X"),
            Config::get<float>("SIMULATION_CONTROLLER.CAMERA_AGENT.SPAWN_POSITION_Y"),
            Config::get<float>("SIMULATION_CONTROLLER.CAMERA_AGENT.SPAWN_POSITION_Z"));
        spawn_rotation = FRotator(
            Config::get<float>("SIMULATION_CONTROLLER.CAMERA_AGENT.SPAWN_PITCH"),
            Config::get<float>("SIMULATION_CONTROLLER.CAMERA_AGENT.SPAWN_YAW"),
            Config::get<float>("SIMULATION_CONTROLLER.CAMERA_AGENT.SPAWN_ROLL"));
    } else {
        ASSERT(false);
    }
    FActorSpawnParameters actor_spawn_params;
    actor_spawn_params.Name = Unreal::toFName(Config::get<std::string>("SIMULATION_CONTROLLER.CAMERA_AGENT.CAMERA_ACTOR_NAME"));
    actor_spawn_params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    camera_actor_ = world->SpawnActor<ACameraActor>(spawn_location, spawn_rotation, actor_spawn_params);
    ASSERT(camera_actor_);

    auto observation_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.CAMERA_AGENT.OBSERVATION_COMPONENTS");

    if (Std::contains(observation_components, "camera")) {
        camera_sensor_ = std::make_unique<CameraSensor>(
            camera_actor_->GetCameraComponent(),
            Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.CAMERA_AGENT.CAMERA.RENDER_PASSES"),
            Config::get<unsigned int>("SIMULATION_CONTROLLER.CAMERA_AGENT.CAMERA.IMAGE_WIDTH"),
            Config::get<unsigned int>("SIMULATION_CONTROLLER.CAMERA_AGENT.CAMERA.IMAGE_HEIGHT"));
        ASSERT(camera_sensor_);

        // update FOV
        for (auto& pass : Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.CAMERA_AGENT.CAMERA.RENDER_PASSES")) {
            camera_sensor_->render_passes_.at(pass).scene_capture_component_->FOVAngle = Config::get<float>("SIMULATION_CONTROLLER.CAMERA_AGENT.CAMERA.FOV");
        }
    }
}

CameraAgent::~CameraAgent()
{
    auto observation_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.CAMERA_AGENT.OBSERVATION_COMPONENTS");

    if (Std::contains(observation_components, "camera")) {
        ASSERT(camera_sensor_);
        camera_sensor_ = nullptr;
    }

    ASSERT(camera_actor_);
    camera_actor_->Destroy();
    camera_actor_ = nullptr;
}

void CameraAgent::findObjectReferences(UWorld* world)
{
    auto step_info_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.CAMERA_AGENT.STEP_INFO_COMPONENTS");

    if (Std::contains(step_info_components, "random_points")) {
        nav_sys_ = FNavigationSystem::GetCurrent<UNavigationSystemV1>(world);
        ASSERT(nav_sys_);

        FNavAgentProperties agent_properties;
        agent_properties.AgentHeight     = Config::get<float>("SIMULATION_CONTROLLER.CAMERA_AGENT.NAVMESH.AGENT_HEIGHT");
        agent_properties.AgentRadius     = Config::get<float>("SIMULATION_CONTROLLER.CAMERA_AGENT.NAVMESH.AGENT_RADIUS");
        agent_properties.AgentStepHeight = Config::get<float>("SIMULATION_CONTROLLER.CAMERA_AGENT.NAVMESH.AGENT_MAX_STEP_HEIGHT");

        ANavigationData* nav_data = nav_sys_->GetNavDataForProps(agent_properties);
        ASSERT(nav_data);

        nav_mesh_ = dynamic_cast<ARecastNavMesh*>(nav_data);
        ASSERT(nav_mesh_);

        buildNavMesh();
    }
}

void CameraAgent::cleanUpObjectReferences()
{
    auto step_info_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.CAMERA_AGENT.STEP_INFO_COMPONENTS");

    if (Std::contains(step_info_components, "random_points")) {
        nav_mesh_ = nullptr;
    }
}

std::map<std::string, Box> CameraAgent::getActionSpace() const
{
    std::map<std::string, Box> action_space;
    auto action_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.CAMERA_AGENT.ACTION_COMPONENTS");

    if (Std::contains(action_components, "set_pose")) {
        Box box;
        box.low_ = std::numeric_limits<float>::lowest();
        box.high_ = std::numeric_limits<float>::max();
        box.shape_ = {6}; // x,y,z in cms and then p,y,r in degs
        box.datatype_ = DataType::Float32;
        action_space["set_pose"] = std::move(box);
    }

    if (Std::contains(action_components, "set_num_random_points")) {
        Box box;
        box.low_ = std::numeric_limits<uint32_t>::lowest();
        box.high_ = std::numeric_limits<uint32_t>::max();
        box.shape_ = {1};
        box.datatype_ = DataType::UInteger32;
        action_space["set_num_random_points"] = std::move(box);
    }

    return action_space;
}

std::map<std::string, Box> CameraAgent::getObservationSpace() const
{
    std::map<std::string, Box> observation_space;
    auto observation_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.CAMERA_AGENT.OBSERVATION_COMPONENTS");

    std::map<std::string, Box> camera_sensor_observation_space = camera_sensor_->getObservationSpace(observation_components);
    for (auto& camera_sensor_observation_space_component : camera_sensor_observation_space) {
        observation_space[camera_sensor_observation_space_component.first] = std::move(camera_sensor_observation_space_component.second);
    }

    return observation_space;
}

std::map<std::string, Box> CameraAgent::getStepInfoSpace() const
{
    std::map<std::string, Box> step_info_space;
    auto step_info_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.CAMERA_AGENT.STEP_INFO_COMPONENTS");

    if (Std::contains(step_info_components, "random_points")) {
        Box box;
        box.low_ = std::numeric_limits<float>::lowest();
        box.high_ = std::numeric_limits<float>::max();
        box.shape_ = {-1, 3};
        box.datatype_ = DataType::Float32;
        step_info_space["random_points"] = std::move(box);
    }

    return step_info_space;
}

void CameraAgent::applyAction(const std::map<std::string, std::vector<uint8_t>>& action)
{
    auto action_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.CAMERA_AGENT.ACTION_COMPONENTS");

    if (Std::contains(action_components, "set_pose")) {
        std::vector<float> component_data = Std::reinterpret_as<float>(action.at("set_pose"));
        FVector agent_location(component_data.at(0), component_data.at(1), component_data.at(2));
        FRotator agent_rotation(component_data.at(3), component_data.at(4), component_data.at(5));
        bool sweep = false;
        FHitResult* hit_result = nullptr;
        camera_actor_->SetActorLocationAndRotation(agent_location, agent_rotation, sweep, hit_result, ETeleportType::ResetPhysics);
    }

    // store action because we might use it in getStepInfo(...)
    action_ = action;
}

std::map<std::string, std::vector<uint8_t>> CameraAgent::getObservation() const
{
    std::map<std::string, std::vector<uint8_t>> observation;
    auto observation_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.CAMERA_AGENT.OBSERVATION_COMPONENTS");

    std::map<std::string, std::vector<uint8_t>> camera_sensor_observation = camera_sensor_->getObservation(observation_components);
    for (auto& camera_sensor_observation_component : camera_sensor_observation) {
        observation[camera_sensor_observation_component.first] = std::move(camera_sensor_observation_component.second);
    }

    return observation;
}

std::map<std::string, std::vector<uint8_t>> CameraAgent::getStepInfo() const
{
    std::map<std::string, std::vector<uint8_t>> step_info;
    auto step_info_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.CAMERA_AGENT.STEP_INFO_COMPONENTS");

    if (Std::contains(step_info_components, "random_points")) {

        std::vector<uint32> component_data = Std::reinterpret_as<uint32>(action_.at("set_num_random_points"));
        std::vector<float> random_points;
        int num_random_points = static_cast<int>(component_data.at(0));
        for (int i = 0; i < num_random_points; i++) {
            FVector random_position = nav_mesh_->GetRandomPoint().Location;
            random_points.push_back(random_position.X);
            random_points.push_back(random_position.Y);
            random_points.push_back(random_position.Z);
        }
        step_info["random_points"] = Std::reinterpret_as<uint8_t>(random_points);
    }

    return step_info;
}

void CameraAgent::reset() {}

bool CameraAgent::isReady() const
{
    return true;
}

void CameraAgent::buildNavMesh()
{
    // set the navmesh properties
    nav_mesh_->AgentRadius            = Config::get<float>("SIMULATION_CONTROLLER.CAMERA_AGENT.NAVMESH.AGENT_RADIUS");
    nav_mesh_->AgentHeight            = Config::get<float>("SIMULATION_CONTROLLER.CAMERA_AGENT.NAVMESH.AGENT_HEIGHT");
    nav_mesh_->CellSize               = Config::get<float>("SIMULATION_CONTROLLER.CAMERA_AGENT.NAVMESH.CELL_SIZE");
    nav_mesh_->CellHeight             = Config::get<float>("SIMULATION_CONTROLLER.CAMERA_AGENT.NAVMESH.CELL_HEIGHT");
    nav_mesh_->AgentMaxStepHeight     = Config::get<float>("SIMULATION_CONTROLLER.CAMERA_AGENT.NAVMESH.AGENT_MAX_STEP_HEIGHT");
    nav_mesh_->AgentMaxSlope          = Config::get<float>("SIMULATION_CONTROLLER.CAMERA_AGENT.NAVMESH.AGENT_MAX_SLOPE");
    nav_mesh_->MergeRegionSize        = Config::get<float>("SIMULATION_CONTROLLER.CAMERA_AGENT.NAVMESH.MERGE_REGION_SIZE");
    nav_mesh_->MinRegionArea          = Config::get<float>("SIMULATION_CONTROLLER.CAMERA_AGENT.NAVMESH.MIN_REGION_AREA");
    nav_mesh_->TileSizeUU             = Config::get<float>("SIMULATION_CONTROLLER.CAMERA_AGENT.NAVMESH.TILE_SIZE_UU");
    nav_mesh_->TilePoolSize           = Config::get<float>("SIMULATION_CONTROLLER.CAMERA_AGENT.NAVMESH.TILE_POOL_SIZE");
    nav_mesh_->MaxSimplificationError = Config::get<float>("SIMULATION_CONTROLLER.CAMERA_AGENT.NAVMESH.MAX_SIMPLIFICATION_ERROR");

    // get bounds volume
    FBox bounds_volume(EForceInit::ForceInit);
    auto tags = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.CAMERA_AGENT.NAVMESH.BOUNDS_VOLUME_ACTOR_TAGS");
    for (auto& actor : Unreal::findActorsByTagAny(camera_actor_->GetWorld(), tags)) {
        bounds_volume += actor->GetComponentsBoundingBox(false, true);
    }

    // get references to ANavMeshBoundsVolume and ANavModifierVolume
    ANavMeshBoundsVolume* nav_mesh_bounds_volume = Unreal::findActorByType<ANavMeshBoundsVolume>(camera_actor_->GetWorld());
    ASSERT(nav_mesh_bounds_volume);

    ANavModifierVolume* nav_modifier_volume = Unreal::findActorByType<ANavModifierVolume>(camera_actor_->GetWorld());
    ASSERT(nav_modifier_volume);

    // update ANavMeshBoundsVolume
    nav_mesh_bounds_volume->GetRootComponent()->SetMobility(EComponentMobility::Movable);
    nav_mesh_bounds_volume->SetActorLocation(bounds_volume.GetCenter(), false);
    nav_mesh_bounds_volume->SetActorRelativeScale3D(bounds_volume.GetSize() / 200.0f);
    nav_mesh_bounds_volume->GetRootComponent()->UpdateBounds();
    nav_sys_->OnNavigationBoundsUpdated(nav_mesh_bounds_volume);
    nav_mesh_bounds_volume->GetRootComponent()->SetMobility(EComponentMobility::Static);

    // update ANavModifierVolume
    nav_modifier_volume->GetRootComponent()->SetMobility(EComponentMobility::Movable);
    nav_modifier_volume->SetActorLocation(bounds_volume.GetCenter(), false);
    nav_modifier_volume->SetActorRelativeScale3D(bounds_volume.GetSize() / 200.0f);
    nav_modifier_volume->AddActorWorldOffset(FVector(
        Config::get<float>("SIMULATION_CONTROLLER.CAMERA_AGENT.NAVMESH.NAV_MODIFIER_OFFSET_X"),
        Config::get<float>("SIMULATION_CONTROLLER.CAMERA_AGENT.NAVMESH.NAV_MODIFIER_OFFSET_Y"),
        Config::get<float>("SIMULATION_CONTROLLER.CAMERA_AGENT.NAVMESH.NAV_MODIFIER_OFFSET_Z")));
    nav_modifier_volume->GetRootComponent()->UpdateBounds();
    nav_modifier_volume->GetRootComponent()->SetMobility(EComponentMobility::Static);
    nav_modifier_volume->RebuildNavigationData();

    // rebuild navmesh
    nav_sys_->Build();

    // We need to wrap this call with guards because ExportNavigationData(...) is only implemented in non-shipping builds, see:
    //     Engine/Source/Runtime/Engine/Public/AI/NavDataGenerator.h
    //     Engine/Source/Runtime/NavigationSystem/Public/NavMesh/RecastNavMeshGenerator.h
    //     Engine/Source/Runtime/NavigationSystem/Private/NavMesh/RecastNavMeshGenerator.cpp
    #if !(UE_BUILD_SHIPPING || UE_BUILD_TEST)
        if (Config::get<bool>("SIMULATION_CONTROLLER.CAMERA_AGENT.NAVMESH.EXPORT_NAV_DATA_OBJ")) {
            nav_mesh_->GetGenerator()->ExportNavigationData(FPaths::Combine(
                Unreal::toFString(Config::get<std::string>("SIMULATION_CONTROLLER.CAMERA_AGENT.NAVMESH.EXPORT_NAV_DATA_OBJ_DIR")),
                camera_actor_->GetWorld()->GetName()));
        }
    #endif
}
