////------ BEGIN UE5 MIGRATION ------////
//// Uncomment this file when OpenBot is supported in UE5.
/*
//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SimulationController/OpenBotAgent.h"

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <AI/NavDataGenerator.h>
#include <Components/BoxComponent.h>
#include <Components/PrimitiveComponent.h>
#include <Components/SceneCaptureComponent2D.h>
#include <Components/StaticMeshComponent.h>
#include <Engine/World.h>
#include <EngineUtils.h>
#include <GameFramework/Actor.h>
#include <NavigationSystem.h>
#include <NavMesh/NavMeshBoundsVolume.h>
#include <NavMesh/RecastNavMesh.h>
#include <NavModifierVolume.h>

#include "CoreUtils/ArrayDesc.h"
#include "CoreUtils/Assert.h"
#include "CoreUtils/Config.h"
#include "CoreUtils/Std.h"
#include "CoreUtils/Unreal.h"
#include "OpenBot/OpenBotPawn.h"
#include "SimulationController/CameraSensor.h"
#include "SimulationController/ImuSensor.h"
#include "SimulationController/SonarSensor.h"

OpenBotAgent::OpenBotAgent(UWorld* world)
{
    FVector spawn_location = FVector::ZeroVector;
    FRotator spawn_rotation = FRotator::ZeroRotator;
    std::string spawn_mode = Config::get<std::string>("SIMULATION_CONTROLLER.OPENBOT_AGENT.SPAWN_MODE");
    if (spawn_mode == "specify_existing_actor") {
        AActor* spawn_actor = Unreal::findActorByName(world, Config::get<std::string>("SIMULATION_CONTROLLER.OPENBOT_AGENT.SPAWN_ACTOR_NAME"));
        ASSERT(spawn_actor);
        spawn_location = spawn_actor->GetActorLocation();
        spawn_rotation = spawn_actor->GetActorRotation();
    } else if (spawn_mode == "specify_pose") {
        spawn_location = FVector(
            Config::get<float>("SIMULATION_CONTROLLER.OPENBOT_AGENT.SPAWN_POSITION_X"),
            Config::get<float>("SIMULATION_CONTROLLER.OPENBOT_AGENT.SPAWN_POSITION_Y"),
            Config::get<float>("SIMULATION_CONTROLLER.OPENBOT_AGENT.SPAWN_POSITION_Z"));
        spawn_rotation = FRotator(
            Config::get<float>("SIMULATION_CONTROLLER.OPENBOT_AGENT.SPAWN_PITCH"),
            Config::get<float>("SIMULATION_CONTROLLER.OPENBOT_AGENT.SPAWN_YAW"),
            Config::get<float>("SIMULATION_CONTROLLER.OPENBOT_AGENT.SPAWN_ROLL"));
    } else {
        ASSERT(false);
    }
    FActorSpawnParameters actor_spawn_params;
    actor_spawn_params.Name = Unreal::toFName(Config::get<std::string>("SIMULATION_CONTROLLER.OPENBOT_AGENT.OPENBOT_ACTOR_NAME"));
    actor_spawn_params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    open_bot_pawn_ = world->SpawnActor<AOpenBotPawn>(spawn_location, spawn_rotation, actor_spawn_params);
    ASSERT(open_bot_pawn_);

    auto observation_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.OPENBOT_AGENT.OBSERVATION_COMPONENTS");

    if (Std::contains(observation_components, "camera")) {
        camera_sensor_ = std::make_unique<CameraSensor>(
            open_bot_pawn_->camera_component_,
            Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.OPENBOT_AGENT.CAMERA.RENDER_PASSES"),
            Config::get<unsigned int>("SIMULATION_CONTROLLER.OPENBOT_AGENT.CAMERA.IMAGE_WIDTH"),
            Config::get<unsigned int>("SIMULATION_CONTROLLER.OPENBOT_AGENT.CAMERA.IMAGE_HEIGHT"));
        ASSERT(camera_sensor_);

        // update FOV
        for (auto& pass : Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.OPENBOT_AGENT.CAMERA.RENDER_PASSES")) {
            camera_sensor_->render_passes_.at(pass).scene_capture_component_->FOVAngle =
                Config::get<float>("SIMULATION_CONTROLLER.OPENBOT_AGENT.CAMERA.FOV");
        }
    }

    if (Std::contains(observation_components, "imu")) {
        imu_sensor_ = std::make_unique<ImuSensor>(open_bot_pawn_->imu_component_);
        ASSERT(imu_sensor_);
    }
    
    if (Std::contains(observation_components, "sonar")) {
        sonar_sensor_ = std::make_unique<SonarSensor>(open_bot_pawn_->sonar_component_);
        ASSERT(sonar_sensor_);
    }
}

OpenBotAgent::~OpenBotAgent()
{
    auto observation_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.OPENBOT_AGENT.OBSERVATION_COMPONENTS");

    if (Std::contains(observation_components, "sonar")) {
        ASSERT(sonar_sensor_);
        sonar_sensor_ = nullptr;
    }

    if (Std::contains(observation_components, "imu")) {
        ASSERT(imu_sensor_);
        imu_sensor_ = nullptr;
    }

    if (Std::contains(observation_components, "camera")) {
        ASSERT(camera_sensor_);
        camera_sensor_ = nullptr;
    }

    ASSERT(open_bot_pawn_);
    open_bot_pawn_->Destroy();
    open_bot_pawn_ = nullptr;
}

void OpenBotAgent::findObjectReferences(UWorld* world)
{
    auto step_info_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.OPENBOT_AGENT.STEP_INFO_COMPONENTS");

    if (Std::contains(step_info_components, "trajectory_data")) {
        goal_actor_ = Unreal::findActorByName(world, Config::get<std::string>("SIMULATION_CONTROLLER.OPENBOT_AGENT.TRAJECTORY_DATA.GOAL_ACTOR_NAME"));
        ASSERT(goal_actor_);

        nav_sys_ = FNavigationSystem::GetCurrent<UNavigationSystemV1>(world);
        ASSERT(nav_sys_);

        FNavAgentProperties agent_properties;
        agent_properties.AgentHeight     = Config::get<float>("SIMULATION_CONTROLLER.OPENBOT_AGENT.NAVMESH.AGENT_HEIGHT");
        agent_properties.AgentRadius     = Config::get<float>("SIMULATION_CONTROLLER.OPENBOT_AGENT.NAVMESH.AGENT_RADIUS");
        agent_properties.AgentStepHeight = Config::get<float>("SIMULATION_CONTROLLER.OPENBOT_AGENT.NAVMESH.AGENT_MAX_STEP_HEIGHT");

        ANavigationData* nav_data = nav_sys_->GetNavDataForProps(agent_properties);
        ASSERT(nav_data);

        nav_mesh_ = dynamic_cast<ARecastNavMesh*>(nav_data);
        ASSERT(nav_mesh_);

        buildNavMesh();
    }
}

void OpenBotAgent::cleanUpObjectReferences()
{
    auto step_info_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.OPENBOT_AGENT.STEP_INFO_COMPONENTS");

    if (Std::contains(step_info_components, "trajectory_data")) {
        ASSERT(nav_mesh_);
        nav_mesh_ = nullptr;

        ASSERT(nav_sys_);
        nav_sys_ = nullptr;

        ASSERT(goal_actor_);
        goal_actor_ = nullptr;
    }
}

std::map<std::string, ArrayDesc> OpenBotAgent::getActionSpace() const
{
    
    std::map<std::string, ArrayDesc> action_space;
    auto action_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.OPENBOT_AGENT.ACTION_COMPONENTS");

    if (Std::contains(action_components, "apply_voltage")) {
        ArrayDesc array_desc;
        array_desc.low_ = -1.f;
        array_desc.high_ = 1.f;
        array_desc.shape_ = {2};
        array_desc.datatype_ = DataType::Float32;
        action_space["apply_voltage"] = std::move(array_desc);
    }

    if (Std::contains(action_components, "set_position_xyz_centimeters")) {
        ArrayDesc array_desc;
        array_desc.low_ = std::numeric_limits<float>::lowest();
        array_desc.high_ = std::numeric_limits<float>::max();
        array_desc.shape_ = {3};
        array_desc.datatype_ = DataType::Float32;
        action_space["set_position_xyz_centimeters"] = std::move(array_desc);
    }

    if (Std::contains(action_components, "set_orientation_pyr_radians")) {
        ArrayDesc array_desc;
        array_desc.low_ = std::numeric_limits<float>::lowest();
        array_desc.high_ = std::numeric_limits<float>::max();
        array_desc.shape_ = {3};
        array_desc.datatype_ = DataType::Float32;
        action_space["set_orientation_pyr_radians"] = std::move(array_desc);
    }

    return action_space;
}

std::map<std::string, ArrayDesc> OpenBotAgent::getObservationSpace() const
{
    std::map<std::string, ArrayDesc> observation_space;
    auto observation_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.OPENBOT_AGENT.OBSERVATION_COMPONENTS");

    if (Std::contains(observation_components, "state_data")) {
        ArrayDesc array_desc;
        array_desc.low_ = std::numeric_limits<float>::lowest();
        array_desc.high_ = std::numeric_limits<float>::max();
        array_desc.datatype_ = DataType::Float32;
        array_desc.shape_ = {6};
        observation_space["state_data"] = std::move(array_desc); // position (X, Y, Z) and orientation (Roll, Pitch, Yaw) of the agent relative to the world frame.
    }

    if (Std::contains(observation_components, "control_data")) {
        ArrayDesc array_desc;
        array_desc.low_ = std::numeric_limits<float>::lowest();
        array_desc.high_ = std::numeric_limits<float>::max();
        array_desc.datatype_ = DataType::Float32;
        array_desc.shape_ = {2};
        observation_space["control_data"] = std::move(array_desc); // ctrl_left, ctrl_right
    }

    if (Std::contains(observation_components, "encoder")) {
        ArrayDesc array_desc;
        array_desc.low_ = std::numeric_limits<float>::lowest();
        array_desc.high_ = std::numeric_limits<float>::max();
        array_desc.datatype_ = DataType::Float32;
        array_desc.shape_ = {4};
        observation_space["encoder"] = std::move(array_desc); // FL, FR, RL, RR
    }

    if (Std::contains(observation_components, "imu")) {
        ArrayDesc array_desc;
        array_desc.low_ = std::numeric_limits<float>::lowest();
        array_desc.high_ = std::numeric_limits<float>::max();
        array_desc.datatype_ = DataType::Float32;
        array_desc.shape_ = {6};
        observation_space["imu"] = std::move(array_desc); // a_x, a_y, a_z, g_x, g_y, g_z
    }

    if (Std::contains(observation_components, "sonar")) {
        ArrayDesc array_desc;
        array_desc.low_ = std::numeric_limits<float>::lowest();
        array_desc.high_ = std::numeric_limits<float>::max();
        array_desc.datatype_ = DataType::Float32;
        array_desc.shape_ = {1};
        observation_space["sonar"] = std::move(array_desc); // Front obstacle distance in [m]
    }
    
    std::map<std::string, ArrayDesc> camera_sensor_observation_space = camera_sensor_->getObservationSpace(observation_components);
    for (auto& camera_sensor_observation_space_component : camera_sensor_observation_space) {
        observation_space[camera_sensor_observation_space_component.first] = std::move(camera_sensor_observation_space_component.second);
    }

    return observation_space;
}

std::map<std::string, ArrayDesc> OpenBotAgent::getStepInfoSpace() const
{
    std::map<std::string, ArrayDesc> step_info_space;
    auto step_info_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.OPENBOT_AGENT.STEP_INFO_COMPONENTS");

    if (Std::contains(step_info_components, "trajectory_data")) {
        ArrayDesc array_desc;
        array_desc.low_ = std::numeric_limits<float>::lowest();
        array_desc.high_ = std::numeric_limits<float>::max();
        array_desc.datatype_ = DataType::Float32;
        array_desc.shape_ = {-1, 3};
        step_info_space["trajectory_data"] = std::move(array_desc); // Vector of the waypoints x,y,z in the world frame.
    }

    return step_info_space;
}

void OpenBotAgent::applyAction(const std::map<std::string, std::vector<uint8_t>>& action)
{
    auto action_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.OPENBOT_AGENT.ACTION_COMPONENTS");

    if (Std::contains(action_components, "apply_voltage")) {
        std::vector<float> component_data = Std::reinterpret_as<float>(action.at("apply_voltage"));
        open_bot_pawn_->setDutyCycle(Eigen::Vector4f(
            component_data.at(0), component_data.at(1), component_data.at(0), component_data.at(1)));
        open_bot_pawn_->setBrakeTorques(Eigen::Vector4f(0.0f, 0.0f, 0.0f, 0.0f));
    }

    if (Std::contains(action_components, "set_position_xyz_centimeters")) {
        std::vector<float> component_data = Std::reinterpret_as<float>(action.at("set_position_xyz_centimeters"));
        FVector location(component_data.at(0), component_data.at(1), component_data.at(2));
        bool sweep = false;
        FHitResult* hit_result = nullptr;
        open_bot_pawn_->SetActorLocation(location, sweep, hit_result, ETeleportType::TeleportPhysics);
        open_bot_pawn_->setBrakeTorques(Eigen::Vector4f(1000.0f, 1000.0f, 1000.0f, 1000.0f)); // TODO: get value from the config system
    }

    if (Std::contains(action_components, "set_orientation_pyr_radians")) {
        std::vector<float> component_data = Std::reinterpret_as<float>(action.at("set_orientation_pyr_radians"));
        FRotator rotation(
            FMath::RadiansToDegrees(component_data.at(0)),
            FMath::RadiansToDegrees(component_data.at(1)),
            FMath::RadiansToDegrees(component_data.at(2)));
        open_bot_pawn_->SetActorRotation(rotation, ETeleportType::TeleportPhysics);
        open_bot_pawn_->setBrakeTorques(Eigen::Vector4f(1000.0f, 1000.0f, 1000.0f, 1000.0f)); // TODO: get value from the config system
    }
}

std::map<std::string, std::vector<uint8_t>> OpenBotAgent::getObservation() const
{
    std::map<std::string, std::vector<uint8_t>> observation;

    auto observation_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.OPENBOT_AGENT.OBSERVATION_COMPONENTS");

    if (Std::contains(observation_components, "state_data")) {
        FVector location = open_bot_pawn_->GetActorLocation();
        FRotator rotation = open_bot_pawn_->GetActorRotation();
        observation["state_data"] = Std::reinterpret_as<uint8_t>(std::vector<float>{
            location.X,
            location.Y,
            location.Z,
            FMath::DegreesToRadians(rotation.Pitch),
            FMath::DegreesToRadians(rotation.Yaw),
            FMath::DegreesToRadians(rotation.Roll)});
    }

    if (Std::contains(observation_components, "control_data")) {
        Eigen::Vector4f duty_cycle = open_bot_pawn_->getDutyCycle();
        Eigen::Vector2f control_state;
        control_state(0) = (duty_cycle(0) + duty_cycle(2)) / 2;
        control_state(1) = (duty_cycle(1) + duty_cycle(3)) / 2;
        observation["control_data"] = Std::reinterpret_as<uint8_t>(std::vector<float>{
            control_state(0),
            control_state(1)});
    }

    if (Std::contains(observation_components, "encoder")) {
        Eigen::Vector4f wheel_rotation_speeds = open_bot_pawn_->getWheelRotationSpeeds();
        observation["encoder"] = Std::reinterpret_as<uint8_t>(std::vector<float>{
            wheel_rotation_speeds(0),
            wheel_rotation_speeds(1),
            wheel_rotation_speeds(2),
            wheel_rotation_speeds(3)});
    }

    if (Std::contains(observation_components, "imu")) {
        observation["imu"] = Std::reinterpret_as<uint8_t>(std::vector<float>{
            imu_sensor_->linear_acceleration_.X,
            imu_sensor_->linear_acceleration_.Y,
            imu_sensor_->linear_acceleration_.Z,
            imu_sensor_->angular_rate_.X,
            imu_sensor_->angular_rate_.Y,
            imu_sensor_->angular_rate_.Z});
    }

    if (Std::contains(observation_components, "sonar")) {
        observation["sonar"] = Std::reinterpret_as<uint8_t>(std::vector<float>{
            sonar_sensor_->range_});
    }

    std::map<std::string, std::vector<uint8_t>> camera_sensor_observation = camera_sensor_->getObservation(observation_components);
    for (auto& camera_sensor_observation_component : camera_sensor_observation) {
        observation[camera_sensor_observation_component.first] = std::move(camera_sensor_observation_component.second);
    }

    return observation;
}

std::map<std::string, std::vector<uint8_t>> OpenBotAgent::getStepInfo() const
{
    std::map<std::string, std::vector<uint8_t>> step_info;

    auto step_info_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.OPENBOT_AGENT.STEP_INFO_COMPONENTS");

    if (Std::contains(step_info_components, "trajectory_data")) {
        step_info["trajectory_data"] = Std::reinterpret_as<uint8_t>(trajectory_);
    }

    return step_info;
}

void OpenBotAgent::reset()
{
    // For some reason, the pose of AOpenBotPawn needs to be set using ETeleportType::TeleportPhysics, which maintains
    // velocity information across calls to SetActorPositionAndRotation(...). Since tasks are supposed to be implemented
    // in a general way, they must therefore use ETeleportType::TeleportPhysics to set the pose of actors, because the
    // actor they're attempting to reset might be an AOpenBotPawn. But this means that our velocity will be maintained
    // unless we explicitly reset it, so we reset our velocity here.
    open_bot_pawn_->skeletal_mesh_component_->SetPhysicsLinearVelocity(FVector::ZeroVector, false);
    open_bot_pawn_->skeletal_mesh_component_->SetPhysicsAngularVelocityInRadians(FVector::ZeroVector, false);
    open_bot_pawn_->skeletal_mesh_component_->GetBodyInstance()->ClearTorques();
    open_bot_pawn_->skeletal_mesh_component_->GetBodyInstance()->ClearForces();

    // Reset wheels
    open_bot_pawn_->resetWheels();
    open_bot_pawn_->setBrakeTorques(Eigen::Vector4f(1000.0f, 1000.0f, 1000.0f, 1000.0f)); // TODO: get value from the config system

    // Compute a new trajectory for step_info["trajectory_data"]
    auto step_info_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.OPENBOT_AGENT.STEP_INFO_COMPONENTS");

    if (Std::contains(step_info_components, "trajectory_data")) {
        generateTrajectoryToGoal();
    }
}

bool OpenBotAgent::isReady() const
{
    return open_bot_pawn_->GetVelocity().Size() <= Config::get<float>("SIMULATION_CONTROLLER.OPENBOT_AGENT.IS_READY_VELOCITY_THRESHOLD");
}

void OpenBotAgent::buildNavMesh()
{
    // Set the navmesh properties
    nav_mesh_->AgentRadius            = Config::get<float>("SIMULATION_CONTROLLER.OPENBOT_AGENT.NAVMESH.AGENT_RADIUS");
    nav_mesh_->AgentHeight            = Config::get<float>("SIMULATION_CONTROLLER.OPENBOT_AGENT.NAVMESH.AGENT_HEIGHT");
    nav_mesh_->CellSize               = Config::get<float>("SIMULATION_CONTROLLER.OPENBOT_AGENT.NAVMESH.CELL_SIZE");
    nav_mesh_->CellHeight             = Config::get<float>("SIMULATION_CONTROLLER.OPENBOT_AGENT.NAVMESH.CELL_HEIGHT");
    nav_mesh_->AgentMaxStepHeight     = Config::get<float>("SIMULATION_CONTROLLER.OPENBOT_AGENT.NAVMESH.AGENT_MAX_STEP_HEIGHT");
    nav_mesh_->AgentMaxSlope          = Config::get<float>("SIMULATION_CONTROLLER.OPENBOT_AGENT.NAVMESH.AGENT_MAX_SLOPE");
    nav_mesh_->MergeRegionSize        = Config::get<float>("SIMULATION_CONTROLLER.OPENBOT_AGENT.NAVMESH.MERGE_REGION_SIZE");
    nav_mesh_->MinRegionArea          = Config::get<float>("SIMULATION_CONTROLLER.OPENBOT_AGENT.NAVMESH.MIN_REGION_AREA");
    nav_mesh_->TileSizeUU             = Config::get<float>("SIMULATION_CONTROLLER.OPENBOT_AGENT.NAVMESH.TILE_SIZE_UU");
    nav_mesh_->TilePoolSize           = Config::get<float>("SIMULATION_CONTROLLER.OPENBOT_AGENT.NAVMESH.TILE_POOL_SIZE");
    nav_mesh_->MaxSimplificationError = Config::get<float>("SIMULATION_CONTROLLER.OPENBOT_AGENT.NAVMESH.MAX_SIMPLIFICATION_ERROR");

    // get bounds volume
    FBox bounds_volume(EForceInit::ForceInit);
    auto tags = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.OPENBOT_AGENT.NAVMESH.BOUNDS_VOLUME_ACTOR_TAGS");
    for (auto& actor : Unreal::findActorsByTagAny(open_bot_pawn_->GetWorld(), tags)) {
        bounds_volume += actor->GetComponentsBoundingBox(false, true);
    }

    // get references to ANavMeshBoundsVolume and ANavModifierVolume
    ANavMeshBoundsVolume* nav_mesh_bounds_volume = Unreal::findActorByType<ANavMeshBoundsVolume>(open_bot_pawn_->GetWorld());
    ASSERT(nav_mesh_bounds_volume);

    ANavModifierVolume* nav_modifier_volume = Unreal::findActorByType<ANavModifierVolume>(open_bot_pawn_->GetWorld());
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
    nav_modifier_volume->SetActorRelativeScale3D(bounds_volume.GetSize() / 200.f);
    nav_modifier_volume->AddActorWorldOffset(FVector(
        Config::get<float>("SIMULATION_CONTROLLER.OPENBOT_AGENT.NAVMESH.NAV_MODIFIER_OFFSET_X"),
        Config::get<float>("SIMULATION_CONTROLLER.OPENBOT_AGENT.NAVMESH.NAV_MODIFIER_OFFSET_Y"),
        Config::get<float>("SIMULATION_CONTROLLER.OPENBOT_AGENT.NAVMESH.NAV_MODIFIER_OFFSET_Z")));
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
        if (Config::get<bool>("SIMULATION_CONTROLLER.OPENBOT_AGENT.NAVMESH.EXPORT_NAV_DATA_OBJ")) {
            nav_mesh_->GetGenerator()->ExportNavigationData(FPaths::Combine(
                Unreal::toFString(Config::get<std::string>("SIMULATION_CONTROLLER.CAMERA_AGENT.NAVMESH.EXPORT_NAV_DATA_OBJ_DIR")),
                open_bot_pawn_->GetWorld()->GetName()));
        }
    #endif
}

void OpenBotAgent::generateTrajectoryToGoal()
{
    trajectory_.clear();
    
    // Update navigation query with the new agent position and goal position
    FPathFindingQuery nav_query = FPathFindingQuery(open_bot_pawn_, *nav_mesh_, open_bot_pawn_->GetActorLocation(), goal_actor_->GetActorLocation());

    // Generate a collision-free path between the agent position and the goal position
    FPathFindingResult path = nav_sys_->FindPathSync(nav_query, EPathFindingMode::Type::Regular);
    
    // Ensure that path generation process was successful and that the generated path is valid
    ASSERT(path.IsSuccessful());
    ASSERT(path.Path.IsValid());

    // Update trajectory_
    TArray<FNavPathPoint> path_points = path.Path->GetPathPoints(); 
    ASSERT(path_points.Num() > 1); // There should be at least a starting point and a goal point

    for(auto& path_point : path_points) {
        trajectory_.push_back(path_point.Location.X);
        trajectory_.push_back(path_point.Location.Y);
        trajectory_.push_back(path_point.Location.Z);
    }

    // Debug output
    if (path.IsPartial()) {
        std::cout << "[SPEAR | OpenBotAgent.cpp] Only a partial path could be found..." << std::endl;
    }

    int num_waypoints = path.Path->GetPathPoints().Num();
    float trajectory_length = 0.0f;
    for (int i = 0; i < num_waypoints - 1; i++) {
        trajectory_length += FVector::Dist(path_points[i].Location, path_points[i + 1].Location);
    }
    trajectory_length /= open_bot_pawn_->GetWorld()->GetWorldSettings()->WorldToMeters;
    FVector2D relative_position_to_goal(
        (goal_actor_->GetActorLocation() - open_bot_pawn_->GetActorLocation()).X, (goal_actor_->GetActorLocation() - open_bot_pawn_->GetActorLocation()).Y);

    std::cout << "[SPEAR | OpenBotAgent.cpp] Number of waypoints: " << num_waypoints << std::endl;
    std::cout << "[SPEAR | OpenBotAgent.cpp] Goal distance: " <<
        relative_position_to_goal.Size() / open_bot_pawn_->GetWorld()->GetWorldSettings()->WorldToMeters << "m" <<
        std::endl;
    std::cout << "[SPEAR | OpenBotAgent.cpp] Path length: " << trajectory_length << "m" << std::endl;
    std::cout << "[SPEAR | OpenBotAgent.cpp] Initial position: [" <<
        open_bot_pawn_->GetActorLocation().X << ", " << open_bot_pawn_->GetActorLocation().Y << ", " << open_bot_pawn_->GetActorLocation().Z << "]." <<
        std::endl;
    std::cout << "[SPEAR | OpenBotAgent.cpp] Goal position: [" <<
        goal_actor_->GetActorLocation().X << ", " << goal_actor_->GetActorLocation().Y << ", " << goal_actor_->GetActorLocation().Z << "]." <<
        std::endl;
    std::cout << "[SPEAR | OpenBotAgent.cpp] ----------------------" << std::endl;
    std::cout << "[SPEAR | OpenBotAgent.cpp] Waypoints: " << std::endl;
    for (auto& point : path_points) {
        std::cout << "[SPEAR | OpenBotAgent.cpp] [" << point.Location.X << ", " << point.Location.Y << ", " << point.Location.Z << "]" << std::endl;
    }
    std::cout << "[SPEAR | OpenBotAgent.cpp] ----------------------" << std::endl;
}
*/
////------ END UE5 MIGRATION ------////
