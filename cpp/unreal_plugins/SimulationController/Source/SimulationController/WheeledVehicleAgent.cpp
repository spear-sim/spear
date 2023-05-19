//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SimulationController/WheeledVehicleAgent.h"

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
#include "CoreUtils/Log.h"
#include "CoreUtils/Std.h"
#include "CoreUtils/Unreal.h"
#include "WheeledVehicle/VehiclePawn.h"
#include "SimulationController/CameraSensor.h"
#include "SimulationController/ImuSensor.h"
#include "SimulationController/SonarSensor.h"

WheeledVehicleAgent::WheeledVehicleAgent(UWorld* world)
{
    SP_LOG_CURRENT_FUNCTION();

    FVector spawn_location = FVector::ZeroVector;
    FRotator spawn_rotation = FRotator::ZeroRotator;
    std::string spawn_mode = Config::get<std::string>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.SPAWN_MODE");
    if (spawn_mode == "specify_existing_actor") {
        AActor* spawn_actor = Unreal::findActorByName(world, Config::get<std::string>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.SPAWN_ACTOR_NAME"));
        SP_ASSERT(spawn_actor);
        spawn_location = spawn_actor->GetActorLocation();
        spawn_rotation = spawn_actor->GetActorRotation();
    }
    else if (spawn_mode == "specify_pose") {
        spawn_location = FVector(
            Config::get<float>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.SPAWN_POSITION_X"),
            Config::get<float>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.SPAWN_POSITION_Y"),
            Config::get<float>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.SPAWN_POSITION_Z"));
        spawn_rotation = FRotator(
            Config::get<float>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.SPAWN_PITCH"),
            Config::get<float>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.SPAWN_YAW"),
            Config::get<float>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.SPAWN_ROLL"));
    }
    else {
        SP_ASSERT(false);
    }
    FActorSpawnParameters actor_spawn_params;
    actor_spawn_params.Name = Unreal::toFName(Config::get<std::string>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.WHEELED_VEHICLE_ACTOR_NAME"));
    actor_spawn_params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    wheeled_vehicle_pawn_ = world->SpawnActor<AVehiclePawn>(spawn_location, spawn_rotation, actor_spawn_params);
    SP_ASSERT(wheeled_vehicle_pawn_);

    auto observation_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.OBSERVATION_COMPONENTS");

    if (Std::contains(observation_components, "camera")) {
        camera_sensor_ = std::make_unique<CameraSensor>(
            wheeled_vehicle_pawn_->camera_component_,
            Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.CAMERA.RENDER_PASSES"),
            Config::get<unsigned int>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.CAMERA.IMAGE_WIDTH"),
            Config::get<unsigned int>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.CAMERA.IMAGE_HEIGHT"),
            Config::get<float>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.CAMERA.FOV"));
        SP_ASSERT(camera_sensor_);
    }

    if (Std::contains(observation_components, "imu")) {
        imu_sensor_ = std::make_unique<ImuSensor>(wheeled_vehicle_pawn_->imu_component_);
        SP_ASSERT(imu_sensor_);
    }

    // TODO: uncomment when SonarSensor is supported
    //if (Std::contains(observation_components, "sonar")) {
    //    sonar_sensor_ = std::make_unique<SonarSensor>(wheeled_vehicle_pawn_->sonar_component_);
    //    SP_ASSERT(sonar_sensor_);
    //}
}

WheeledVehicleAgent::~WheeledVehicleAgent()
{
    SP_LOG_CURRENT_FUNCTION();

    auto observation_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.OBSERVATION_COMPONENTS");

    // TODO: uncomment when SonarSensor is supported
    //if (Std::contains(observation_components, "sonar")) {
    //    SP_ASSERT(sonar_sensor_);
    //    sonar_sensor_ = nullptr;
    //}

    if (Std::contains(observation_components, "imu")) {
        SP_ASSERT(imu_sensor_);
        imu_sensor_ = nullptr;
    }

    if (Std::contains(observation_components, "camera")) {
        SP_ASSERT(camera_sensor_);
        camera_sensor_ = nullptr;
    }

    SP_ASSERT(wheeled_vehicle_pawn_);
    wheeled_vehicle_pawn_->Destroy();
    wheeled_vehicle_pawn_ = nullptr;
}

void WheeledVehicleAgent::findObjectReferences(UWorld* world)
{
    auto step_info_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.STEP_INFO_COMPONENTS");

    if (Std::contains(step_info_components, "trajectory_data")) {
        goal_actor_ = Unreal::findActorByName(world, Config::get<std::string>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.TRAJECTORY_DATA.GOAL_ACTOR_NAME"));
        SP_ASSERT(goal_actor_);

        nav_sys_ = FNavigationSystem::GetCurrent<UNavigationSystemV1>(world);
        SP_ASSERT(nav_sys_);

        FNavAgentProperties agent_properties;
        agent_properties.AgentHeight = Config::get<float>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.NAVMESH.AGENT_HEIGHT");
        agent_properties.AgentRadius = Config::get<float>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.NAVMESH.AGENT_RADIUS");
        agent_properties.AgentStepHeight = Config::get<float>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.NAVMESH.AGENT_MAX_STEP_HEIGHT");

        ANavigationData* nav_data = nav_sys_->GetNavDataForProps(agent_properties);
        SP_ASSERT(nav_data);

        nav_mesh_ = dynamic_cast<ARecastNavMesh*>(nav_data);
        SP_ASSERT(nav_mesh_);

        buildNavMesh();
    }
}

void WheeledVehicleAgent::cleanUpObjectReferences()
{
    auto step_info_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.STEP_INFO_COMPONENTS");

    if (Std::contains(step_info_components, "trajectory_data")) {
        SP_ASSERT(nav_mesh_);
        nav_mesh_ = nullptr;

        SP_ASSERT(nav_sys_);
        nav_sys_ = nullptr;

        SP_ASSERT(goal_actor_);
        goal_actor_ = nullptr;
    }
}

std::map<std::string, ArrayDesc> WheeledVehicleAgent::getActionSpace() const
{
    std::map<std::string, ArrayDesc> action_space;
    auto action_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.ACTION_COMPONENTS");

    if (Std::contains(action_components, "apply_wheel_torques")) {
        ArrayDesc array_desc;
        array_desc.low_ = std::numeric_limits<float>::lowest();;
        array_desc.high_ = std::numeric_limits<float>::max();;
        array_desc.shape_ = { 4 };
        array_desc.datatype_ = DataType::Float64;
        action_space["apply_wheel_torques"] = std::move(array_desc);
    }

    if (Std::contains(action_components, "set_position_xyz_centimeters")) {
        ArrayDesc array_desc;
        array_desc.low_ = std::numeric_limits<float>::lowest();
        array_desc.high_ = std::numeric_limits<float>::max();
        array_desc.shape_ = { 3 };
        array_desc.datatype_ = DataType::Float64;
        action_space["set_position_xyz_centimeters"] = std::move(array_desc);
    }

    if (Std::contains(action_components, "set_orientation_pyr_radians")) {
        ArrayDesc array_desc;
        array_desc.low_ = std::numeric_limits<float>::lowest();
        array_desc.high_ = std::numeric_limits<float>::max();
        array_desc.shape_ = { 3 };
        array_desc.datatype_ = DataType::Float64;
        action_space["set_orientation_pyr_radians"] = std::move(array_desc);
    }

    return action_space;
}

std::map<std::string, ArrayDesc> WheeledVehicleAgent::getObservationSpace() const
{
    std::map<std::string, ArrayDesc> observation_space;
    auto observation_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.OBSERVATION_COMPONENTS");

    if (Std::contains(observation_components, "state_data")) {
        ArrayDesc array_desc;
        array_desc.low_ = std::numeric_limits<float>::lowest();
        array_desc.high_ = std::numeric_limits<float>::max();
        array_desc.datatype_ = DataType::Float64;
        array_desc.shape_ = { 6 };
        observation_space["state_data"] = std::move(array_desc); // position (X, Y, Z) and orientation (Roll, Pitch, Yaw) of the agent relative to the world frame.
    }

    //if (Std::contains(observation_components, "control_data")) {
    //    ArrayDesc array_desc;
    //    array_desc.low_ = std::numeric_limits<float>::lowest();
    //    array_desc.high_ = std::numeric_limits<float>::max();
    //    array_desc.datatype_ = DataType::Float64;
    //    array_desc.shape_ = { 2 };
    //    observation_space["control_data"] = std::move(array_desc); // ctrl_left, ctrl_right
    //}

    if (Std::contains(observation_components, "encoder")) {
        ArrayDesc array_desc;
        array_desc.low_ = std::numeric_limits<float>::lowest();
        array_desc.high_ = std::numeric_limits<float>::max();
        array_desc.datatype_ = DataType::Float64;
        array_desc.shape_ = { 4 };
        observation_space["encoder"] = std::move(array_desc); // FL, FR, RL, RR
    }

    if (Std::contains(observation_components, "imu")) {
        ArrayDesc array_desc;
        array_desc.low_ = std::numeric_limits<float>::lowest();
        array_desc.high_ = std::numeric_limits<float>::max();
        array_desc.datatype_ = DataType::Float64;
        array_desc.shape_ = { 6 };
        observation_space["imu"] = std::move(array_desc); // a_x, a_y, a_z, g_x, g_y, g_z
    }

    //if (Std::contains(observation_components, "sonar")) {
    //    ArrayDesc array_desc;
    //    array_desc.low_ = std::numeric_limits<float>::lowest();
    //    array_desc.high_ = std::numeric_limits<float>::max();
    //    array_desc.datatype_ = DataType::Float64;
    //    array_desc.shape_ = { 1 };
    //    observation_space["sonar"] = std::move(array_desc); // Front obstacle distance in [m]
    //}

    std::map<std::string, ArrayDesc> camera_sensor_observation_space = camera_sensor_->getObservationSpace(observation_components);
    for (auto& camera_sensor_observation_space_component : camera_sensor_observation_space) {
        observation_space[camera_sensor_observation_space_component.first] = std::move(camera_sensor_observation_space_component.second);
    }

    return observation_space;
}

std::map<std::string, ArrayDesc> WheeledVehicleAgent::getStepInfoSpace() const
{
    std::map<std::string, ArrayDesc> step_info_space;
    auto step_info_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.STEP_INFO_COMPONENTS");

    if (Std::contains(step_info_components, "trajectory_data")) {
        ArrayDesc array_desc;
        array_desc.low_ = std::numeric_limits<float>::lowest();
        array_desc.high_ = std::numeric_limits<float>::max();
        array_desc.datatype_ = DataType::Float64;
        array_desc.shape_ = { -1, 3 };
        step_info_space["trajectory_data"] = std::move(array_desc); // Vector of the waypoints x,y,z in the world frame.
    }

    return step_info_space;
}

void WheeledVehicleAgent::applyAction(const std::map<std::string, std::vector<uint8_t>>& action)
{
    auto action_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.ACTION_COMPONENTS");

    if (Std::contains(action_components, "apply_wheel_torques")) {
        std::vector<double> component_data = Std::reinterpretAs<double>(action.at("apply_wheel_torques"));
        wheeled_vehicle_pawn_->setDriveTorques(Eigen::Vector4d(
            component_data.at(0), component_data.at(1), component_data.at(2), component_data.at(3)));
        wheeled_vehicle_pawn_->setBrakeTorques(Eigen::Vector4d(0.0f, 0.0f, 0.0f, 0.0f));
    }

    if (Std::contains(action_components, "set_position_xyz_centimeters")) {
        std::vector<double> component_data = Std::reinterpretAs<double>(action.at("set_position_xyz_centimeters"));
        FVector location(component_data.at(0), component_data.at(1), component_data.at(2));
        bool sweep = false;
        FHitResult* hit_result = nullptr;
        wheeled_vehicle_pawn_->SetActorLocation(location, sweep, hit_result, ETeleportType::TeleportPhysics);
        wheeled_vehicle_pawn_->setBrakeTorques(Eigen::Vector4d(1000.0f, 1000.0f, 1000.0f, 1000.0f)); // TODO: get value from the config system
    }

    if (Std::contains(action_components, "set_orientation_pyr_radians")) {
        std::vector<double> component_data = Std::reinterpretAs<double>(action.at("set_orientation_pyr_radians"));
        FRotator rotation(
            FMath::RadiansToDegrees(component_data.at(0)),
            FMath::RadiansToDegrees(component_data.at(1)),
            FMath::RadiansToDegrees(component_data.at(2)));
        wheeled_vehicle_pawn_->SetActorRotation(rotation, ETeleportType::TeleportPhysics);
        wheeled_vehicle_pawn_->setBrakeTorques(Eigen::Vector4d(1000.0f, 1000.0f, 1000.0f, 1000.0f)); // TODO: get value from the config system
    }
}

std::map<std::string, std::vector<uint8_t>> WheeledVehicleAgent::getObservation() const
{
    std::map<std::string, std::vector<uint8_t>> observation;

    auto observation_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.OBSERVATION_COMPONENTS");

    if (Std::contains(observation_components, "state_data")) {
        FVector location = wheeled_vehicle_pawn_->GetActorLocation();
        FRotator rotation = wheeled_vehicle_pawn_->GetActorRotation();
        observation["state_data"] = Std::reinterpretAs<uint8_t>(std::vector<double>{
            location.X,
            location.Y,
            location.Z,
            FMath::DegreesToRadians(rotation.Pitch),
            FMath::DegreesToRadians(rotation.Yaw),
            FMath::DegreesToRadians(rotation.Roll)});
    }

    // This should be available in python code, move to OpenBotEnv class
    //if (Std::contains(observation_components, "control_data")) {
    //    Eigen::Vector4d duty_cycle = wheeled_vehicle_pawn_->getDutyCycle();
    //    Eigen::Vector2f control_state;
    //    control_state(0) = (duty_cycle(0) + duty_cycle(2)) / 2;
    //    control_state(1) = (duty_cycle(1) + duty_cycle(3)) / 2;
    //    observation["control_data"] = Std::reinterpretAs<uint8_t>(std::vector<double>{
    //        control_state(0),
    //        control_state(1)});
    //}

    if (Std::contains(observation_components, "encoder")) {
        Eigen::Vector4d wheel_rotation_speeds = wheeled_vehicle_pawn_->getWheelRotationSpeeds();
        observation["encoder"] = Std::reinterpretAs<uint8_t>(std::vector<double>{
            wheel_rotation_speeds(0),
            wheel_rotation_speeds(1),
            wheel_rotation_speeds(2),
            wheel_rotation_speeds(3)});
    }

    if (Std::contains(observation_components, "imu")) {
        observation["imu"] = Std::reinterpretAs<uint8_t>(std::vector<double>{
            imu_sensor_->linear_acceleration_body_.X,
            imu_sensor_->linear_acceleration_body_.Y,
            imu_sensor_->linear_acceleration_body_.Z,
            imu_sensor_->angular_velocity_body_.X,
            imu_sensor_->angular_velocity_body_.Y,
            imu_sensor_->angular_velocity_body_.Z});
    }

    //if (Std::contains(observation_components, "sonar")) {
    //    observation["sonar"] = Std::reinterpretAs<uint8_t>(std::vector<double>{
    //        sonar_sensor_->range_});
    //}

    std::map<std::string, std::vector<uint8_t>> camera_sensor_observation = camera_sensor_->getObservation(observation_components);
    for (auto& camera_sensor_observation_component : camera_sensor_observation) {
        observation[camera_sensor_observation_component.first] = std::move(camera_sensor_observation_component.second);
    }

    return observation;
}

std::map<std::string, std::vector<uint8_t>> WheeledVehicleAgent::getStepInfo() const
{
    std::map<std::string, std::vector<uint8_t>> step_info;

    auto step_info_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.STEP_INFO_COMPONENTS");

    if (Std::contains(step_info_components, "trajectory_data")) {
        step_info["trajectory_data"] = Std::reinterpretAs<uint8_t>(trajectory_);
    }

    return step_info;
}

void WheeledVehicleAgent::reset()
{
    // For some reason, the pose of AOpenBotPawn needs to be set using ETeleportType::TeleportPhysics, which maintains
    // velocity information across calls to SetActorPositionAndRotation(...). Since tasks are supposed to be implemented
    // in a general way, they must therefore use ETeleportType::TeleportPhysics to set the pose of actors, because the
    // actor they're attempting to reset might be an AOpenBotPawn. But this means that our velocity will be maintained
    // unless we explicitly reset it, so we reset our velocity here.
    wheeled_vehicle_pawn_->skeletal_mesh_component_->SetPhysicsLinearVelocity(FVector::ZeroVector, false);
    wheeled_vehicle_pawn_->skeletal_mesh_component_->SetPhysicsAngularVelocityInRadians(FVector::ZeroVector, false);
    wheeled_vehicle_pawn_->skeletal_mesh_component_->GetBodyInstance()->ClearTorques();
    wheeled_vehicle_pawn_->skeletal_mesh_component_->GetBodyInstance()->ClearForces();

    // Reset vehicle
    wheeled_vehicle_pawn_->resetVehicle();
    wheeled_vehicle_pawn_->setBrakeTorques(Eigen::Vector4d(1000.0f, 1000.0f, 1000.0f, 1000.0f)); // TODO: get value from the config system

    // Compute a new trajectory for step_info["trajectory_data"]
    auto step_info_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.STEP_INFO_COMPONENTS");

    if (Std::contains(step_info_components, "trajectory_data")) {
        generateTrajectoryToGoal();
    }
}

bool WheeledVehicleAgent::isReady() const
{
    return wheeled_vehicle_pawn_->GetVelocity().Size() <= Config::get<float>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.IS_READY_VELOCITY_THRESHOLD");
}

void WheeledVehicleAgent::buildNavMesh()
{
    // Set the navmesh properties
    nav_mesh_->AgentRadius = Config::get<float>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.NAVMESH.AGENT_RADIUS");
    nav_mesh_->AgentHeight = Config::get<float>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.NAVMESH.AGENT_HEIGHT");
    nav_mesh_->CellSize = Config::get<float>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.NAVMESH.CELL_SIZE");
    nav_mesh_->CellHeight = Config::get<float>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.NAVMESH.CELL_HEIGHT");
    nav_mesh_->AgentMaxStepHeight = Config::get<float>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.NAVMESH.AGENT_MAX_STEP_HEIGHT");
    nav_mesh_->AgentMaxSlope = Config::get<float>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.NAVMESH.AGENT_MAX_SLOPE");
    nav_mesh_->MergeRegionSize = Config::get<float>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.NAVMESH.MERGE_REGION_SIZE");
    nav_mesh_->MinRegionArea = Config::get<float>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.NAVMESH.MIN_REGION_AREA");
    nav_mesh_->TileSizeUU = Config::get<float>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.NAVMESH.TILE_SIZE_UU");
    nav_mesh_->TilePoolSize = Config::get<float>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.NAVMESH.TILE_POOL_SIZE");
    nav_mesh_->MaxSimplificationError = Config::get<float>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.NAVMESH.MAX_SIMPLIFICATION_ERROR");

    // get bounds volume
    FBox bounds_volume(EForceInit::ForceInit);
    auto tags = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.NAVMESH.BOUNDS_VOLUME_ACTOR_TAGS");
    for (auto& actor : Unreal::findActorsByTagAny(wheeled_vehicle_pawn_->GetWorld(), tags)) {
        bounds_volume += actor->GetComponentsBoundingBox(false, true);
    }

    // get references to ANavMeshBoundsVolume and ANavModifierVolume
    ANavMeshBoundsVolume* nav_mesh_bounds_volume = Unreal::findActorByType<ANavMeshBoundsVolume>(wheeled_vehicle_pawn_->GetWorld());
    SP_ASSERT(nav_mesh_bounds_volume);

    ANavModifierVolume* nav_modifier_volume = Unreal::findActorByType<ANavModifierVolume>(wheeled_vehicle_pawn_->GetWorld());
    SP_ASSERT(nav_modifier_volume);

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
        Config::get<float>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.NAVMESH.NAV_MODIFIER_OFFSET_X"),
        Config::get<float>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.NAVMESH.NAV_MODIFIER_OFFSET_Y"),
        Config::get<float>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.NAVMESH.NAV_MODIFIER_OFFSET_Z")));
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
    if (Config::get<bool>("SIMULATION_CONTROLLER.WHEELED_VEHICLE_AGENT.NAVMESH.EXPORT_NAV_DATA_OBJ")) {
        nav_mesh_->GetGenerator()->ExportNavigationData(FPaths::Combine(
            Unreal::toFString(Config::get<std::string>("SIMULATION_CONTROLLER.CAMERA_AGENT.NAVMESH.EXPORT_NAV_DATA_OBJ_DIR")),
            wheeled_vehicle_pawn_->GetWorld()->GetName()));
    }
#endif
}

void WheeledVehicleAgent::generateTrajectoryToGoal()
{
    trajectory_.clear();

    // Update navigation query with the new agent position and goal position
    FPathFindingQuery nav_query = FPathFindingQuery(wheeled_vehicle_pawn_, *nav_mesh_, wheeled_vehicle_pawn_->GetActorLocation(), goal_actor_->GetActorLocation());

    // Generate a collision-free path between the agent position and the goal position
    FPathFindingResult path = nav_sys_->FindPathSync(nav_query, EPathFindingMode::Type::Regular);

    // Ensure that path generation process was successful and that the generated path is valid
    SP_ASSERT(path.IsSuccessful());
    SP_ASSERT(path.Path.IsValid());

    // Update trajectory_
    TArray<FNavPathPoint> path_points = path.Path->GetPathPoints();
    SP_ASSERT(path_points.Num() > 1); // There should be at least a starting point and a goal point

    for (auto& path_point : path_points) {
        trajectory_.push_back(path_point.Location.X);
        trajectory_.push_back(path_point.Location.Y);
        trajectory_.push_back(path_point.Location.Z);
    }

    // Debug output
    if (path.IsPartial()) {
        SP_LOG("Only a partial path could be found...");
    }

    int num_waypoints = path.Path->GetPathPoints().Num();
    float trajectory_length = 0.0f;
    for (int i = 0; i < num_waypoints - 1; i++) {
        trajectory_length += FVector::Dist(path_points[i].Location, path_points[i + 1].Location);
    }
    trajectory_length /= wheeled_vehicle_pawn_->GetWorld()->GetWorldSettings()->WorldToMeters;
    FVector2D relative_position_to_goal(
        (goal_actor_->GetActorLocation() - wheeled_vehicle_pawn_->GetActorLocation()).X, (goal_actor_->GetActorLocation() - wheeled_vehicle_pawn_->GetActorLocation()).Y);

    SP_LOG("Number of waypoints: ", num_waypoints);
    SP_LOG("Goal distance: ", relative_position_to_goal.Size() / wheeled_vehicle_pawn_->GetWorld()->GetWorldSettings()->WorldToMeters, "m");
    SP_LOG("Path length: ", trajectory_length, "m");
    SP_LOG("Initial position: [", wheeled_vehicle_pawn_->GetActorLocation().X, ", ", wheeled_vehicle_pawn_->GetActorLocation().Y, ", ", wheeled_vehicle_pawn_->GetActorLocation().Z, "].");
    SP_LOG("Goal position: [", goal_actor_->GetActorLocation().X, ", ", goal_actor_->GetActorLocation().Y, ", ", goal_actor_->GetActorLocation().Z, "].");
    SP_LOG("----------------------");
    SP_LOG("Waypoints: ");
    for (auto& point : path_points) {
        SP_LOG("[", point.Location.X, ", ", point.Location.Y, ", ", point.Location.Z, "]");
    }
    SP_LOG("----------------------");
}
