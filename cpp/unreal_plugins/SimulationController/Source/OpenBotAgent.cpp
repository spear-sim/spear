#include "OpenBotAgent.h"

#include <algorithm>

#include <AI/NavDataGenerator.h>
#include <Components/SceneCaptureComponent2D.h>
#include <Components/BoxComponent.h>
#include <Components/PrimitiveComponent.h>
#include <Components/StaticMeshComponent.h>
#include <Engine/TextureRenderTarget2D.h>
#include <Engine/World.h>
#include <EngineUtils.h>
#include <GameFramework/Actor.h>
#include <Kismet/GameplayStatics.h>
#include <NavigationSystem.h>
#include <NavMesh/NavMeshBoundsVolume.h>
#include <NavMesh/RecastNavMesh.h>
#include <NavModifierVolume.h>
#include <UObject/UObjectGlobals.h>

#include "Assert/Assert.h"
#include "Box.h"
#include "CameraSensor.h"
#include "Config.h"
#include "ImuSensor.h"
#include "OpenBotPawn.h"
#include "Serialize.h"
#include "SonarSensor.h"

OpenBotAgent::OpenBotAgent(UWorld* world)
{
    FActorSpawnParameters spawn_params;
    spawn_params.Name = FName(Config::getValue<std::string>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT", "OPENBOT_ACTOR_NAME"}).c_str());
    spawn_params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    open_bot_pawn_ = world->SpawnActor<AOpenBotPawn>(FVector(0, 0, 0), FRotator(0, 0, 0), spawn_params);
    ASSERT(open_bot_pawn_);

    auto observation_components = Config::getValue<std::vector<std::string>>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT", "OBSERVATION_COMPONENTS"});

    //
    // observation["camera"]
    //
    if (std::find(observation_components.begin(), observation_components.end(), "camera") != observation_components.end()) {
        camera_sensor_ = std::make_unique<CameraSensor>(
            open_bot_pawn_->camera_component_,
            Config::getValue<std::vector<std::string>>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT", "CAMERA", "RENDER_PASSES"}),
            Config::getValue<unsigned int>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT", "CAMERA", "IMAGE_WIDTH"}),
            Config::getValue<unsigned int>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT", "CAMERA", "IMAGE_HEIGHT"}));
        ASSERT(camera_sensor_);

        // update FOV
        for (auto& pass : Config::getValue<std::vector<std::string>>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT", "CAMERA", "RENDER_PASSES"})) {
            camera_sensor_->render_passes_.at(pass).scene_capture_component_->FOVAngle = Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT", "CAMERA", "FOV"});
        }
    }

    //
    // observation["imu"]
    //
    if (std::find(observation_components.begin(), observation_components.end(), "imu") != observation_components.end()) {
        imu_sensor_ = std::make_unique<ImuSensor>(open_bot_pawn_->imu_component_);
        ASSERT(imu_sensor_);
    }
    
    //
    // observation["sonar"]
    //
    if (std::find(observation_components.begin(), observation_components.end(), "sonar")!= observation_components.end()) {
        sonar_sensor_ = std::make_unique<SonarSensor>(open_bot_pawn_->sonar_component_);
        ASSERT(sonar_sensor_);
    }
}

OpenBotAgent::~OpenBotAgent()
{
    auto observation_components = Config::getValue<std::vector<std::string>>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT", "OBSERVATION_COMPONENTS"});

    //
    // observation["sonar"]
    //
    if (std::find(observation_components.begin(), observation_components.end(), "sonar")!= observation_components.end()) {
        ASSERT(sonar_sensor_);
        sonar_sensor_ = nullptr;
    }

    //
    // observation["imu"]
    //
    if (std::find(observation_components.begin(), observation_components.end(), "imu") != observation_components.end()) {
        ASSERT(imu_sensor_);
        imu_sensor_ = nullptr;
    }

    //
    // observation["camera"]
    //
    if (std::find(observation_components.begin(), observation_components.end(), "camera") != observation_components.end()) {
        ASSERT(camera_sensor_);
        camera_sensor_ = nullptr;
    }

    ASSERT(open_bot_pawn_);
    open_bot_pawn_->Destroy();
    open_bot_pawn_ = nullptr;
}

void OpenBotAgent::findObjectReferences(UWorld* world)
{
    auto step_info_components = Config::getValue<std::vector<std::string>>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT", "STEP_INFO_COMPONENTS"});

    //
    // step_info["trajectory_data"]
    //
    if (std::find(step_info_components.begin(), step_info_components.end(), "trajectory_data") != step_info_components.end()) {

        for (TActorIterator<AActor> actor_itr(world); actor_itr; ++actor_itr) {
            std::string actor_name = TCHAR_TO_UTF8(*((*actor_itr)->GetName()));
            if (actor_name == Config::getValue<std::string>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT", "TRAJECTORY_DATA", "GOAL_ACTOR_NAME"})) {
                ASSERT(!goal_actor_);
                goal_actor_ = *actor_itr;
                break;
            }
        }
        ASSERT(goal_actor_);

        nav_sys_ = FNavigationSystem::GetCurrent<UNavigationSystemV1>(world);
        ASSERT(nav_sys_);

        FNavAgentProperties agent_properties;
        agent_properties.AgentHeight     = Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT", "NAVMESH", "AGENT_HEIGHT"});
        agent_properties.AgentRadius     = Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT", "NAVMESH", "AGENT_RADIUS"});
        agent_properties.AgentStepHeight = Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT", "NAVMESH", "AGENT_MAX_STEP_HEIGHT"});

        ANavigationData* nav_data = nav_sys_->GetNavDataForProps(agent_properties);
        ASSERT(nav_data);

        nav_mesh_ = dynamic_cast<ARecastNavMesh*>(nav_data);
        ASSERT(nav_mesh_);

        buildNavMesh();
    }
}

void OpenBotAgent::cleanUpObjectReferences()
{
    auto step_info_components = Config::getValue<std::vector<std::string>>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT", "STEP_INFO_COMPONENTS"});

    //
    // step_info["trajectory_data"]
    //
    if (std::find(step_info_components.begin(), step_info_components.end(), "trajectory_data") != step_info_components.end()) {
        ASSERT(nav_mesh_);
        nav_mesh_ = nullptr;

        ASSERT(nav_sys_);
        nav_sys_ = nullptr;

        ASSERT(goal_actor_);
        goal_actor_ = nullptr;
    }
}

std::map<std::string, Box> OpenBotAgent::getActionSpace() const
{
    
    std::map<std::string, Box> action_space;
    Box box;

    auto action_components = Config::getValue<std::vector<std::string>>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT", "ACTION_COMPONENTS"});

    //
    // action["apply_voltage"]
    //
    if (std::find(action_components.begin(), action_components.end(), "apply_voltage") != action_components.end()) {
        box.low = -1.f;
        box.high = 1.f;
        box.shape = {2};
        box.dtype = DataType::Float32;
        action_space["apply_voltage"] = std::move(box);
    }

    //
    // action["set_position_xyz_centimeters"]
    //
    if (std::find(action_components.begin(), action_components.end(), "set_position_xyz_centimeters") != action_components.end()) {
        box.low = std::numeric_limits<float>::lowest();
        box.high = std::numeric_limits<float>::max();
        box.shape = {3};
        box.dtype = DataType::Float32;
        action_space["set_position_xyz_centimeters"] = std::move(box);
    }

    //
    // action["set_orientation_pyr_radians"]
    //
    if (std::find(action_components.begin(), action_components.end(), "set_orientation_pyr_radians") != action_components.end()) {
        box.low = std::numeric_limits<float>::lowest();
        box.high = std::numeric_limits<float>::max();
        box.shape = {3};
        box.dtype = DataType::Float32;
        action_space["set_orientation_pyr_radians"] = std::move(box);
    }

    return action_space;
}

std::map<std::string, Box> OpenBotAgent::getObservationSpace() const
{
    std::map<std::string, Box> observation_space;
    Box box;

    auto observation_components = Config::getValue<std::vector<std::string>>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT", "OBSERVATION_COMPONENTS"});

    //
    // observation["state_data"]
    //
    if (std::find(observation_components.begin(), observation_components.end(), "state_data") != observation_components.end()) {
        box.low = std::numeric_limits<float>::lowest();
        box.high = std::numeric_limits<float>::max();
        box.dtype = DataType::Float32;
        box.shape = {6};
        observation_space["state_data"] = std::move(box); // position (X, Y, Z) and orientation (Roll, Pitch, Yaw) of the agent relative to the world frame.
    }

    //
    // observation["control_data"]
    //
    if (std::find(observation_components.begin(), observation_components.end(), "control_data") != observation_components.end()) {
        box.low = std::numeric_limits<float>::lowest();
        box.high = std::numeric_limits<float>::max();
        box.dtype = DataType::Float32;
        box.shape = {2};
        observation_space["control_data"] = std::move(box); // ctrl_left, ctrl_right
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

    //
    // observation["imu"]
    //
    if (std::find(observation_components.begin(), observation_components.end(), "imu") != observation_components.end()) {
        box.low = std::numeric_limits<float>::lowest();
        box.high = std::numeric_limits<float>::max();
        box.dtype = DataType::Float32;
        box.shape = {6};
        observation_space["imu"] = std::move(box); // a_x, a_y, a_z, g_x, g_y, g_z
    }

    //
    // observation["sonar"]
    //
    if (std::find(observation_components.begin(), observation_components.end(), "sonar")!= observation_components.end()) {
        box.low = std::numeric_limits<float>::lowest();
        box.high = std::numeric_limits<float>::max();
        box.dtype = DataType::Float32;
        box.shape = {1};
        observation_space["sonar"] = std::move(box); // Front obstacle distance in [m]
    }
    
    return observation_space;
}

std::map<std::string, Box> OpenBotAgent::getStepInfoSpace() const
{
    std::map<std::string, Box> step_info_space;
    Box box;

    auto step_info_components = Config::getValue<std::vector<std::string>>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT", "STEP_INFO_COMPONENTS"});

    //
    // step_info["trajectory_data"]
    //
    if (std::find(step_info_components.begin(), step_info_components.end(), "trajectory_data") != step_info_components.end()) {
        box.low = std::numeric_limits<float>::lowest();
        box.high = std::numeric_limits<float>::max();
        box.dtype = DataType::Float32;
        box.shape = {-1, 3};
        step_info_space["trajectory_data"] = std::move(box); // Vector of the waypoints (X, Y, Z) building the desired trajectory relative to the world frame.
    }

    return step_info_space;
}

void OpenBotAgent::applyAction(const std::map<std::string, std::vector<float>>& action)
{
    auto action_components = Config::getValue<std::vector<std::string>>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT", "ACTION_COMPONENTS"});

    //
    // action["apply_voltage"]
    //
    if (std::find(action_components.begin(), action_components.end(), "apply_voltage") != action_components.end()) {
        Eigen::Vector4f duty_cycle(action.at("apply_voltage").at(0), action.at("apply_voltage").at(1), action.at("apply_voltage").at(0), action.at("apply_voltage").at(1));
        open_bot_pawn_->deactivateBrakes();
        open_bot_pawn_->setDutyCycleAndClamp(duty_cycle);
    }

    //
    // action["set_position_xyz_centimeters"]
    //
    if (std::find(action_components.begin(), action_components.end(), "set_position_xyz_centimeters") != action_components.end()) {
        FVector location(action.at("set_position_xyz_centimeters").at(0), action.at("set_position_xyz_centimeters").at(1), action.at("set_position_xyz_centimeters").at(2));
        bool sweep = false;
        FHitResult* hit_result_info = nullptr;
        open_bot_pawn_->SetActorLocation(location, sweep, hit_result_info, ETeleportType::TeleportPhysics);
        open_bot_pawn_->activateBrakes();
    }

    //
    // action["set_orientation_pyr_radians"]
    //
    if (std::find(action_components.begin(), action_components.end(), "set_orientation_pyr_radians") != action_components.end()) {
        FRotator rotation{FMath::RadiansToDegrees(action.at("set_orientation_pyr_radians").at(0)), FMath::RadiansToDegrees(action.at("set_orientation_pyr_radians").at(1)), FMath::RadiansToDegrees(action.at("set_orientation_pyr_radians").at(2))};
        open_bot_pawn_->SetActorRotation(rotation, ETeleportType::TeleportPhysics);
        open_bot_pawn_->activateBrakes();
    }
}

std::map<std::string, std::vector<uint8_t>> OpenBotAgent::getObservation() const
{
    std::map<std::string, std::vector<uint8_t>> observation;

    auto observation_components = Config::getValue<std::vector<std::string>>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT", "OBSERVATION_COMPONENTS"});

    //
    // observation["state_data"]
    //
    if (std::find(observation_components.begin(), observation_components.end(), "state_data") != observation_components.end()) {
        FVector location = open_bot_pawn_->GetActorLocation();
        FRotator rotation = open_bot_pawn_->GetActorRotation();
        observation["state_data"] = Serialize::toUint8(std::vector<float>{
            location.X,
            location.Y,
            location.Z,
            FMath::DegreesToRadians(rotation.Pitch),
            FMath::DegreesToRadians(rotation.Yaw),
            FMath::DegreesToRadians(rotation.Roll)});
    }

    //
    // observation["control_data"]
    //
    if (std::find(observation_components.begin(), observation_components.end(), "control_data") != observation_components.end()) {
        Eigen::Vector4f duty_cycle = open_bot_pawn_->getDutyCycle();
        Eigen::Vector2f control_state;
        control_state(0) = (duty_cycle(0) + duty_cycle(2)) / 2;
        control_state(1) = (duty_cycle(1) + duty_cycle(3)) / 2;
        observation["control_data"] = Serialize::toUint8(std::vector<float>{control_state(0), control_state(1)});
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

    //
    // observation["imu"]
    //
    if (std::find(observation_components.begin(), observation_components.end(), "imu") != observation_components.end()) {
        observation["imu"] = Serialize::toUint8(std::vector<float>{imu_sensor_->linear_acceleration_.X, imu_sensor_->linear_acceleration_.Y, imu_sensor_->linear_acceleration_.Z, imu_sensor_->angular_rate_.X, imu_sensor_->angular_rate_.Y, imu_sensor_->angular_rate_.Z});
    }

    //
    // observation["sonar"]
    //
    if (std::find(observation_components.begin(), observation_components.end(), "sonar")!= observation_components.end()) {
        observation["sonar"] = Serialize::toUint8(std::vector<float>{sonar_sensor_->range_});
    }

    return observation;
}

std::map<std::string, std::vector<uint8_t>> OpenBotAgent::getStepInfo() const
{
    std::map<std::string, std::vector<uint8_t>> step_info;

    auto step_info_components = Config::getValue<std::vector<std::string>>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT", "STEP_INFO_COMPONENTS"});

    //
    // step_info["trajectory_data"]
    //
    if (std::find(step_info_components.begin(), step_info_components.end(), "trajectory_data") != step_info_components.end()) {
        step_info["trajectory_data"] = Serialize::toUint8(trajectory_);
    }

    return step_info;
}

void OpenBotAgent::reset()
{
    FVector location = open_bot_pawn_->GetActorLocation();
    bool sweep = false;
    FHitResult* hit_result_info = nullptr;    
    open_bot_pawn_->SetActorLocationAndRotation(location, FQuat(FRotator(0)), sweep, hit_result_info, ETeleportType::TeleportPhysics);
    open_bot_pawn_->resetPhysicsState();
    open_bot_pawn_->activateBrakes();

    auto step_info_components = Config::getValue<std::vector<std::string>>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT", "STEP_INFO_COMPONENTS"});

    //
    // step_info["trajectory_data"]
    //
    if (std::find(step_info_components.begin(), step_info_components.end(), "trajectory_data") != step_info_components.end()) {
        generateTrajectoryToGoal();
    }
}

bool OpenBotAgent::isReady() const
{
    return open_bot_pawn_->GetVelocity().Size() <= Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT", "IS_READY_VELOCITY_THRESHOLD"});
}

void OpenBotAgent::buildNavMesh()
{
    // Set the navmesh properties
    nav_mesh_->AgentRadius            = Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT", "NAVMESH", "AGENT_RADIUS"});
    nav_mesh_->AgentHeight            = Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT", "NAVMESH", "AGENT_HEIGHT"});
    nav_mesh_->CellSize               = Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT", "NAVMESH", "CELL_SIZE"});
    nav_mesh_->CellHeight             = Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT", "NAVMESH", "CELL_HEIGHT"});
    nav_mesh_->AgentMaxStepHeight     = Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT", "NAVMESH", "AGENT_MAX_STEP_HEIGHT"});
    nav_mesh_->AgentMaxSlope          = Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT", "NAVMESH", "AGENT_MAX_SLOPE"});
    nav_mesh_->MergeRegionSize        = Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT", "NAVMESH", "MERGE_REGION_SIZE"});
    nav_mesh_->MinRegionArea          = Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT", "NAVMESH", "MIN_REGION_AREA"});
    nav_mesh_->TileSizeUU             = Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT", "NAVMESH", "TILE_SIZE_UU"});
    nav_mesh_->TilePoolSize           = Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT", "NAVMESH", "TILE_POOL_SIZE"});
    nav_mesh_->MaxSimplificationError = Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT", "NAVMESH", "MAX_SIMPLIFICATION_ERROR"});

    // Get world bounding box
    FBox world_box(ForceInit);
    auto world_bound_tag_names = Config::getValue<std::vector<std::string>>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT", "NAVMESH", "WORLD_BOUND_TAG_NAMES"});
    for (TActorIterator<AActor> actor_itr(open_bot_pawn_->GetWorld()); actor_itr; ++actor_itr) {     
        for (auto& name : world_bound_tag_names) { 
            if (actor_itr->ActorHasTag(name.c_str())) { 
                world_box += actor_itr->GetComponentsBoundingBox(false, true);
            }
        }
    }

    // get references to ANavMeshBoundsVolume and ANavModifierVolume
    ANavMeshBoundsVolume* nav_mesh_bounds_volume = nullptr;
    for (TActorIterator<ANavMeshBoundsVolume> actor_itr(open_bot_pawn_->GetWorld()); actor_itr; ++actor_itr) {
        nav_mesh_bounds_volume = *actor_itr;
    }
    ASSERT(nav_mesh_bounds_volume);

    ANavModifierVolume* nav_modifier_volume = nullptr;
    for (TActorIterator<ANavModifierVolume> actor_itr(open_bot_pawn_->GetWorld()); actor_itr; ++actor_itr) {
        nav_modifier_volume = *actor_itr;
    }
    ASSERT(nav_modifier_volume);

    // update ANavMeshBoundsVolume
    nav_mesh_bounds_volume->GetRootComponent()->SetMobility(EComponentMobility::Movable);
    nav_mesh_bounds_volume->SetActorLocation(world_box.GetCenter(), false);
    nav_mesh_bounds_volume->SetActorRelativeScale3D(world_box.GetSize() / 200.0f);
    nav_mesh_bounds_volume->GetRootComponent()->UpdateBounds();
    nav_sys_->OnNavigationBoundsUpdated(nav_mesh_bounds_volume);
    nav_mesh_bounds_volume->GetRootComponent()->SetMobility(EComponentMobility::Static);

    // update ANavModifierVolume
    nav_modifier_volume->GetRootComponent()->SetMobility(EComponentMobility::Movable);
    nav_modifier_volume->SetActorLocation(world_box.GetCenter(), false);
    nav_modifier_volume->SetActorRelativeScale3D(world_box.GetSize() / 200.f);
    nav_modifier_volume->AddActorWorldOffset(FVector(
        Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT", "NAVMESH", "NAV_MODIFIER_OFFSET_X"}),
        Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT", "NAVMESH", "NAV_MODIFIER_OFFSET_Y"}),
        Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT", "NAVMESH", "NAV_MODIFIER_OFFSET_Z"})));
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
    if (Config::getValue<bool>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT", "NAVMESH", "EXPORT_NAV_DATA_OBJ"})) {
        nav_mesh_->GetGenerator()->ExportNavigationData(FString(Config::getValue<std::string>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT", "NAVMESH", "EXPORT_NAV_DATA_OBJ_DIR"}).c_str()) + "/" + open_bot_pawn_->GetWorld()->GetName() + "/");
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
    
    // If path generation is successful, update trajectory_
    if (path.IsSuccessful() && path.Path.IsValid()) {

        TArray<FNavPathPoint> path_points = path.Path->GetPathPoints();        
        for(auto& path_point : path_points) {
            trajectory_.push_back(path_point.Location.X);
            trajectory_.push_back(path_point.Location.Y);
            trajectory_.push_back(path_point.Location.Z);
        }

        // Debug output
        if (path.IsPartial()) {
            std::cout << "Only a partial path could be found..." << std::endl;
        }

        int num_waypoints = path.Path->GetPathPoints().Num();
        float trajectory_length = 0.0;
        for (size_t i = 0; i < num_waypoints - 1; i++) {
            trajectory_length += FVector::Dist(path_points[i].Location, path_points[i + 1].Location);
        }
        trajectory_length /= open_bot_pawn_->GetWorld()->GetWorldSettings()->WorldToMeters;
        FVector2D relative_position_to_goal((goal_actor_->GetActorLocation() - open_bot_pawn_->GetActorLocation()).X, (goal_actor_->GetActorLocation() - open_bot_pawn_->GetActorLocation()).Y);

        std::cout << std::endl;
        std::cout << "Number of waypoints: " << num_waypoints << std::endl;
        std::cout << "Goal distance: " << relative_position_to_goal.Size() / open_bot_pawn_->GetWorld()->GetWorldSettings()->WorldToMeters << "m" << std::endl;
        std::cout << "Path length: " << trajectory_length << "m" << std::endl;
        std::cout << "Initial position: [" << open_bot_pawn_->GetActorLocation().X << ", " << open_bot_pawn_->GetActorLocation().Y << ", " << open_bot_pawn_->GetActorLocation().Z << "]." << std::endl;
        std::cout << "Goal position: [" << goal_actor_->GetActorLocation().X << ", " << goal_actor_->GetActorLocation().Y << ", " << goal_actor_->GetActorLocation().Z << "]." << std::endl;
        std::cout << "-----------------------------------------------------------" << std::endl;
        std::cout << "Waypoints: " << std::endl;
        for (auto& point : path_points) {
            std::cout << "[" << point.Location.X << ", " << point.Location.Y << ", " << point.Location.Z << "]" << std::endl;
        }
        std::cout << "-----------------------------------------------------------" << std::endl;
    }
}
