#include "OpenBotAgentController.h"

#include <algorithm>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include <Components/SceneCaptureComponent2D.h>
#include <Components/StaticMeshComponent.h>
#include <Engine/TextureRenderTarget2D.h>
#include <Engine/World.h>
#include <EngineUtils.h>
#include <GameFramework/Actor.h>
#include <NavMesh/NavMeshBoundsVolume.h>
#include <NavMesh/RecastNavMesh.h>
#include <NavigationSystem.h>
#include <UObject/UObjectGlobals.h>

#include "Assert/Assert.h"
#include "Box.h"
#include "CameraSensor.h"
#include "Config.h"
#include "IMUSensor.h"
#include "OpenBotPawn.h"
#include "Serialize.h"
#include "SonarSensor.h"

OpenBotAgentController::OpenBotAgentController(UWorld* world)
{
    FActorSpawnParameters SpawnParams;
    SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;
    open_bot_pawn_ = world->SpawnActor<AOpenBotPawn>(AOpenBotPawn::StaticClass(), FVector::ZeroVector, FRotator::ZeroRotator, SpawnParams);
    ASSERT(open_bot_pawn_);

    // Setup sensors:
    for (const auto& sensor : Config::getValue<std::vector<std::string>>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "SENSORS"})) {

        if (sensor == "camera") {
            // Create camera sensor
            camera_sensor_ = std::make_unique<CameraSensor>(
                open_bot_pawn_->camera_component_,
                Config::getValue<std::vector<std::string>>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "MIXED_MODE", "RENDER_PASSES"}),
                Config::getValue<unsigned long>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "MIXED_MODE", "IMAGE_WIDTH"}),
                Config::getValue<unsigned long>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "MIXED_MODE", "IMAGE_HEIGHT"}));
            ASSERT(camera_sensor_);
        }

        if (sensor == "sonar") {
            // Create sonar sensor
            sonar_sensor_ = std::make_unique<SonarSensor>(open_bot_pawn_,
                                                          Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "SONAR_PARAMETERS", "RANGE", "MIN"}),
                                                          Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "SONAR_PARAMETERS", "RANGE", "MAX"}),
                                                          Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "SONAR_PARAMETERS", "HORIZONTAL_FOV"}),
                                                          Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "SONAR_PARAMETERS", "VERTICAL_FOV"}),
                                                          Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "SONAR_PARAMETERS", "NOISE_STD"}),
                                                          Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "SONAR_PARAMETERS", "MAX_REFLECTION_ANGLE"}),
                                                          FVector(Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "SONAR_PARAMETERS", "OFFSET_POSITION", "X"}),
                                                                  Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "SONAR_PARAMETERS", "OFFSET_POSITION", "Y"}),
                                                                  Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "SONAR_PARAMETERS", "OFFSET_POSITION", "Z"})),
                                                          FRotator(Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "SONAR_PARAMETERS", "OFFSET_ORIENTATION", "ROLL"}),
                                                                   Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "SONAR_PARAMETERS", "OFFSET_ORIENTATION", "PITCH"}),
                                                                   Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "SONAR_PARAMETERS", "OFFSET_ORIENTATION", "YAW"})),
                                                          Config::getValue<bool>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "SONAR_PARAMETERS", "DEBUG"}));
            ASSERT(sonar_sensor_);
        }

        if (sensor == "imu") {
            // Create IMU sensor
            inertial_sensor_ = std::make_unique<IMUSensor>(open_bot_pawn_,
                                                           FVector(Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "IMU_PARAMETERS", "ACCELEROMETER_NOISE_STD", "X"}),
                                                                   Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "IMU_PARAMETERS", "ACCELEROMETER_NOISE_STD", "Y"}),
                                                                   Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "IMU_PARAMETERS", "ACCELEROMETER_NOISE_STD", "Z"})),
                                                           FVector(Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "IMU_PARAMETERS", "GYROSCOPE_NOISE_STD", "X"}),
                                                                   Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "IMU_PARAMETERS", "GYROSCOPE_NOISE_STD", "Y"}),
                                                                   Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "IMU_PARAMETERS", "GYROSCOPE_NOISE_STD", "Z"})),
                                                           FVector(Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "IMU_PARAMETERS", "GYROSCOPE_BIAS", "X"}),
                                                                   Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "IMU_PARAMETERS", "GYROSCOPE_BIAS", "Y"}),
                                                                   Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "IMU_PARAMETERS", "GYROSCOPE_BIAS", "Z"})),
                                                           FVector(Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "IMU_PARAMETERS", "OFFSET_POSITION", "X"}),
                                                                   Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "IMU_PARAMETERS", "OFFSET_POSITION", "Y"}),
                                                                   Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "IMU_PARAMETERS", "OFFSET_POSITION", "Z"})),
                                                           FRotator(Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "IMU_PARAMETERS", "OFFSET_ORIENTATION", "ROLL"}),
                                                                    Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "IMU_PARAMETERS", "OFFSET_ORIENTATION", "PITCH"}),
                                                                    Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "IMU_PARAMETERS", "OFFSET_ORIENTATION", "YAW"})),
                                                           Config::getValue<bool>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "IMU_PARAMETERS", "DEBUG"}));
            ASSERT(inertial_sensor_);
        }
    }

    // Setup observation camera
    if (Config::getValue<std::string>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "OBSERVATION_MODE"}) == "mixed") {
    }
}

OpenBotAgentController::~OpenBotAgentController()
{
    for (const auto& sensor : Config::getValue<std::vector<std::string>>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "SENSORS"})) {

        if (sensor == "camera") {
            ASSERT(camera_sensor_);
            camera_sensor_ = nullptr;
        }

        if (sensor == "sonar") {
            ASSERT(sonar_sensor_);
            sonar_sensor_ = nullptr;
        }

        if (sensor == "imu") {
            ASSERT(inertial_sensor_);
            inertial_sensor_ = nullptr;
        }
    }

    ASSERT(open_bot_pawn_);
    open_bot_pawn_->Destroy();
    open_bot_pawn_ = nullptr;
}

void OpenBotAgentController::findObjectReferences(UWorld* world)
{
    for (TActorIterator<AActor> actor_itr(world, AActor::StaticClass()); actor_itr; ++actor_itr) {
        std::string actor_name = TCHAR_TO_UTF8(*(*actor_itr)->GetName());
        if (actor_name == Config::getValue<std::string>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "GOAL_ACTOR_NAME"})) {
            ASSERT(!goal_actor_);
            goal_actor_ = *actor_itr;
            break;
        }
    }
    ASSERT(goal_actor_);

    // Agent Navigation:
    // Get a pointer to the navigation system
    nav_sys_ = FNavigationSystem::GetCurrent<UNavigationSystemV1>(world);
    ASSERT(nav_sys_);

    // Get a pointer to the agent's navigation data
    const INavAgentInterface* actor_as_nav_agent = CastChecked<INavAgentInterface>(open_bot_pawn_);
    ASSERT(actor_as_nav_agent);
    nav_data_ = nav_sys_->GetNavDataForProps(actor_as_nav_agent->GetNavAgentPropertiesRef(), actor_as_nav_agent->GetNavAgentLocation());
    ASSERT(nav_data_);

    // Get a pointer to the navigation mesh
    nav_mesh_ = Cast<ARecastNavMesh>(nav_data_);
    ASSERT(nav_mesh_);

    // Rebuild navigation mesh with the desired properties before executing trajectory planning
    buildNavMesh();
}

void OpenBotAgentController::cleanUpObjectReferences()
{
    ASSERT(nav_mesh_);
    nav_mesh_ = nullptr;

    ASSERT(nav_data_);
    nav_data_ = nullptr;

    ASSERT(nav_sys_);
    nav_sys_ = nullptr;

    ASSERT(goal_actor_);
    goal_actor_ = nullptr;
}

std::map<std::string, Box> OpenBotAgentController::getActionSpace() const
{

    std::map<std::string, Box> action_space;
    Box box;

    if (Config::getValue<std::string>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "ACTION_MODE"}) == "low_level_control") {
        box.low = -1.f;
        box.high = 1.f;
        box.shape = {2};
        box.dtype = DataType::Float32;
        action_space["apply_voltage"] = std::move(box);
    }
    else if (Config::getValue<std::string>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "ACTION_MODE"}) == "teleport") {
        box.low = std::numeric_limits<float>::lowest();
        box.high = std::numeric_limits<float>::max();
        box.shape = {3};
        box.dtype = DataType::Float32;
        action_space["set_position_xyz_centimeters"] = std::move(box);

        box.low = std::numeric_limits<float>::lowest();
        box.high = std::numeric_limits<float>::max();
        box.shape = {3};
        box.dtype = DataType::Float32;
        action_space["set_orientation_pyr_radians"] = std::move(box);
    }
    else {
        ASSERT(false);
    }

    return action_space;
}

std::map<std::string, Box> OpenBotAgentController::getObservationSpace() const
{
    std::map<std::string, Box> observation_space;
    Box box;

    box.low = std::numeric_limits<float>::lowest();
    box.high = std::numeric_limits<float>::max();
    box.dtype = DataType::Float32;

    box.shape = {2};
    observation_space["control_data"] = std::move(box); // ctrl_left, ctrl_right

    box.shape = {6};
    observation_space["state_data"] = std::move(box); // position (X, Y, Z) and orientation (Roll, Pitch, Yaw) of the agent relative to the world frame.

    for (const auto& sensor : Config::getValue<std::vector<std::string>>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "SENSORS"})) {

        if (sensor == "camera") {
            for (const auto& render_pass : Config::getValue<std::vector<std::string>>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "RENDER_PASSES"})) {
                // RGB camera
                if (render_pass == "final_color") {
                    box.low = 0;
                    box.high = 255;
                    box.shape = {Config::getValue<long>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "IMAGE_HEIGHT"}),
                                 Config::getValue<long>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "IMAGE_WIDTH"}),
                                 3};
                    box.dtype = DataType::UInteger8;
                    observation_space["rgb"] = std::move(box);
                }

                // Depth camera
                if (render_pass == "depth_glsl") {
                    box.low = 0;
                    box.high = 255;
                    box.shape = {Config::getValue<long>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "IMAGE_HEIGHT"}),
                                 Config::getValue<long>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "IMAGE_WIDTH"}),
                                 1};
                    box.dtype = DataType::UInteger8;
                    observation_space["depth"] = std::move(box);
                }

                // Segmentation camera
                if (render_pass == "segmentation") {
                    box.low = 0;
                    box.high = 255;
                    box.shape = {Config::getValue<long>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "IMAGE_HEIGHT"}),
                                 Config::getValue<long>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "IMAGE_WIDTH"}),
                                 3};
                    box.dtype = DataType::UInteger8;
                    observation_space["segmentation"] = std::move(box);
                }
            }
        }

        if (sensor == "sonar") {
            box.shape = {1};
            observation_space["sonar_data"] = std::move(box); // Front obstacle distance
        }

        if (sensor == "imu") {
            box.shape = {6};
            observation_space["imu_data"] = std::move(box); // a_x, a_y, a_z, g_x, g_y, g_z
        }
    }

    return observation_space;
}

std::map<std::string, Box> OpenBotAgentController::getStepInfoSpace() const
{
    std::map<std::string, Box> step_info_space;
    Box box;

    box.low = std::numeric_limits<float>::lowest();
    box.high = std::numeric_limits<float>::max();
    box.dtype = DataType::Float32;
    box.shape = {-1};
    step_info_space["trajectory_data"] = std::move(box); // Vector of the waypoints (X, Y, Z) building the desired trajectory relative to the world frame.

    return step_info_space;
}

void OpenBotAgentController::applyAction(const std::map<std::string, std::vector<float>>& action)
{
    if (Config::getValue<std::string>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "ACTION_MODE"}) == "low_level_control") {
        Eigen::Vector4f duty_cycle(action.at("apply_voltage").at(0), action.at("apply_voltage").at(1), action.at("apply_voltage").at(0), action.at("apply_voltage").at(1));
        open_bot_pawn_->setDutyCycleAndClamp(duty_cycle);
    }
    else if (Config::getValue<std::string>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "ACTION_MODE"}) == "teleport") {
        const FVector agent_location{action.at("set_position_xyz_centimeters").at(0), action.at("set_position_xyz_centimeters").at(1), action.at("set_position_xyz_centimeters").at(2)};
        const FRotator agent_rotation{FMath::RadiansToDegrees(action.at("set_orientation_pyr_radians").at(0)), FMath::RadiansToDegrees(action.at("set_orientation_pyr_radians").at(1)), FMath::RadiansToDegrees(action.at("set_orientation_pyr_radians").at(2))};

        constexpr bool sweep = false;
        constexpr FHitResult* hit_result_info = nullptr;
        open_bot_pawn_->SetActorLocationAndRotation(agent_location, FQuat(agent_rotation), sweep, hit_result_info, ETeleportType::TeleportPhysics);
    }
    else {
        ASSERT(false);
    }
}

std::map<std::string, std::vector<uint8_t>> OpenBotAgentController::getObservation() const
{
    std::map<std::string, std::vector<uint8_t>> observation;

    // Get observations
    const FVector agent_current_location = open_bot_pawn_->GetActorLocation();
    const FRotator agent_current_orientation = open_bot_pawn_->GetActorRotation();

    Eigen::Vector4f duty_cycle = open_bot_pawn_->getDutyCycle();
    Eigen::Vector2f control_state;
    
    // Fill the observed action vector to be used for RL purposes:
    control_state(0) = (duty_cycle(0) + duty_cycle(2)) / 2; // leftCtrl
    control_state(1) = (duty_cycle(1) + duty_cycle(3)) / 2; // rightCtrl

    observation["control_data"] = Serialize::toUint8(std::vector<float>{control_state(0), control_state(1)});
    observation["state_data"] = Serialize::toUint8(std::vector<float>{agent_current_location.X, agent_current_location.Y, agent_current_location.Z, FMath::DegreesToRadians(agent_current_orientation.Roll), FMath::DegreesToRadians(agent_current_orientation.Pitch), FMath::DegreesToRadians(agent_current_orientation.Yaw)});

    for (const auto& sensor : Config::getValue<std::vector<std::string>>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "SENSORS"})) {

        if (sensor == "camera") {
            ASSERT(IsInGameThread());

            // Get render data
            std::map<std::string, TArray<FColor>> render_data = camera_sensor_->getRenderData();
            ASSERT(render_data.size() > 0);

            // Parallelize the camera sampling process
            std::vector<std::string> render_passes = Config::getValue<std::vector<std::string>>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "RENDER_PASSES"});
            ParallelFor(render_passes.size(), [&](int idx) {
                // RGB camera
                if (render_passes.at(idx) == "final_color") {

                    std::vector<uint8_t> image(Config::getValue<int>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "IMAGE_HEIGHT"}) *
                                               Config::getValue<int>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "IMAGE_WIDTH"}) *
                                               3);

                    const auto& data = render_data.at("final_color");

                    for (uint32 i = 0; i < static_cast<uint32>(data.Num()); ++i) {
                        image.at(3 * i + 0) = data[i].R;
                        image.at(3 * i + 1) = data[i].G;
                        image.at(3 * i + 2) = data[i].B;
                    }

                    observation["rgb"] = std::move(image);
                }

                // Depth camera
                if (render_passes.at(idx) == "depth_glsl") {

                    std::vector<uint8_t> image(Config::getValue<int>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "IMAGE_HEIGHT"}) *
                                               Config::getValue<int>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "IMAGE_WIDTH"}));

                    // Convert RGB-encoded depth image to metric depth image
                    const auto& data = CameraSensor::getFloatDepthFromColorDepth(render_data.at("depth_glsl"));

                    for (uint32 i = 0; i < static_cast<uint32>(data.size()); ++i) {
                        image.at(i) = static_cast<uint8_t>(25.5 * data[i]);
                    }

                    observation["depth"] = std::move(image);
                }

                // Segmentation camera
                if (render_passes.at(idx) == "segmentation") {

                    std::vector<uint8_t> image(Config::getValue<int>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "IMAGE_HEIGHT"}) *
                                               Config::getValue<int>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "IMAGE_WIDTH"}) *
                                               3);

                    const auto& data = render_data.at("segmentation");

                    for (uint32 i = 0; i < static_cast<uint32>(data.Num()); ++i) {
                        image.at(3 * i + 0) = data[i].R;
                        image.at(3 * i + 1) = data[i].G;
                        image.at(3 * i + 2) = data[i].B;
                    }

                    observation["segmentation"] = std::move(image);
                }
            });
        }

        if (sensor == "sonar") {
            // Get sonar measurement:
            float sonar_range = Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "SONAR_PARAMETERS", "RANGE", "MAX"});
            sonar_sensor_->update(sonar_range);
            observation["sonar_data"] = Serialize::toUint8(std::vector<float>{sonar_range});
        }

        if (sensor == "imu") {
            // Get IMU measurement:
            FVector acceleration = FVector::ZeroVector;
            FVector angular_rate = FVector::ZeroVector;
            inertial_sensor_->update(acceleration, angular_rate, Config::getValue<float>({"SIMULATION_CONTROLLER", "SIMULATION_STEP_TIME_SECONDS"}));
            observation["imu_data"] = Serialize::toUint8(std::vector<float>{acceleration.X, acceleration.Y, acceleration.Z, angular_rate.X, angular_rate.Y, angular_rate.Z});
        }
    }

    return observation;
}

std::map<std::string, std::vector<uint8_t>> OpenBotAgentController::getStepInfo() const
{
    std::map<std::string, std::vector<uint8_t>> step_info;

    step_info["trajectory_data"] = Serialize::toUint8(trajectory_);

    return step_info;
}

void OpenBotAgentController::reset()
{
    const FVector agent_location = open_bot_pawn_->GetActorLocation();
    open_bot_pawn_->SetActorLocationAndRotation(agent_location, FQuat(FRotator(0)), false, nullptr, ETeleportType::TeleportPhysics);
    open_bot_pawn_->resetPhysicsState();

    // Trajectory generation between the start and goal points
    generateTrajectoryToTarget();
}

bool OpenBotAgentController::isReady() const
{
    ASSERT(open_bot_pawn_);
    return open_bot_pawn_->GetVelocity().Size() < Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "AGENT_READY_VELOCITY_THRESHOLD"});
}

void OpenBotAgentController::buildNavMesh()
{
    ASSERT(nav_sys_);
    ASSERT(nav_mesh_);

    // Set the navigation mesh properties:
    nav_mesh_->AgentRadius = Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "NAVMESH", "AGENT_RADIUS"});
    nav_mesh_->AgentHeight = Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "NAVMESH", "AGENT_HEIGHT"});
    nav_mesh_->CellSize = Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "NAVMESH", "CELL_SIZE"});
    nav_mesh_->CellHeight = Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "NAVMESH", "CELL_HEIGHT"});
    nav_mesh_->AgentMaxSlope = Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "NAVMESH", "AGENT_MAX_SLOPE"});
    nav_mesh_->AgentMaxStepHeight = Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "NAVMESH", "AGENT_MAX_STEP_HEIGHT"});
    nav_mesh_->MergeRegionSize = Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "NAVMESH", "MERGE_REGION_SIZE"});
    nav_mesh_->MinRegionArea = Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "NAVMESH", "MIN_REGION_AREA"}); // ignore region that are too small
    nav_mesh_->MaxSimplificationError = Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "NAVMESH", "MAX_SIMPLIFINCATION_ERROR"});

    // Bounding box around the appartment (in the world coordinate system)
    FBox box(ForceInit);
    std::vector<std::string> world_bound_tag_names = Config::getValue<std::vector<std::string>>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "WORLD_BOUND_TAG_NAMES"});

    for (TActorIterator<AActor> actor_itr(open_bot_pawn_->GetWorld(), AActor::StaticClass()); actor_itr; ++actor_itr) {

        for (std::string tag_itr : world_bound_tag_names) {

            if (actor_itr->ActorHasTag(tag_itr.c_str())) {
                box += actor_itr->GetComponentsBoundingBox(false, true);
            }
        }
    }
    FBox environment_bounds = box.ExpandBy(box.GetSize() * 0.1f).ShiftBy(FVector(0, 0, -0.3f * box.GetSize().Z)); // Remove ceiling

    // Dynamic update navMesh location and size:
    ANavMeshBoundsVolume* nav_meshbounds_volume = nullptr;

    for (TActorIterator<ANavMeshBoundsVolume> it(open_bot_pawn_->GetWorld()); it; ++it) {
        nav_meshbounds_volume = *it;
    }
    ASSERT(nav_meshbounds_volume);

    nav_meshbounds_volume->GetRootComponent()->SetMobility(EComponentMobility::Movable); // Hack
    nav_meshbounds_volume->SetActorLocation(environment_bounds.GetCenter(), false);      // Place the navmesh at the center of the map
    std::cout << "environment_bounds: X: " << environment_bounds.GetCenter().X << ", Y: " << environment_bounds.GetCenter().Y << ", Z: " << environment_bounds.GetCenter().Z << std::endl;
    nav_meshbounds_volume->SetActorRelativeScale3D(environment_bounds.GetSize() / 200.f); // Rescale the navmesh so it matches the whole world
    std::cout << "environment_bounds.GetSize(): X: " << environment_bounds.GetSize().X << ", Y: " << environment_bounds.GetSize().Y << ", Z: " << environment_bounds.GetSize().Z << std::endl;
    nav_meshbounds_volume->GetRootComponent()->UpdateBounds();
    nav_sys_->OnNavigationBoundsUpdated(nav_meshbounds_volume);
    nav_meshbounds_volume->GetRootComponent()->SetMobility(EComponentMobility::Static);
    nav_sys_->Build(); // Rebuild NavMesh, required for update AgentRadius
}

void OpenBotAgentController::generateTrajectoryToTarget()
{
    int number_of_way_points = 0;
    FNavLocation target_location;
    FVector2D relative_position_to_target(0.0f, 0.0f);
    FPathFindingQuery nav_query;
    FPathFindingResult collision_free_path;
    TArray<FNavPathPoint> path_points;
    trajectory_.clear();

    // Sanity checks
    ASSERT(nav_data_);
    ASSERT(nav_sys_);

    // Initial agent position
    ASSERT(open_bot_pawn_);
    agent_initial_position_ = open_bot_pawn_->GetActorLocation();

    // Goal position
    ASSERT(goal_actor_);
    agent_goal_position_ = goal_actor_->GetActorLocation();

    // Update relative position between the agent and its new target:
    relative_position_to_target.X = (agent_goal_position_ - agent_initial_position_).X;
    relative_position_to_target.Y = (agent_goal_position_ - agent_initial_position_).Y;

    // Update navigation query with the new target:
    nav_query = FPathFindingQuery(open_bot_pawn_, *nav_data_, agent_initial_position_, agent_goal_position_);

    // Genrate a collision-free path between the robot position and the target point:
    collision_free_path = nav_sys_->FindPathSync(nav_query, EPathFindingMode::Type::Regular);

    // If path generation is sucessful, analyze the obtained path (it should not be too simple):
    if (collision_free_path.IsSuccessful() && collision_free_path.Path.IsValid()) {

        if (collision_free_path.IsPartial()) {
            std::cout << "Only a partial path could be found by the planner..." << std::endl;
        }

        number_of_way_points = collision_free_path.Path->GetPathPoints().Num();

        path_points = collision_free_path.Path->GetPathPoints();

        for (auto path_point : path_points) {
            trajectory_.push_back(path_point.Location.X);
            trajectory_.push_back(path_point.Location.Y);
            trajectory_.push_back(path_point.Location.Z);
        }

        std::cout << "Number of way points: " << number_of_way_points << std::endl;
        std::cout << "Target distance: " << relative_position_to_target.Size() / open_bot_pawn_->GetWorld()->GetWorldSettings()->WorldToMeters << "m" << std::endl;

        float trajectory_length = 0.0;
        for (size_t i = 0; i < number_of_way_points - 1; i++) {
            trajectory_length += FVector::Dist(path_points[i].Location, path_points[i + 1].Location);
        }

        // Scaling to meters
        trajectory_length /= open_bot_pawn_->GetWorld()->GetWorldSettings()->WorldToMeters;

        std::cout << "Path length " << trajectory_length << "m" << std::endl;
    }

    std::cout << "Initial position: [" << agent_initial_position_.X << ", " << agent_initial_position_.Y << ", " << agent_initial_position_.Z << "]." << std::endl;
    std::cout << "Reachable position: [" << agent_goal_position_.X << ", " << agent_goal_position_.Y << ", " << agent_goal_position_.Z << "]." << std::endl;
    std::cout << "-----------------------------------------------------------" << std::endl;
    std::cout << "Way points: " << std::endl;
    for (auto wayPoint : path_points) {
        std::cout << "[" << wayPoint.Location.X << ", " << wayPoint.Location.Y << ", " << wayPoint.Location.Z << "]" << std::endl;
    }
    std::cout << "-----------------------------------------------------------" << std::endl;
}
