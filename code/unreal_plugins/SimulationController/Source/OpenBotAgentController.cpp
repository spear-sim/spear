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
#include <NavigationSystem.h>
#include <NavMesh/NavMeshBoundsVolume.h>
#include <NavMesh/RecastNavMesh.h>
#include <PxRigidDynamic.h>
#include <SimpleWheeledVehicleMovementComponent.h>
#include <UObject/UObjectGlobals.h>

#include "PIPCamera.h"
#include "SimpleVehicle/SimpleVehiclePawn.h"

#include "Assert.h"
#include "Box.h"
#include "Config.h"
#include "Serialize.h"

OpenBotAgentController::OpenBotAgentController(UWorld* world)
{
    for (TActorIterator<AActor> actor_itr(world, AActor::StaticClass()); actor_itr; ++actor_itr) {
        std::string actor_name = TCHAR_TO_UTF8(*(*actor_itr)->GetName());
        if (actor_name == Config::getValue<std::string>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "AGENT_ACTOR_NAME"})) {
            ASSERT(!simple_vehicle_pawn_);
            simple_vehicle_pawn_ = dynamic_cast<ASimpleVehiclePawn*>(*actor_itr);
            ASSERT(simple_vehicle_pawn_);
        } else if (actor_name == Config::getValue<std::string>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "GOAL_ACTOR_NAME"})){
            ASSERT(!goal_actor_);
            goal_actor_ = *actor_itr;
            ASSERT(goal_actor_);
        }
    }

    ASSERT(simple_vehicle_pawn_);
    ASSERT(goal_actor_);

    // Setup observation camera
    if (Config::getValue<std::string>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "OBSERVATION_MODE"}) == "mixed") {

        TArray<AActor*> all_attached_actors;
        simple_vehicle_pawn_->GetAttachedActors(all_attached_actors, true);

        for (const auto& actor : all_attached_actors) {
            std::string actor_name = TCHAR_TO_UTF8(*actor->GetName());
            if (actor_name == Config::getValue<std::string>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "MIXED_MODE", "OBSERVATION_CAMERA_ACTOR_NAME"})) {
                ASSERT(!pip_camera_);
                pip_camera_ = dynamic_cast<APIPCamera*>(actor);
                break;
            }
        }
        ASSERT(pip_camera_);

        // create SceneCaptureComponent2D and TextureRenderTarget2D
        scene_capture_component_ = pip_camera_->GetSceneCaptureComponent();
        ASSERT(scene_capture_component_);

        // Set Camera Properties
        scene_capture_component_->bAlwaysPersistRenderingState = 1;
        scene_capture_component_->bCaptureEveryFrame = 0;
        scene_capture_component_->FOVAngle = Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "MIXED_MODE", "SMARTPHONE_FOV"}); // Smartphone FOV
        scene_capture_component_->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
        scene_capture_component_->ShowFlags.SetTemporalAA(false);
        scene_capture_component_->ShowFlags.SetAntiAliasing(true);

        new_object_parent_actor_ = world->SpawnActor<AActor>();
        ASSERT(new_object_parent_actor_);

        // Adjust RenderTarget
        texture_render_target_ = NewObject<UTextureRenderTarget2D>(new_object_parent_actor_, TEXT("TextureRenderTarget2D"));
        ASSERT(texture_render_target_);

        texture_render_target_->TargetGamma = GEngine->GetDisplayGamma(); // Set FrameWidth and FrameHeight: 1.2f; for Vulkan | GEngine->GetDisplayGamma(); for DX11/12
        texture_render_target_->InitAutoFormat(Config::getValue<unsigned long>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "MIXED_MODE", "IMAGE_WIDTH"}),
                                               Config::getValue<unsigned long>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "MIXED_MODE", "IMAGE_HEIGHT"})); // Setup the RenderTarget capture format: some random format, got crashing otherwise frameWidht = 2048 and frameHeight = 2048.
        texture_render_target_->InitCustomFormat(Config::getValue<unsigned long>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "MIXED_MODE", "IMAGE_WIDTH"}),
                                                 Config::getValue<unsigned long>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "MIXED_MODE", "IMAGE_HEIGHT"}),
                                                 PF_B8G8R8A8,
                                                 true); // PF_B8G8R8A8 disables HDR which will boost storing to disk due to less image information
        texture_render_target_->RenderTargetFormat = ETextureRenderTargetFormat::RTF_RGBA8;
        texture_render_target_->bGPUSharedFlag = true; // demand buffer on GPU
        scene_capture_component_->TextureTarget = texture_render_target_;

        // Set post processing parameters:
        FPostProcessSettings post_process_settings;
        post_process_settings.MotionBlurAmount = Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "MIXED_MODE", "MOTION_BLUR_AMOUNT"}); // Strength of motion blur, 0:off, should be renamed to intensity
        post_process_settings.MotionBlurMax = Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "MIXED_MODE", "MOTION_BLUR_MAX"});       // Max distortion caused by motion blur, in percent of the screen width, 0:off
        scene_capture_component_->PostProcessSettings = post_process_settings;
        scene_capture_component_->PostProcessBlendWeight = Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "MIXED_MODE", "POST_PROC_BLEND_WEIGHT"}); // Range (0.0, 1.0) where 0 indicates no effect, 1 indicates full effect.
    }

    // Agent Navigation:
    // Get a pointer to the navigation system
    nav_sys_ = FNavigationSystem::GetCurrent<UNavigationSystemV1>(world);
    ASSERT(nav_sys_ != nullptr);

    // Get a pointer to the agent's navigation data
    const INavAgentInterface* actor_as_nav_agent = CastChecked<INavAgentInterface>(simple_vehicle_pawn_);
    ASSERT(actor_as_nav_agent != nullptr);
    nav_data_ = nav_sys_->GetNavDataForProps(actor_as_nav_agent->GetNavAgentPropertiesRef(), actor_as_nav_agent->GetNavAgentLocation());
    ASSERT(nav_data_ != nullptr);

    // Get a pointer to the navigation mesh
    nav_mesh_ = Cast<ARecastNavMesh>(nav_data_);
    ASSERT(nav_mesh_ != nullptr);

    // Rebuild navigation mesh with the desired properties before executing trajectory planning
    buildNavMesh();
}

OpenBotAgentController::~OpenBotAgentController()
{
    if (Config::getValue<std::string>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "OBSERVATION_MODE"}) == "mixed") {

        ASSERT(texture_render_target_);
        texture_render_target_->MarkPendingKill();
        texture_render_target_ = nullptr;

        ASSERT(new_object_parent_actor_);
        new_object_parent_actor_->Destroy();
        new_object_parent_actor_ = nullptr;

        ASSERT(scene_capture_component_);
        scene_capture_component_ = nullptr;

        ASSERT(pip_camera_);
        pip_camera_ = nullptr;
    }

    ASSERT(simple_vehicle_pawn_);
    simple_vehicle_pawn_ = nullptr;

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

    box.shape = {-1};
    observation_space["trajectory_data"] = std::move(box); // Vector of the waypoints (X, Y, Z) building the desired trajectory relative to the world frame.

    ASSERT(Config::getValue<std::string>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "OBSERVATION_MODE"}) == "mixed" or Config::getValue<std::string>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "OBSERVATION_MODE"}) == "physical");
    if (Config::getValue<std::string>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "OBSERVATION_MODE"}) == "mixed") {
        box.low = 0;
        box.high = 255;
        box.shape = {Config::getValue<long>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "MIXED_MODE", "IMAGE_HEIGHT"}),
                     Config::getValue<long>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "MIXED_MODE", "IMAGE_WIDTH"}),
                     3};
        box.dtype = DataType::UInteger8;
        observation_space["visual_observation"] = std::move(box);
    }
    
    return observation_space;
}

void OpenBotAgentController::applyAction(const std::map<std::string, std::vector<float>>& action)
{
    if (Config::getValue<std::string>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "ACTION_MODE"}) == "low_level_control") {
        ASSERT(action.count("apply_voltage") == 2);
        // @TODO: This can be checked in python?
        ASSERT(action.at("apply_voltage").at(0) >= getActionSpace().at("apply_voltage").low && action.at("apply_voltage").at(0) <= getActionSpace().at("apply_voltage").high, "%f", action.at("apply_voltage").at(0));
        ASSERT(action.at("apply_voltage").at(1) >= getActionSpace().at("apply_voltage").low && action.at("apply_voltage").at(1) <= getActionSpace().at("apply_voltage").high, "%f", action.at("apply_voltage").at(1));
        simple_vehicle_pawn_->MoveLeftRight(action.at("apply_voltage").at(0), action.at("apply_voltage").at(1));
    }
    else if (Config::getValue<std::string>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "ACTION_MODE"}) == "teleport") {
        ASSERT(action.count("set_position_xyz_centimeters") == 3);
        ASSERT(isfinite(action.at("set_position_xyz_centimeters").at(0)));
        ASSERT(isfinite(action.at("set_position_xyz_centimeters").at(1)));
        ASSERT(isfinite(action.at("set_position_xyz_centimeters").at(2)));
        const FVector agent_location{action.at("set_position_xyz_centimeters").at(0), action.at("set_position_xyz_centimeters").at(1), action.at("set_position_xyz_centimeters").at(2)};

        ASSERT(action.count("set_orientation_pyr_radians") == 3);
        ASSERT(isfinite(action.at("set_orientation_pyr_radians").at(0)));
        ASSERT(isfinite(action.at("set_orientation_pyr_radians").at(1)));
        ASSERT(isfinite(action.at("set_orientation_pyr_radians").at(2)));
        const FRotator agent_rotation{FMath::RadiansToDegrees(action.at("set_orientation_pyr_radians").at(0)), FMath::RadiansToDegrees(action.at("set_orientation_pyr_radians").at(1)), FMath::RadiansToDegrees(action.at("set_orientation_pyr_radians").at(2))};

        constexpr bool sweep = false;
        constexpr FHitResult* hit_result_info = nullptr;
        simple_vehicle_pawn_->SetActorLocationAndRotation(agent_location, FQuat(agent_rotation), sweep, hit_result_info, ETeleportType::TeleportPhysics);
    }
    else {
        ASSERT(false);
    }
}

std::map<std::string, std::vector<uint8_t>> OpenBotAgentController::getObservation() const
{
    std::map<std::string, std::vector<uint8_t>> observation;

    // get observations
    if (Config::getValue<std::string>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "OBSERVATION_MODE"}) == "mixed") {
        ASSERT(IsInGameThread());

        FTextureRenderTargetResource* target_resource = scene_capture_component_->TextureTarget->GameThread_GetRenderTargetResource();
        ASSERT(target_resource);

        TArray<FColor> pixels;

        struct FReadSurfaceContext {
            FRenderTarget* src_render_target_;
            TArray<FColor>& out_data_;
            FIntRect rect_;
            FReadSurfaceDataFlags flags_;
        };

        FReadSurfaceContext context = {target_resource, pixels, FIntRect(0, 0, target_resource->GetSizeXY().X, target_resource->GetSizeXY().Y), FReadSurfaceDataFlags(RCM_UNorm, CubeFace_MAX)};

        // Required for uint8 read mode
        context.flags_.SetLinearToGamma(false);

        ENQUEUE_RENDER_COMMAND(ReadSurfaceCommand)
        ([context](FRHICommandListImmediate& RHICmdList) {
            RHICmdList.ReadSurfaceData(context.src_render_target_->GetRenderTargetTexture(), context.rect_, context.out_data_, context.flags_);
        });

        FRenderCommandFence ReadPixelFence;
        ReadPixelFence.BeginFence(true);
        ReadPixelFence.Wait(true);

        std::vector<uint8_t> image(Config::getValue<int>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "MIXED_MODE", "IMAGE_HEIGHT"}) *
                                   Config::getValue<int>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "MIXED_MODE", "IMAGE_WIDTH"}) *
                                   3);

        for (uint32 i = 0; i < static_cast<uint32>(pixels.Num()); ++i) {
            image.at(3 * i + 0) = pixels[i].R;
            image.at(3 * i + 1) = pixels[i].G;
            image.at(3 * i + 2) = pixels[i].B;
        }

        observation["visual_observation"] = std::move(image);
    }

    const FVector agent_current_location = simple_vehicle_pawn_->GetActorLocation();
    const FRotator agent_current_orientation = simple_vehicle_pawn_->GetActorRotation();

    Eigen::Vector2f control_state = simple_vehicle_pawn_->GetControlState();

    observation["control_data"] = Serialize::toUint8(std::vector<float>{control_state(0), control_state(1)});
    observation["state_data"] = Serialize::toUint8(std::vector<float>{agent_current_location.X, agent_current_location.Y, agent_current_location.Z, FMath::DegreesToRadians(agent_current_orientation.Roll), FMath::DegreesToRadians(agent_current_orientation.Pitch), FMath::DegreesToRadians(agent_current_orientation.Yaw)});
    observation["trajectory_data"] = Serialize::toUint8(trajectory_);

    return observation;
}

void OpenBotAgentController::reset()
{
    const FVector agent_location = simple_vehicle_pawn_->GetActorLocation();
    simple_vehicle_pawn_->SetActorLocationAndRotation(agent_location, FQuat(FRotator(0)), false, nullptr, ETeleportType::TeleportPhysics);

    USimpleWheeledVehicleMovementComponent* vehicle_movement_component = dynamic_cast<USimpleWheeledVehicleMovementComponent*>(simple_vehicle_pawn_->GetVehicleMovementComponent());
    ASSERT(vehicle_movement_component);

    PxRigidDynamic* rigid_body_dynamic_actor = vehicle_movement_component->PVehicle->getRigidDynamicActor();
    ASSERT(rigid_body_dynamic_actor);

    // We want to reset the physics state of OpenBot, so we are inlining the below code from
    // Engine/Source/ThirdParty/PhysX3/PhysX_3.4/Source/PhysXVehicle/src/PxVehicleDrive.cpp::setToRestState(), and
    // Engine/Source/ThirdParty/PhysX3/PhysX_3.4/Source/PhysXVehicle/src/PxVehicleWheels.cpp::setToRestState(), because these functions are protected.
    if (!(rigid_body_dynamic_actor->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC)) {
        rigid_body_dynamic_actor->setLinearVelocity(PxVec3(0, 0, 0));
        rigid_body_dynamic_actor->setAngularVelocity(PxVec3(0, 0, 0));
        rigid_body_dynamic_actor->clearForce(PxForceMode::eACCELERATION);
        rigid_body_dynamic_actor->clearForce(PxForceMode::eVELOCITY_CHANGE);
        rigid_body_dynamic_actor->clearTorque(PxForceMode::eACCELERATION);
        rigid_body_dynamic_actor->clearTorque(PxForceMode::eVELOCITY_CHANGE);
    }
    vehicle_movement_component->PVehicle->mWheelsDynData.setToRestState();
    // PVehicleDrive is not intiliazed, and so PVehicleDrive->mDriveDynData.setToRestState() is commented out. We want to know if this changes at some point.
    ASSERT(!vehicle_movement_component->PVehicleDrive);
    // vehicle_movement_component->PVehicleDrive->mDriveDynData.setToRestState(); // throws seg fault
    
    // Trajectory generation between the start and goal points
    generateTrajectoryToTarget();
}

bool OpenBotAgentController::isReady() const
{
    ASSERT(simple_vehicle_pawn_);
    

    return simple_vehicle_pawn_->GetVelocity().Size() < Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "AGENT_READY_VELOCITY_THRESHOLD"});
}

void OpenBotAgentController::buildNavMesh()
{
    ASSERT(nav_sys_ != nullptr);
    ASSERT(nav_mesh_ != nullptr);

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

    for (TActorIterator<AActor> actor_itr(simple_vehicle_pawn_->GetWorld(), AActor::StaticClass()); actor_itr; ++actor_itr) { 
        
        std::string actor_name = TCHAR_TO_UTF8(*(*actor_itr)->GetName());
        
        if (std::find(world_bound_tag_names.begin(), world_bound_tag_names.end(), actor_name) != world_bound_tag_names.end()) { // it->ActorHasTag(
            box += actor_itr->GetComponentsBoundingBox(false, true);
        }
    }
    FBox environment_bounds = box.ExpandBy(box.GetSize() * 0.1f).ShiftBy(FVector(0, 0, -0.3f * box.GetSize().Z)); // Remove ceiling

    // Dynamic update navMesh location and size:
    ANavMeshBoundsVolume* nav_meshbounds_volume = nullptr;
    
    for (TActorIterator<ANavMeshBoundsVolume> it(simple_vehicle_pawn_->GetWorld()); it; ++it) {
        nav_meshbounds_volume = *it;
    }
    ASSERT(nav_meshbounds_volume != nullptr);

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
    ASSERT(nav_data_ != nullptr);
    ASSERT(nav_sys_ != nullptr);
    
    // Initial agent position
    ASSERT(simple_vehicle_pawn_);
    agent_initial_position_ = simple_vehicle_pawn_->GetActorLocation();

    // Goal position
    ASSERT(goal_actor_);
    agent_goal_position_ = goal_actor_->GetActorLocation();
    
    // Update relative position between the agent and its new target:
    relative_position_to_target.X = (agent_goal_position_ - agent_initial_position_).X;
    relative_position_to_target.Y = (agent_goal_position_ - agent_initial_position_).Y;
    
    // Update navigation query with the new target:
    nav_query = FPathFindingQuery(simple_vehicle_pawn_, *nav_data_, agent_initial_position_, agent_goal_position_);
    
    // Genrate a collision-free path between the robot position and the target point:
    collision_free_path = nav_sys_->FindPathSync(nav_query, EPathFindingMode::Type::Regular);
    
    // If path generation is sucessful, analyze the obtained path (it should not be too simple):
    if (collision_free_path.IsSuccessful() and collision_free_path.Path.IsValid()) {

        if (collision_free_path.IsPartial()) {
            std::cout << "Only a partial path could be found by the planner..." << std::endl;
        }

        number_of_way_points = collision_free_path.Path->GetPathPoints().Num();

        path_points = collision_free_path.Path->GetPathPoints();
        
        for(auto path_point : path_points) {
            trajectory_.push_back(path_point.Location.X);
            trajectory_.push_back(path_point.Location.Y);
            trajectory_.push_back(path_point.Location.Z);
        }

        std::cout << "Number of way points: " << number_of_way_points << std::endl;
        std::cout << "Target distance: " << relative_position_to_target.Size() / simple_vehicle_pawn_->GetWorld()->GetWorldSettings()->WorldToMeters << "m" << std::endl;

        float trajectory_length = 0.0;
        for (size_t i = 0; i < number_of_way_points - 1; i++) {
            trajectory_length += FVector::Dist(path_points[i].Location, path_points[i + 1].Location);
        }

        // Scaling to meters
        trajectory_length /= simple_vehicle_pawn_->GetWorld()->GetWorldSettings()->WorldToMeters;

        std::cout << "Path length " << trajectory_length << "m" << std::endl;
    }
    
    ASSERT(path_points.Num() > 1);
    
    std::cout << "Initial position: [" << agent_initial_position_.X << ", " << agent_initial_position_.Y << ", " << agent_initial_position_.Z << "]." << std::endl;
    std::cout << "Reachable position: [" << agent_goal_position_.X << ", " << agent_goal_position_.Y << ", " << agent_goal_position_.Z << "]." << std::endl;
    std::cout << "-----------------------------------------------------------" << std::endl;
    std::cout << "Way points: " << std::endl;
    for (auto wayPoint : path_points) {
        std::cout << "[" << wayPoint.Location.X << ", " << wayPoint.Location.Y << ", " << wayPoint.Location.Z << "]" << std::endl;
    }
    std::cout << "-----------------------------------------------------------" << std::endl;
}
