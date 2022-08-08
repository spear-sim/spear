#include "SlamDatasetAgentController.h"

#include <algorithm>
#include <map>
#include <math.h>
#include <string>
#include <utility>
#include <vector>

#include <Camera/CameraActor.h>
#include "CameraSensor.h"

#include <Components/SceneCaptureComponent2D.h>
#include <Components/StaticMeshComponent.h>

#include <Engine/EngineTypes.h>
#include <Engine/TextureRenderTarget2D.h>
#include <Engine/World.h>

#include <Engine/PointLight.h>
#include <Engine/SpotLight.h>
#include <Engine/DirectionalLight.h>
#include <Engine/RectLight.h>

#include <EngineUtils.h>
#include <GameFramework/Actor.h>
#include "Kismet/GameplayStatics.h"
#include <UObject/UObjectGlobals.h>

#include "Assert.h"
#include "Box.h"
#include "Config.h"
#include "Serialize.h"

SlamDatasetAgentController::SlamDatasetAgentController(UWorld* world)
{
    // store ref to world
    world_ = world;

    rebuildNavSystem();

    //world_->Exec(world_, TEXT("r.RayTracing.SceneCaptures 1"));

    FVector spawn_location {Config::getValue<float>({"SIMULATION_CONTROLLER", "SLAM_DATASET_AGENT_CONTROLLER", "TRAJECTORY", "START_POS_X"}),
                            Config::getValue<float>({"SIMULATION_CONTROLLER", "SLAM_DATASET_AGENT_CONTROLLER", "TRAJECTORY", "START_POS_Y"}),
                            Config::getValue<float>({"SIMULATION_CONTROLLER", "SLAM_DATASET_AGENT_CONTROLLER", "TRAJECTORY", "START_POS_Z"})};
    // RobotSim::NavMeshUtil::GetRandomPoint(nav_mesh_, spawn_location, Config::getValue<float>({"SIMULATION_CONTROLLER", "SLAM_DATASET_AGENT_CONTROLLER", "NAVMESH", "HEIGHT_LIMIT"}));

    FActorSpawnParameters spawn_params;
    spawn_params.Name = FName(Config::getValue<std::string>({"SIMULATION_CONTROLLER", "SLAM_DATASET_AGENT_CONTROLLER", "CAMERA_ACTOR_NAME"}).c_str());
    spawn_params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

    //camera
    AActor* new_camera_ = world->SpawnActor<ACameraActor>(spawn_location, FRotator(0, 0, 0), spawn_params);
    rgb_camera_sensor_ = new CameraSensor(world, new_camera_);
    ASSERT(rgb_camera_sensor_);

    depth_camera_sensor_ = new CameraSensor(world, new_camera_);
    ASSERT(depth_camera_sensor_);
    
    std::vector<std::string> passes_ = {"Depth_GLSL"};
    //rgb_camera_sensor_->SetPostProcessBlendables(passes_);
    depth_camera_sensor_->SetPostProcessBlendables(passes_);

    new_object_parent_actor_ = world->SpawnActor<AActor>();
    ASSERT(new_object_parent_actor_);

    // create SceneCaptureComponent2D and TextureRenderTarget2D
    //scene_capture_component_ = NewObject<USceneCaptureComponent2D>(new_object_parent_actor_, TEXT("SceneCaptureComponent2D"));
    //ASSERT(scene_capture_component_);

    // set camera properties
    rgb_camera_sensor_->scene_capture_component_->bAlwaysPersistRenderingState = 1;
    rgb_camera_sensor_->scene_capture_component_->FOVAngle = Config::getValue<float>({"SIMULATION_CONTROLLER", "SLAM_DATASET_AGENT_CONTROLLER", "CAMERA_FOV"});
    
    depth_camera_sensor_->scene_capture_component_->bAlwaysPersistRenderingState = 1;
    depth_camera_sensor_->scene_capture_component_->FOVAngle = Config::getValue<float>({ "SIMULATION_CONTROLLER", "SLAM_DATASET_AGENT_CONTROLLER", "CAMERA_FOV" });
    //scene_capture_component_->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
    // scene_capture_component_->ShowFlags.SetTemporalAA(false);
    // scene_capture_component_->ShowFlags.SetAntiAliasing(true);
    //scene_capture_component_->AttachToComponent(camera_actor_->GetRootComponent(), FAttachmentTransformRules::SnapToTargetNotIncludingScale);
    //scene_capture_component_->SetVisibility(true);
    //scene_capture_component_->RegisterComponent();


    // adjust renderTarget
    rgb_camera_sensor_->SetRenderTarget(
        Config::getValue<unsigned long>({ "SIMULATION_CONTROLLER", "SLAM_DATASET_AGENT_CONTROLLER", "IMAGE_WIDTH" }),
        Config::getValue<unsigned long>({ "SIMULATION_CONTROLLER", "SLAM_DATASET_AGENT_CONTROLLER", "IMAGE_HEIGHT" }));

    depth_camera_sensor_->SetRenderTarget(
        Config::getValue<unsigned long>({ "SIMULATION_CONTROLLER", "SLAM_DATASET_AGENT_CONTROLLER", "IMAGE_WIDTH" }),
        Config::getValue<unsigned long>({ "SIMULATION_CONTROLLER", "SLAM_DATASET_AGENT_CONTROLLER", "IMAGE_HEIGHT" }));

    //rgb_camera_sensor_->ActivateBlendablePass("finalColor");
    depth_camera_sensor_->ActivateBlendablePass("Depth_GLSL");
    //texture_render_target_ = NewObject<UTextureRenderTarget2D>(new_object_parent_actor_, TEXT("TextureRenderTarget2D"));
    //ASSERT(texture_render_target_);

    //texture_render_target_->TargetGamma = GEngine->GetDisplayGamma(); // Set FrameWidth and FrameHeight: 1.2f; for Vulkan | GEngine->GetDisplayGamma(); for DX11/12
    //texture_render_target_->InitCustomFormat(Config::getValue<unsigned long>({"SIMULATION_CONTROLLER", "SLAM_DATASET_AGENT_CONTROLLER", "IMAGE_WIDTH"}),
    //                                         Config::getValue<unsigned long>({"SIMULATION_CONTROLLER", "SLAM_DATASET_AGENT_CONTROLLER", "IMAGE_HEIGHT"}),
    //                                         PF_B8G8R8A8,
    //                                         true); // PF_B8G8R8A8 disables HDR which will boost storing to disk due to less image information
    //texture_render_target_->RenderTargetFormat = ETextureRenderTargetFormat::RTF_RGBA8;
    //texture_render_target_->bGPUSharedFlag = true; // demand buffer on GPU
    //scene_capture_component_->TextureTarget = texture_render_target_;
    RemoveVirtualLights();
}

SlamDatasetAgentController::~SlamDatasetAgentController()
{
    ASSERT(new_object_parent_actor_);
    new_object_parent_actor_->Destroy();
    new_object_parent_actor_ = nullptr;

    ASSERT(rgb_camera_sensor_);
    rgb_camera_sensor_ = nullptr;

    ASSERT(depth_camera_sensor_);
    depth_camera_sensor_ = nullptr;

    ASSERT(world_);
    world_ = nullptr;
}

std::map<std::string, Box> SlamDatasetAgentController::getActionSpace() const
{
    std::map<std::string, Box> action_space;
    Box box;

    if (Config::getValue<std::string>({"SIMULATION_CONTROLLER", "SLAM_DATASET_AGENT_CONTROLLER", "ACTION_MODE"}) == "get_ceiling_images") {
        box.low = std::numeric_limits<float>::lowest();
        box.high = std::numeric_limits<float>::max();
        box.shape = {3};
        box.dtype = DataType::Float32;
        action_space["set_orientation_pyr_deg"] = std::move(box);

        box.low = std::numeric_limits<float>::lowest();
        box.high = std::numeric_limits<float>::max();
        box.shape = {3};
        box.dtype = DataType::Float32;
        action_space["set_position_cms"] = std::move(box);
    } else if (Config::getValue<std::string>({"SIMULATION_CONTROLLER", "SLAM_DATASET_AGENT_CONTROLLER", "ACTION_MODE"}) == "follow_trajectory") {
        box.low = std::numeric_limits<float>::lowest();
        box.high = std::numeric_limits<float>::max();
        box.shape = {3};
        box.dtype = DataType::Float32;
        action_space["set_orientation_pyr_deg"] = std::move(box);

        box.low = std::numeric_limits<float>::lowest();
        box.high = std::numeric_limits<float>::max();
        box.shape = {3};
        box.dtype = DataType::Float32;
        action_space["set_position_cms"] = std::move(box);
    } else {
        ASSERT(false);
    }
    
    return action_space;
}

std::map<std::string, Box> SlamDatasetAgentController::getObservationSpace() const
{
    std::map<std::string, Box> observation_space;
    Box box;
    box.low = 0;
    box.high = 255;
    box.shape = {Config::getValue<unsigned long>({"SIMULATION_CONTROLLER", "SLAM_DATASET_AGENT_CONTROLLER", "IMAGE_HEIGHT"}),
                 Config::getValue<unsigned long>({"SIMULATION_CONTROLLER", "SLAM_DATASET_AGENT_CONTROLLER", "IMAGE_WIDTH"}),
                 3};
    box.dtype = DataType::UInteger8;
    observation_space["visual_observation"] = std::move(box);

    box.low = std::numeric_limits<float>::lowest();
    box.high = std::numeric_limits<float>::max();
    box.shape = { Config::getValue<unsigned long>({"SIMULATION_CONTROLLER", "SLAM_DATASET_AGENT_CONTROLLER", "IMAGE_HEIGHT"}),
                 Config::getValue<unsigned long>({"SIMULATION_CONTROLLER", "SLAM_DATASET_AGENT_CONTROLLER", "IMAGE_WIDTH"})};
    box.dtype = DataType::Float32;
    observation_space["visual_observation_depth"] = std::move(box);

    box.low = std::numeric_limits<float>::lowest();
    box.high = std::numeric_limits<float>::max();
    box.shape = {6};
    box.dtype = DataType::Float32;
    observation_space["pose"] = std::move(box);

    box.low = std::numeric_limits<float>::lowest();
    box.high = std::numeric_limits<float>::max();
    box.shape = {1};
    box.dtype = DataType::Float32;
    observation_space["camera_horizontal_fov"] = std::move(box);

    box.low = std::numeric_limits<float>::lowest();
    box.high = std::numeric_limits<float>::max();
    box.shape = {1};
    box.dtype = DataType::Float32;
    observation_space["camera_vertical_fov"] = std::move(box);
    
    return observation_space;
}

void SlamDatasetAgentController::applyAction(const std::map<std::string, std::vector<float>>& action)
{
    if (Config::getValue<std::string>({"SIMULATION_CONTROLLER", "SLAM_DATASET_AGENT_CONTROLLER", "ACTION_MODE"}) == "get_ceiling_images") {
        ASSERT(action.count("set_orientation_pyr_deg"));
        ASSERT(action.count("set_position_cms"));

        ASSERT(std::all_of(action.at("set_orientation_pyr_deg").begin(), action.at("set_orientation_pyr_deg").end(), [](float i) -> bool {return isfinite(i);}));
        ASSERT(std::all_of(action.at("set_position_cms").begin(), action.at("set_position_cms").end(), [](float i) -> bool {return isfinite(i);}));

        const FRotator orientation {action.at("set_orientation_pyr_deg").at(1), action.at("set_orientation_pyr_deg").at(2), action.at("set_orientation_pyr_deg").at(0)};
        const FVector position = {action.at("set_position_cms").at(0), action.at("set_position_cms").at(1), action.at("set_position_cms").at(2)};
        constexpr bool sweep = false;
        constexpr FHitResult* hit_result_info = nullptr;

        rgb_camera_sensor_->camera_actor_->SetActorLocationAndRotation(position, orientation, sweep, hit_result_info, ETeleportType::TeleportPhysics);
    } else if (Config::getValue<std::string>({"SIMULATION_CONTROLLER", "SLAM_DATASET_AGENT_CONTROLLER", "ACTION_MODE"}) == "follow_trajectory") {
        // generateTrajectoryToPredefinedTarget();
        ASSERT(action.count("set_orientation_pyr_deg"));
        ASSERT(action.count("set_position_cms"));

        ASSERT(std::all_of(action.at("set_orientation_pyr_deg").begin(), action.at("set_orientation_pyr_deg").end(), [](float i) -> bool {return isfinite(i);}));
        ASSERT(std::all_of(action.at("set_position_cms").begin(), action.at("set_position_cms").end(), [](float i) -> bool {return isfinite(i);}));

        const FRotator orientation {action.at("set_orientation_pyr_deg").at(1), action.at("set_orientation_pyr_deg").at(2), action.at("set_orientation_pyr_deg").at(0)};
        const FVector position = {action.at("set_position_cms").at(0), action.at("set_position_cms").at(1), action.at("set_position_cms").at(2)};
        constexpr bool sweep = false;
        constexpr FHitResult* hit_result_info = nullptr;

        rgb_camera_sensor_->camera_actor_->SetActorLocationAndRotation(position, orientation, sweep, hit_result_info, ETeleportType::TeleportPhysics);
    } else {
        ASSERT(false);
    }
}

std::map<std::string, std::vector<uint8_t>> SlamDatasetAgentController::getObservation() const
{
    std::map<std::string, std::vector<uint8_t>> observation;

    const FVector position = rgb_camera_sensor_->camera_actor_->GetActorLocation();
    const FRotator orientation = rgb_camera_sensor_->camera_actor_->GetActorRotation();
    observation["pose"] = Serialize::toUint8(std::vector<float>{position.X, position.Y, position.Z, orientation.Roll, orientation.Pitch, orientation.Yaw});

    observation["camera_vertical_fov"] = Serialize::toUint8(std::vector<float>{rgb_camera_sensor_->scene_capture_component_->FOVAngle});
    float vfov = 2 * 180.0 * atan(tan(rgb_camera_sensor_->scene_capture_component_->FOVAngle * 3.14159 / 360.0) * (float)Config::getValue<unsigned long>({ "SIMULATION_CONTROLLER", "SLAM_DATASET_AGENT_CONTROLLER", "IMAGE_HEIGHT" }) / Config::getValue<unsigned long>({ "SIMULATION_CONTROLLER", "SLAM_DATASET_AGENT_CONTROLLER", "IMAGE_WIDTH" })) / 3.14159;
    observation["camera_horizontal_fov"] = Serialize::toUint8(std::vector<float>{vfov});
    
    ASSERT(IsInGameThread());

    //FTextureRenderTargetResource* target_resource = camera_sensor_->scene_capture_component_->TextureTarget->GameThread_GetRenderTargetResource();
    //ASSERT(target_resource);

    TArray<FColor> rgb_pixels = rgb_camera_sensor_->GetRenderData();
    TArray<FColor> depth_pixels = depth_camera_sensor_->GetRenderData();

    //struct FReadSurfaceContext
    //{
    //    FRenderTarget* src_render_target_;
    //    TArray<FColor>& out_data_;
    //    FIntRect rect_;
    //    FReadSurfaceDataFlags flags_;
    //};

    //FReadSurfaceContext context = {target_resource, pixels, FIntRect(0, 0, target_resource->GetSizeXY().X, target_resource->GetSizeXY().Y), FReadSurfaceDataFlags(RCM_UNorm, CubeFace_MAX)};

    // Required for uint8 read mode
    //context.flags_.SetLinearToGamma(false);

    //ENQUEUE_RENDER_COMMAND(ReadSurfaceCommand)([context](FRHICommandListImmediate& RHICmdList) {
    //    RHICmdList.ReadSurfaceData(context.src_render_target_->GetRenderTargetTexture(), context.rect_, context.out_data_, context.flags_);
    //});

    //FRenderCommandFence ReadPixelFence;
    //ReadPixelFence.BeginFence(true);
    //ReadPixelFence.Wait(true);

    std::vector<uint8_t> image(Config::getValue<unsigned long>({"SIMULATION_CONTROLLER", "SLAM_DATASET_AGENT_CONTROLLER", "IMAGE_HEIGHT"}) *
                                Config::getValue<unsigned long>({"SIMULATION_CONTROLLER", "SLAM_DATASET_AGENT_CONTROLLER", "IMAGE_WIDTH"}) *
                                3);
    std::vector<float> depth_image(Config::getValue<unsigned long>({ "SIMULATION_CONTROLLER", "SLAM_DATASET_AGENT_CONTROLLER", "IMAGE_HEIGHT" }) *
                                    Config::getValue<unsigned long>({ "SIMULATION_CONTROLLER", "SLAM_DATASET_AGENT_CONTROLLER", "IMAGE_WIDTH" }));

    for (uint32 i = 0; i < static_cast<uint32>(rgb_pixels.Num()); ++i) {
        image.at(3 * i + 0) = rgb_pixels[i].R;
        image.at(3 * i + 1) = rgb_pixels[i].G;
        image.at(3 * i + 2) = rgb_pixels[i].B;
    }

    //depth codification
    //far clip plane is defined to 1000 meters
    //decode formula :
    //                  depth = ((r) + (g * 256) + (b * 256 * 256)) / ((256 * 256 * 256) - 1) * f
    //far clip plane will be defined in the camera or in the shader ??
    //take UE4 info and convert to meters directly 
    float max_depth = 0.0f;
    float min_depth = 100.0f;

    for (uint32 i = 0; i < static_cast<uint32>(depth_pixels.Num()); ++i) {
        float depth = depth_pixels[i].R + (depth_pixels[i].G * 256) + (depth_pixels[i].B * 256 * 256);
        float normalized_depth = depth / ((256 * 256 * 256) - 1);
        float dist = normalized_depth * 10; // apply 1000 meters as a far clip plane as a test purpouse

        if (max_depth < dist) max_depth = dist;
        if (min_depth > dist) min_depth = dist;

        depth_image.at(i) = dist;
    }
    printf("min depth: %f  --  max depth %f \n", min_depth, max_depth);
    
    observation["visual_observation"] = std::move(image);
    observation["visual_observation_depth"] = Serialize::toUint8(depth_image);
    
    return observation;
}

void SlamDatasetAgentController::reset()
{}

bool SlamDatasetAgentController::isReady() const
{
    return true;
}

void SlamDatasetAgentController::RemoveVirtualLights(){
    TArray<AActor*> FoundActors;
    UGameplayStatics::GetAllActorsOfClass(world_, ALight::StaticClass(), FoundActors);

    for (int i = 0; i < FoundActors.Num(); i++) {
        //ASpotLight* sl = Cast<ASpotLight>(FoundActors[i]);
        //APointLight* pl = Cast<APointLight>(FoundActors[i]);
        //ARectLight* rl = Cast<ARectLight>(FoundActors[i]);
        ////ADirectionalLight* dl = Cast<ADirectionalLight>(FoundActors[i]);
        //if (rl != nullptr) {
        //    rl->SetEnabled(false);
        //    UE_LOG(LogTemp, Warning, TEXT("virtual light founded"));
        //}
        //ALight* l = Cast<ALight>(FoundActors[i]);
        //if (l != nullptr) {
        //    l->SetMobility(EComponentMobility::Movable);
        //}
    }
}

void SlamDatasetAgentController::rebuildNavSystem()
{
    nav_sys_ = FNavigationSystem::GetCurrent<UNavigationSystemV1>(world_);
    ASSERT(nav_sys_);

    FNavAgentProperties agent_properties;
    agent_properties.AgentHeight = Config::getValue<float>({"SIMULATION_CONTROLLER", "SLAM_DATASET_AGENT_CONTROLLER", "NAVMESH", "AGENT_HEIGHT"});
    agent_properties.AgentRadius = Config::getValue<float>({"SIMULATION_CONTROLLER", "SLAM_DATASET_AGENT_CONTROLLER", "NAVMESH", "AGENT_RADIUS"});

    nav_data_ = nav_sys_->GetNavDataForProps(agent_properties);
    ASSERT(nav_data_);

    nav_mesh_ = Cast<ARecastNavMesh>(nav_data_);
    ASSERT(nav_mesh_);

    ANavMeshBoundsVolume* nav_mesh_bounds = nullptr;
    for (TActorIterator<ANavMeshBoundsVolume> it(world_); it; ++it) {
        nav_mesh_bounds = *it;
    }
    ASSERT(nav_mesh_bounds);

    // Set the NavMesh properties:
    nav_mesh_->CellSize = Config::getValue<float>({"SIMULATION_CONTROLLER", "SLAM_DATASET_AGENT_CONTROLLER", "NAVMESH", "CELL_SIZE"});
    nav_mesh_->CellHeight = Config::getValue<float>({"SIMULATION_CONTROLLER", "SLAM_DATASET_AGENT_CONTROLLER", "NAVMESH", "CELL_HEIGHT"});
    nav_mesh_->AgentMaxStepHeight = Config::getValue<float>({"SIMULATION_CONTROLLER", "SLAM_DATASET_AGENT_CONTROLLER", "NAVMESH", "AGENT_MAX_STEP_HEIGHT"});

    // Dynamic update navMesh location and size
    FBox worldBox = getWorldBoundingBox();
    nav_mesh_bounds->GetRootComponent()->SetMobility(EComponentMobility::Static);
    nav_mesh_bounds->SetActorLocation(worldBox.GetCenter(), false);          // Place the navmesh at the center of the map
    // nav_mesh_bounds->SetActorRelativeScale3D(worldBox.GetSize() / 200.0f);   // Rescale the navmesh
    nav_mesh_bounds->GetRootComponent()->UpdateBounds();
    nav_sys_->OnNavigationBoundsUpdated(nav_mesh_bounds);        

    nav_sys_->Build(); // Rebuild NavMesh, required for update AgentRadius
}

FBox SlamDatasetAgentController::getWorldBoundingBox(bool bScaleCeiling)
{
    FBox box(ForceInit);
    for (TActorIterator<AActor> it(world_); it; ++it) {
        if (it->ActorHasTag("architecture") || it->ActorHasTag("furniture")) {
            box += it->GetComponentsBoundingBox(false, true);
        }
    }
    // Remove ceiling
    return !bScaleCeiling ? box : box.ExpandBy(box.GetSize() * 0.1f).ShiftBy(FVector(0, 0, -0.3f * box.GetSize().Z));
}

void SlamDatasetAgentController::generateTrajectoryToPredefinedTarget()
{
    int numIter = 0;
    int numberOfWayPoints = 0;
    float pathCriterion = 0.0f;
    float bestPathCriterion = 0.0f;
    FNavLocation bestTargetLocation;
    FVector2D relativePositionToTarget(0.0f, 0.0f);

    // DIRTY HACK for neurips:
    FVector target_position {Config::getValue<float>({"SIMULATION_CONTROLLER", "SLAM_DATASET_AGENT_CONTROLLER", "TRAJECTORY", "TARGET_POS_X"}),
                             Config::getValue<float>({"SIMULATION_CONTROLLER", "SLAM_DATASET_AGENT_CONTROLLER", "TRAJECTORY", "TARGET_POS_Y"}),
                             Config::getValue<float>({"SIMULATION_CONTROLLER", "SLAM_DATASET_AGENT_CONTROLLER", "TRAJECTORY", "TARGET_POS_Z"})};

    FVector initial_position = rgb_camera_sensor_->camera_actor_->GetActorLocation();

    // Path generation polling to get "interesting" paths in every experiment:
    float trajLength = 0.0;
    while (numIter < Config::getValue<int>({"SIMULATION_CONTROLLER", "SLAM_DATASET_AGENT_CONTROLLER", "TRAJECTORY", "MAX_ITER_REPLAN"})) // Try to generate interesting trajectories with multiple waypoints
    {
        // Update relative position between the agent and its new target:
        relativePositionToTarget.X = (target_position - initial_position).X;
        relativePositionToTarget.Y = (target_position - initial_position).Y;

        // Update navigation query with the new target:
        nav_query_ = FPathFindingQuery(rgb_camera_sensor_->camera_actor_, *nav_data_, initial_position, target_position);
        nav_query_.SetAllowPartialPaths(true);

        // Genrate a collision-free path between the initial position and the target point:
        FPathFindingResult collisionFreePath = nav_sys_->FindPathSync(nav_query_, EPathFindingMode::Type::Regular);

        // If path generation is sucessful, analyze the obtained path (it should not be too simple):
        if (collisionFreePath.IsSuccessful() and collisionFreePath.Path.IsValid()) {

            if (collisionFreePath.IsPartial()) {
                std::cout << "Only a partial path could be found by the planner..." << std::endl;
            }

            numberOfWayPoints = collisionFreePath.Path->GetPathPoints().Num();
            pathCriterion = relativePositionToTarget.Size() * Config::getValue<float>({"SIMULATION_CONTROLLER", "SLAM_DATASET_AGENT_CONTROLLER", "TRAJECTORY", "PATH_WEIGHT_DIST"}) + numberOfWayPoints * relativePositionToTarget.Size() * Config::getValue<float>({"SIMULATION_CONTROLLER", "SLAM_DATASET_AGENT_CONTROLLER", "TRAJECTORY", "PATH_WEIGHT_NUM_WAYPOINTS"});

            if (bestPathCriterion <= pathCriterion) {
                bestPathCriterion = pathCriterion;
                bestTargetLocation.Location = target_position;
                way_points_.Empty();
                way_points_ = collisionFreePath.Path->GetPathPoints();
                std::cout << "Iteration: " << numIter << std::endl;
                std::cout << "Cost: " << bestPathCriterion << std::endl;
                std::cout << "Number of way points: " << numberOfWayPoints << std::endl;
                std::cout << "Target distance: " << relativePositionToTarget.Size() * 0.01 << "m" << std::endl;

                trajLength = 0.0;
                for (size_t i = 0; i < numberOfWayPoints-1; i++) {
                    trajLength += FVector::Dist(way_points_[i].Location, way_points_[i+1].Location);
                }           
                std::cout << "Path length " << trajLength * 0.01 << "m" << std::endl;
            }
        }
        numIter++;
    }

    ASSERT(way_points_.Num() > 1);

    target_position = bestTargetLocation;

    // trajectoryLength_ = trajLength * 0.01;

    std::cout << "Initial position: [" << initial_position.X << ", " << initial_position.Y << ", " << initial_position.Z << "]." << std::endl;
    std::cout << "Reachable position: [" << target_position.X << ", " << target_position.Y << ", " << target_position.Z << "]." << std::endl;
    std::cout << "-----------------------------------------------------------" << std::endl;
    std::cout << "Way points: " << std::endl;
    for (auto wayPoint : way_points_) {
        std::cout << "[" << wayPoint.Location.X << ", " << wayPoint.Location.Y << ", " << wayPoint.Location.Z << "]" << std::endl;
    }
    std::cout << "-----------------------------------------------------------" << std::endl;
    // indexPath_ = 1; // Path point 0 is the initial robot position. getCurrentPathPoint() should therefore return the next point.
    // executionCounter_++;
}
