#include "ImageSamplingAgentController.h"

#include <algorithm>
#include <map>
#include <string>
#include <utility>
#include <vector>

// remove before commit
#include <fstream>

#include "AI/NavDataGenerator.h"
#include <Camera/CameraActor.h>
#include <Components/SceneCaptureComponent2D.h>
#include <Components/StaticMeshComponent.h>
#include <Engine/EngineTypes.h>
#include <Engine/TextureRenderTarget2D.h>
#include <Engine/World.h>
#include <EngineUtils.h>
#include <GameFramework/Actor.h>
#include "Kismet/GameplayStatics.h"
#include <NavMesh/NavMeshBoundsVolume.h>
#include <UObject/UObjectGlobals.h>

#include <common_utils/NavMeshUtil.hpp>

//#include <VWLevelManager.h>

#include "Assert.h"
#include "Box.h"
#include "Config.h"
#include "Serialize.h"

#include "NavMeshManager.h"

ImageSamplingAgentController::ImageSamplingAgentController(UWorld* world)
{
    // store ref to world
    world_ = world;

    rebuildNavSystem();

    // NavMeshManager::navSystemRebuild(world,10);
    NavMeshManager::exportData(world);

    FVector spawn_location;
    RobotSim::NavMeshUtil::GetRandomPoint(nav_mesh_, spawn_location, Config::getValue<float>({"SIMULATION_CONTROLLER", "IMAGE_SAMPLING_AGENT_CONTROLLER", "NAVMESH", "HEIGHT_LIMIT"}));

    FActorSpawnParameters spawn_params;
    spawn_params.Name = FName(Config::getValue<std::string>({"SIMULATION_CONTROLLER", "IMAGE_SAMPLING_AGENT_CONTROLLER", "CAMERA_ACTOR_NAME"}).c_str());
    spawn_params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    camera_actor_ = world->SpawnActor<ACameraActor>(spawn_location, FRotator(0,0,0), spawn_params);
    ASSERT(camera_actor_);

    new_object_parent_actor_ = world->SpawnActor<AActor>();
    ASSERT(new_object_parent_actor_);

    // create SceneCaptureComponent2D and TextureRenderTarget2D
    scene_capture_component_ = NewObject<USceneCaptureComponent2D>(new_object_parent_actor_, TEXT("SceneCaptureComponent2D"));
    ASSERT(scene_capture_component_);

    // set camera properties
    scene_capture_component_->bAlwaysPersistRenderingState = 1;
    scene_capture_component_->FOVAngle = 90.f;
    scene_capture_component_->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
    // scene_capture_component_->ShowFlags.SetTemporalAA(false);
    // scene_capture_component_->ShowFlags.SetAntiAliasing(true);
    scene_capture_component_->AttachToComponent(camera_actor_->GetRootComponent(), FAttachmentTransformRules::SnapToTargetNotIncludingScale);
    scene_capture_component_->SetVisibility(true);
    scene_capture_component_->RegisterComponent();


    // adjust renderTarget
    texture_render_target_ = NewObject<UTextureRenderTarget2D>(new_object_parent_actor_, TEXT("TextureRenderTarget2D"));
    ASSERT(texture_render_target_);

    texture_render_target_->TargetGamma = GEngine->GetDisplayGamma(); // Set FrameWidth and FrameHeight: 1.2f; for Vulkan | GEngine->GetDisplayGamma(); for DX11/12
    texture_render_target_->InitCustomFormat(Config::getValue<unsigned long>({"SIMULATION_CONTROLLER", "IMAGE_SAMPLING_AGENT_CONTROLLER", "IMAGE_WIDTH"}),
                                             Config::getValue<unsigned long>({"SIMULATION_CONTROLLER", "IMAGE_SAMPLING_AGENT_CONTROLLER", "IMAGE_HEIGHT"}),
                                             PF_B8G8R8A8,
                                             true); // PF_B8G8R8A8 disables HDR which will boost storing to disk due to less image information
    texture_render_target_->RenderTargetFormat = ETextureRenderTargetFormat::RTF_RGBA8;
    texture_render_target_->bGPUSharedFlag = true; // demand buffer on GPU
    scene_capture_component_->TextureTarget = texture_render_target_;

    // spawn vwlevelmanager
    //virtual_world_level_manager_ = world->SpawnActor<AVWLevelManager>();
    //ASSERT(virtual_world_level_manager_);
    //AVWLevelManager* vw_level_manager = dynamic_cast<AVWLevelManager*>(virtual_world_level_manager_);
    //ASSERT(vw_level_manager);
    
    // set post processing parameters
    //if (Config::getValue<std::string>({"SIMULATION_CONTROLLER", "IMAGE_SAMPLING_AGENT_CONTROLLER", "IMAGE_TYPE"}) == "seg") {
        //scene_capture_component_->AddOrUpdateBlendable(vw_level_manager->getPostProcessMaterial(EPostProcessMaterialType::Semantic));
    //}
}

ImageSamplingAgentController::~ImageSamplingAgentController()
{
    //ASSERT(virtual_world_level_manager_);
    //virtual_world_level_manager_->Destroy();
    //virtual_world_level_manager_ = nullptr;

    ASSERT(texture_render_target_);
    texture_render_target_->MarkPendingKill();
    texture_render_target_ = nullptr;

    ASSERT(scene_capture_component_);
    scene_capture_component_ = nullptr;

    ASSERT(new_object_parent_actor_);
    new_object_parent_actor_->Destroy();
    new_object_parent_actor_ = nullptr;

    ASSERT(camera_actor_);
    camera_actor_ = nullptr;

    ASSERT(world_);
    world_ = nullptr;
}

std::map<std::string, Box> ImageSamplingAgentController::getActionSpace() const
{
    std::map<std::string, Box> action_space;
    Box box;

    if (Config::getValue<std::string>({"SIMULATION_CONTROLLER", "IMAGE_SAMPLING_AGENT_CONTROLLER", "ACTION_MODE"}) == "sample_images") {
        box.low = std::numeric_limits<float>::lowest();
        box.high = std::numeric_limits<float>::max();
        box.shape = {3};
        box.dtype = DataType::Float32;
        action_space["set_random_orientation_pyr_deg"] = std::move(box);

        box.low = std::numeric_limits<float>::lowest();
        box.high = std::numeric_limits<float>::max();
        box.shape = {1};
        box.dtype = DataType::Float32;
        action_space["set_random_agent_height_cms"] = std::move(box);
    } else if (Config::getValue<std::string>({"SIMULATION_CONTROLLER", "IMAGE_SAMPLING_AGENT_CONTROLLER", "ACTION_MODE"}) == "replay_sampled_images") {
        box.low = std::numeric_limits<float>::lowest();
        box.high = std::numeric_limits<float>::max();
        box.shape = {3};
        box.dtype = DataType::Float32;
        action_space["set_position_xyz_centimeters"] = std::move(box);

        box.low = std::numeric_limits<float>::lowest();
        box.high = std::numeric_limits<float>::max();
        box.shape = {3};
        box.dtype = DataType::Float32;
        action_space["set_orientation_pyr_degrees"] = std::move(box);
    } else {
        ASSERT(false);
    }
    
    return action_space;
}

std::map<std::string, Box> ImageSamplingAgentController::getObservationSpace() const
{
    std::map<std::string, Box> observation_space;
    Box box;
    box.low = 0;
    box.high = 255;
    box.shape = {Config::getValue<unsigned long>({"SIMULATION_CONTROLLER", "IMAGE_SAMPLING_AGENT_CONTROLLER", "IMAGE_HEIGHT"}),
                 Config::getValue<unsigned long>({"SIMULATION_CONTROLLER", "IMAGE_SAMPLING_AGENT_CONTROLLER", "IMAGE_WIDTH"}),
                 3};
    box.dtype = DataType::UInteger8;
    observation_space["visual_observation"] = std::move(box);

    box.low = std::numeric_limits<float>::lowest();
    box.high = std::numeric_limits<float>::max();
    box.shape = {6};
    box.dtype = DataType::Float32;
    observation_space["pose"] = std::move(box);
    
    return observation_space;
}

void ImageSamplingAgentController::applyAction(const std::map<std::string, std::vector<float>>& action)
{
    if (Config::getValue<std::string>({"SIMULATION_CONTROLLER", "IMAGE_SAMPLING_AGENT_CONTROLLER", "ACTION_MODE"}) == "sample_images") {
        ASSERT(action.count("set_random_orientation_pyr_deg"));
        ASSERT(action.count("set_random_agent_height_cms"));

        ASSERT(std::all_of(action.at("set_random_orientation_pyr_deg").begin(), action.at("set_random_orientation_pyr_deg").end(), [](float i) -> bool {return isfinite(i);}));
        ASSERT(std::all_of(action.at("set_random_agent_height_cms").begin(), action.at("set_random_agent_height_cms").end(), [](float i) -> bool {return isfinite(i);}));

        const FRotator random_orientation {action.at("set_random_orientation_pyr_deg").at(0), action.at("set_random_orientation_pyr_deg").at(1), action.at("set_random_orientation_pyr_deg").at(2)};

        // sample directly from navmesh
        FVector random_position = nav_mesh_->GetRandomPoint().Location;

        // FVector random_position = FVector(0);
        // int count = 0;
        // while(random_position.Size() <= 0 and count < 20) {
            // RobotSim::NavMeshUtil::GetRandomPoint(nav_mesh_, random_position, Config::getValue<float>({"SIMULATION_CONTROLLER", "IMAGE_SAMPLING_AGENT_CONTROLLER", "NAVMESH", "HEIGHT_LIMIT"}));
            // count++;
        // }
        random_position.Z = action.at("set_random_agent_height_cms").at(0);

        constexpr bool sweep = false;
        constexpr FHitResult* hit_result_info = nullptr;
        camera_actor_->SetActorLocationAndRotation(random_position, random_orientation, sweep, hit_result_info, ETeleportType::TeleportPhysics);
    } else if (Config::getValue<std::string>({"SIMULATION_CONTROLLER", "IMAGE_SAMPLING_AGENT_CONTROLLER", "ACTION_MODE"}) == "replay_sampled_images") {
        ASSERT(action.count("set_position_xyz_centimeters"));
        ASSERT(action.count("set_orientation_pyr_degrees"));
        ASSERT(std::all_of(action.at("set_position_xyz_centimeters").begin(), action.at("set_position_xyz_centimeters").end(), [](float i) -> bool {return isfinite(i);}));
        ASSERT(std::all_of(action.at("set_orientation_pyr_degrees").begin(), action.at("set_orientation_pyr_degrees").end(), [](float i) -> bool {return isfinite(i);}));

        const FVector agent_location {action.at("set_position_xyz_centimeters").at(0), action.at("set_position_xyz_centimeters").at(1), action.at("set_position_xyz_centimeters").at(2)};
        const FRotator agent_rotation {action.at("set_orientation_pyr_degrees").at(0), action.at("set_orientation_pyr_degrees").at(1), action.at("set_orientation_pyr_degrees").at(2)};

        constexpr bool sweep = false;
        constexpr FHitResult* hit_result_info = nullptr;
        camera_actor_->SetActorLocationAndRotation(agent_location, agent_rotation, sweep, hit_result_info, ETeleportType::TeleportPhysics);
    } else {
        ASSERT(false);
    }
}

std::map<std::string, std::vector<uint8_t>> ImageSamplingAgentController::getObservation() const
{
    std::map<std::string, std::vector<uint8_t>> observation;

    const FVector position = camera_actor_->GetActorLocation();
    const FRotator orientation = camera_actor_->GetActorRotation();
    observation["pose"] = Serialize::toUint8(std::vector<float>{position.X, position.Y, position.Z, orientation.Roll, orientation.Pitch, orientation.Yaw});

    std::ofstream myfile;
    std::string file = Config::getValue<std::string>({ "SIMULATION_CONTROLLER", "IMAGE_SAMPLING_AGENT_CONTROLLER", "DEBUG_POSES_DIR" }) + "/" + std::string(TCHAR_TO_UTF8(*(world_->GetName()))) + "/poses_for_debug.txt";
    UE_LOG(LogTemp, Warning, TEXT("printing filename for storing debug poses %s"), UTF8_TO_TCHAR(file.c_str()));
    myfile.open(file);
    myfile << "pos_x_cm,pos_y_cm,pos_z_cm\n";
    for (size_t i = 0u; i < Config::getValue<size_t>({ "SIMULATION_CONTROLLER", "IMAGE_SAMPLING_AGENT_CONTROLLER", "DEBUG_POSES_NUM"}); ++i) {
        FVector random_position = nav_mesh_->GetRandomPoint().Location;
        myfile << random_position.X << "," << random_position.Y << "," << random_position.Z << "\n";
    }
    myfile.close();

    ASSERT(IsInGameThread());

    FTextureRenderTargetResource* target_resource = scene_capture_component_->TextureTarget->GameThread_GetRenderTargetResource();
    ASSERT(target_resource);

    TArray<FColor> pixels;

    struct FReadSurfaceContext
    {
        FRenderTarget* src_render_target_;
        TArray<FColor>& out_data_;
        FIntRect rect_;
        FReadSurfaceDataFlags flags_;
    };

    FReadSurfaceContext context = {target_resource, pixels, FIntRect(0, 0, target_resource->GetSizeXY().X, target_resource->GetSizeXY().Y), FReadSurfaceDataFlags(RCM_UNorm, CubeFace_MAX)};

    // Required for uint8 read mode
    context.flags_.SetLinearToGamma(false);

    ENQUEUE_RENDER_COMMAND(ReadSurfaceCommand)([context](FRHICommandListImmediate& RHICmdList) {
        RHICmdList.ReadSurfaceData(context.src_render_target_->GetRenderTargetTexture(), context.rect_, context.out_data_, context.flags_);
    });

    FRenderCommandFence ReadPixelFence;
    ReadPixelFence.BeginFence(true);
    ReadPixelFence.Wait(true);

    std::vector<uint8_t> image(Config::getValue<unsigned long>({"SIMULATION_CONTROLLER", "IMAGE_SAMPLING_AGENT_CONTROLLER", "IMAGE_HEIGHT"}) *
                                Config::getValue<unsigned long>({"SIMULATION_CONTROLLER", "IMAGE_SAMPLING_AGENT_CONTROLLER", "IMAGE_WIDTH"}) *
                                3);

    for (uint32 i = 0; i < static_cast<uint32>(pixels.Num()); ++i) {
        image.at(3 * i + 0) = pixels[i].R;
        image.at(3 * i + 1) = pixels[i].G;
        image.at(3 * i + 2) = pixels[i].B;
    }
    
    observation["visual_observation"] = std::move(image);
    
    return observation;
}

void ImageSamplingAgentController::reset()
{}

bool ImageSamplingAgentController::isReady() const
{
    return true;
}

void ImageSamplingAgentController::rebuildNavSystem()
{
    UNavigationSystemV1* nav_sys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(world_);
    ASSERT(nav_sys);

    FNavAgentProperties agent_properties;
    agent_properties.AgentHeight = Config::getValue<float>({"SIMULATION_CONTROLLER", "IMAGE_SAMPLING_AGENT_CONTROLLER", "NAVMESH", "AGENT_HEIGHT"});
    agent_properties.AgentRadius = Config::getValue<float>({"SIMULATION_CONTROLLER", "IMAGE_SAMPLING_AGENT_CONTROLLER", "NAVMESH", "AGENT_RADIUS"});
    agent_properties.AgentStepHeight = Config::getValue<float>({"SIMULATION_CONTROLLER", "IMAGE_SAMPLING_AGENT_CONTROLLER", "NAVMESH", "AGENT_MAX_STEP_HEIGHT"});

    ANavigationData* nav_data = nav_sys->GetNavDataForProps(agent_properties);
    ASSERT(nav_data);

    nav_mesh_ = Cast<ARecastNavMesh>(nav_data);
    ASSERT(nav_mesh_);

    ANavMeshBoundsVolume* nav_mesh_bounds = nullptr;
    for (TActorIterator<ANavMeshBoundsVolume> it(world_); it; ++it) {
        nav_mesh_bounds = *it;
    }
    ASSERT(nav_mesh_bounds);

    // Set the NavMesh properties:
    nav_mesh_->CellSize = Config::getValue<float>({"SIMULATION_CONTROLLER", "IMAGE_SAMPLING_AGENT_CONTROLLER", "NAVMESH", "CELL_SIZE"});
    nav_mesh_->CellHeight = Config::getValue<float>({"SIMULATION_CONTROLLER", "IMAGE_SAMPLING_AGENT_CONTROLLER", "NAVMESH", "CELL_HEIGHT"});
    nav_mesh_->MergeRegionSize = Config::getValue<float>({"SIMULATION_CONTROLLER", "IMAGE_SAMPLING_AGENT_CONTROLLER", "NAVMESH", "MERGE_REGION_SIZE"});
    nav_mesh_->MinRegionArea = Config::getValue<float>({"SIMULATION_CONTROLLER", "IMAGE_SAMPLING_AGENT_CONTROLLER", "NAVMESH", "MIN_REGION_AREA"});
    nav_mesh_->AgentMaxStepHeight = Config::getValue<float>({"SIMULATION_CONTROLLER", "IMAGE_SAMPLING_AGENT_CONTROLLER", "NAVMESH", "AGENT_MAX_STEP_HEIGHT"});
    nav_mesh_->AgentMaxSlope = Config::getValue<float>({"SIMULATION_CONTROLLER", "IMAGE_SAMPLING_AGENT_CONTROLLER", "NAVMESH", "AGENT_MAX_SLOPE"});
    nav_mesh_->TileSizeUU = Config::getValue<float>({"SIMULATION_CONTROLLER", "IMAGE_SAMPLING_AGENT_CONTROLLER", "NAVMESH", "TILE_SIZE_UU"});
    nav_mesh_->AgentRadius = Config::getValue<float>({"SIMULATION_CONTROLLER", "IMAGE_SAMPLING_AGENT_CONTROLLER", "NAVMESH", "AGENT_RADIUS"});
    nav_mesh_->AgentHeight = Config::getValue<float>({"SIMULATION_CONTROLLER", "IMAGE_SAMPLING_AGENT_CONTROLLER", "NAVMESH", "AGENT_HEIGHT"});
    // Dynamic update navMesh location and size

    nav_mesh_bounds->GetRootComponent()->SetMobility(EComponentMobility::Movable);
    FBox worldBox = getWorldBoundingBox(true);
    // std::cout << "worldbox center (X,Y,Z) : (" << worldBox.GetCenter().X << worldBox.GetCenter().Y << worldBox.GetCenter().Z << ")" << std::endl;
    // UE_LOG(LogTemp, Warning, TEXT("worldbox center (X,Y,Z) : (%f, %f, %f)"), worldBox.GetCenter().X, worldBox.GetCenter().Y, worldBox.GetCenter().Z);
    nav_mesh_bounds->SetActorLocation(worldBox.GetCenter(), false);          // Place the navmesh at the center of the map
    nav_mesh_bounds->SetActorRelativeScale3D(worldBox.GetSize() / 200.0f);   // Rescale the navmesh
    nav_mesh_bounds->GetRootComponent()->UpdateBounds();
    nav_sys->OnNavigationBoundsUpdated(nav_mesh_bounds);        
    nav_mesh_bounds->GetRootComponent()->SetMobility(EComponentMobility::Static);
    nav_sys->Build(); // Rebuild NavMesh, required for update AgentRadius

    nav_mesh_->GetGenerator()->ExportNavigationData(FString(Config::getValue<std::string>({ "SIMULATION_CONTROLLER", "IMAGE_SAMPLING_AGENT_CONTROLLER", "DEBUG_POSES_DIR" }).c_str()) + "/" + world_->GetName() + "/");
}

FBox ImageSamplingAgentController::getWorldBoundingBox(bool remove_ceiling)
{
    FBox box(ForceInit);
    for (TActorIterator<AActor> it(world_); it; ++it) {
        if (it->ActorHasTag("architecture") || it->ActorHasTag("furniture")) {
            box += it->GetComponentsBoundingBox(false, true);
        }
    }
    // Remove ceiling
    // UE_LOG(LogTemp, Warning, TEXT("Before: Box Min Vector is (%f, %f, %f)"), box.Min.X, box.Min.Y, box.Min.Z);
    // UE_LOG(LogTemp, Warning, TEXT("Before: Box Max Vector is (%f, %f, %f)"), box.Max.X, box.Max.Y, box.Max.Z);
    return remove_ceiling ? FBox(box.Min, box.Max - FVector(0, 0, 0.7f * box.GetSize().Z)) : box;
    // UE_LOG(LogTemp, Warning, TEXT("After Max-(0,0,0.7*(Max-Min).Z): Box Max Vector is (%f, %f, %f)"), box.Max.X, box.Max.Y, (box.Max-FVector(0, 0, 0.7f * box.GetSize().Z)).Z);
    // FBox new_box = box.ExpandBy(box.GetSize() * 0.1f).ShiftBy(FVector(0, 0, -0.3f * box.GetSize().Z));
    // UE_LOG(LogTemp, Warning, TEXT("After box.expandby: Box Min Vector is (%f, %f, %f)"), new_box.Min.X, new_box.Min.Y, new_box.Min.Z);
    // UE_LOG(LogTemp, Warning, TEXT("After box.expandby: Box Max Vector is (%f, %f, %f)"), new_box.Max.X, new_box.Max.Y, new_box.Max.Z);
    
    // return remove_ceiling ? box.ExpandBy(box.GetSize() * 0.1f).ShiftBy(FVector(0, 0, -0.3f * box.GetSize().Z)) : box;
    // return remove_ceiling ? box.ExpandBy(box.GetSize() * 0.1f) : box; // export full box + some buffer
}
