#include "ImageSamplingAgentController.h"

#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <AI/NavDataGenerator.h>
#include <Camera/CameraActor.h>
#include <Engine/SpotLight.h>
#include <Engine/World.h>
#include <EngineUtils.h>
#include <GameFramework/Actor.h>
#include <Kismet/GameplayStatics.h>
#include <NavigationSystem.h>
#include <NavMesh/NavMeshBoundsVolume.h>
#include <NavMesh/RecastNavMesh.h>
#include <NavModifierVolume.h>

#include "Assert/Assert.h"
#include "Box.h"
#include "CameraSensor.h"
#include "Config.h"
#include "Serialize.h"

ImageSamplingAgentController::ImageSamplingAgentController(UWorld* world)
{
    // store ref to world
    world_ = world;

    // create camera sensor
    FActorSpawnParameters spawn_params;
    spawn_params.Name = FName(Config::getValue<std::string>({"SIMULATION_CONTROLLER", "IMAGE_SAMPLING_AGENT_CONTROLLER", "CAMERA_ACTOR_NAME"}).c_str());
    spawn_params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

    camera_actor_ = world_->SpawnActor<ACameraActor>(FVector(0, 0, Config::getValue<float>({ "SIMULATION_CONTROLLER", "IMAGE_SAMPLING_AGENT_CONTROLLER", "NAVMESH", "AGENT_HEIGHT" })), FRotator(0, 0, 0), spawn_params);
    ASSERT(camera_actor_);

    camera_sensor_ = std::make_unique<CameraSensor>(camera_actor_,
        Config::getValue<std::vector<std::string>>({ "SIMULATION_CONTROLLER", "IMAGE_SAMPLING_AGENT_CONTROLLER", "RENDER_PASSES" }),
        Config::getValue<unsigned long>({ "SIMULATION_CONTROLLER", "IMAGE_SAMPLING_AGENT_CONTROLLER", "IMAGE_WIDTH" }),
        Config::getValue<unsigned long>({ "SIMULATION_CONTROLLER", "IMAGE_SAMPLING_AGENT_CONTROLLER", "IMAGE_HEIGHT" }));
    ASSERT(camera_sensor_);

    // update FOV as required by this controller
    if (Config::getValue<std::string>({ "SIMULATION_CONTROLLER", "IMAGE_SAMPLING_AGENT_CONTROLLER", "IMAGE_TYPE" }) == "rgb") {
        camera_sensor_->camera_passes_.at("final_color").scene_capture_component_->FOVAngle = 90.f;
    } else if (Config::getValue<std::string>({ "SIMULATION_CONTROLLER", "IMAGE_SAMPLING_AGENT_CONTROLLER", "IMAGE_TYPE" }) == "seg") {
        camera_sensor_->camera_passes_.at("segmentation").scene_capture_component_->FOVAngle = 90.f;
    }
}

ImageSamplingAgentController::~ImageSamplingAgentController()
{
    ASSERT(camera_sensor_);
    camera_sensor_ = nullptr;

    ASSERT(camera_actor_);
    camera_actor_->Destroy();
    camera_actor_ = nullptr;

    ASSERT(world_);
    world_ = nullptr;
}

void ImageSamplingAgentController::findObjectReferences(UWorld* world)
{
    // find references to navmeshboundvolume and navmodifier volume and rebuild navsystem. Storing nav_mesh_ reference here
    rebuildNavSystem();

    // find references to spotlights and remove them
    TArray<AActor*> actors;
    UGameplayStatics::GetAllActorsOfClass(world_, ALight::StaticClass(), actors);

    for (int i = 0; i < actors.Num(); i++) {
        ASpotLight* spot_light = Cast<ASpotLight>(actors[i]);
        if (spot_light != nullptr) {
            spot_light->Destroy();
        }
    }
}

void ImageSamplingAgentController::cleanUpObjectReferences()
{
    // unassign nav_mesh_ reference
    nav_mesh_ = nullptr;
}

std::map<std::string, Box> ImageSamplingAgentController::getActionSpace() const
{
    std::map<std::string, Box> action_space;
    Box box;

    box.low = std::numeric_limits<float>::lowest();
    box.high = std::numeric_limits<float>::max();
    box.shape = { 3 };
    box.dtype = DataType::Float32;
    action_space["set_orientation_pyr_deg"] = std::move(box);

    if (Config::getValue<std::string>({"SIMULATION_CONTROLLER", "IMAGE_SAMPLING_AGENT_CONTROLLER", "ACTION_MODE"}) == "sample_images") {
        box.low = std::numeric_limits<float>::lowest();
        box.high = std::numeric_limits<float>::max();
        box.shape = {1};
        box.dtype = DataType::Float32;
        action_space["set_agent_height_cm"] = std::move(box);
    } else if (Config::getValue<std::string>({"SIMULATION_CONTROLLER", "IMAGE_SAMPLING_AGENT_CONTROLLER", "ACTION_MODE"}) == "replay_sampled_images") {
        box.low = std::numeric_limits<float>::lowest();
        box.high = std::numeric_limits<float>::max();
        box.shape = {3};
        box.dtype = DataType::Float32;
        action_space["set_position_xyz_cm"] = std::move(box);
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
    observation_space["image"] = std::move(box);

    box.low = std::numeric_limits<float>::lowest();
    box.high = std::numeric_limits<float>::max();
    box.shape = {6};
    box.dtype = DataType::Float32;
    observation_space["pose"] = std::move(box);
    
    return observation_space;
}

std::map<std::string, Box> ImageSamplingAgentController::getStepInfoSpace() const
{
    return {};
}

void ImageSamplingAgentController::applyAction(const std::map<std::string, std::vector<float>>& action)
{
    if (Config::getValue<std::string>({"SIMULATION_CONTROLLER", "IMAGE_SAMPLING_AGENT_CONTROLLER", "ACTION_MODE"}) == "sample_images") {
        ASSERT(action.count("set_orientation_pyr_deg"));
        ASSERT(action.count("set_agent_height_cm"));

        ASSERT(std::all_of(action.at("set_orientation_pyr_deg").begin(), action.at("set_orientation_pyr_deg").end(), [](float i) -> bool {return isfinite(i);}));
        ASSERT(std::all_of(action.at("set_agent_height_cm").begin(), action.at("set_agent_height_cm").end(), [](float i) -> bool {return isfinite(i);}));

        const FRotator random_orientation{ action.at("set_orientation_pyr_deg").at(0), action.at("set_orientation_pyr_deg").at(1), action.at("set_orientation_pyr_deg").at(2) };

        // sample directly from navmesh
        FVector random_position = nav_mesh_->GetRandomPoint().Location;

        random_position.Z = action.at("set_agent_height_cm").at(0);

        constexpr bool sweep = false;
        constexpr FHitResult* hit_result_info = nullptr;

        camera_actor_->SetActorLocationAndRotation(random_position, random_orientation, sweep, hit_result_info, ETeleportType::TeleportPhysics);
        
    } else if (Config::getValue<std::string>({"SIMULATION_CONTROLLER", "IMAGE_SAMPLING_AGENT_CONTROLLER", "ACTION_MODE"}) == "replay_sampled_images") {
        ASSERT(action.count("set_position_xyz_cm"));
        ASSERT(action.count("set_orientation_pyr_deg"));
        ASSERT(std::all_of(action.at("set_position_xyz_cm").begin(), action.at("set_position_xyz_cm").end(), [](float i) -> bool {return isfinite(i);}));
        ASSERT(std::all_of(action.at("set_orientation_pyr_deg").begin(), action.at("set_orientation_pyr_deg").end(), [](float i) -> bool {return isfinite(i);}));

        const FVector agent_location {action.at("set_position_xyz_cm").at(0), action.at("set_position_xyz_cm").at(1), action.at("set_position_xyz_cm").at(2)};
        const FRotator agent_rotation {action.at("set_orientation_pyr_deg").at(0), action.at("set_orientation_pyr_deg").at(1), action.at("set_orientation_pyr_deg").at(2)};

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
    
    // initialize image data
    std::vector<uint8_t> image(Config::getValue<unsigned long>({ "SIMULATION_CONTROLLER", "IMAGE_SAMPLING_AGENT_CONTROLLER", "IMAGE_HEIGHT" }) * Config::getValue<unsigned long>({ "SIMULATION_CONTROLLER", "IMAGE_SAMPLING_AGENT_CONTROLLER", "IMAGE_WIDTH" }) * 3);

    // get render data
    std::map<std::string, TArray<FColor>> render_data = camera_sensor_->GetRenderData();

    if (Config::getValue<std::string>({ "SIMULATION_CONTROLLER", "IMAGE_SAMPLING_AGENT_CONTROLLER", "IMAGE_TYPE" }) == "rgb") {
        TArray<FColor>& pixels = render_data.at("final_color");
        for (uint32 i = 0; i < static_cast<uint32>(pixels.Num()); ++i) {
            image.at(3 * i + 0) = pixels[i].R;
            image.at(3 * i + 1) = pixels[i].G;
            image.at(3 * i + 2) = pixels[i].B;
        }
    } else if (Config::getValue<std::string>({ "SIMULATION_CONTROLLER", "IMAGE_SAMPLING_AGENT_CONTROLLER", "IMAGE_TYPE" }) == "seg") {
        TArray<FColor>& pixels = render_data.at("segmentation");
        for (uint32 i = 0; i < static_cast<uint32>(pixels.Num()); ++i) {
            image.at(3 * i + 0) = pixels[i].R;
            image.at(3 * i + 1) = pixels[i].G;
            image.at(3 * i + 2) = pixels[i].B;
        }
    }

    observation["image"] = std::move(image);

    return observation;
}

std::map<std::string, std::vector<uint8_t>> ImageSamplingAgentController::getStepInfo() const
{
    return {};
}

void ImageSamplingAgentController::reset()
{
    // for debugging purposes
    if (Config::getValue<bool>({ "SIMULATION_CONTROLLER", "IMAGE_SAMPLING_AGENT_CONTROLLER", "EXPORT_NAV_DATA_DEBUG_POSES" })) {
        std::ofstream myfile;
        std::string file = Config::getValue<std::string>({ "SIMULATION_CONTROLLER", "IMAGE_SAMPLING_AGENT_CONTROLLER", "DEBUG_POSES_DIR" }) + "/" + std::string(TCHAR_TO_UTF8(*(world_->GetName()))) + "/poses_for_debug.txt";
        //UE_LOG(LogTemp, Warning, TEXT("printing filename for storing debug poses %s"), UTF8_TO_TCHAR(file.c_str()));
        myfile.open(file);
        myfile << "pos_x_cm,pos_y_cm,pos_z_cm\n";
        for (size_t i = 0u; i < Config::getValue<size_t>({ "SIMULATION_CONTROLLER", "IMAGE_SAMPLING_AGENT_CONTROLLER", "DEBUG_POSES_NUM" }); ++i) {
            FVector random_position = nav_mesh_->GetRandomPoint().Location;
            myfile << random_position.X << "," << random_position.Y << "," << random_position.Z << "\n";
        }
        myfile.close();
    }
}

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

    ANavMeshBoundsVolume* nav_mesh_bounds_volume_1 = nullptr;
    ANavMeshBoundsVolume* nav_mesh_bounds_volume_2 = nullptr;
    //ANavMeshBoundsVolume* nav_mesh_bounds_volume = nullptr;

    for (TActorIterator<ANavMeshBoundsVolume> it(world_); it; ++it) {
        std::string actor_name = TCHAR_TO_UTF8(*(*it)->GetName());
        UE_LOG(LogTemp, Warning, TEXT("NavmeshBoundsVolume name is %s"), *(*it)->GetName());
        //nav_mesh_bounds_volume = *it;

        if (!nav_mesh_bounds_volume_1) {
            nav_mesh_bounds_volume_1 = *it;
        } else if (!nav_mesh_bounds_volume_2) {
            nav_mesh_bounds_volume_2 = *it;
        }
    }
   
    //ASSERT(nav_mesh_bounds_volume);
    ASSERT(nav_mesh_bounds_volume_1);
    ASSERT(nav_mesh_bounds_volume_2);
    
    if (nav_mesh_bounds_volume_2) {
        nav_mesh_bounds_volume_2->Destroy();
        nav_mesh_bounds_volume_2 = nullptr;
    }

    ANavModifierVolume* nav_modifier_volume = nullptr;
    for (TActorIterator<ANavModifierVolume> it(world_); it; ++it) {
        nav_modifier_volume = *it;
    }
    ASSERT(nav_modifier_volume);
 
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

    // get world bounding box
    FBox worldBox(ForceInit);
    for (TActorIterator<AActor> it(world_); it; ++it) {
        if (it->ActorHasTag("architecture") || it->ActorHasTag("furniture")) {
            worldBox += it->GetComponentsBoundingBox(false, true);
        }
    }

    //FBox worldBox(box.Min, FVector(box.Max.X, box.Max.Y, box.Min.Z + Config::getValue<float>({ "SIMULATION_CONTROLLER", "IMAGE_SAMPLING_AGENT_CONTROLLER", "NAVMESH", "NAV_BOUND_BOX_HEIGHT_ABOVE_MIN" })));

    nav_mesh_bounds_volume_1->GetRootComponent()->SetMobility(EComponentMobility::Movable);
    nav_mesh_bounds_volume_1->SetActorLocation(worldBox.GetCenter(), false);
    nav_mesh_bounds_volume_1->SetActorRelativeScale3D(worldBox.GetSize() / 200.0f);
    nav_mesh_bounds_volume_1->GetRootComponent()->UpdateBounds();
    nav_sys->OnNavigationBoundsUpdated(nav_mesh_bounds_volume_1);
    nav_mesh_bounds_volume_1->GetRootComponent()->SetMobility(EComponentMobility::Static);

    nav_modifier_volume->GetRootComponent()->SetMobility(EComponentMobility::Movable);
    nav_modifier_volume->SetActorLocation(worldBox.GetCenter(), false);
    nav_modifier_volume->SetActorRelativeScale3D(worldBox.GetSize() / 200.f);
    nav_modifier_volume->AddActorWorldOffset(FVector(0, 0, 10.f));
    nav_modifier_volume->GetRootComponent()->UpdateBounds();
    nav_modifier_volume->GetRootComponent()->SetMobility(EComponentMobility::Static);
    nav_modifier_volume->RebuildNavigationData();

    nav_sys->Build(); // Rebuild NavMesh, required for update AgentRadius

    const FBox box_1 = nav_mesh_bounds_volume_1->GetComponentsBoundingBox(true, true);

    UE_LOG(LogTemp, Warning, TEXT("After nav_mesh_bounds_volume_1 : Box Min Vector is (%f, %f, %f)"), box_1.Min.X, box_1.Min.Y, box_1.Min.Z);
    UE_LOG(LogTemp, Warning, TEXT("After nav_mesh_bounds_volume_1 : Box Max Vector is (%f, %f, %f)"), box_1.Max.X, box_1.Max.Y, box_1.Max.Z);

    const FBox box_mod = nav_modifier_volume->GetComponentsBoundingBox(true, true);

    UE_LOG(LogTemp, Warning, TEXT("After nav_modifier_volume : Box Min Vector is (%f, %f, %f)"), box_mod.Min.X, box_mod.Min.Y, box_mod.Min.Z);
    UE_LOG(LogTemp, Warning, TEXT("After nav_modifier_volume : Box Max Vector is (%f, %f, %f)"), box_mod.Max.X, box_mod.Max.Y, box_mod.Max.Z);

    const FVector nav_mesh_scale_relative = nav_mesh_bounds_volume_1->GetActorRelativeScale3D();
    const FVector nav_modifier_scale_relative = nav_modifier_volume->GetActorRelativeScale3D();
    UE_LOG(LogTemp, Warning, TEXT("navmesh relative scale 3D is (%f, %f, %f)"), nav_mesh_scale_relative.X, nav_mesh_scale_relative.Y, nav_mesh_scale_relative.Z);
    UE_LOG(LogTemp, Warning, TEXT("navmodifier relative scale 3D is (%f, %f, %f)"), nav_modifier_scale_relative.X, nav_modifier_scale_relative.Y, nav_modifier_scale_relative.Z);

    const FVector nav_mesh_scale_world = nav_mesh_bounds_volume_1->GetActorScale3D();
    const FVector nav_modifier_scale_world = nav_modifier_volume->GetActorScale3D();
    UE_LOG(LogTemp, Warning, TEXT("navmesh world scale 3D is (%f, %f, %f)"), nav_mesh_scale_world.X, nav_mesh_scale_world.Y, nav_mesh_scale_world.Z);
    UE_LOG(LogTemp, Warning, TEXT("navmodifier world scale 3D is (%f, %f, %f)"), nav_modifier_scale_world.X, nav_modifier_scale_world.Y, nav_modifier_scale_world.Z);

    if (Config::getValue<bool>({ "SIMULATION_CONTROLLER", "IMAGE_SAMPLING_AGENT_CONTROLLER", "EXPORT_NAV_DATA_DEBUG_POSES" })) {
        std::ofstream myfile;
        std::string file = Config::getValue<std::string>({ "SIMULATION_CONTROLLER", "IMAGE_SAMPLING_AGENT_CONTROLLER", "DEBUG_POSES_DIR" }) + "/" + std::string(TCHAR_TO_UTF8(*(world_->GetName()))) + "/navmeshboundsvolume_bounds.txt";
        myfile.open(file);
        myfile << "pos_x_cm,pos_y_cm,pos_z_cm\n";
        myfile << box_1.Min.X << "," << box_1.Min.Y << "," << box_1.Min.Z << "\n";
        myfile << box_1.Max.X << "," << box_1.Max.Y << "," << box_1.Max.Z << "\n";
        myfile.close();

        file = Config::getValue<std::string>({ "SIMULATION_CONTROLLER", "IMAGE_SAMPLING_AGENT_CONTROLLER", "DEBUG_POSES_DIR" }) + "/" + std::string(TCHAR_TO_UTF8(*(world_->GetName()))) + "/navmodvolume_bounds.txt";
        myfile.open(file);
        myfile << "pos_x_cm,pos_y_cm,pos_z_cm\n";
        myfile << box_mod.Min.X << "," << box_mod.Min.Y << "," << box_mod.Min.Z << "\n";
        myfile << box_mod.Max.X << "," << box_mod.Max.Y << "," << box_mod.Max.Z << "\n";
        myfile.close();
    }

    if (Config::getValue<bool>({ "SIMULATION_CONTROLLER", "IMAGE_SAMPLING_AGENT_CONTROLLER", "EXPORT_NAV_DATA_OBJ"})) {
        nav_mesh_->GetGenerator()->ExportNavigationData(FString(Config::getValue<std::string>({ "SIMULATION_CONTROLLER", "IMAGE_SAMPLING_AGENT_CONTROLLER", "DEBUG_POSES_DIR" }).c_str()) + "/" + world_->GetName() + "/");
    }
}