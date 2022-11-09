#include "CameraAgent.h"

#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <AI/NavDataGenerator.h>
#include <Camera/CameraActor.h>
#include <Components/SceneCaptureComponent2D.h>
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

CameraAgent::CameraAgent(UWorld* world)
{
    auto observation_components = Config::getValue<std::vector<std::string>>({"SIMULATION_CONTROLLER", "CAMERA_AGENT", "OBSERVATION_COMPONENTS"});

    //
    // observation["camera"]
    //
    if (std::find(observation_components.begin(), observation_components.end(), "camera") != observation_components.end()) {

        FActorSpawnParameters spawn_params;
        spawn_params.Name = FName(Config::getValue<std::string>({"SIMULATION_CONTROLLER", "CAMERA_AGENT", "CAMERA", "CAMERA_ACTOR_NAME"}).c_str());
        spawn_params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
        camera_actor_ = world->SpawnActor<ACameraActor>(FVector(0, 0, 0), FRotator(0, 0, 0), spawn_params);
        ASSERT(camera_actor_);

        camera_sensor_ = std::make_unique<CameraSensor>(
            camera_actor_->GetCameraComponent(),
            Config::getValue<std::vector<std::string>>({"SIMULATION_CONTROLLER", "CAMERA_AGENT", "CAMERA", "RENDER_PASSES"}),
            Config::getValue<unsigned long>({"SIMULATION_CONTROLLER", "CAMERA_AGENT", "CAMERA", "IMAGE_WIDTH"}),
            Config::getValue<unsigned long>({"SIMULATION_CONTROLLER", "CAMERA_AGENT", "CAMERA", "IMAGE_HEIGHT"}));
        ASSERT(camera_sensor_);

        // update FOV
        for (auto& pass : camera_sensor_->camera_passes_) {
            pass.second.scene_capture_component_->FOVAngle = Config::getValue<float>({"SIMULATION_CONTROLLER", "CAMERA_AGENT", "CAMERA", "FOV"});
        }

        // update auto-exposure settings
        if (camera_sensor_->camera_passes_.contains("final_color")) {
            camera_sensor_->camera_passes_["final_color"].scene_capture_component_->PostProcessSettings.bOverride_AutoExposureSpeedUp   = Config::getValue<bool>({"SIMULATION_CONTROLLER", "CAMERA_AGENT", "CAMERA", "FINAL_COLOR_AUTO_EXPOSURE_OVERRIDE_SPEED_UP"});
            camera_sensor_->camera_passes_["final_color"].scene_capture_component_->PostProcessSettings.AutoExposureSpeedUp             = Config::getValue<float>({"SIMULATION_CONTROLLER", "CAMERA_AGENT", "CAMERA", "FINAL_COLOR_AUTO_EXPOSURE_SPEED_UP"});
            camera_sensor_->camera_passes_["final_color"].scene_capture_component_->PostProcessSettings.bOverride_AutoExposureSpeedDown = Config::getValue<bool>({"SIMULATION_CONTROLLER", "CAMERA_AGENT", "CAMERA", "FINAL_COLOR_AUTO_EXPOSURE_OVERRIDE_SPEED_DOWN"});
            camera_sensor_->camera_passes_["final_color"].scene_capture_component_->PostProcessSettings.AutoExposureSpeedDown           = Config::getValue<float>({"SIMULATION_CONTROLLER", "CAMERA_AGENT", "CAMERA", "FINAL_COLOR_AUTO_EXPOSURE_SPEED_DOWN"});
        }
    }
}

CameraAgent::~CameraAgent()
{
    auto observation_components = Config::getValue<std::vector<std::string>>({"SIMULATION_CONTROLLER", "CAMERA_AGENT", "OBSERVATION_COMPONENTS"});

    //
    // observation["camera"]
    //
    if (std::find(observation_components.begin(), observation_components.end(), "camera") != observation_components.end()) {
        ASSERT(camera_sensor_);
        camera_sensor_ = nullptr;

        ASSERT(camera_actor_);
        camera_actor_->Destroy();
        camera_actor_ = nullptr;
    }
}

void CameraAgent::findObjectReferences(UWorld* world)
{
    // HACK: find references to spotlights and remove them
    TArray<AActor*> spot_light_actors;
    UGameplayStatics::GetAllActorsOfClass(world, ASpotLight::StaticClass(), spot_light_actors);
    for (auto spot_light_actor : spot_light_actors) {
        spot_light_actor->Destroy();
    }

    auto step_info_components = Config::getValue<std::vector<std::string>>({"SIMULATION_CONTROLLER", "CAMERA_AGENT", "STEP_INFO_COMPONENTS"});

    //
    // step_info["random_points"]
    //
    if (std::find(step_info_components.begin(), step_info_components.end(), "random_points") != step_info_components.end()) {

        UNavigationSystemV1* nav_sys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(world);
        ASSERT(nav_sys);

        FNavAgentProperties agent_properties;
        agent_properties.AgentHeight     = Config::getValue<float>({"SIMULATION_CONTROLLER", "CAMERA_AGENT", "NAVMESH", "AGENT_HEIGHT"});
        agent_properties.AgentRadius     = Config::getValue<float>({"SIMULATION_CONTROLLER", "CAMERA_AGENT", "NAVMESH", "AGENT_RADIUS"});
        agent_properties.AgentStepHeight = Config::getValue<float>({"SIMULATION_CONTROLLER", "CAMERA_AGENT", "NAVMESH", "AGENT_MAX_STEP_HEIGHT"});

        ANavigationData* nav_data = nav_sys->GetNavDataForProps(agent_properties);
        ASSERT(nav_data);

        nav_mesh_ = dynamic_cast<ARecastNavMesh*>(nav_data);
        ASSERT(nav_mesh_);

        buildNavMesh(nav_sys);
    }
}

void CameraAgent::cleanUpObjectReferences()
{
    auto step_info_components = Config::getValue<std::vector<std::string>>({"SIMULATION_CONTROLLER", "CAMERA_AGENT", "STEP_INFO_COMPONENTS"});

    //
    // step_info["random_points"]
    //
    if (std::find(step_info_components.begin(), step_info_components.end(), "random_points") != step_info_components.end()) {
        nav_mesh_ = nullptr;
    }
}

std::map<std::string, Box> CameraAgent::getActionSpace() const
{
    std::map<std::string, Box> action_space;
    Box box;

    auto action_components = Config::getValue<std::vector<std::string>>({"SIMULATION_CONTROLLER", "CAMERA_AGENT", "ACTION_COMPONENTS"});

    //
    // action["set_pose"]
    //
    if (std::find(action_components.begin(), action_components.end(), "set_pose") != action_components.end()) {
        box.low = std::numeric_limits<float>::lowest();
        box.high = std::numeric_limits<float>::max();
        box.shape = { 6 }; // x,y,z in cms and then p,y,r in degs
        box.dtype = DataType::Float32;
        action_space["set_pose"] = std::move(box);
    }

    //
    // action["set_num_random_points"]
    //
    if (std::find(action_components.begin(), action_components.end(), "set_num_random_points") != action_components.end()) {
        box.low = std::numeric_limits<uint32_t>::lowest();
        box.high = std::numeric_limits<uint32_t>::max();
        box.shape = { 1 };
        box.dtype = DataType::UInteger32;
        action_space["set_num_random_points"] = std::move(box);
    }

    return action_space;
}

std::map<std::string, Box> CameraAgent::getObservationSpace() const
{
    std::map<std::string, Box> observation_space;
    Box box;

    auto observation_components = Config::getValue<std::vector<std::string>>({"SIMULATION_CONTROLLER", "CAMERA_AGENT", "OBSERVATION_COMPONENTS"});

    //
    // observation["camera"]
    //
    if (std::find(observation_components.begin(), observation_components.end(), "camera") != observation_components.end()) {

        auto passes = Config::getValue<std::vector<std::string>>({"SIMULATION_CONTROLLER", "CAMERA_AGENT", "CAMERA", "RENDER_PASSES"});
        for (auto& pass : passes) {
            box.low = 0;
            box.high = 255;
            box.shape = { Config::getValue<int64_t>({"SIMULATION_CONTROLLER", "CAMERA_AGENT", "CAMERA", "IMAGE_HEIGHT"}),
                          Config::getValue<int64_t>({"SIMULATION_CONTROLLER", "CAMERA_AGENT", "CAMERA", "IMAGE_WIDTH"}),
                          3 };
            box.dtype = DataType::UInteger8;
            observation_space["camera_" + pass] = std::move(box);
        }
    }

    return observation_space;
}

std::map<std::string, Box> CameraAgent::getStepInfoSpace() const
{
    std::map<std::string, Box> step_info_space;
    Box box;

    auto step_info_components = Config::getValue<std::vector<std::string>>({"SIMULATION_CONTROLLER", "CAMERA_AGENT", "STEP_INFO_COMPONENTS"});

    //
    // step_info["random_points"]
    //
    if (std::find(step_info_components.begin(), step_info_components.end(), "random_points") != step_info_components.end()) {
        box.low = std::numeric_limits<float>::lowest();
        box.high = std::numeric_limits<float>::max();
        box.shape = { -1, 3 };
        box.dtype = DataType::Float32;
        step_info_space["random_points"] = std::move(box);
    }

    return step_info_space;
}

void CameraAgent::applyAction(const std::map<std::string, std::vector<float>>& action)
{
    auto action_components = Config::getValue<std::vector<std::string>>({"SIMULATION_CONTROLLER", "CAMERA_AGENT", "ACTION_COMPONENTS"});

    //
    // action["set_pose"]
    //
    if (std::find(action_components.begin(), action_components.end(), "set_pose") != action_components.end()) {
        FVector agent_location(action.at("set_pose").at(0), action.at("set_pose").at(1), action.at("set_pose").at(2));
        FRotator agent_rotation(action.at("set_pose").at(3), action.at("set_pose").at(4), action.at("set_pose").at(5));
        bool sweep = false;
        FHitResult* hit_result = nullptr;
        camera_actor_->SetActorLocationAndRotation(agent_location, agent_rotation, sweep, hit_result, ETeleportType::TeleportPhysics);
    }

    // store action because we might use it in getStepInfo(...)
    action_ = action;
}

std::map<std::string, std::vector<uint8_t>> CameraAgent::getObservation() const
{
    std::map<std::string, std::vector<uint8_t>> observation;

    auto observation_components = Config::getValue<std::vector<std::string>>({"SIMULATION_CONTROLLER", "CAMERA_AGENT", "OBSERVATION_COMPONENTS"});

    //
    // observation["camera"]
    //
    if (std::find(observation_components.begin(), observation_components.end(), "camera") != observation_components.end()) {

        std::map<std::string, TArray<FColor>> render_data_components = camera_sensor_->getRenderData();
        for (auto& render_data_component : render_data_components) {
            std::vector<uint8_t> image(
                Config::getValue<unsigned long>({"SIMULATION_CONTROLLER", "CAMERA_AGENT", "CAMERA", "IMAGE_HEIGHT"}) *
                Config::getValue<unsigned long>({"SIMULATION_CONTROLLER", "CAMERA_AGENT", "CAMERA", "IMAGE_WIDTH"}) *
                3);

            for (int i = 0; i < render_data_component.second.Num(); i++) {
                image[3 * i + 0] = render_data_component.second[i].R;
                image[3 * i + 1] = render_data_component.second[i].G;
                image[3 * i + 2] = render_data_component.second[i].B;
            }

            observation["camera_" + render_data_component.first] = std::move(image);
        }
    }

    return observation;
}

std::map<std::string, std::vector<uint8_t>> CameraAgent::getStepInfo() const
{
    std::map<std::string, std::vector<uint8_t>> step_info;

    auto step_info_components = Config::getValue<std::vector<std::string>>({"SIMULATION_CONTROLLER", "CAMERA_AGENT", "STEP_INFO_COMPONENTS"});

    //
    // step_info["random_points"]
    //
    if (std::find(step_info_components.begin(), step_info_components.end(), "random_points") != step_info_components.end()) {

        std::vector<float> random_points;
        uint32_t num_random_points = static_cast<uint32_t>(action_.at("set_num_random_points").at(0));
        for (int i = 0; i < num_random_points; i++) {
            FVector random_position = nav_mesh_->GetRandomPoint().Location;
            random_points.emplace_back(random_position.X);
            random_points.emplace_back(random_position.Y);
            random_points.emplace_back(random_position.Z);
        }

        step_info["random_points"] = Serialize::toUint8(random_points);
    }

    return step_info;
}

void CameraAgent::reset()
{}

bool CameraAgent::isReady() const
{
    return true;
}

void CameraAgent::buildNavMesh(UNavigationSystemV1* nav_sys)
{
    ASSERT(nav_mesh_);
    ASSERT(nav_sys);

    // set the navmesh properties
    nav_mesh_->AgentRadius            = Config::getValue<float>({"SIMULATION_CONTROLLER", "CAMERA_AGENT", "NAVMESH", "AGENT_RADIUS"});
    nav_mesh_->AgentHeight            = Config::getValue<float>({"SIMULATION_CONTROLLER", "CAMERA_AGENT", "NAVMESH", "AGENT_HEIGHT"});
    nav_mesh_->CellSize               = Config::getValue<float>({"SIMULATION_CONTROLLER", "CAMERA_AGENT", "NAVMESH", "CELL_SIZE"});
    nav_mesh_->CellHeight             = Config::getValue<float>({"SIMULATION_CONTROLLER", "CAMERA_AGENT", "NAVMESH", "CELL_HEIGHT"});
    nav_mesh_->AgentMaxStepHeight     = Config::getValue<float>({"SIMULATION_CONTROLLER", "CAMERA_AGENT", "NAVMESH", "AGENT_MAX_STEP_HEIGHT"});
    nav_mesh_->AgentMaxSlope          = Config::getValue<float>({"SIMULATION_CONTROLLER", "CAMERA_AGENT", "NAVMESH", "AGENT_MAX_SLOPE"});
    nav_mesh_->MergeRegionSize        = Config::getValue<float>({"SIMULATION_CONTROLLER", "CAMERA_AGENT", "NAVMESH", "MERGE_REGION_SIZE"});
    nav_mesh_->MinRegionArea          = Config::getValue<float>({"SIMULATION_CONTROLLER", "CAMERA_AGENT", "NAVMESH", "MIN_REGION_AREA"});
    nav_mesh_->TileSizeUU             = Config::getValue<float>({"SIMULATION_CONTROLLER", "CAMERA_AGENT", "NAVMESH", "TILE_SIZE_UU"});
    nav_mesh_->MaxSimplificationError = Config::getValue<float>({"SIMULATION_CONTROLLER", "CAMERA_AGENT", "NAVMESH", "MAX_SIMPLIFICATION_ERROR"});

    // get world bounding box
    FBox world_box(ForceInit);
    auto world_bound_tag_names = Config::getValue<std::vector<std::string>>({"SIMULATION_CONTROLLER", "CAMERA_AGENT", "NAVMESH", "WORLD_BOUND_TAG_NAMES"});
    for (TActorIterator<AActor> actor_itr(camera_actor_->GetWorld()); actor_itr; ++actor_itr) {
        for (auto& name : world_bound_tag_names) {
            if (actor_itr->ActorHasTag(name.c_str())) {
                world_box += actor_itr->GetComponentsBoundingBox(false, true);
            }
        }
    }

    // get references to ANavMeshBoundsVolume and ANavModifierVolume
    ANavMeshBoundsVolume* nav_mesh_bounds_volume = nullptr;
    for (TActorIterator<ANavMeshBoundsVolume> actor_itr(camera_actor_->GetWorld()); actor_itr; ++actor_itr) {
        nav_mesh_bounds_volume = *actor_itr;
    }
    ASSERT(nav_mesh_bounds_volume);

    ANavModifierVolume* nav_modifier_volume = nullptr;
    for (TActorIterator<ANavModifierVolume> actor_itr(camera_actor_->GetWorld()); actor_itr; ++actor_itr) {
        nav_modifier_volume = *actor_itr;
    }
    ASSERT(nav_modifier_volume);

    // Uupdate ANavMeshBoundsVolume
    nav_mesh_bounds_volume->GetRootComponent()->SetMobility(EComponentMobility::Movable);
    nav_mesh_bounds_volume->SetActorLocation(world_box.GetCenter(), false);
    nav_mesh_bounds_volume->SetActorRelativeScale3D(world_box.GetSize() / 200.0f);
    nav_mesh_bounds_volume->GetRootComponent()->UpdateBounds();
    nav_sys->OnNavigationBoundsUpdated(nav_mesh_bounds_volume);
    nav_mesh_bounds_volume->GetRootComponent()->SetMobility(EComponentMobility::Static);

    // update ANavModifierVolume
    nav_modifier_volume->GetRootComponent()->SetMobility(EComponentMobility::Movable);
    nav_modifier_volume->SetActorLocation(world_box.GetCenter(), false);
    nav_modifier_volume->SetActorRelativeScale3D(world_box.GetSize() / 200.f);
    nav_modifier_volume->AddActorWorldOffset(FVector(
        Config::getValue<float>({"SIMULATION_CONTROLLER", "CAMERA_AGENT", "NAVMESH", "NAV_MODIFIER_OFFSET_X"}),
        Config::getValue<float>({"SIMULATION_CONTROLLER", "CAMERA_AGENT", "NAVMESH", "NAV_MODIFIER_OFFSET_Y"}),
        Config::getValue<float>({"SIMULATION_CONTROLLER", "CAMERA_AGENT", "NAVMESH", "NAV_MODIFIER_OFFSET_Z"})));
    nav_modifier_volume->GetRootComponent()->UpdateBounds();
    nav_modifier_volume->GetRootComponent()->SetMobility(EComponentMobility::Static);
    nav_modifier_volume->RebuildNavigationData();

    // rebuild navmesh
    nav_sys->Build();

    // We need to wrap this call with guards because ExportNavigationData(...) is only implemented in non-shipping builds, see:
    //     Engine/Source/Runtime/Engine/Public/AI/NavDataGenerator.h
    //     Engine/Source/Runtime/NavigationSystem/Public/NavMesh/RecastNavMeshGenerator.h
    //     Engine/Source/Runtime/NavigationSystem/Private/NavMesh/RecastNavMeshGenerator.cpp
#if !(UE_BUILD_SHIPPING || UE_BUILD_TEST)
    if (Config::getValue<bool>({"SIMULATION_CONTROLLER", "CAMERA_AGENT", "NAVMESH", "EXPORT_NAV_DATA_OBJ"})) {
        nav_mesh_->GetGenerator()->ExportNavigationData(FString(Config::getValue<std::string>({"SIMULATION_CONTROLLER", "CAMERA_AGENT", "NAVMESH", "EXPORT_NAV_DATA_OBJ_DIR"}).c_str()) + "/" + camera_actor_->GetWorld()->GetName() + "/");
    }
#endif

}
