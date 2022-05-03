#include "ImageSamplingAgentController.h"

#include <algorithm>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include <Camera/CameraActor.h>
#include <Components/SceneCaptureComponent2D.h>
#include <Components/StaticMeshComponent.h>
#include <Engine/EngineTypes.h>
#include <Engine/TextureRenderTarget2D.h>
#include <Engine/World.h>
#include <EngineUtils.h>
#include <GameFramework/Actor.h>
#include "Kismet/GameplayStatics.h"
#include <UObject/UObjectGlobals.h>

#include <common_utils/NavMeshUtil.hpp>

#include <VWLevelManager.h>

#include "Assert.h"
#include "Box.h"
#include "Config.h"
#include "Serialize.h"

ImageSamplingAgentController::ImageSamplingAgentController(UWorld* world)
{
    // store ref to world
    world_ = world;

    // get scenes
    scenes_ = Config::getValue<std::vector<std::string>>({"SIMULATION_CONTROLLER", "IMAGE_SAMPLING_AGENT_CONTROLLER", "SCENES"});

    agent_properties_.AgentHeight = Config::getValue<float>({"SIMULATION_CONTROLLER", "IMAGE_SAMPLING_AGENT_CONTROLLER", "AGENT_PROPERTIES", "AGENT_HEIGHT"});
    agent_properties_.AgentRadius = Config::getValue<float>({"SIMULATION_CONTROLLER", "IMAGE_SAMPLING_AGENT_CONTROLLER", "AGENT_PROPERTIES", "AGENT_RADIUS"});

    FVector spawn_location = getRandomPointOnNavMesh();

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
    scene_capture_component_->FOVAngle = 60.f;
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
    virtual_world_level_manager_ = world->SpawnActor<AVWLevelManager>();
    ASSERT(virtual_world_level_manager_);
    AVWLevelManager* vw_level_manager = dynamic_cast<AVWLevelManager*>(virtual_world_level_manager_);
    ASSERT(vw_level_manager);
    
    // set post processing parameters
    scene_capture_component_->AddOrUpdateBlendable(vw_level_manager->getPostProcessMaterial(EPostProcessMaterialType::Semantic));
}

ImageSamplingAgentController::~ImageSamplingAgentController()
{
    ASSERT(virtual_world_level_manager_);
    virtual_world_level_manager_->Destroy();
    virtual_world_level_manager_ = nullptr;

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
}

std::map<std::string, Box> ImageSamplingAgentController::getActionSpace() const
{
    std::map<std::string, Box> action_space;
    Box box;

    box.low = std::numeric_limits<float>::lowest();
    box.high = std::numeric_limits<float>::max();
    box.shape = {3};
    box.dtype = DataType::Float32;
    action_space["set_orientation_pyr_degrees"] = std::move(box);

    box.low = std::numeric_limits<uint16_t>::lowest();
    box.high = std::numeric_limits<uint16_t>::max();
    box.shape = {1};
    box.dtype = DataType::UInteger16;
    action_space["change_scene"] = std::move(box);

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
    if (action.count("set_orientation_pyr_degrees")) {
        ASSERT(std::all_of(action.at("set_orientation_pyr_degrees").begin(), action.at("set_orientation_pyr_degrees").end(), [](float i) -> bool {return isfinite(i);}));
        const FRotator new_orientation {action.at("set_orientation_pyr_degrees").at(0), action.at("set_orientation_pyr_degrees").at(1), action.at("set_orientation_pyr_degrees").at(2)};

        const FVector new_position = getRandomPointOnNavMesh();

        constexpr bool sweep = false;
        constexpr FHitResult* hit_result_info = nullptr;
        camera_actor_->SetActorLocationAndRotation(new_position, new_orientation, sweep, hit_result_info, ETeleportType::TeleportPhysics);

    } else if (action.count("change_scene")) {
        TArray<FString> all_available_maps;
        AVWLevelManager* vw_level_manager = dynamic_cast<AVWLevelManager*>(virtual_world_level_manager_);
        ASSERT(vw_level_manager);
        vw_level_manager->getAllMapsInPak(all_available_maps);
        UE_LOG(LogTemp, Warning, TEXT("Listing all available maps"));
        for(auto map: all_available_maps) {
            UE_LOG(LogTemp, Warning, TEXT("%s"), *map);
        }
        FString new_map = FString(("/Game/Maps/Map_"+scenes_.at(action.at("change_scene").at(0))).c_str());
        UE_LOG(LogTemp, Warning, TEXT("Loading map %s"), *new_map);
        UGameplayStatics::OpenLevel(world_, *new_map, true);
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

FVector ImageSamplingAgentController::getRandomPointOnNavMesh()
{
    // get position
    UNavigationSystemV1* nav_sys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(world_);
    ASSERT(nav_sys);
    ANavigationData* nav_data = nav_sys->GetNavDataForProps(agent_properties_);
    ASSERT(nav_data);
    ARecastNavMesh* nav_mesh = Cast<ARecastNavMesh>(nav_data);
    ASSERT(nav_mesh);

    FVector position;
    RobotSim::NavMeshUtil::GetRandomPoint(nav_mesh, position, Config::getValue<float>({"SIMULATION_CONTROLLER", "IMAGE_SAMPLING_AGENT_CONTROLLER", "AGENT_PROPERTIES", "AGENT_HEIGHT"}));
    ASSERT(position.Size() > 0);
    position.Z = Config::getValue<float>({"SIMULATION_CONTROLLER", "IMAGE_SAMPLING_AGENT_CONTROLLER", "AGENT_PROPERTIES", "AGENT_HEIGHT"});
    UE_LOG(LogTemp, Warning, TEXT("Spawn position is %f, %f, %f"), position.X, position.Y, position.Z);

    return position;
}
