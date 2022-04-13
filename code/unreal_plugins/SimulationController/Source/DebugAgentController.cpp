#include "DebugAgentController.h"

#include <iostream>
#include <map>
#include <string>
#include <vector>

#include <Components/StaticMeshComponent.h>
#include <Components/SceneCaptureComponent2D.h>
#include <Engine/TextureRenderTarget2D.h>
#include <EngineUtils.h>

#include "Assert.h"
#include "Box.h"
#include "Config.h"

DebugAgentController::DebugAgentController(UWorld* world)
{
    for (TActorIterator<AActor> actor_itr(world, AActor::StaticClass()); actor_itr; ++actor_itr) {
        std::string actor_name = TCHAR_TO_UTF8(*(*actor_itr)->GetName());

        if (actor_name == Config::getValue<std::string>({"SIMULATION_CONTROLLER", "DEBUG_AGENT_CONTROLLER", "ACTOR_NAME"})) {
            std::cout << "Sphere actor found!" << std::endl;
            ASSERT(!agent_actor_);
            agent_actor_ = *actor_itr;
        } else if (actor_name == Config::getValue<std::string>({"SIMULATION_CONTROLLER", "DEBUG_AGENT_CONTROLLER", "FIRST_OBSERVATION_CAMERA_NAME"})) {
            std::cout << "Observation camera 1 actor found!" << std::endl;
            ASSERT(!first_observation_camera_);
            first_observation_camera_ = *actor_itr;
        } else if (actor_name == Config::getValue<std::string>({"SIMULATION_CONTROLLER", "DEBUG_AGENT_CONTROLLER", "SECOND_OBSERVATION_CAMERA_NAME"})) {
            std::cout << "Observation camera 2 actor found!" << std::endl;
            ASSERT(!second_observation_camera_);
            second_observation_camera_ = *actor_itr;
        }
    }

    ASSERT(agent_actor_);
    ASSERT(first_observation_camera_);
    ASSERT(second_observation_camera_);

    new_object_parent_actor_ = world->SpawnActor<AActor>();
    ASSERT(new_object_parent_actor_);

    // Create SceneCaptureComponent2D and TextureRenderTarget2D
    first_scene_capture_component_ = NewObject<USceneCaptureComponent2D>(new_object_parent_actor_, TEXT("SceneCaptureComponent2D_1"));
    ASSERT(first_scene_capture_component_);
    first_scene_capture_component_->AttachToComponent(first_observation_camera_->GetRootComponent(), FAttachmentTransformRules::SnapToTargetNotIncludingScale);
    first_scene_capture_component_->SetVisibility(true);
    first_scene_capture_component_->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
    first_scene_capture_component_->FOVAngle = 60.f;
    first_scene_capture_component_->ShowFlags.SetTemporalAA(false);

    UTextureRenderTarget2D* texture_render_target_first_camera = NewObject<UTextureRenderTarget2D>(world, TEXT("TextureRenderTarget2D_1"));
    ASSERT(texture_render_target_first_camera);
    // texture_render_target_first_camera->bHDR_DEPRECATED = false;
    texture_render_target_first_camera->InitCustomFormat(Config::getValue<unsigned long>({"SIMULATION_CONTROLLER", "DEBUG_AGENT_CONTROLLER", "FIRST_OBSERVATION_CAMERA_HEIGHT"}),
                                             Config::getValue<unsigned long>({"SIMULATION_CONTROLLER", "DEBUG_AGENT_CONTROLLER", "FIRST_OBSERVATION_CAMERA_WIDTH"}),
                                             PF_B8G8R8A8,
                                             true); // PF_B8G8R8A8 disables HDR;
    texture_render_target_first_camera->RenderTargetFormat = ETextureRenderTargetFormat::RTF_RGBA8;
    texture_render_target_first_camera->bGPUSharedFlag = true; // demand buffer on GPU - might improve performance?
    texture_render_target_first_camera->TargetGamma = 1;
    texture_render_target_first_camera->SRGB = false; // false for pixels to be stored in linear space
    texture_render_target_first_camera->bAutoGenerateMips = false;
    texture_render_target_first_camera->UpdateResourceImmediate(true);

    first_scene_capture_component_->TextureTarget = texture_render_target_first_camera;
    first_scene_capture_component_->RegisterComponent();

    // Create SceneCaptureComponent2D and TextureRenderTarget2D
    second_scene_capture_component_ = NewObject<USceneCaptureComponent2D>(new_object_parent_actor_, TEXT("SceneCaptureComponent2D_2"));
    ASSERT(second_scene_capture_component_);
    second_scene_capture_component_->AttachToComponent(first_observation_camera_->GetRootComponent(), FAttachmentTransformRules::SnapToTargetNotIncludingScale);
    second_scene_capture_component_->SetVisibility(true);
    second_scene_capture_component_->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
    second_scene_capture_component_->FOVAngle = 60.f;
    second_scene_capture_component_->ShowFlags.SetTemporalAA(false);

    UTextureRenderTarget2D* texture_render_target_second_camera = NewObject<UTextureRenderTarget2D>(new_object_parent_actor_, TEXT("TextureRenderTarget2D_2"));
    ASSERT(texture_render_target_second_camera);
    // texture_render_target_second_camera->bHDR_DEPRECATED = false;
    texture_render_target_second_camera->InitCustomFormat(Config::getValue<unsigned long>({"SIMULATION_CONTROLLER", "DEBUG_AGENT_CONTROLLER", "SECOND_OBSERVATION_CAMERA_HEIGHT"}),
                                             Config::getValue<unsigned long>({"SIMULATION_CONTROLLER", "DEBUG_AGENT_CONTROLLER", "SECOND_OBSERVATION_CAMERA_WIDTH"}),
                                             PF_B8G8R8A8,
                                             true); // PF_B8G8R8A8 disables HDR;
    texture_render_target_second_camera->RenderTargetFormat = ETextureRenderTargetFormat::RTF_RGBA8;
    texture_render_target_second_camera->bGPUSharedFlag = true; // demand buffer on GPU - might improve performance?
    texture_render_target_second_camera->TargetGamma = 1;
    texture_render_target_second_camera->SRGB = false; // false for pixels to be stored in linear space
    texture_render_target_second_camera->bAutoGenerateMips = false;
    texture_render_target_second_camera->UpdateResourceImmediate(true);

    second_scene_capture_component_->TextureTarget = texture_render_target_second_camera;
    second_scene_capture_component_->RegisterComponent();    
}

DebugAgentController::~DebugAgentController()
{
    ASSERT(second_scene_capture_component_);
    second_scene_capture_component_->DestroyComponent();
    second_scene_capture_component_ = nullptr;

    ASSERT(first_scene_capture_component_);
    first_scene_capture_component_->DestroyComponent();
    first_scene_capture_component_ = nullptr;

    ASSERT(new_object_parent_actor_);
    new_object_parent_actor_->Destroy();
    new_object_parent_actor_  = nullptr;
}

std::map<std::string, Box> DebugAgentController::getActionSpace() const
{
    std::map<std::string, Box> action_space;
    
    Box box;
    box.low = std::numeric_limits<float>::lowest();
    box.high = std::numeric_limits<float>::max();
    box.shape = {3};
    box.dtype = DataType::Float32;
    action_space["set_location"] = std::move(box);

    box = Box();
    box.low = std::numeric_limits<float>::lowest();
    box.high = std::numeric_limits<float>::max();
    box.shape = {1};
    box.dtype = DataType::Float32;
    action_space["apply_force"] = std::move(box);

    return action_space;
}

std::map<std::string, Box> DebugAgentController::getObservationSpace() const
{
    std::map<std::string, Box> observation_space;

    Box box;
    box.low = std::numeric_limits<float>::lowest();
    box.high = std::numeric_limits<float>::max();
    box.shape = {3};
    box.dtype = DataType::Float32;
    observation_space["location"] = std::move(box);

    box = Box();
    box.low = 0;
    box.high = 255;
    box.shape = {Config::getValue<unsigned long>({"SIMULATION_CONTROLLER", "DEBUG_AGENT_CONTROLLER", "FIRST_OBSERVATION_CAMERA_HEIGHT"}),
                 Config::getValue<unsigned long>({"SIMULATION_CONTROLLER", "DEBUG_AGENT_CONTROLLER", "FIRST_OBSERVATION_CAMERA_WIDTH"}),
                 3};
    box.dtype = DataType::UInteger8;
    observation_space["camera_1_image"] = std::move(box);

    box = Box();
    box.low = 0;
    box.high = 255;
    box.shape = {Config::getValue<unsigned long>({"SIMULATION_CONTROLLER", "DEBUG_AGENT_CONTROLLER", "SECOND_OBSERVATION_CAMERA_HEIGHT"}),
                 Config::getValue<unsigned long>({"SIMULATION_CONTROLLER", "DEBUG_AGENT_CONTROLLER", "SECOND_OBSERVATION_CAMERA_WIDTH"}),
                 3};
    box.dtype = DataType::UInteger8;
    observation_space["camera_2_image"] = std::move(box);

    return observation_space;
}

void DebugAgentController::applyAction(const std::map<std::string, std::vector<float>>& action)
{
    std::cout << "c++: Received actions!" << std::endl;
    for (const auto& a : action) {
        std::cout << "c++: Name: " << a.first << ", action: ";
        for(const auto& b: a.second) {
            std::cout << b << ", ";
        }
        std::cout << std::endl;
    }

    if (action.count("set_location")) {
        std::vector<float> action_vec = action.at("set_location");
        agent_actor_->SetActorLocation(FVector(action_vec.at(0), action_vec.at(1), action_vec.at(2)));
    }

    if (action.count("apply_force")) {
        float force = action.at("apply_force").at(0);
        Cast<UStaticMeshComponent>(agent_actor_->GetRootComponent())->AddForce(agent_actor_->GetActorForwardVector() * force);
    }
}

std::map<std::string, std::vector<uint8_t>> DebugAgentController::getObservation() const
{
    std::map<std::string, std::vector<uint8_t>> observation;

    FVector agent_actor_location = agent_actor_->GetActorLocation();
    std::vector<float> src = {agent_actor_location.X, agent_actor_location.Y, agent_actor_location.Z};
    observation["location"] = serializeToUint8(src);
    
    ASSERT(IsInGameThread());

    FTextureRenderTargetResource* target_resource = first_scene_capture_component_->TextureTarget->GameThread_GetRenderTargetResource();
    ASSERT(target_resource);

    TArray<FColor> raw_pixels;

    struct FReadSurfaceContext
    {
        FRenderTarget* src_render_target_;
        TArray<FColor>& out_data_;
        FIntRect rect_;
        FReadSurfaceDataFlags flags_;
    };

    FReadSurfaceContext Context = {target_resource, raw_pixels, FIntRect(0, 0, target_resource->GetSizeXY().X, target_resource->GetSizeXY().Y), FReadSurfaceDataFlags(RCM_UNorm, CubeFace_MAX)};

    // Required for uint8 read mode
    Context.flags_.SetLinearToGamma(false);

    ENQUEUE_RENDER_COMMAND(ReadSurfaceCommand1)
    ([Context](FRHICommandListImmediate& RHICmdList) {
        RHICmdList.ReadSurfaceData(Context.src_render_target_->GetRenderTargetTexture(), Context.rect_, Context.out_data_, Context.flags_);
    });

    FRenderCommandFence ReadPixelFence;
    ReadPixelFence.BeginFence(true);
    ReadPixelFence.Wait(true);

    std::vector<uint8_t> image(Config::getValue<int>({"SIMULATION_CONTROLLER", "DEBUG_AGENT_CONTROLLER", "FIRST_OBSERVATION_CAMERA_HEIGHT"}) *
                               Config::getValue<int>({"SIMULATION_CONTROLLER", "DEBUG_AGENT_CONTROLLER", "FIRST_OBSERVATION_CAMERA_WIDTH"}) *
                               3);
    for (uint32 i = 0; i < static_cast<uint32>(raw_pixels.Num()); ++i) {
        image.at(3 * i + 0) = raw_pixels[i].R;
        image.at(3 * i + 1) = raw_pixels[i].G;
        image.at(3 * i + 2) = raw_pixels[i].B;
    }

    observation["camera_1_image"] = std::move(image);

    target_resource = second_scene_capture_component_->TextureTarget->GameThread_GetRenderTargetResource();
    ASSERT(target_resource);

    raw_pixels.Reset();

    FReadSurfaceContext context_2 = {target_resource,  raw_pixels, FIntRect(0, 0, target_resource->GetSizeXY().X, target_resource->GetSizeXY().Y), FReadSurfaceDataFlags(RCM_UNorm, CubeFace_MAX)};

    // Required for uint8 read mode
    context_2.flags_.SetLinearToGamma(false);

    ENQUEUE_RENDER_COMMAND(ReadSurfaceCommand2)
    ([context_2](FRHICommandListImmediate& RHICmdList) {
        RHICmdList.ReadSurfaceData(context_2.src_render_target_->GetRenderTargetTexture(), context_2.rect_, context_2.out_data_, context_2.flags_);
    });

    ReadPixelFence.BeginFence(true);
    ReadPixelFence.Wait(true);

    image.clear();
    image.resize(Config::getValue<int>({"SIMULATION_CONTROLLER", "DEBUG_AGENT_CONTROLLER", "FIRST_OBSERVATION_CAMERA_HEIGHT"}) *
                 Config::getValue<int>({"SIMULATION_CONTROLLER", "DEBUG_AGENT_CONTROLLER", "FIRST_OBSERVATION_CAMERA_WIDTH"}) *
                 3);

    for (uint32 i = 0; i < static_cast<uint32>(raw_pixels.Num()); ++i) {
        image.at(3 * i + 0) = raw_pixels[i].R;
        image.at(3 * i + 1) = raw_pixels[i].G;
        image.at(3 * i + 2) = raw_pixels[i].B;
    }
    observation["camera_2_image"] = std::move(image);

    return observation;
}
