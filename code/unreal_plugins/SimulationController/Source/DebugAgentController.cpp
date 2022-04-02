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
    for (TActorIterator<AActor> ActorItr(world, AActor::StaticClass()); ActorItr; ++ActorItr) {
        if ((*ActorItr)->GetName().Equals(TEXT("SphereActor"), ESearchCase::IgnoreCase)) { 
            UE_LOG(LogTemp, Warning, TEXT("Sphere actor found!"));
            sphere_actor_ = (*ActorItr);
        }
        else if ((*ActorItr)->GetName().Equals(TEXT("FirstObservationCamera"), ESearchCase::IgnoreCase)) {
            UE_LOG(LogTemp, Warning, TEXT("Debug camera 1 actor found!"));
            first_observation_camera_ = (*ActorItr);
        }
        else if ((*ActorItr)->GetName().Equals(TEXT("SecondObservationCamera"), ESearchCase::IgnoreCase)) {
            UE_LOG(LogTemp, Warning, TEXT("Debug camera 2 actor found!"));
            second_observation_camera_ = (*ActorItr);
        }
    }

    ASSERT(sphere_actor_);
    ASSERT(first_observation_camera_);
    ASSERT(second_observation_camera_);

    // Set observation active camera as active camera
    APlayerController* Controller = world->GetFirstPlayerController();
    ASSERT(Controller);
    Controller->SetViewTarget(first_observation_camera_);

    // Create SceneCaptureComponent2D and TextureRenderTarget2D
    first_scene_capture_component_ = NewObject<USceneCaptureComponent2D>(first_observation_camera_, TEXT("SceneCaptureComponent2D_1"));
    ASSERT(first_scene_capture_component_);
    first_scene_capture_component_->AttachToComponent(first_observation_camera_->GetRootComponent(), FAttachmentTransformRules::SnapToTargetNotIncludingScale);
    first_scene_capture_component_->SetVisibility(true);
    first_scene_capture_component_->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
    first_scene_capture_component_->FOVAngle = 60.f;
    first_scene_capture_component_->ShowFlags.SetTemporalAA(false);

    texture_render_target_ = NewObject<UTextureRenderTarget2D>(first_scene_capture_component_, TEXT("TextureRenderTarget2D_1"));
    ASSERT(texture_render_target_);
    // texture_render_target_->bHDR_DEPRECATED = false;
    texture_render_target_->InitCustomFormat(Config::getValue<unsigned long>({"DEBUG_PROJECT", "IMAGE_HEIGHT"}), Config::getValue<unsigned long>({"DEBUG_PROJECT", "IMAGE_WIDTH"}), PF_B8G8R8A8, true); // PF_B8G8R8A8 disables HDR;
    texture_render_target_->RenderTargetFormat = ETextureRenderTargetFormat::RTF_RGBA8;
    texture_render_target_->bGPUSharedFlag = true; // demand buffer on GPU - might improve performance?
    texture_render_target_->TargetGamma = 1;
    texture_render_target_->SRGB = false; // false for pixels to be stored in linear space
    texture_render_target_->bAutoGenerateMips = false;
    texture_render_target_->UpdateResourceImmediate(true);

    first_scene_capture_component_->TextureTarget = texture_render_target_;
    first_scene_capture_component_->RegisterComponent();

    // Create SceneCaptureComponent2D and TextureRenderTarget2D
    second_scene_capture_component_ = NewObject<USceneCaptureComponent2D>(first_observation_camera_, TEXT("SceneCaptureComponent2D_2"));
    ASSERT(second_scene_capture_component_);
    second_scene_capture_component_->AttachToComponent(first_observation_camera_->GetRootComponent(), FAttachmentTransformRules::SnapToTargetNotIncludingScale);
    second_scene_capture_component_->SetVisibility(true);
    second_scene_capture_component_->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
    second_scene_capture_component_->FOVAngle = 60.f;
    second_scene_capture_component_->ShowFlags.SetTemporalAA(false);

    texture_render_target_ = NewObject<UTextureRenderTarget2D>(second_scene_capture_component_, TEXT("TextureRenderTarget2D_2"));
    ASSERT(texture_render_target_);
    // texture_render_target_->bHDR_DEPRECATED = false;
    texture_render_target_->InitCustomFormat(Config::getValue<unsigned long>({"DEBUG_PROJECT", "IMAGE_HEIGHT"}), Config::getValue<unsigned long>({"DEBUG_PROJECT", "IMAGE_WIDTH"}), PF_B8G8R8A8, true); // PF_B8G8R8A8 disables HDR;
    texture_render_target_->RenderTargetFormat = ETextureRenderTargetFormat::RTF_RGBA8;
    texture_render_target_->bGPUSharedFlag = true; // demand buffer on GPU - might improve performance?
    texture_render_target_->TargetGamma = 1;
    texture_render_target_->SRGB = false; // false for pixels to be stored in linear space
    texture_render_target_->bAutoGenerateMips = false;
    texture_render_target_->UpdateResourceImmediate(true);

    second_scene_capture_component_->TextureTarget = texture_render_target_;
    second_scene_capture_component_->RegisterComponent();    
}

std::map<std::string, Box> DebugAgentController::getActionSpace() const
{
    std::map<std::string, Box> action_space;
    
    Box box;
    box.low = std::numeric_limits<float>::lowest();
    box.high = std::numeric_limits<float>::max();
    box.shape = {3};
    box.dtype = DataType::Float32;
    action_space["set_location"] = std::move(box); // @Todo: implement move assignment operator

    box = Box();
    box.low = std::numeric_limits<float>::lowest();
    box.high = std::numeric_limits<float>::max();
    box.shape = {1};
    box.dtype = DataType::Float32;
    action_space["apply_force"] = std::move(box);  // @Todo: implement move assignment operator

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
    box.shape = {Config::getValue<unsigned long>({"DEBUG_PROJECT", "IMAGE_HEIGHT"}), Config::getValue<unsigned long>({"DEBUG_PROJECT", "IMAGE_WIDTH"}), 3};
    box.dtype = DataType::UInteger8;
    observation_space["camera_1_image"] = std::move(box);

    box = Box();
    box.low = 0;
    box.high = 255;
    box.shape = {Config::getValue<unsigned long>({"DEBUG_PROJECT", "IMAGE_HEIGHT"}), Config::getValue<unsigned long>({"DEBUG_PROJECT", "IMAGE_WIDTH"}), 3};
    box.dtype = DataType::UInteger8;
    observation_space["camera_2_image"] = std::move(box);

    return observation_space;
}

void DebugAgentController::applyAction(const std::map<std::string, std::vector<float>>& action)
{
    std::cout << "c++: Received actions!" << std::endl;
    for (auto a : action)
    {
        std::cout << "c++: Name: " << a.first << ", action: ";
        for(auto b: a.second)
        {
            std::cout << b << ", ";
        }
        std::cout << std::endl;
    }

    if (action.count("set_location")) {
        std::vector<float> action_vec = action.at("set_location");
        sphere_actor_->SetActorLocation(FVector(action_vec.at(0), action_vec.at(1), action_vec.at(2)));
    }

    if (action.count("apply_force")) {
        float force = action.at("apply_force").at(0);
        Cast<UStaticMeshComponent>(sphere_actor_->GetRootComponent())->AddForce(sphere_actor_->GetActorForwardVector() * force);
    }
}

std::map<std::string, std::vector<uint8_t>> DebugAgentController::getObservation() const
{
    std::map<std::string, std::vector<uint8_t>> observation;

    FVector sphere_actor_location = sphere_actor_->GetActorLocation();
    std::vector<float> src = {sphere_actor_location.X, sphere_actor_location.Y, sphere_actor_location.Z};
    observation["location"] = serializeToUint8(src);
    
    // --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
    ASSERT(IsInGameThread());

    FTextureRenderTargetResource* target_resource = first_scene_capture_component_->TextureTarget->GameThread_GetRenderTargetResource();
    if (target_resource == nullptr) {
        ASSERT(false, "Could not get RenderTarget Resource from GameThread!! :(");
    }

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
    ([Context](FRHICommandListImmediate& RHICmdList)
    {
        RHICmdList.ReadSurfaceData(Context.src_render_target_->GetRenderTargetTexture(), Context.rect_, Context.out_data_, Context.flags_);
    });

    FRenderCommandFence ReadPixelFence;
    ReadPixelFence.BeginFence(true);
    ReadPixelFence.Wait(true);

    std::vector<uint8_t> image(Config::getValue<int>({"DEBUG_PROJECT", "IMAGE_HEIGHT"}) * Config::getValue<int>({"DEBUG_PROJECT", "IMAGE_WIDTH"}) * 3);
    for (uint32 i = 0; i < static_cast<uint32>(raw_pixels.Num()); ++i)
    {
        image.at(3 * i + 0) = raw_pixels[i].R;
        image.at(3 * i + 1) = raw_pixels[i].G;
        image.at(3 * i + 2) = raw_pixels[i].B;
    }

    observation["camera_1_image"] = image;

    // --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

    target_resource = second_scene_capture_component_->TextureTarget->GameThread_GetRenderTargetResource();
    if (target_resource == nullptr)
    {
        ASSERT(false, "Could not get RenderTarget Resource from GameThread!! :(");
    }

    raw_pixels.Reset();

    FReadSurfaceContext context_2 = {target_resource,  raw_pixels, FIntRect(0, 0, target_resource->GetSizeXY().X, target_resource->GetSizeXY().Y), FReadSurfaceDataFlags(RCM_UNorm, CubeFace_MAX)};

    // Required for uint8 read mode
    context_2.flags_.SetLinearToGamma(false);

    ENQUEUE_RENDER_COMMAND(ReadSurfaceCommand2)
    ([context_2](FRHICommandListImmediate& RHICmdList)
    {
        RHICmdList.ReadSurfaceData(context_2.src_render_target_->GetRenderTargetTexture(), context_2.rect_, context_2.out_data_, context_2.flags_);
    });

    ReadPixelFence.BeginFence(true);
    ReadPixelFence.Wait(true);

    image.clear();
    image.resize(Config::getValue<int>({"DEBUG_PROJECT", "IMAGE_HEIGHT"}) * Config::getValue<int>({"DEBUG_PROJECT", "IMAGE_WIDTH"}) * 3);

    for (uint32 i = 0; i < static_cast<uint32>(raw_pixels.Num()); ++i)
    {
        image.at(3 * i + 0) = raw_pixels[i].R;
        image.at(3 * i + 1) = raw_pixels[i].G;
        image.at(3 * i + 2) = raw_pixels[i].B;
    }
    observation["camera_2_image"] = image;

    return observation;
}
