#include "SphereAgentController.h"

//remove
#include <iostream>
//this

#include <map>
#include <string>
#include <vector>

#include <Components/StaticMeshComponent.h>
#include <Components/SceneCaptureComponent2D.h>
#include <Engine/TextureRenderTarget2D.h>


#include "Box.h"

SphereAgentController::SphereAgentController(UWorld* world)
{
    for (TActorIterator<AActor> ActorItr(world, AActor::StaticClass()); ActorItr; ++ActorItr)
    {
        if ((*ActorItr)->GetName().Equals(TEXT("SphereAgent"), ESearchCase::IgnoreCase)) { 
            UE_LOG(LogTemp, Warning, TEXT("Sphere actor found!"));
            sphere_actor_ = (*ActorItr);
        }
        else if ((*ActorItr)->GetName().Equals(TEXT("ObservationCamera"), ESearchCase::IgnoreCase)) {
            UE_LOG(LogTemp, Warning, TEXT("Observation camera actor found!"));
            observation_camera_ = (*ActorItr);
        }
    }

    ASSERT(sphere_actor_);
    ASSERT(observation_camera_);

    APlayerController* Controller = world->GetFirstPlayerController();
    ASSERT(Controller);

    // Set active camera
    Controller->SetViewTarget(observation_camera_);

    // Create SceneCaptureComponent2D and TextureRenderTarget2D
    scene_capture_component_ = NewObject<USceneCaptureComponent2D>(observation_camera_, TEXT("SceneCaptureComponent2D"));
    ASSERT(scene_capture_component_);

    scene_capture_component_->AttachToComponent(observation_camera_->GetRootComponent(), FAttachmentTransformRules::SnapToTargetNotIncludingScale);
    scene_capture_component_->SetVisibility(true);
    scene_capture_component_->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
    scene_capture_component_->FOVAngle = 60.f;
    scene_capture_component_->ShowFlags.SetTemporalAA(false);

    UTextureRenderTarget2D* texture_render_target_ = NewObject<UTextureRenderTarget2D>(scene_capture_component_, TEXT("TextureRenderTarget2D"));
    ASSERT(texture_render_target_);

    // texture_render_target_->bHDR_DEPRECATED = false;
    texture_render_target_->InitCustomFormat(Config::getValue<unsigned long>({"DEBUG_PROJECT", "IMAGE_HEIGHT"}), Config::getValue<unsigned long>({"DEBUG_PROJECT", "IMAGE_WIDTH"}), PF_B8G8R8A8, true); // PF_B8G8R8A8 disables HDR;
    texture_render_target_->RenderTargetFormat = ETextureRenderTargetFormat::RTF_RGBA8;
    texture_render_target_->bGPUSharedFlag = true; // demand buffer on GPU - might improve performance?
    texture_render_target_->TargetGamma = 1;
    texture_render_target_->SRGB = false; // false for pixels to be stored in linear space
    texture_render_target_->bAutoGenerateMips = false;
    texture_render_target_->UpdateResourceImmediate(true);

    scene_capture_component_->TextureTarget = texture_render_target_;
    scene_capture_component_->RegisterComponent();
}

std::map<std::string, Box> SphereAgentController::getActionSpace() const
{
    std::map<std::string, Box> action_space;
    Box box;

    box.low = std::numeric_limits<float>::lowest();
    box.high = std::numeric_limits<float>::max();
    box.shape = {3};
    box.dtype = DataType::Float32;
    action_space["set_location"] = box; // @Todo: implement move assignment operator

    box = Box();
    box.low = std::numeric_limits<float>::lowest();
    box.high = std::numeric_limits<float>::max();
    box.shape = {1};
    box.dtype = DataType::Float32;
    action_space["apply_force"] = box;

    return action_space;
}

std::map<std::string, Box> SphereAgentController::getObservationSpace() const
{
    std::map<std::string, Box> observation_space;

    Box box;
    box.low = std::numeric_limits<float>::lowest();
    box.high = std::numeric_limits<float>::max();
    box.shape = {3};
    box.dtype = DataType::Float32;
    observation_space["location"] = box;

    box = Box();
    box.low = 0;
    box.high = 255;
    box.shape = {Config::getValue<unsigned long>({"DEBUG_PROJECT", "IMAGE_HEIGHT"}), Config::getValue<unsigned long>({"DEBUG_PROJECT", "IMAGE_WIDTH"}), 3};
    box.dtype = DataType::UInteger8;
    observation_space["image"] = box;

    return observation_space;
}

void SphereAgentController::applyAction(const std::map<std::string, std::vector<float>>& action)
{
    std::cout << "c++: Received actions!" << std::endl;
    for (auto a : action)
    {
        std::cout << "Name: " << a.first << ", action: ";
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

std::map<std::string, std::vector<uint8_t>> SphereAgentController::getObservation() const
{
    std::map<std::string, std::vector<uint8_t>> observation;

    FVector sphere_actor_location = sphere_actor_->GetActorLocation();
    std::vector<float> src = {sphere_actor_location.X, sphere_actor_location.Y, sphere_actor_location.Z};
    observation["location"] = AgentController::serializeToUint8(src);

    ASSERT(IsInGameThread());

    FTextureRenderTargetResource* target_resource = scene_capture_component_->TextureTarget->GameThread_GetRenderTargetResource();
    if (target_resource == nullptr)
    {
        ASSERT(false, "Could not get RenderTarget Resource from GameThread!! :(");
    }

    TArray<FColor> raw_pixels;
    raw_pixels.Reset();

    struct FReadSurfaceContext
    {
        FRenderTarget* src_render_target_;
        TArray<FColor>* out_data_;
        FIntRect rect_;
        FReadSurfaceDataFlags flags_;
    };

    FReadSurfaceContext Context = {target_resource,  &raw_pixels, FIntRect(0, 0, target_resource->GetSizeXY().X, target_resource->GetSizeXY().Y), FReadSurfaceDataFlags(RCM_UNorm, CubeFace_MAX)};

    // Required for uint8 read mode
    Context.flags_.SetLinearToGamma(false);

    ENQUEUE_RENDER_COMMAND(ReadSurfaceCommand)
    ([Context](FRHICommandListImmediate& RHICmdList)
    {
        RHICmdList.ReadSurfaceData(Context.src_render_target_->GetRenderTargetTexture(), Context.rect_, *Context.out_data_, Context.flags_);
    });

    FRenderCommandFence ReadPixelFence;
    ReadPixelFence.BeginFence();
    ReadPixelFence.Wait(true);

    std::vector<uint8_t> image;
    image.resize(Config::getValue<int>({"DEBUG_PROJECT", "IMAGE_HEIGHT"}) * Config::getValue<int>({"DEBUG_PROJECT", "IMAGE_WIDTH"}) * 3);

    for (uint32 i = 0; i < static_cast<uint32>(raw_pixels.Num()); ++i)
    {
        image.at(3 * i + 0) = raw_pixels[i].R;
        image.at(3 * i + 1) = raw_pixels[i].G;
        image.at(3 * i + 2) = raw_pixels[i].B;
    }
    observation["image"] = image;

    return observation;
}
