#include "SphereAgentController.h"

#include <map>
#include <string>
#include <utility>
#include <vector>

#include <Components/SceneCaptureComponent2D.h>
#include <Components/StaticMeshComponent.h>
#include <Engine/TextureRenderTarget2D.h>
#include <EngineUtils.h>

#include "Assert.h"
#include "Box.h"
#include "Config.h"

SphereAgentController::SphereAgentController(UWorld* world)
{
    for (TActorIterator<AActor> ActorItr(world, AActor::StaticClass()); ActorItr; ++ActorItr) {
        if ((*ActorItr)->GetName().Equals(TEXT("SphereActor"), ESearchCase::IgnoreCase)) { 
            sphere_actor_ = (*ActorItr);
        }
        else if ((*ActorItr)->GetName().Equals(TEXT("FirstObservationCamera"), ESearchCase::IgnoreCase)) {
            observation_camera_ = (*ActorItr);
        }
    }

    ASSERT(sphere_actor_);
    ASSERT(observation_camera_);

    // Set observation active camera as active camera
    APlayerController* Controller = world->GetFirstPlayerController();
    ASSERT(Controller);
    Controller->SetViewTarget(observation_camera_);

    // create SceneCaptureComponent2D
    scene_capture_component_ = NewObject<USceneCaptureComponent2D>(observation_camera_, TEXT("SceneCaptureComponent2D_1"));
    ASSERT(scene_capture_component_);
    scene_capture_component_->AttachToComponent(observation_camera_->GetRootComponent(), FAttachmentTransformRules::SnapToTargetNotIncludingScale);
    scene_capture_component_->SetVisibility(true);
    scene_capture_component_->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
    scene_capture_component_->FOVAngle = 60.f;
    scene_capture_component_->ShowFlags.SetTemporalAA(false);

    // create TextureRenderTarget2D
    texture_render_target_ = NewObject<UTextureRenderTarget2D>(scene_capture_component_, TEXT("TextureRenderTarget2D_1"));
    ASSERT(texture_render_target_);
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
    action_space["set_location"] = std::move(box); // @Todo: implement move assignment operator

    box = Box();
    box.low = std::numeric_limits<float>::lowest();
    box.high = std::numeric_limits<float>::max();
    box.shape = {1};
    box.dtype = DataType::Float32;
    action_space["apply_force"] = std::move(box); // @Todo: implement move assignment operator

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
    observation_space["location"] = std::move(box);

    box = Box();
    box.low = 0;
    box.high = 255;
    box.shape = {Config::getValue<unsigned long>({"DEBUG_PROJECT", "IMAGE_HEIGHT"}), Config::getValue<unsigned long>({"DEBUG_PROJECT", "IMAGE_WIDTH"}), 3}; // RGB image
    box.dtype = DataType::UInteger8;
    observation_space["image"] = std::move(box);

    return observation_space;
}

void SphereAgentController::applyAction(const std::map<std::string, std::vector<float>>& action)
{
    if (action.count("set_location")) {
        std::vector<float> action_vec = action.at("set_location");
        sphere_actor_->SetActorLocation(FVector(action_vec.at(0), action_vec.at(1), action_vec.at(2)), false, nullptr, ETeleportType::TeleportPhysics);
    }

    if (action.count("apply_force")) {
        float force = action.at("apply_force").at(0);
        Cast<UStaticMeshComponent>(sphere_actor_->GetRootComponent())->AddForce(sphere_actor_->GetActorForwardVector() * force);
    }
}

std::map<std::string, std::vector<uint8_t>> SphereAgentController::getObservation() const
{
    std::map<std::string, std::vector<uint8_t>> observation;

    // get location observation
    FVector sphere_actor_location = sphere_actor_->GetActorLocation();
    std::vector<float> src = {sphere_actor_location.X, sphere_actor_location.Y, sphere_actor_location.Z};
    observation["location"] = serializeToUint8(src);
    
    // get image observation
    ASSERT(IsInGameThread());

    FTextureRenderTargetResource* target_resource = scene_capture_component_->TextureTarget->GameThread_GetRenderTargetResource();
    if (target_resource == nullptr) {
        ASSERT(false, "Could not get RenderTarget Resource from GameThread!! :(");
    }

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

    std::vector<uint8_t> image(Config::getValue<int>({"DEBUG_PROJECT", "IMAGE_HEIGHT"}) * Config::getValue<int>({"DEBUG_PROJECT", "IMAGE_WIDTH"}) * 3);

    for (uint32 i = 0; i < static_cast<uint32>(pixels.Num()); ++i) {
        image.at(3 * i + 0) = pixels[i].R;
        image.at(3 * i + 1) = pixels[i].G;
        image.at(3 * i + 2) = pixels[i].B;
    }
    
    observation["image"] = image;

    return observation;
}
