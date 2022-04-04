#include "SphereAgentController.h"

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

#include "Assert.h"
#include "Box.h"
#include "Config.h"

SphereAgentController::SphereAgentController(UWorld* world)
{
    for (TActorIterator<AActor> ActorItr(world, AActor::StaticClass()); ActorItr; ++ActorItr) {
        if ((*ActorItr)->GetName().Equals(Config::getValue<std::string>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "ACTOR_NAME"}).c_str(), ESearchCase::IgnoreCase)) { 
            ASSERT(sphere_actor_ == nullptr);
            sphere_actor_ = (*ActorItr);
            ASSERT(sphere_actor_);
        } else if ((*ActorItr)->GetName().Equals(Config::getValue<std::string>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "OBSERVATION_CAMERA_NAME"}).c_str(), ESearchCase::IgnoreCase)) {
            ASSERT(observation_camera_actor_ == nullptr);
            observation_camera_actor_ = (*ActorItr);
            ASSERT(observation_camera_actor_);
        } else if ((*ActorItr)->GetName().Equals(Config::getValue<std::string>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "CONE_NAME"}).c_str(), ESearchCase::IgnoreCase)) {
            ASSERT(cone_actor_ == nullptr);
            cone_actor_ = *ActorItr;
            ASSERT(cone_actor_);
        } else if ((*ActorItr)->GetName().Equals(Config::getValue<std::string>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "DEBUG_CAMERA_NAME"}).c_str(), ESearchCase::IgnoreCase)) {
            ASSERT(debug_camera_actor_ == nullptr);
            debug_camera_actor_ = *ActorItr;
            ASSERT(debug_camera_actor_);
        }
    }

    // set active camera
    APlayerController* Controller = world->GetFirstPlayerController();
    ASSERT(Controller);
    if (Config::getValue<std::string>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "VISUALIZATION_MODE"}) == "debug") {
        Controller->SetViewTarget(debug_camera_actor_);
    } else if (Config::getValue<std::string>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "VISUALIZATION_MODE"}) == "observation") {
        Controller->SetViewTarget(observation_camera_actor_);
    } else {
        ASSERT(false);
    }

    // Set debug camera
    if (Config::getValue<bool>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "DEBUG_CAMERA", "SET_POSE"})) {
        const FVector debug_camera_pose(
            Config::getValue<float>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "DEBUG_CAMERA", "POSITION_X"}),
            Config::getValue<float>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "DEBUG_CAMERA", "POSITION_Y"}),
            Config::getValue<float>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "DEBUG_CAMERA", "POSITION_Z"}));
        debug_camera_actor_->SetActorLocation(debug_camera_pose);
        debug_camera_actor_->SetActorRotation(FRotator(
            Config::getValue<float>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "DEBUG_CAMERA", "PITCH"}),
            Config::getValue<float>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "DEBUG_CAMERA", "YAW"}),
            Config::getValue<float>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "DEBUG_CAMERA", "ROLL"})));
    }

    if (Config::getValue<std::string>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "OBSERVATION_MODE"}) == "mixed") {
        // Create SceneCaptureComponent2D and TextureRenderTarget2D
        scene_capture_component_ = NewObject<USceneCaptureComponent2D>(observation_camera_actor_, TEXT("SceneCaptureComponent2D"));
        ASSERT(scene_capture_component_);

        scene_capture_component_->AttachToComponent(observation_camera_actor_->GetRootComponent(), FAttachmentTransformRules::SnapToTargetNotIncludingScale);
        scene_capture_component_->SetVisibility(true);
        scene_capture_component_->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
        scene_capture_component_->FOVAngle = 60.f;
        scene_capture_component_->ShowFlags.SetTemporalAA(false);

        UTextureRenderTarget2D* texture_render_target = NewObject<UTextureRenderTarget2D>(scene_capture_component_, TEXT("TextureRenderTarget2D"));
        ASSERT(texture_render_target);

        // texture_render_target->bHDR_DEPRECATED = false;
        texture_render_target->InitCustomFormat(
            Config::getValue<unsigned long>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "MIXED_MODE", "IMAGE_HEIGHT"}),
            Config::getValue<unsigned long>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "MIXED_MODE", "IMAGE_WIDTH"}),
            PF_B8G8R8A8,
            true); // PF_B8G8R8A8 disables HDR;
        texture_render_target->RenderTargetFormat = ETextureRenderTargetFormat::RTF_RGBA8;
        texture_render_target->bGPUSharedFlag = true; // demand buffer on GPU - might improve performance?
        texture_render_target->TargetGamma = 1;
        texture_render_target->SRGB = false; // false for pixels to be stored in linear space
        texture_render_target->bAutoGenerateMips = false;
        texture_render_target->UpdateResourceImmediate(true);

        scene_capture_component_->TextureTarget = texture_render_target;
        scene_capture_component_->RegisterComponent();
    }

    // Set collision callback
    // sphere_actor_->OnActorHit.AddDynamic(this, &SphereAgentController::OnActorHit);

    sphere_static_mesh_component_ = Cast<UStaticMeshComponent>(sphere_actor_->GetRootComponent());
    ASSERT(sphere_static_mesh_component_);

    cone_static_mesh_component_ = Cast<UStaticMeshComponent>(cone_actor_->GetRootComponent());
    ASSERT(cone_static_mesh_component_);

    // Need to set this to apply forces or move objects
    sphere_static_mesh_component_->SetMobility(EComponentMobility::Type::Movable);
    cone_static_mesh_component_->SetMobility(EComponentMobility::Type::Movable);

    // Set physics state
    sphere_static_mesh_component_->BodyInstance.SetCollisionProfileName(UCollisionProfile::PhysicsActor_ProfileName);
    sphere_static_mesh_component_->SetSimulatePhysics(true);
    sphere_static_mesh_component_->SetAngularDamping(Config::getValue<float>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "SPHERE", "ANGULAR_DAMPING"}));
    sphere_static_mesh_component_->SetLinearDamping(Config::getValue<float>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "SPHERE", "LINEAR_DAMPING"}));
    sphere_static_mesh_component_->BodyInstance.MaxAngularVelocity = FMath::RadiansToDegrees(Config::getValue<float>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "SPHERE", "MAX_ANGULAR_VELOCITY"}));
    sphere_static_mesh_component_->BodyInstance.MassScale = FMath::RadiansToDegrees(Config::getValue<float>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "SPHERE", "MASS_SCALE"}));
    sphere_static_mesh_component_->SetNotifyRigidBodyCollision(true);
}

std::map<std::string, Box> SphereAgentController::getActionSpace() const
{
    std::map<std::string, Box> action_space;
    
    Box box;
    box.low = -1.f;
    box.high = 1.f;
    box.shape = {2};
    box.dtype = DataType::Float32;
    action_space["apply_force"] = std::move(box);

    return action_space;
}

std::map<std::string, Box> SphereAgentController::getObservationSpace() const
{
    std::map<std::string, Box> observation_space;

    if (Config::getValue<std::string>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "OBSERVATION_MODE"}) == "mixed") {
        Box box;
        box.low = std::numeric_limits<float>::lowest();
        box.high = std::numeric_limits<float>::max();
        box.shape = {5};
        box.dtype = DataType::Float32;
        observation_space["physical_observation"] = std::move(box);

        box = Box();
        box.low = 0;
        box.high = 255;
        box.shape = {Config::getValue<unsigned long>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "MIXED_MODE", "IMAGE_HEIGHT"}), Config::getValue<unsigned long>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "MIXED_MODE", "IMAGE_WIDTH"}), 3};
        box.dtype = DataType::UInteger8;
        observation_space["visual_observation"] = std::move(box);
    } else if (Config::getValue<std::string>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "OBSERVATION_MODE"}) == "physical") {
        Box box;
        box.low = std::numeric_limits<float>::lowest();
        box.high = std::numeric_limits<float>::max();
        box.shape = {4};
        box.dtype = DataType::Float32;
        observation_space["physical_observation"] = std::move(box);
    } else {
        ASSERT(false);
    }

    return observation_space;
}

void SphereAgentController::applyAction(const std::map<std::string, std::vector<float>>& action)
{
    ASSERT(action.count("apply_force"));
    ASSERT(isfinite(action.at("apply_force").at(0)));
    ASSERT(isfinite(action.at("apply_force").at(1)));

    // @TODO: This can be checked in python?
    ASSERT(action.at("apply_force").at(0) >= getActionSpace()["apply_force"].low && action.at("apply_force").at(0) <= getActionSpace()["apply_force"].high, "%f", action.at("apply_force").at(0));
    ASSERT(action.at("apply_force").at(1) >= getActionSpace()["apply_force"].low && action.at("apply_force").at(1) <= getActionSpace()["apply_force"].high, "%f", action.at("apply_force").at(1));

    if (Config::getValue<std::string>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "OBSERVATION_MODE"}) == "mixed") {
        // Get yaw from the observation camera, apply force to the sphere in that direction
        FVector force = observation_camera_actor_->GetActorRotation().RotateVector(FVector(action.at("apply_force").at(0), 0.0f, 0.0f)) * Config::getValue<float>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "MIXED_MODE", "ACTION_APPLY_FORCE_SCALE"});

        ASSERT(isfinite(force.X));
        ASSERT(isfinite(force.Y));
        ASSERT(isfinite(force.Z));

        sphere_static_mesh_component_->AddForce(force);

        // Set observation camera yaw by adding to the current observation camera yaw
        FRotator rotation = observation_camera_actor_->GetActorRotation().Add(0.0f, action.at("apply_force").at(1) * Config::getValue<float>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "MIXED_MODE", "ACTION_ROTATE_OBSERVATION_CAMERA_SCALE"}), 0.0f);

        ASSERT(isfinite(rotation.Pitch));
        ASSERT(isfinite(rotation.Yaw));
        ASSERT(isfinite(rotation.Roll));
        ASSERT(rotation.Pitch >= -360.0 && rotation.Pitch <= 360.0, "%f", rotation.Pitch);
        ASSERT(rotation.Yaw   >= -360.0 && rotation.Yaw   <= 360.0, "%f", rotation.Yaw);
        ASSERT(rotation.Roll  >= -360.0 && rotation.Roll  <= 360.0, "%f", rotation.Roll);

        observation_camera_actor_->SetActorRotation(rotation);
    } else if (Config::getValue<std::string>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "OBSERVATION_MODE"}) == "physical") {
        FVector force = FVector(action.at("apply_force").at(0), action.at("apply_force").at(1), 0.0f) * Config::getValue<float>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "PHYSICAL_MODE", "ACTION_APPLY_FORCE_SCALE"});

        ASSERT(isfinite(force.X));
        ASSERT(isfinite(force.Y));
        ASSERT(isfinite(force.Z));

        sphere_static_mesh_component_->AddForce(force);
    } else {
        ASSERT(false);
    }

    // Set the observation camera's position to be offset relative to the sphere's position. This is an imperfect place to set the
    // position because this function will be called before a physics update, but rendering will happen after a physics update. So
    // it's possible to see the sphere in the observation camera's rendered view, even if we set the sphere to be directly above
    // the sphere here.
    const FVector observation_camera_pose(
        sphere_actor_->GetActorLocation() +
        FVector(Config::getValue<float>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "OBSERVATION_CAMERA", "POSITION_OFFSET_X"}),
                Config::getValue<float>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "OBSERVATION_CAMERA", "POSITION_OFFSET_Y"}),
                Config::getValue<float>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "OBSERVATION_CAMERA", "POSITION_OFFSET_Z"})));
    observation_camera_actor_->SetActorLocation(observation_camera_pose);
}

std::map<std::string, std::vector<uint8_t>> SphereAgentController::getObservation() const
{
    std::map<std::string, std::vector<uint8_t>> observation;

    // get game state
    const FVector sphere_to_cone = cone_actor_->GetActorLocation() - sphere_actor_->GetActorLocation();
    const FVector linear_velocity = sphere_static_mesh_component_->GetPhysicsLinearVelocity();
    const float observation_camera_yaw = observation_camera_actor_->GetActorRotation().Yaw;

    // get observations
    if (Config::getValue<std::string>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "OBSERVATION_MODE"}) == "mixed") {

        observation["physical_observation"] = serializeToUint8(std::vector<float>{
            Config::getValue<float>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "OBSERVATION", "OFFSET_TO_GOAL_SCALE"}) * sphere_to_cone.X,
            Config::getValue<float>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "OBSERVATION", "OFFSET_TO_GOAL_SCALE"}) * sphere_to_cone.Y,
            Config::getValue<float>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "OBSERVATION", "LINEAR_VELOCITY_SCALE"}) * linear_velocity.X,
            Config::getValue<float>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "OBSERVATION", "LINEAR_VELOCITY_SCALE"}) * linear_velocity.Y,
            Config::getValue<float>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "OBSERVATION", "YAW_SCALE"}) * observation_camera_yaw});

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


        std::vector<uint8_t> image(Config::getValue<int>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "OBSERVATION_CAMERA_HEIGHT"}) * Config::getValue<int>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "OBSERVATION_CAMERA_WIDTH"}) * 3);

        for (uint32 i = 0; i < static_cast<uint32>(pixels.Num()); ++i) {
            image.at(3 * i + 0) = pixels[i].R;
            image.at(3 * i + 1) = pixels[i].G;
            image.at(3 * i + 2) = pixels[i].B;
        }
        
        observation["visual_observation"] = image;
    } else if (Config::getValue<std::string>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "OBSERVATION_MODE"}) == "physical") {

        observation["physical_observation"] = serializeToUint8(std::vector<float>{
            Config::getValue<float>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "OBSERVATION", "OFFSET_TO_GOAL_SCALE"}) * sphere_to_cone.X,
            Config::getValue<float>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "OBSERVATION", "OFFSET_TO_GOAL_SCALE"}) * sphere_to_cone.Y,
            Config::getValue<float>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "OBSERVATION", "LINEAR_VELOCITY_SCALE"}) * linear_velocity.X,
            Config::getValue<float>({"SIMULATION_CONTROLLER", "SPHERE_AGENT_CONTROLLER", "OBSERVATION", "LINEAR_VELOCITY_SCALE"}) * linear_velocity.Y});
    } else {
        ASSERT(false);
    }

    return observation;
}

AActor* SphereAgentController::getConeActor() const
{
    return cone_actor_;
}

AActor* SphereAgentController::getSphereActor() const
{
    return sphere_actor_;
}

AActor* SphereAgentController::getObservationCameraActor() const
{
    return observation_camera_actor_;
}
