#include "SimpleVehicleBrain.h"

USimpleVehicleBrain::USimpleVehicleBrain(const FObjectInitializer &objectInitializer) : Super(objectInitializer)
{
}

void USimpleVehicleBrain::Init()
{
    for (TActorIterator<ASimpleVehiclePawn> it(this->GetWorld()); it; ++it)
    {
        ownerPawn = *it;
    }

    ASSERT(ownerPawn != nullptr);

    APlayerController *Controller = GetWorld()->GetFirstPlayerController();
    ASSERT(Controller != nullptr);

    // Look for the desired observation camera among all available camera actors:
    for (TActorIterator<APIPCamera> it(this->GetWorld()); it; ++it)
    {
        // This is quick and dirty fix to get access to the camera actors
        // attached to the vehicle. Note that these actors are defined within
        // the settings.json parameter file.
        // TODO: point to a specific PIPcamera actor, defined within a parameter
        // file.
        mainCamera_ = *it;
    }

    ASSERT(mainCamera_ != nullptr);

    // Set the observation camera:
    Controller->SetViewTarget(mainCamera_);

    if(unrealrl::Config::GetValue<bool>({"INTERIOR_SIM_BRIDGE", "USE_IMAGE_OBSERVATIONS"}))
    {
        // Point to the vehicle camera capture component:
        captureComponent2D_ = mainCamera_->GetSceneCaptureComponent();
    }

    ownerPawn->OnActorHit.AddDynamic(this, &USimpleVehicleBrain::OnActorHit);

    // Store actor refs required during simulation.
    for (TActorIterator<AActor> ActorItr(GetWorld(), AActor::StaticClass());
         ActorItr; ++ActorItr)
    {
        if ((*ActorItr)->ActorHasTag("goal"))
        {
            goalActor = *ActorItr;
            break;
        }
    }

    ASSERT(goalActor != nullptr);

    // Initialize ObservationSpec and ActionSpec for this agent

    std::string SimpleVehicleActionDescription =
        "The OpenBot action space consists of a 2D vector containing the "
        "percentages of "
        "battery voltage to be applied to the left (resp. right) vehicle "
        "motors.\n"
        "These commands are in the [-1, 1] range.\n";
    unrealrl::ActionSpec SimpleVehicleActionSpec(false, unrealrl::DataType::Float32, {2}, std::make_pair(-1.0f, 1.0f), SimpleVehicleActionDescription); // HACK: should specify constants in a config file

    std::string SimpleVehicleObservationDescription =
        "The agent has following observations:\n"
        "left wheel commands in the range [-1, 1]\n"
        "right wheel commands in the range [-1, 1]\n"
        "Euclidean distance between current x-y position and target x-y "
        "position.\n"
        "Sinus of the relative yaw between current pose and target pose.\n"
        "Cosinus of the relative yaw between current pose and target pose.\n";

    unrealrl::ObservationSpec SimpleVehicleObservationSpec({5}, unrealrl::DataType::Float32, SimpleVehicleObservationDescription);

    if(unrealrl::Config::GetValue<bool>({"INTERIOR_SIM_BRIDGE", "USE_IMAGE_OBSERVATIONS"}))
    {
        std::string CameraObservationDescription = "This observation is an egocentric RGB image.\n";
        unrealrl::ObservationSpec CameraObservationSpec({{height_, width_, 3}}, unrealrl::DataType::UInteger8, CameraObservationDescription); // HACK: should specify constants in a config file

        SetObservationSpecs({SimpleVehicleObservationSpec, CameraObservationSpec});
    }
    else
    {
        SetObservationSpecs({SimpleVehicleObservationSpec});
    }
    SetActionSpecs({SimpleVehicleActionSpec});
}

bool USimpleVehicleBrain::IsAgentReady()
{
    ASSERT(ownerPawn != nullptr);
    return (ownerPawn->GetVelocity().Size() >= 0.0 && ownerPawn->GetVelocity().Size() < 0.1); // Wait for the agent to settle...
}

void USimpleVehicleBrain::OnEpisodeBegin()
{
    // reset by reload entire map
    UGameplayStatics::OpenLevel(this, FName(*GetWorld()->GetName()), false);
}

void USimpleVehicleBrain::SetAction(const std::vector<unrealrl::Action> &actionVector)
{
    ASSERT(ownerPawn != nullptr);
    ASSERT(actionVector.size() == 1);
    
    std::vector<float> controlState;
    controlState = actionVector.at(0).GetActions();

    ASSERT(controlState.size() == 2);

    ownerPawn->MoveLeftRight(controlState[0], controlState[1]); // controlState are in the [-1.0; 1.0] range
}

void USimpleVehicleBrain::GetObservation(std::vector<unrealrl::Observation> &observationVector)
{
    ASSERT(ownerPawn != nullptr);
    ASSERT(goalActor != nullptr);

    // Get OpenBot position and orientation:
    const FVector currentLocation = ownerPawn->GetActorLocation();     // Relative to global coordinate system
    const FRotator currentOrientation = ownerPawn->GetActorRotation(); // Relative to global coordinate system
    Eigen::Vector2f controlState;
    
    if (unrealrl::Config::GetValue<std::string>({"INTERIOR_SIM_BRIDGE", "ROBOTSIM_LEARNING_MODE"}) == "reinforcement_learning")
    {
        // Get relative position to target in the global coordinate system:
        const FVector2D relativePositionToTarget(
            (goalActor->GetActorLocation() - currentLocation).X,
            (goalActor->GetActorLocation() - currentLocation).Y);

        // Compute Euclidean distance to target:
        float dist = relativePositionToTarget.Size();

        // Compute robot forward axis (global coordinate system)
        FVector forwardAxis = FVector(1.f, 0.f, 0.f); // Front axis is the X axis.
        FVector forwardAxisRotated = currentOrientation.RotateVector(forwardAxis);

        // Compute yaw in [rad]:
        float deltaYaw = std::atan2f(forwardAxisRotated.Y, forwardAxisRotated.X) - std::atan2f(relativePositionToTarget.Y, relativePositionToTarget.X);

        // Fit to range [-pi, pi]:
        if (deltaYaw > PI)
        {
            deltaYaw -= 2 * PI;
        }
        else
        {
            if (deltaYaw <= -PI)
            {
                deltaYaw += 2 * PI;
            }
        }

        // Check the actual OpenBot code:
        // https://github.com/isl-org/OpenBot/blob/7868c54742f8ba3df0ba2a886247a753df982772/android/app/src/main/java/org/openbot/pointGoalNavigation/PointGoalNavigationFragment.java#L103
        float sinYaw = std::sinf(deltaYaw);
        float cosYaw = std::cosf(deltaYaw);

        // Get observation data
        observationVector.resize(GetObservationSpecs().size());

        // Vector observations
        controlState = ownerPawn->GetControlState(); // Fuses the actions received from the python client
                                                  // with those received from the keyboard interface (if
                                                  // this interface is activated in the settings.json
                                                  // file)

        observationVector.at(0).Copy(std::vector<float>{controlState(0), controlState(1), dist, sinYaw, cosYaw});

        switch (hitInfo_)
        {
        case UHitInfo::Goal: // NOTE: so far we never get there...
            SetCurrentReward(100);
            EndEpisode();
            break;
        case UHitInfo::Edge: // NOTE: so far we never get there...
            SetCurrentReward(-100);
            EndEpisode();
            break;
        case UHitInfo::NoHit:
            SetCurrentReward(-dist / 100000); // TODO: set proper reward !!!!
            break;
        default:
            SetCurrentReward(-dist / 100000); // TODO: set proper reward !!!!
            break;
        }

    }
    if (unrealrl::Config::GetValue<std::string>({"INTERIOR_SIM_BRIDGE", "ROBOTSIM_LEARNING_MODE"}) == "imitation_learning")
    {
            // Get observation data
        observationVector.resize(GetObservationSpecs().size());

        // Vector observations
        controlState = ownerPawn->GetControlState(); // Fuses the actions received from the python client
                                                  // with those received from the keyboard interface (if
                                                  // this interface is activated in the settings.json
                                                  // file)

        observationVector.at(0).Copy(std::vector<float>{controlState(0), controlState(1), FMath::DegreesToRadians(currentOrientation.Yaw), currentLocation.X, currentLocation.Y});
    }
    else
    {
       ASSERT(false);
    }


    // Reset hitInfo_ .
    hitInfo_ = UHitInfo::NoHit;

    if(unrealrl::Config::GetValue<bool>({"INTERIOR_SIM_BRIDGE", "USE_IMAGE_OBSERVATIONS"}))
    {
        ASSERT(IsInGameThread());

        FTextureRenderTargetResource *targetResource = captureComponent2D_->TextureTarget->GameThread_GetRenderTargetResource();

        if (targetResource == nullptr)
        {
            UE_LOG(LogTemp, Error, TEXT("Could not get RenderTarget Resource from GameThread!! :("));
            ASSERT(false);
        }

        TArray<FColor> rawPixels;
        rawPixels.Reset();

        struct FReadSurfaceContext
        {
            FRenderTarget *srcRenderTarget;
            TArray<FColor> *outData;
            FIntRect rect;
            FReadSurfaceDataFlags flags;
        };

        FReadSurfaceContext context = {
            targetResource,
            &rawPixels,
            FIntRect(0, 0, targetResource->GetSizeXY().X, targetResource->GetSizeXY().Y),
            FReadSurfaceDataFlags(RCM_UNorm, CubeFace_MAX)};

        // Required for uint8 read mode
        context.flags.SetLinearToGamma(false);

        ENQUEUE_RENDER_COMMAND(ReadSurfaceCommand)
        (
            [context](FRHICommandListImmediate &RHICmdList)
            {
                RHICmdList.ReadSurfaceData(context.srcRenderTarget->GetRenderTargetTexture(), context.rect, *context.outData, context.flags);
            });

        FRenderCommandFence readPixelFence;
        readPixelFence.BeginFence();
        readPixelFence.Wait(true); // true if you want to process gamethreadtasks
        // in parallel while waiting for RT to complete the enqueued task

        std::vector<uint8_t> *pixelData = &observationVector.at(1).Data;

        pixelData->resize(height_ * width_ * 3); // HACK: should specify constants in a config file

        for (uint32 i = 0; i < static_cast<uint32>(rawPixels.Num()); ++i)
        {
            uint32 j = i;
            pixelData->at(3 * j) = rawPixels[i].R;
            pixelData->at(3 * j + 1) = rawPixels[i].G;
            pixelData->at(3 * j + 2) = rawPixels[i].B;
        }

        observationVector.at(1).bIsSet = true;
    }
}

void USimpleVehicleBrain::OnActorHit(AActor *selfActor,
                                     AActor *otherActor,
                                     FVector normalImpulse,
                                     const FHitResult &hitFlag)
{
    ASSERT(otherActor != nullptr);

    if (otherActor->ActorHasTag("goal"))
    {
        hitInfo_ = UHitInfo::Goal;
    }
    // TODO: Does instid1227 apply to all obstacles?
    // If not, include all obstacles or provide an user interface to specify
    // obstacles
    else if (!otherActor->GetName().Contains(TEXT("instid1227"), ESearchCase::IgnoreCase))
    {
        hitInfo_ = UHitInfo::Edge;
    }
}
