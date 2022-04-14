#include "SimpleVehicleBrain.h"

USimpleVehicleBrain::USimpleVehicleBrain(const FObjectInitializer& objectInitializer) : Super(objectInitializer)
{
}

void USimpleVehicleBrain::Init()
{
    for (TActorIterator<ASimpleVehiclePawn> it(GetWorld()); it; ++it)
    {
        ownerPawn = *it;
    }

    ASSERT(ownerPawn != nullptr);

    APlayerController* Controller = GetWorld()->GetFirstPlayerController();
    ASSERT(Controller != nullptr);

    // Look for the desired observation camera among all available camera actors:
    for (TActorIterator<APIPCamera> it(GetWorld()); it; ++it)
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

    if (unrealrl::Config::GetValue<bool>({"INTERIOR_SIM_BRIDGE", "USE_IMAGE_OBSERVATIONS"}))
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

    /*
     * The OpenBot action space consists of a 2D vector containing the percentages of
     * battery voltage to be applied to the left (resp. right) vehicle motors.
     * These commands are in the [-1, 1] range.
     */
    std::string SimpleVehicleActionDescription = "";

    unrealrl::ActionSpec SimpleVehicleActionSpec(false, unrealrl::DataType::Float32, {2}, std::make_pair(-1.0f, 1.0f), SimpleVehicleActionDescription); // HACK: should specify constants in a config file

    /*
     * The agent has following observations:
     *   --> left wheel commands in the range [-1, 1]
     *   --> right wheel commands in the range [-1, 1]
     *   --> Euclidean distance between current x-y position and target x-y position.
     *   --> Sinus of the relative yaw between current pose and target pose.
     *   --> Cosinus of the relative yaw between current pose and target pose.
     */
    std::string SimpleVehicleObservationDescription = "";

    unrealrl::ObservationSpec SimpleVehicleObservationSpec({5}, unrealrl::DataType::Float32, SimpleVehicleObservationDescription);

    if (unrealrl::Config::GetValue<bool>({"INTERIOR_SIM_BRIDGE", "USE_IMAGE_OBSERVATIONS"}))
    {
        // This observation is an egocentric RGB image.
        std::string CameraObservationDescription = "";
        unrealrl::ObservationSpec CameraObservationSpec({{unrealrl::Config::GetValue<unsigned long>({"INTERIOR_SIM_BRIDGE", "IMAGE_HEIGHT"}), unrealrl::Config::GetValue<unsigned long>({"INTERIOR_SIM_BRIDGE", "IMAGE_WIDTH"}), 3}}, unrealrl::DataType::UInteger8, CameraObservationDescription); // HACK: should specify constants in a config file

        SetObservationSpecs({SimpleVehicleObservationSpec, CameraObservationSpec});
    }
    else
    {
        SetObservationSpecs({SimpleVehicleObservationSpec});
    }
    SetActionSpecs({SimpleVehicleActionSpec});

    // Trajectory planning:
    if (unrealrl::Config::GetValue<bool>({"INTERIOR_SIM_BRIDGE", "ACTIVATE_AUTOPILOT"}))
    {
        // Initialize navigation variables:
        int numberOfWayPoints = 0;
        int numIter = 0;
        int numIterTot = 0;
        int numCycle = 0;
        FVector initialPosition = ownerPawn->GetActorLocation();
        FNavLocation targetLocation;
        FVector2D relativePositionToTarget(0.0f, 0.0f);

        // Initialize navigation:
        UNavigationSystemV1* navSys = Cast<UNavigationSystemV1>(ownerPawn->GetWorld()->GetNavigationSystem());
        ANavigationData* navData = (navSys == nullptr) ? nullptr : navSys->GetNavDataForProps(ownerPawn->GetNavAgentPropertiesRef());

        // Set path generation query:
        FPathFindingQuery Query = FPathFindingQuery(*ownerPawn, *navData, initialPosition, targetLocation.Location);
        // Set the path query such that case no path to the target can be found, a path that brings the agent as close as possible to the target can still be generated
        Query.SetAllowPartialPaths(true);

        // Path generation polling to get "interesting" paths in every experiment:
        while ((numberOfWayPoints < unrealrl::Config::GetValue<int>({"INTERIOR_SIM_BRIDGE", "MAX_WAY_POINTS"}) - numCycle and relativePositionToTarget.Size() < unrealrl::Config::GetValue<int>({"INTERIOR_SIM_BRIDGE", "MIN_TARGET_DISTANCE"})) or numIterTot < unrealrl::Config::GetValue<int>({"INTERIOR_SIM_BRIDGE", "MAX_ITER_REPLAN"}) * unrealrl::Config::GetValue<int>({"INTERIOR_SIM_BRIDGE", "MAX_WAY_POINTS"})) // Try to generate interesting trajectories with multiple waypoints
        {
            std::cout << "Iteration: " << numIterTot << std::endl;
            if (numIter >= unrealrl::Config::GetValue<int>({"INTERIOR_SIM_BRIDGE", "MAX_ITER_REPLAN"}))
            {
                numCycle++;
                numIter = 0;
            }
            pathPoints_.Empty();

            // Ret a random target point, to be reached by the agent:
            if (not navSys->GetRandomReachablePointInRadius(initialPosition, unrealrl::Config::GetValue<float>({"INTERIOR_SIM_BRIDGE", "TARGET_RADIUS"}), targetLocation))
            {
                ASSERT(false);
            }
            relativePositionToTarget.X = (targetLocation.Location - initialPosition).X;
            relativePositionToTarget.Y = (targetLocation.Location - initialPosition).Y;

            // Genrate a collision-free path between the robot position and the target point:
            FPathFindingResult collisionFreePath = navSys->FindPathSync(Query, EPathFindingMode::Type::Regular);

            // If path generation is sucessful, analyze the obtained path (it should not be too simple):
            if (collisionFreePath.IsSuccessful() && collisionFreePath.Path.IsValid())
            {
                if (collisionFreePath.IsPartial())
                {
                    std::cout << "Only a partial path could be found by the planner..." << std::endl;
                }
                pathPoints_ = collisionFreePath.Path->GetPathPoints();
                numberOfWayPoints = pathPoints_.Num();
            }
            std::cout << "Number of way points: " << numberOfWayPoints << std::endl;
            std::cout << "Target distance: " << relativePositionToTarget.Size() * 0.01 << "m" << std::endl;
            numIter++;
            numIterTot++;
        }

        std::cout << "Current position: [" << initialPosition.X << ", " << initialPosition.Y << ", " << initialPosition.Z << "]." << std::endl;
        std::cout << "Reachable position: [" << targetLocation.Location.X << ", " << targetLocation.Location.Y << ", " << targetLocation.Location.Z << "]." << std::endl;
        std::cout << "-----------------------------------------------------------" << std::endl;
        std::cout << "Way points: " << std::endl;
        for (auto wayPoint : pathPoints_)
        {
            std::cout << "[" << wayPoint.Location.X << ", " << wayPoint.Location.Y << ", " << wayPoint.Location.Z << "]" << std::endl;
        }
        std::cout << "-----------------------------------------------------------" << std::endl;
        currentPathPoint_.X = pathPoints_[indexPath_].Location.X;
        currentPathPoint_.Y = pathPoints_[indexPath_].Location.Y;
        indexPath_++;
    }
}

bool USimpleVehicleBrain::IsAgentReady()
{
    ASSERT(ownerPawn != nullptr);
    return (ownerPawn->GetVelocity().Size() >= 0.0 && ownerPawn->GetVelocity().Size() < 0.1); // Wait for the agent to settle...
}

void USimpleVehicleBrain::OnEpisodeBegin()
{
    // Reset by reloading the entire map:
    UGameplayStatics::OpenLevel(this, FName(*GetWorld()->GetName()), false);
}

void USimpleVehicleBrain::SetAction(const std::vector<unrealrl::Action>& actionVector)
{
    ASSERT(ownerPawn != nullptr);
    ASSERT(actionVector.size() == 1);

    if (unrealrl::Config::GetValue<bool>({"INTERIOR_SIM_BRIDGE", "ACTIVATE_AUTOPILOT"}))
    {
        if (ownerPawn->MoveTo(currentPathPoint_))
        {
            if (indexPath_ == pathPoints_.Num() - 1) // Move to the next waypoint
            {
                std::cout << "######## Reached waypoint " << indexPath_ << " ########" << std::endl;
                currentPathPoint_.X = pathPoints_[indexPath_].Location.X;
                currentPathPoint_.Y = pathPoints_[indexPath_].Location.Y;
                indexPath_++;
            }
            else // We reached the target
            {
                std::cout << "############ Reached the target location ! ############" << std::endl;
                hitInfo_ = UHitInfo::Goal;
            }
        }
    }
    else
    {
        std::vector<float> controlState;
        controlState = actionVector.at(0).GetActions(); // Get actions from the python interface

        ASSERT(controlState.size() == 2);

        ownerPawn->MoveLeftRight(controlState[0], controlState[1]); // controlState are in the [-1.0; 1.0] range
    }
}

void USimpleVehicleBrain::GetObservation(std::vector<unrealrl::Observation>& observationVector)
{
    ASSERT(ownerPawn != nullptr);
    ASSERT(goalActor != nullptr);

    // Get OpenBot position and orientation:
    const FVector currentLocation = ownerPawn->GetActorLocation();     // Relative to global coordinate system
    const FRotator currentOrientation = ownerPawn->GetActorRotation(); // Relative to global coordinate system
    Eigen::Vector2f controlState;
    float dist;
    float deltaYaw;
    float sinYaw;
    float cosYaw;

    if (unrealrl::Config::GetValue<std::string>({"INTERIOR_SIM_BRIDGE", "OBSERVATION_VECTOR"}) == "dist-sin-cos")
    {
        // Get relative position to target in the global coordinate system:
        const FVector2D relativePositionToTarget(
            (goalActor->GetActorLocation() - currentLocation).X,
            (goalActor->GetActorLocation() - currentLocation).Y);

        // Compute Euclidean distance to target:
        dist = relativePositionToTarget.Size();

        // Compute robot forward axis (global coordinate system)
        FVector forwardAxis = FVector(1.f, 0.f, 0.f); // Front axis is the X axis.
        FVector forwardAxisRotated = currentOrientation.RotateVector(forwardAxis);

        // Compute yaw in [rad]:
        deltaYaw = std::atan2f(forwardAxisRotated.Y, forwardAxisRotated.X) - std::atan2f(relativePositionToTarget.Y, relativePositionToTarget.X);

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
        sinYaw = std::sinf(deltaYaw);
        cosYaw = std::cosf(deltaYaw);

        // Get observation data
        observationVector.resize(GetObservationSpecs().size());

        // Vector observations
        controlState = ownerPawn->GetControlState(); // Fuses the actions received from the python client
                                                     // with those received from the keyboard interface (if
                                                     // this interface is activated in the settings.json
                                                     // file)

        observationVector.at(0).Copy(std::vector<float>{controlState(0), controlState(1), dist, sinYaw, cosYaw});

        // Hack:
        if (dist < 10) // in [cm]
        {
            hitInfo_ = UHitInfo::Goal;
        }
    }
    else if (unrealrl::Config::GetValue<std::string>({"INTERIOR_SIM_BRIDGE", "OBSERVATION_VECTOR"}) == "yaw-x-y")
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

    // Reward function is defined based on https://github.com/isl-org/OpenBot-Distributed/blob/main/trainer/ob_agents/envs/open_bot_replay_env.py
    switch (hitInfo_)
    {
    case UHitInfo::Goal:
        // Goal reached reward:
        AddReward(unrealrl::Config::GetValue<float>({"INTERIOR_SIM_BRIDGE", "REWARD_GOAL_REACHED"}));
        EndEpisode();
        break;
    case UHitInfo::Edge:
        // Obstacle penalty
        AddReward(-unrealrl::Config::GetValue<float>({"INTERIOR_SIM_BRIDGE", "REWARD_COLLISION"}));
        EndEpisode();
        break;
    case UHitInfo::NoHit:
        // Constant timestep penalty:
        AddReward(-1 / unrealrl::Config::GetValue<float>({"INTERIOR_SIM_BRIDGE", "MAX_STEPS_PER_EPISODE"}));
        // Goal distance penalty:
        AddReward(-dist * unrealrl::Config::GetValue<float>({"INTERIOR_SIM_BRIDGE", "REWARD_DISTANCE_GAIN"}));
        AddReward(-unrealrl::Config::GetValue<float>({"INTERIOR_SIM_BRIDGE", "REWARD_DISTANCE_BIAS"}));
        break;
    default:
        SetCurrentReward(0.0);
        break;
    }

    // Reset hitInfo_ .
    hitInfo_ = UHitInfo::NoHit;

    if (unrealrl::Config::GetValue<bool>({"INTERIOR_SIM_BRIDGE", "USE_IMAGE_OBSERVATIONS"}))
    {
        ASSERT(IsInGameThread());

        FTextureRenderTargetResource* targetResource = captureComponent2D_->TextureTarget->GameThread_GetRenderTargetResource();

        if (targetResource == nullptr)
        {
            UE_LOG(LogTemp, Error, TEXT("Could not get RenderTarget Resource from GameThread!! :("));
            ASSERT(false);
        }

        TArray<FColor> rawPixels;
        rawPixels.Reset();

        struct FReadSurfaceContext
        {
            FRenderTarget* srcRenderTarget;
            TArray<FColor>* outData;
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
            [context](FRHICommandListImmediate& RHICmdList)
            {
                RHICmdList.ReadSurfaceData(context.srcRenderTarget->GetRenderTargetTexture(), context.rect, *context.outData, context.flags);
            });

        FRenderCommandFence readPixelFence;
        readPixelFence.BeginFence();
        readPixelFence.Wait(true); // true if you want to process gamethreadtasks
        // in parallel while waiting for RT to complete the enqueued task

        std::vector<uint8_t>* pixelData = &observationVector.at(1).Data;

        pixelData->resize(unrealrl::Config::GetValue<unsigned long>({"INTERIOR_SIM_BRIDGE", "IMAGE_HEIGHT"}) * unrealrl::Config::GetValue<unsigned long>({"INTERIOR_SIM_BRIDGE", "IMAGE_WIDTH"}) * 3); // HACK: should specify constants in a config file

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

void USimpleVehicleBrain::OnActorHit(AActor* selfActor,
                                     AActor* otherActor,
                                     FVector normalImpulse,
                                     const FHitResult& hitFlag)
{
    ASSERT(otherActor != nullptr);

    if (otherActor->ActorHasTag("goal"))
    {
        hitInfo_ = UHitInfo::Goal;
    }
    else
    {
        hitInfo_ = UHitInfo::Edge;
    }
}