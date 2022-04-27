#include "SimpleVehicleBrain.h"

USimpleVehicleBrain::USimpleVehicleBrain(const FObjectInitializer& objectInitializer) : Super(objectInitializer)
{
}

void USimpleVehicleBrain::Init()
{
    for (TActorIterator<ASimpleVehiclePawn> it(GetWorld()); it; ++it) {
        ownerPawn = *it;
    }

    ASSERT(ownerPawn != nullptr);

    APlayerController* Controller = GetWorld()->GetFirstPlayerController();
    ASSERT(Controller != nullptr);

    // Look for the desired observation camera among all available camera actors:
    for (TActorIterator<APIPCamera> it(GetWorld()); it; ++it) {
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

    if (unrealrl::Config::GetValue<bool>({"INTERIOR_SIM_BRIDGE", "USE_IMAGE_OBSERVATIONS"})) {
        // Point to the vehicle camera capture component:
        captureComponent2D_ = mainCamera_->GetSceneCaptureComponent();

        // Set Camera Properties
        captureComponent2D_->bAlwaysPersistRenderingState = 1;
        captureComponent2D_->bCaptureEveryFrame = 0;
        captureComponent2D_->FOVAngle = unrealrl::Config::GetValue<float>({"INTERIOR_SIM_BRIDGE", "SMARTPHONE_FOV"}); // Smartphone FOV
        captureComponent2D_->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
        captureComponent2D_->ShowFlags.SetTemporalAA(false);
        captureComponent2D_->ShowFlags.SetAntiAliasing(true);

        // Adjust RenderTarget
        UTextureRenderTarget2D* renderTarget2D = NewObject<UTextureRenderTarget2D>();
        renderTarget2D->TargetGamma = GEngine->GetDisplayGamma();                                                                                                                                                                   // Set FrameWidth and FrameHeight: 1.2f; for Vulkan | GEngine->GetDisplayGamma(); for DX11/12
        renderTarget2D->InitAutoFormat(unrealrl::Config::GetValue<unsigned long>({"INTERIOR_SIM_BRIDGE", "IMAGE_WIDTH"}), unrealrl::Config::GetValue<unsigned long>({"INTERIOR_SIM_BRIDGE", "IMAGE_HEIGHT"}));                      // Setup the RenderTarget capture format: some random format, got crashing otherwise frameWidht = 2048 and frameHeight = 2048.
        renderTarget2D->InitCustomFormat(unrealrl::Config::GetValue<unsigned long>({"INTERIOR_SIM_BRIDGE", "IMAGE_WIDTH"}), unrealrl::Config::GetValue<unsigned long>({"INTERIOR_SIM_BRIDGE", "IMAGE_HEIGHT"}), PF_B8G8R8A8, true); // PF_B8G8R8A8 disables HDR which will boost storing to disk due to less image information
        renderTarget2D->RenderTargetFormat = ETextureRenderTargetFormat::RTF_RGBA8;
        renderTarget2D->bGPUSharedFlag = true; // demand buffer on GPU
        captureComponent2D_->TextureTarget = renderTarget2D;

        // Set post processing parameters:
        FPostProcessSettings postProcSet;
        postProcSet.MotionBlurAmount = unrealrl::Config::GetValue<float>({"INTERIOR_SIM_BRIDGE", "MOTION_BLUR_AMMOUNT"}); // Strength of motion blur, 0:off, should be renamed to intensity
        postProcSet.MotionBlurMax = unrealrl::Config::GetValue<float>({"INTERIOR_SIM_BRIDGE", "MOTION_BLUR_MAX"});    // Max distortion caused by motion blur, in percent of the screen width, 0:off
        captureComponent2D_->PostProcessSettings = postProcSet;
        captureComponent2D_->PostProcessBlendWeight = unrealrl::Config::GetValue<float>({"INTERIOR_SIM_BRIDGE", "POST_PROC_BLEND_WEIGHT"}); // Range (0.0, 1.0) where 0 indicates no effect, 1 indicates full effect.
    }

    ownerPawn->OnActorHit.AddDynamic(this, &USimpleVehicleBrain::OnActorHit);

    // Store actor refs required during simulation.
    for (TActorIterator<AActor> ActorItr(GetWorld(), AActor::StaticClass());
         ActorItr; ++ActorItr) {
        if ((*ActorItr)->ActorHasTag("goal")) {
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

    unrealrl::ObservationSpec SimpleVehicleObservationSpec; 

    if (unrealrl::Config::GetValue<std::string>({"INTERIOR_SIM_BRIDGE", "OBSERVATION_VECTOR"}) == "dist-sin-cos") {
        SimpleVehicleObservationSpec = unrealrl::ObservationSpec({5}, unrealrl::DataType::Float32, SimpleVehicleObservationDescription);
    }
    else if (unrealrl::Config::GetValue<std::string>({"INTERIOR_SIM_BRIDGE", "OBSERVATION_VECTOR"}) == "yaw-x-y") {
        SimpleVehicleObservationSpec = unrealrl::ObservationSpec({8}, unrealrl::DataType::Float32, SimpleVehicleObservationDescription);
    }
    else{
        SimpleVehicleObservationSpec = unrealrl::ObservationSpec({5}, unrealrl::DataType::Float32, SimpleVehicleObservationDescription);
    }

    if (unrealrl::Config::GetValue<bool>({"INTERIOR_SIM_BRIDGE", "USE_IMAGE_OBSERVATIONS"})) {
        // This observation is an egocentric RGB image.
        std::string CameraObservationDescription = "";
        unrealrl::ObservationSpec CameraObservationSpec({{unrealrl::Config::GetValue<unsigned long>({"INTERIOR_SIM_BRIDGE", "IMAGE_HEIGHT"}), unrealrl::Config::GetValue<unsigned long>({"INTERIOR_SIM_BRIDGE", "IMAGE_WIDTH"}), 3}}, unrealrl::DataType::UInteger8, CameraObservationDescription); // HACK: should specify constants in a config file

        SetObservationSpecs({SimpleVehicleObservationSpec, CameraObservationSpec});
    }
    else {
        SetObservationSpecs({SimpleVehicleObservationSpec});
    }
    SetActionSpecs({SimpleVehicleActionSpec});

    // Trajectory planning:
    if (unrealrl::Config::GetValue<bool>({"INTERIOR_SIM_BRIDGE", "ACTIVATE_AUTOPILOT"})) {
        // Initialize navigation variables:
        int numberOfWayPoints = 0;
        int numIter = 0;
        float pathCriterion = 0.f;
        FVector initialPosition = ownerPawn->GetActorLocation();
        FNavLocation bestTargetLocation;
        FVector2D relativePositionToTarget(0.0f, 0.0f);

        // Initialize navigation:
        //UNavigationSystemV1* navSys = Cast<UNavigationSystemV1>(ownerPawn->GetWorld()->GetNavigationSystem());
        UNavigationSystemV1* navSys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(GetWorld());

        if (not navSys) {
            ASSERT(false);
        }
        
        //auto navData = navSys->GetNavDataForAgentName(FName(ownerPawn->GetName()));

        ANavigationData* navData = (navSys == nullptr) ? nullptr : navSys->GetNavDataForProps(ownerPawn->GetNavAgentPropertiesRef());
        if (not navData) {
            ASSERT(false);
        }
        

        // ARecastNavMesh* navMesh = Cast<ARecastNavMesh>(navData);

        // if (not navMesh) {
        //     ASSERT(false);
        // }

        // Set path generation query:
        FPathFindingQuery Query = FPathFindingQuery(*ownerPawn, *navData, initialPosition, targetLocation_.Location);
        // Set the path query such that case no path to the target can be found, a path that brings the agent as close as possible to the target can still be generated
        Query.SetAllowPartialPaths(true);

        // Path generation polling to get "interesting" paths in every experiment:
        while (numIter < unrealrl::Config::GetValue<int>({"INTERIOR_SIM_BRIDGE", "MAX_ITER_REPLAN"})) // Try to generate interesting trajectories with multiple waypoints
        {
            // Ret a random target point, to be reached by the agent:
            if (not navSys->GetRandomReachablePointInRadius(initialPosition, unrealrl::Config::GetValue<float>({"INTERIOR_SIM_BRIDGE", "TARGET_RADIUS"}), targetLocation_)) {
                ASSERT(false);
            }
            relativePositionToTarget.X = (targetLocation_.Location - initialPosition).X;
            relativePositionToTarget.Y = (targetLocation_.Location - initialPosition).Y;

            // Update navigation query with the new target: 
            Query = FPathFindingQuery(*ownerPawn, *navData, initialPosition, targetLocation_.Location);

            // Genrate a collision-free path between the robot position and the target point:
            FPathFindingResult collisionFreePath = navSys->FindPathSync(Query, EPathFindingMode::Type::Regular);

            // If path generation is sucessful, analyze the obtained path (it should not be too simple):
            if (collisionFreePath.IsSuccessful() and collisionFreePath.Path.IsValid()) {
                if (collisionFreePath.IsPartial()) {
                    std::cout << "Only a partial path could be found by the planner..." << std::endl;
                }
                numberOfWayPoints = collisionFreePath.Path->GetPathPoints().Num();
                float crit = relativePositionToTarget.Size()*unrealrl::Config::GetValue<float>({"INTERIOR_SIM_BRIDGE", "PATH_WEIGHT_DIST"}) + numberOfWayPoints * relativePositionToTarget.Size() * unrealrl::Config::GetValue<float>({"INTERIOR_SIM_BRIDGE", "PATH_WEIGHT_NUM_WAYPOINTS"});
                
                if (pathCriterion <= crit) {
                    std::cout << "Iteration: " << numIter << std::endl;
                    std::cout << "Cost: " << crit << std::endl;
                    pathCriterion = crit; 
                    bestTargetLocation = targetLocation_;
                    pathPoints_.Empty();
                    pathPoints_ = collisionFreePath.Path->GetPathPoints();
                    std::cout << "Number of way points: " << numberOfWayPoints << std::endl;
                    std::cout << "Target distance: " << relativePositionToTarget.Size() * 0.01 << "m" << std::endl;
                }
                numIter++;
            }
        }

        ASSERT(pathPoints_.Num() != 0);

        targetLocation_ = bestTargetLocation;

        std::cout << "Current position: [" << initialPosition.X << ", " << initialPosition.Y << ", " << initialPosition.Z << "]." << std::endl;
        std::cout << "Reachable position: [" << bestTargetLocation.Location.X << ", " << bestTargetLocation.Location.Y << ", " << bestTargetLocation.Location.Z << "]." << std::endl;
        std::cout << "-----------------------------------------------------------" << std::endl;
        std::cout << "Way points: " << std::endl;
        for (auto wayPoint : pathPoints_) {
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

    if (unrealrl::Config::GetValue<bool>({"INTERIOR_SIM_BRIDGE", "ACTIVATE_AUTOPILOT"})) {
        //std::cout << "Waypoint " << indexPath_ << " over " << pathPoints_.Num() << "." << std::endl;
        if (ownerPawn->MoveTo(currentPathPoint_)) {
            if (indexPath_ < pathPoints_.Num() - 1) // Move to the next waypoint
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
    else {
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
    // Get relative position to target in the global coordinate system:
    // const FVector2D relativePositionToTarget((goalActor->GetActorLocation() - currentLocation).X, (goalActor->GetActorLocation() - currentLocation).Y);
    const FVector2D relativePositionToTarget((targetLocation_.Location - currentLocation).X, (targetLocation_.Location - currentLocation).Y);
    Eigen::Vector2f controlState;
    float dist;
    float deltaYaw;
    float sinYaw;
    float cosYaw;

    if (unrealrl::Config::GetValue<std::string>({"INTERIOR_SIM_BRIDGE", "OBSERVATION_VECTOR"}) == "dist-sin-cos") {
        
        // Compute Euclidean distance to target:
        dist = relativePositionToTarget.Size();

        // Compute robot forward axis (global coordinate system)
        FVector forwardAxis = FVector(1.f, 0.f, 0.f); // Front axis is the X axis.
        FVector forwardAxisRotated = currentOrientation.RotateVector(forwardAxis);

        // Compute yaw in [rad]:
        deltaYaw = std::atan2f(forwardAxisRotated.Y, forwardAxisRotated.X) - std::atan2f(relativePositionToTarget.Y, relativePositionToTarget.X);

        // Fit to range [-pi, pi]:
        if (deltaYaw > PI) {
            deltaYaw -= 2 * PI;
        }
        else {
            if (deltaYaw <= -PI) {
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
    else if (unrealrl::Config::GetValue<std::string>({"INTERIOR_SIM_BRIDGE", "OBSERVATION_VECTOR"}) == "yaw-x-y") {
        // Get observation data
        observationVector.resize(GetObservationSpecs().size());

        //std::cout << "relativePositionToTarget: [" << currentLocation.X << ", " << currentLocation.Y << ", " << currentLocation.Z << "]." << std::endl;

        // Vector observations
        controlState = ownerPawn->GetControlState(); // Fuses the actions received from the python client with those received from the keyboard interface (if
                                                     // this interface is activated in the settings.json file)

        observationVector.at(0).Copy(std::vector<float>{controlState(0), controlState(1), currentLocation.X, currentLocation.Y, currentLocation.Z, FMath::DegreesToRadians(currentOrientation.Roll), FMath::DegreesToRadians(currentOrientation.Pitch), FMath::DegreesToRadians(currentOrientation.Yaw)});
    }
    else {
        ASSERT(false);
    }

    // Reward function is defined based on https://github.com/isl-org/OpenBot-Distributed/blob/main/trainer/ob_agents/envs/open_bot_replay_env.py
    switch (hitInfo_) {
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

    if (unrealrl::Config::GetValue<bool>({"INTERIOR_SIM_BRIDGE", "USE_IMAGE_OBSERVATIONS"})) {
        ASSERT(IsInGameThread());

        FTextureRenderTargetResource* targetResource = captureComponent2D_->TextureTarget->GameThread_GetRenderTargetResource();

        if (targetResource == nullptr) {
            UE_LOG(LogTemp, Error, TEXT("Could not get RenderTarget Resource from GameThread!! :("));
            ASSERT(false);
        }

        TArray<FColor> rawPixels;
        rawPixels.Reset();

        struct FReadSurfaceContext {
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
            [context](FRHICommandListImmediate& RHICmdList) {
                RHICmdList.ReadSurfaceData(context.srcRenderTarget->GetRenderTargetTexture(), context.rect, *context.outData, context.flags);
            });

        FRenderCommandFence readPixelFence;
        readPixelFence.BeginFence();
        readPixelFence.Wait(true); // true if you want to process gamethreadtasks
        // in parallel while waiting for RT to complete the enqueued task

        std::vector<uint8_t>* pixelData = &observationVector.at(1).Data;

        pixelData->resize(unrealrl::Config::GetValue<unsigned long>({"INTERIOR_SIM_BRIDGE", "IMAGE_HEIGHT"}) * unrealrl::Config::GetValue<unsigned long>({"INTERIOR_SIM_BRIDGE", "IMAGE_WIDTH"}) * 3); // HACK: should specify constants in a config file

        for (uint32 i = 0; i < static_cast<uint32>(rawPixels.Num()); ++i) {
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

    if (otherActor->ActorHasTag("goal")) {
        hitInfo_ = UHitInfo::Goal;
    }
    else {
        hitInfo_ = UHitInfo::Edge;
    }
}
