#include "Navigation.h"

Navigation::Navigation(APawn* pawnAgent): pawnAgent_(pawnAgent)
{
    // Initialize navigation:
    resetNavigation(); 
}

Navigation::~Navigation()
{
}

void Navigation::resetNavigation()
{
    indexPath_ = 0; 
    navSystemRebuild();

    initialPosition_ = pawnAgent_->GetActorLocation(); // Initial position of the agent

    navQuery_ = FPathFindingQuery(*pawnAgent_, *navData_, initialPosition_, FVector(0.0f, 0.0f, 0.0f));

    // Set the path query such that case no path to the target can be found, a path that brings the agent as close as possible to the target can still be generated
    navQuery_.SetAllowPartialPaths(true);
}

FVector Navigation::generateRandomInitialPosition()
{
    indexPath_ = 0; 
    FVector initialPosition;

    // Spawn the agent in a random location within the navigation mesh:
    if (Config::getValue<bool>({"SIMULATION_CONTROLLER", "NAVIGATION", "SPAWN_ON_NAV_MESH"})) {

        int trial = 0;
        FNavLocation navLocation = navMesh_->GetRandomPoint();
        while (trial < 100) { 
            if (navLocation.Location.Z >= Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "AGENT_POSITION_Z_MIN"}) and navLocation.Location.Z <= Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "AGENT_POSITION_Z_MAX"})) {
                break;
            }
            std::cout << "navLocation.Location = [" << navLocation.Location.X << ", " << navLocation.Location.Y << ", " << navLocation.Location.Z << "]" << std::endl;
            navLocation = navMesh_->GetRandomPoint();
            trial++;
        } 
        initialPosition_ = navLocation.Location;       
    }
    else { 
        initialPosition_ = FVector(0);
    }

    // TODO: debug this mess after the paper submission... 
    // use BoxTracing to adjust pawn spawn height.
    // use mesh bounding box instead of setting.
    //std::cout << "-------------------------------------------" << std::endl;
    //FRotator spawnRotation = pawnAgent_->GetActorRotation();
    //FVector center = FVector(0.0f, 0.0f, 3.0f);
    //std::cout << "initialPosition = [" << initialPosition_.X << ", " << initialPosition_.Y << ", " << initialPosition_.Z << "]" << std::endl;
    //traceGround(initialPosition_, spawnRotation, FVector(10.0f, 10.0f, 10.0f));
    //initialPosition_ = initialPosition_ + FVector(0.0f, 0.0f, -3.0f);
    //std::cout << "initialPosition_GND = [" << initialPosition_.X << ", " << initialPosition_.Y << ", " << initialPosition_.Z << "]" << std::endl;
    return initialPosition_;
}

void Navigation::generateTrajectory()
{
    int numIter = 0;
    int numberOfWayPoints = 0;
    float pathCriterion = 0.0f;
    float bestPathCriterion = 0.0f;
    FNavLocation bestTargetLocation;
    FVector2D relativePositionToTarget(0.0f, 0.0f);

    // Path generation polling to get "interesting" paths in every experiment:
    while (numIter < Config::getValue<int>({"SIMULATION_CONTROLLER", "NAVIGATION", "MAX_ITER_REPLAN"})) // Try to generate interesting trajectories with multiple waypoints
    {
        // Get a random target point, to be reached by the agent:
        ASSERT(navSys_->GetRandomReachablePointInRadius(initialPosition_, Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "TARGET_RADIUS"}), targetLocation_));

        // Update relative position between the agent and its new target:
        relativePositionToTarget.X = (targetLocation_.Location - initialPosition_).X;
        relativePositionToTarget.Y = (targetLocation_.Location - initialPosition_).Y;

        // Update navigation query with the new target:
        navQuery_ = FPathFindingQuery(*pawnAgent_, *navData_, initialPosition_, targetLocation_.Location);

        // Genrate a collision-free path between the robot position and the target point:
        FPathFindingResult collisionFreePath = navSys_->FindPathSync(navQuery_, EPathFindingMode::Type::Regular);

        // If path generation is sucessful, analyze the obtained path (it should not be too simple):
        if (collisionFreePath.IsSuccessful() and collisionFreePath.Path.IsValid()) {

            if (collisionFreePath.IsPartial()) {
                std::cout << "Only a partial path could be found by the planner..." << std::endl;
            }

            numberOfWayPoints = collisionFreePath.Path->GetPathPoints().Num();
            pathCriterion = relativePositionToTarget.Size() * Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "PATH_WEIGHT_DIST"}) + numberOfWayPoints * relativePositionToTarget.Size() * Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "PATH_WEIGHT_NUM_WAYPOINTS"});

            if (bestPathCriterion <= pathCriterion) {
                bestPathCriterion = pathCriterion;
                bestTargetLocation = targetLocation_;
                pathPoints_.Empty();
                pathPoints_ = collisionFreePath.Path->GetPathPoints();
                std::cout << "Iteration: " << numIter << std::endl;
                std::cout << "Cost: " << bestPathCriterion << std::endl;
                std::cout << "Number of way points: " << numberOfWayPoints << std::endl;
                std::cout << "Target distance: " << relativePositionToTarget.Size() * 0.01 << "m" << std::endl;
            }
            numIter++;
        }
    }

    ASSERT(pathPoints_.Num() > 0);

    targetLocation_ = bestTargetLocation;

    std::cout << "Initial position: [" << initialPosition_.X << ", " << initialPosition_.Y << ", " << initialPosition_.Z << "]." << std::endl;
    std::cout << "Reachable position: [" << bestTargetLocation.Location.X << ", " << bestTargetLocation.Location.Y << ", " << bestTargetLocation.Location.Z << "]." << std::endl;
    std::cout << "-----------------------------------------------------------" << std::endl;
    std::cout << "Way points: " << std::endl;
    for (auto wayPoint : pathPoints_) {
        std::cout << "[" << wayPoint.Location.X << ", " << wayPoint.Location.Y << ", " << wayPoint.Location.Z << "]" << std::endl;
    }
    std::cout << "-----------------------------------------------------------" << std::endl;
    indexPath_ = 1; // Path point 0 is the initial robot position. getCurrentPathPoint() should therefore return the next point.
}

FVector2D Navigation::getPathPoint(size_t index)
{
    ASSERT(pathPoints_.Num() != 0);
    ASSERT(index < pathPoints_.Num());
    return FVector2D(pathPoints_[index].Location.X, pathPoints_[index].Location.Y);
}

FVector2D Navigation::getCurrentPathPoint()
{
    ASSERT(pathPoints_.Num() != 0);
    ASSERT(indexPath_ < pathPoints_.Num());
    return FVector2D(pathPoints_[indexPath_].Location.X, pathPoints_[indexPath_].Location.Y);
}

FVector2D Navigation::updateNavigation()
{
    const FVector agent_current_location = pawnAgent_->GetActorLocation();
    FVector2D relative_position_to_goal = getCurrentPathPoint() - FVector2D(agent_current_location.X, agent_current_location.Y);

    targetReached_ = false;

    // If a waypoint is reached
    if ((relative_position_to_goal.Size() * 0.01) < Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "ACCEPTANCE_RADIUS"})) { 

        if (indexPath_ < pathPoints_.Num() - 1) { // Move to the next waypoint
            std::cout << "######## Reached waypoint " << indexPath_ << " over " << pathPoints_.Num() - 1 << " ########" << std::endl;
            indexPath_++;
        }
        else { // We reached the final target
            std::cout << "############ Reached the target location ! ############" << std::endl;
            targetReached_ = true;
        }
    }

    return getCurrentPathPoint();
}

bool Navigation::navSystemRebuild()
{
    navSys_ = FNavigationSystem::GetCurrent<UNavigationSystemV1>(pawnAgent_->GetWorld());
    ASSERT(navSys_ != nullptr);

    navData_ = navSys_->GetNavDataForProps(pawnAgent_->GetNavAgentPropertiesRef());
    ASSERT(navData_ != nullptr);

    navMesh_ = Cast<ARecastNavMesh>(navData_);
    ASSERT(navMesh_ != nullptr);

    navmeshBounds_ = nullptr;
    for (TActorIterator<ANavMeshBoundsVolume> it(pawnAgent_->GetWorld()); it; ++it) {
        navmeshBounds_ = *it;
    }
    ASSERT(navmeshBounds_ != nullptr);

    // Set the NavMesh properties:
    navMesh_->AgentRadius = Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "NAVMESH", "AGENT_RADIUS"});
    navMesh_->AgentHeight = Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "NAVMESH", "AGENT_HEIGHT"});
    navMesh_->CellSize = Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "NAVMESH", "CELL_SIZE"});
    navMesh_->CellHeight = Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "NAVMESH", "CELL_HEIGHT"});
    navMesh_->AgentMaxSlope = Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "NAVMESH", "AGENT_MAX_SLOPE"});
    navMesh_->AgentMaxStepHeight = Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "NAVMESH", "AGENT_MAX_STEP_HEIGHT"});
    navMesh_->MergeRegionSize = Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "NAVMESH", "MERGE_REGION_SIZE"});
    navMesh_->MinRegionArea = Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "NAVMESH", "MIN_REGION_AREA"}); // ignore region that are too small
    navMesh_->MaxSimplificationError = Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "NAVMESH", "MAX_SIMPLIFINCATION_ERROR"});

    // Dynamic update navMesh location and size
    FBox worldBox = GetWorldBoundingBox();
    navmeshBounds_->GetRootComponent()->SetMobility(EComponentMobility::Movable);
    navmeshBounds_->SetActorLocation(worldBox.GetCenter(), false);          // Place the navmesh at the center of the map
    navmeshBounds_->SetActorRelativeScale3D(worldBox.GetSize() / 200.0f);   // Rescae the navmesh
    navmeshBounds_->GetRootComponent()->UpdateBounds();
    navSys_->OnNavigationBoundsUpdated(navmeshBounds_);

    if (Config::getValue<bool>({"SIMULATION_CONTROLLER", "NAVIGATION", "NAVMESH", "USE_STATIC_NAVMESH"})) {
        navmeshBounds_->GetRootComponent()->SetMobility(EComponentMobility::Static);
    }
    else { // A dynmic navmesh will account for changes occuring in the environment at runtime. But this is more computationally intensive...
        navmeshBounds_->GetRootComponent()->SetMobility(EComponentMobility::Movable); 
    }

    navSys_->Build(); // Rebuild NavMesh, required for update AgentRadius

    return true;
}

void Navigation::traceGround(FVector& spawnPosition, FRotator& spawnRotator, const FVector& boxHalfSize)
{
    FVector startLoc = spawnPosition + FVector(0, 0, 100);
    FVector endLoc = spawnPosition + FVector(0, 0, -1000);

    FCollisionQueryParams collisionParams(FName(TEXT("trace2ground")), true, pawnAgent_);
    FHitResult hit(ForceInit);

    if (UKismetSystemLibrary::BoxTraceSingle(pawnAgent_->GetWorld(), startLoc, endLoc, boxHalfSize, spawnRotator, ETraceTypeQuery::TraceTypeQuery1, false, TArray<AActor*>(), EDrawDebugTrace::Type::ForDuration, hit, true)) {
        spawnPosition = hit.Location;
    }
}

FBox Navigation::GetWorldBoundingBox(bool bScaleCeiling)
{
    FBox box(ForceInit);
    for (TActorIterator<AActor> it(pawnAgent_->GetWorld()); it; ++it) {
        if (it->ActorHasTag("architecture") || it->ActorHasTag("furniture")) {
            box += it->GetComponentsBoundingBox(false, true);
        }
    }
    // Remove ceiling
    return !bScaleCeiling ? box : box.ExpandBy(box.GetSize() * 0.1f).ShiftBy(FVector(0, 0, -0.3f * box.GetSize().Z));
}

