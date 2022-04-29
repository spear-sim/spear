#include "Navigation.h"

Navigation::Navigation(APawn* pawnAgent)
{
    // Initialize navigation:
    indexPath_ = 0; // Initialized at 1 since index 0 refers to the initial position of the agent.

    navSys_ = FNavigationSystem::GetCurrent<UNavigationSystemV1>(pawnAgent->GetWorld());
    ASSERT(navSys_ != nullptr);

    navData_ = navSys_->GetNavDataForProps(pawnAgent->GetNavAgentPropertiesRef());
    ASSERT(navData_ != nullptr);

    navMesh_ = Cast<ARecastNavMesh>(navData_);
    ASSERT(navMesh_ != nullptr);

    initialPosition_ = pawnAgent->GetActorLocation();

    navQuery_ = FPathFindingQuery(pawnAgent, *navData_, initialPosition_, targetLocation_.Location);

    // Set the path query such that case no path to the target can be found, a path that brings the agent as close as possible to the target can still be generated
    navQuery_.SetAllowPartialPaths(true);

    pawnAgent_ = pawnAgent;
}

Navigation::~Navigation()
{
}

FVector Navigation::generateRandomInitialPosition()
{
    // Spawn the agent in a random location within the navigation mesh:
    if (Config::getValue<bool>({"SIMULATION_CONTROLLER", "NAVIGATION", "SPAWN_ON_NAV_MESH"})) {

        FNavLocation navLocation = navMesh_->GetRandomPoint();

        return navLocation.Location;
    }
    else { // Spawn the agent in a random location:
        return FVector(0);
    }
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

    ASSERT(pathPoints_.Num() != 0);

    targetLocation_ = bestTargetLocation;

    std::cout << "Initial position: [" << initialPosition_.X << ", " << initialPosition_.Y << ", " << initialPosition_.Z << "]." << std::endl;
    std::cout << "Reachable position: [" << bestTargetLocation.Location.X << ", " << bestTargetLocation.Location.Y << ", " << bestTargetLocation.Location.Z << "]." << std::endl;
    std::cout << "-----------------------------------------------------------" << std::endl;
    std::cout << "Way points: " << std::endl;
    for (auto wayPoint : pathPoints_) {
        std::cout << "[" << wayPoint.Location.X << ", " << wayPoint.Location.Y << ", " << wayPoint.Location.Z << "]" << std::endl;
    }
    std::cout << "-----------------------------------------------------------" << std::endl;
    indexPath_++; // Path point 0 is the initial robot position. getCurrentPathPoint() should therefore return the next point.
}

FVector2D Navigation::getPathPoint(size_t index)
{
    ASSERT(pathPoints_.Num() != 0);
    ASSERT(index < pathPoints_.Num());
    return FVector2D(pathPoints_[index].Location.X, pathPoints_[index].Location.Y);
}

FVector2D Navigation::getCurrentPathPoint()
{
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
            std::cout << "######## Reached waypoint " << indexPath_ << " ########" << std::endl;
            indexPath_++;
        }
        else { // We reached the final target
            std::cout << "############ Reached the target location ! ############" << std::endl;
            targetReached_ = true;
        }
    }

    return getCurrentPathPoint();
}

