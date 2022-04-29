#include "Navigation.h"

#include "Assert.h"
#include "Config.h"

Navigation::Navigation()
{
    // agentPropertiesRef, vehicle_pawn

    // Initialize navigation variables:

    FVector2D relativePositionToTarget(0.0f, 0.0f);

    // Initialize navigation:
    navSys_ = FNavigationSystem::GetCurrent<UNavigationSystemV1>(vehicle_pawn->GetWorld());

    if (not navSys_)
    {
        ASSERT(false);
    }

    navData_ = navSys_->GetNavDataForProps(agentPropertiesRef);

    if (not navData_)
    {
        ASSERT(false);
    }

    indexPath_ = 1;

    // Set path generation query:
    FPathFindingQuery Query = FPathFindingQuery(*vehicle_pawn, *navData_, agent_location, targetLocation_.Location);
    // Set the path query such that case no path to the target can be found, a path that brings the agent as close as possible to the target can still be generated
    Query.SetAllowPartialPaths(true);
}

Navigation::~Navigation()
{
}

void Navigation::generateTrajectory()
{
    // Path generation polling to get "interesting" paths in every experiment:
    while (numIter < Config::getValue<int>({"SIMULATION_CONTROLLER", "NAVIGATION", "MAX_ITER_REPLAN"}))  // Try to generate interesting trajectories with multiple waypoints
    {
        // Get a random target point, to be reached by the agent:
        if (not navSys->GetRandomReachablePointInRadius(agent_location, Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "TARGET_RADIUS"}), targetLocation_))
        {
            ASSERT(false);
        }

        relativePositionToTarget.X = (targetLocation_.Location - agent_location).X;
        relativePositionToTarget.Y = (targetLocation_.Location - agent_location).Y;

        // Update navigation query with the new target:
        Query = FPathFindingQuery(*vehicle_pawn, *navData, agent_location, targetLocation_.Location);

        // Genrate a collision-free path between the robot position and the target point:
        FPathFindingResult collisionFreePath = navSys->FindPathSync(Query, EPathFindingMode::Type::Regular);

        // If path generation is sucessful, analyze the obtained path (it should not be too simple):
        if (collisionFreePath.IsSuccessful() and collisionFreePath.Path.IsValid())
        {
            if (collisionFreePath.IsPartial())
            {
                std::cout << "Only a partial path could be found by the planner..." << std::endl;
            }
            numberOfWayPoints = collisionFreePath.Path->GetPathPoints().Num();
            float crit = relativePositionToTarget.Size() * Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "PATH_WEIGHT_DIST"}) + numberOfWayPoints * relativePositionToTarget.Size() * Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "PATH_WEIGHT_NUM_WAYPOINTS"});

            if (pathCriterion <= crit)
            {
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

    std::cout << "Current position: [" << agent_location.X << ", " << agent_location.Y << ", " << agent_location.Z << "]." << std::endl;
    std::cout << "Reachable position: [" << bestTargetLocation.Location.X << ", " << bestTargetLocation.Location.Y << ", " << bestTargetLocation.Location.Z << "]." << std::endl;
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
