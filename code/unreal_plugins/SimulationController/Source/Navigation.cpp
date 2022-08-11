#include <Navigation.h>

FVector Navigation::generateRandomNavigablePoint(AActor* agent_actor)
{
    // Get a pointer to the agent's navigation system
    UNavigationSystemV1* nav_sys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(agent_actor->GetWorld());
    ASSERT(nav_sys != nullptr);

    // Get a pointer to the agent's navigation data
    const INavAgentInterface* actor_as_nav_agent = CastChecked<INavAgentInterface>(agent_actor);
    ASSERT(actor_as_nav_agent != nullptr);
    ANavigationData* nav_data = nav_sys->GetNavDataForProps(actor_as_nav_agent->GetNavAgentPropertiesRef(), actor_as_nav_agent->GetNavAgentLocation());
    ASSERT(nav_data != nullptr);

    // Generate a random point in navigable space
    FNavLocation result_location;
    ASSERT(nav_sys->GetRandomPoint(result_location, nav_data) == true);
    return result_location.Location;
}

FVector Navigation::generateRandomReachableTargetPoint(AActor* agent_actor)
{
    // Get a pointer to the agent's navigation system
    UNavigationSystemV1* nav_sys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(agent_actor->GetWorld());
    ASSERT(nav_sys != nullptr);

    // Finds a random target point, reachable by the agent from initial_point
    FNavLocation target_location;
    ASSERT(nav_sys->GetRandomReachablePointInRadius(agent_actor->GetActorLocation(), Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "TARGET_RADIUS"}), target_location));

    return target_location.Location;
}

FVector Navigation::generateRandomReachableTargetPoint(AActor* agent_actor, float radius)
{
    // Get a pointer to the agent's navigation system
    UNavigationSystemV1* nav_sys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(agent_actor->GetWorld());
    ASSERT(nav_sys != nullptr);

    // Finds a random target point, reachable by the agent from initial_point
    FNavLocation target_location;
    ASSERT(nav_sys->GetRandomReachablePointInRadius(agent_actor->GetActorLocation(), radius, target_location));

    return target_location.Location;
}

bool Navigation::generateTrajectoryToTarget(AActor* agent_actor, const FVector& initial_point, const FVector& target_point, TArray<FNavPathPoint>& path_points)
{
    bool status = false;
    int number_of_way_points = 0;
    float trajectory_length = 0.0;
    FNavLocation target_location;
    FVector2D relative_position_to_target(0.0f, 0.0f);
    FPathFindingQuery nav_query;
    FPathFindingResult collision_free_path;
    path_points.Empty();

    // Get a pointer to the agent's navigation system
    UNavigationSystemV1* nav_sys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(agent_actor->GetWorld());
    ASSERT(nav_sys != nullptr);

    // Get a pointer to the agent's navigation data
    const INavAgentInterface* actor_as_nav_agent = CastChecked<INavAgentInterface>(agent_actor);
    ASSERT(actor_as_nav_agent != nullptr);
    ANavigationData* nav_data = nav_sys->GetNavDataForProps(actor_as_nav_agent->GetNavAgentPropertiesRef(), actor_as_nav_agent->GetNavAgentLocation());
    ASSERT(nav_data != nullptr);

    // Update relative position between the agent and its new target:
    relative_position_to_target.X = (target_point - initial_point).X;
    relative_position_to_target.Y = (target_point - initial_point).Y;

    // Update navigation query with the new target:
    nav_query = FPathFindingQuery(agent_actor, *nav_data, initial_point, target_point);

    // Genrate a collision-free path between the robot position and the target point:
    collision_free_path = nav_sys->FindPathSync(nav_query, EPathFindingMode::Type::Regular);

    // If path generation is sucessful, analyze the obtained path (it should not be too simple):
    if (collision_free_path.IsSuccessful() and collision_free_path.Path.IsValid()) {

        if (collision_free_path.IsPartial()) {
            std::cout << "Only a partial path could be found by the planner..." << std::endl;
        }

        number_of_way_points = collision_free_path.Path->GetPathPoints().Num();

        path_points = collision_free_path.Path->GetPathPoints();

        std::cout << "Number of way points: " << number_of_way_points << std::endl;
        std::cout << "Target distance: " << relative_position_to_target.Size() / agent_actor->GetWorld()->GetWorldSettings()->WorldToMeters << "m" << std::endl;

        trajectory_length = 0.0;
        for (size_t i = 0; i < number_of_way_points - 1; i++) {
            trajectory_length += FVector::Dist(path_points[i].Location, path_points[i + 1].Location);
        }

        // Scaling to meters
        trajectory_length /= agent_actor->GetWorld()->GetWorldSettings()->WorldToMeters;

        // Update status
        status = true;

        std::cout << "Path length " << trajectory_length << "m" << std::endl;
    }

    ASSERT(path_points.Num() > 1);

    std::cout << "Initial position: [" << initial_point.X << ", " << initial_point.Y << ", " << initial_point.Z << "]." << std::endl;
    std::cout << "Reachable position: [" << target_point.X << ", " << target_point.Y << ", " << target_point.Z << "]." << std::endl;
    std::cout << "-----------------------------------------------------------" << std::endl;
    std::cout << "Way points: " << std::endl;
    for (auto wayPoint : path_points) {
        std::cout << "[" << wayPoint.Location.X << ", " << wayPoint.Location.Y << ", " << wayPoint.Location.Z << "]" << std::endl;
    }
    std::cout << "-----------------------------------------------------------" << std::endl;

    return status;
}

bool Navigation::generateTrajectoryToRandomTarget(AActor* agent_actor, const FVector& initial_point, TArray<FNavPathPoint>& path_points)
{
    bool status = false;
    int number_of_way_points = 0;
    float trajectory_length = 0.0;
    FNavLocation target_location;
    FVector2D relative_position_to_target(0.0f, 0.0f);
    FPathFindingQuery nav_query;
    FPathFindingResult collision_free_path;
    path_points.Empty();

    // Get a pointer to the agent's navigation system
    UNavigationSystemV1* nav_sys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(agent_actor->GetWorld());
    ASSERT(nav_sys != nullptr);

    // Get a pointer to the agent's navigation data
    const INavAgentInterface* actor_as_nav_agent = CastChecked<INavAgentInterface>(agent_actor);
    ASSERT(actor_as_nav_agent != nullptr);
    ANavigationData* nav_data = nav_sys->GetNavDataForProps(actor_as_nav_agent->GetNavAgentPropertiesRef(), actor_as_nav_agent->GetNavAgentLocation());
    ASSERT(nav_data != nullptr);

    // Get a random reachable target point, to be reached by the agent from initial_point:
    ASSERT(nav_sys->GetRandomReachablePointInRadius(initial_point, Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "TARGET_RADIUS"}), target_location));

    // Update relative position between the agent and its new target:
    relative_position_to_target.X = (target_location.Location - initial_point).X;
    relative_position_to_target.Y = (target_location.Location - initial_point).Y;

    // Update navigation query with the new target:
    nav_query = FPathFindingQuery(agent_actor, *nav_data, initial_point, target_location.Location);

    // Genrate a collision-free path between the robot position and the target point:
    collision_free_path = nav_sys->FindPathSync(nav_query, EPathFindingMode::Type::Regular);

    // If path generation is sucessful, analyze the obtained path (it should not be too simple):
    if (collision_free_path.IsSuccessful() and collision_free_path.Path.IsValid()) {

        if (collision_free_path.IsPartial()) {
            std::cout << "Only a partial path could be found by the planner..." << std::endl;
        }

        number_of_way_points = collision_free_path.Path->GetPathPoints().Num();

        path_points = collision_free_path.Path->GetPathPoints();

        std::cout << "Number of way points: " << number_of_way_points << std::endl;
        std::cout << "Target distance: " << relative_position_to_target.Size() / agent_actor->GetWorld()->GetWorldSettings()->WorldToMeters << "m" << std::endl;

        trajectory_length = 0.0;
        for (size_t i = 0; i < number_of_way_points - 1; i++) {
            trajectory_length += FVector::Dist(path_points[i].Location, path_points[i + 1].Location);
        }

        // Scaling to meters
        trajectory_length /= agent_actor->GetWorld()->GetWorldSettings()->WorldToMeters;

        // Update status
        status = true;

        std::cout << "Path length " << trajectory_length << "m" << std::endl;
    }

    ASSERT(path_points.Num() > 1);

    std::cout << "Initial position: [" << initial_point.X << ", " << initial_point.Y << ", " << initial_point.Z << "]." << std::endl;
    std::cout << "Reachable position: [" << target_location.Location.X << ", " << target_location.Location.Y << ", " << target_location.Location.Z << "]." << std::endl;
    std::cout << "-----------------------------------------------------------" << std::endl;
    std::cout << "Way points: " << std::endl;
    for (auto wayPoint : path_points) {
        std::cout << "[" << wayPoint.Location.X << ", " << wayPoint.Location.Y << ", " << wayPoint.Location.Z << "]" << std::endl;
    }
    std::cout << "-----------------------------------------------------------" << std::endl;

    return status;
}

bool Navigation::generateRandomTrajectory(AActor* agent_actor, TArray<FNavPathPoint>& path_points)
{
    bool status = false;
    int number_of_way_points = 0;
    float trajectory_length = 0.0;
    FNavLocation init_location;
    FNavLocation target_location;
    FVector2D relative_position_to_target(0.0f, 0.0f);
    FPathFindingQuery nav_query;
    FPathFindingResult collision_free_path;
    path_points.Empty();

    // Get a pointer to the agent's navigation system
    UNavigationSystemV1* nav_sys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(agent_actor->GetWorld());
    ASSERT(nav_sys != nullptr);

    // Get a pointer to the agent's navigation data
    const INavAgentInterface* actor_as_nav_agent = CastChecked<INavAgentInterface>(agent_actor);
    ASSERT(actor_as_nav_agent != nullptr);
    ANavigationData* nav_data = nav_sys->GetNavDataForProps(actor_as_nav_agent->GetNavAgentPropertiesRef(), actor_as_nav_agent->GetNavAgentLocation());
    ASSERT(nav_data != nullptr);

    // Get a random initial point:
    ASSERT(nav_sys->GetRandomPoint(init_location, nav_data) == true);

    // Get a random reachable target point, to be reached by the agent from init_location.Location:
    ASSERT(nav_sys->GetRandomReachablePointInRadius(init_location.Location, Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "TARGET_RADIUS"}), target_location));

    // Update relative position between the agent and its new target:
    relative_position_to_target.X = (target_location.Location - init_location.Location).X;
    relative_position_to_target.Y = (target_location.Location - init_location.Location).Y;

    // Update navigation query with the new target:
    nav_query = FPathFindingQuery(agent_actor, *nav_data, init_location.Location, target_location.Location);

    // Genrate a collision-free path between the robot position and the target point:
    collision_free_path = nav_sys->FindPathSync(nav_query, EPathFindingMode::Type::Regular);

    // If path generation is sucessful, analyze the obtained path (it should not be too simple):
    if (collision_free_path.IsSuccessful() and collision_free_path.Path.IsValid()) {

        if (collision_free_path.IsPartial()) {
            std::cout << "Only a partial path could be found by the planner..." << std::endl;
        }

        number_of_way_points = collision_free_path.Path->GetPathPoints().Num();

        path_points = collision_free_path.Path->GetPathPoints();

        std::cout << "Number of way points: " << number_of_way_points << std::endl;
        std::cout << "Target distance: " << relative_position_to_target.Size() / agent_actor->GetWorld()->GetWorldSettings()->WorldToMeters << "m" << std::endl;

        trajectory_length = 0.0;
        for (size_t i = 0; i < number_of_way_points - 1; i++) {
            trajectory_length += FVector::Dist(path_points[i].Location, path_points[i + 1].Location);
        }

        // Scaling to meters
        trajectory_length /= agent_actor->GetWorld()->GetWorldSettings()->WorldToMeters;

        // Update status
        status = true;

        std::cout << "Path length " << trajectory_length << "m" << std::endl;
    }

    ASSERT(path_points.Num() > 1);

    std::cout << "Initial position: [" << init_location.Location.X << ", " << init_location.Location.Y << ", " << init_location.Location.Z << "]." << std::endl;
    std::cout << "Reachable position: [" << target_location.Location.X << ", " << target_location.Location.Y << ", " << target_location.Location.Z << "]." << std::endl;
    std::cout << "-----------------------------------------------------------" << std::endl;
    std::cout << "Way points: " << std::endl;
    for (auto wayPoint : path_points) {
        std::cout << "[" << wayPoint.Location.X << ", " << wayPoint.Location.Y << ", " << wayPoint.Location.Z << "]" << std::endl;
    }
    std::cout << "-----------------------------------------------------------" << std::endl;

    return status;
}

bool Navigation::sampleTrajectoryToRandomTarget(AActor* agent_actor, const FVector& initial_point, TArray<FNavPathPoint>& path_points)
{
    bool status = false;
    int number_iterations = 0;
    int number_of_way_points = 0;
    float trajectory_length = 0.0;
    float path_criterion = 0.0f;
    float best_path_criterion = 0.0f;
    FNavLocation best_target_location;
    FNavLocation target_location;
    FVector2D relative_position_to_target(0.0f, 0.0f);
    FPathFindingQuery nav_query;
    FPathFindingResult collision_free_path;
    path_points.Empty();

    // Get a pointer to the agent's navigation system
    UNavigationSystemV1* nav_sys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(agent_actor->GetWorld());
    ASSERT(nav_sys != nullptr);

    // Get a pointer to the agent's navigation data
    const INavAgentInterface* actor_as_nav_agent = CastChecked<INavAgentInterface>(agent_actor);
    ASSERT(actor_as_nav_agent != nullptr);
    ANavigationData* nav_data = nav_sys->GetNavDataForProps(actor_as_nav_agent->GetNavAgentPropertiesRef(), actor_as_nav_agent->GetNavAgentLocation());
    ASSERT(nav_data != nullptr);

    // Path generation polling to get "interesting" paths in every experiment:
    while (number_iterations < Config::getValue<int>({"SIMULATION_CONTROLLER", "NAVIGATION", "MAX_ITER_REPLAN"})) // Try to generate interesting trajectories with multiple waypoints
    {
        // Get a random reachable target point, to be reached by the agent from initial_point:
        ASSERT(nav_sys->GetRandomReachablePointInRadius(initial_point, Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "TARGET_RADIUS"}), target_location));

        // Update relative position between the agent and its new target:
        relative_position_to_target.X = (target_location.Location - initial_point).X;
        relative_position_to_target.Y = (target_location.Location - initial_point).Y;

        // Update navigation query with the new target:
        nav_query = FPathFindingQuery(agent_actor, *nav_data, initial_point, target_location.Location);

        // Genrate a collision-free path between the robot position and the target point:
        collision_free_path = nav_sys->FindPathSync(nav_query, EPathFindingMode::Type::Regular);

        // If path generation is sucessful, make sure that it is not too simple for better training results:
        if (collision_free_path.IsSuccessful() and collision_free_path.Path.IsValid()) {

            // Partial paths are supported. In this case, the agent is expected to move as close as possible to its target
            if (collision_free_path.IsPartial()) {
                std::cout << "Only a partial path could be found by the planner..." << std::endl;
            }

            // Compute a path metric to evaluate its complexity and determine if it can be used for training purposes
            number_of_way_points = collision_free_path.Path->GetPathPoints().Num();
            path_criterion = relative_position_to_target.Size() * Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "PATH_WEIGHT_DIST"}) + number_of_way_points * relative_position_to_target.Size() * Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "PATH_WEIGHT_NUM_WAYPOINTS"});

            if (best_path_criterion <= path_criterion) {
                best_path_criterion = path_criterion;
                best_target_location = target_location;
                path_points.Empty();
                path_points = collision_free_path.Path->GetPathPoints();
                std::cout << "Iteration: " << number_iterations << std::endl;
                std::cout << "Cost: " << best_path_criterion << std::endl;
                std::cout << "Number of way points: " << number_of_way_points << std::endl;
                std::cout << "Target distance: " << relative_position_to_target.Size() / agent_actor->GetWorld()->GetWorldSettings()->WorldToMeters << "m" << std::endl;

                trajectory_length = 0.0;
                for (size_t i = 0; i < number_of_way_points - 1; i++) {
                    trajectory_length += FVector::Dist(path_points[i].Location, path_points[i + 1].Location);
                }
                std::cout << "Path length " << trajectory_length / agent_actor->GetWorld()->GetWorldSettings()->WorldToMeters << "m" << std::endl;
            }
            number_iterations++;
            status = true;
        }
    }

    ASSERT(path_points.Num() > 1);

    // Scaling to meters
    trajectory_length /= agent_actor->GetWorld()->GetWorldSettings()->WorldToMeters;

    std::cout << "Initial position: [" << initial_point.X << ", " << initial_point.Y << ", " << initial_point.Z << "]." << std::endl;
    std::cout << "Reachable position: [" << best_target_location.Location.X << ", " << best_target_location.Location.Y << ", " << best_target_location.Location.Z << "]." << std::endl;
    std::cout << "-----------------------------------------------------------" << std::endl;
    std::cout << "Way points: " << std::endl;
    for (auto wayPoint : path_points) {
        std::cout << "[" << wayPoint.Location.X << ", " << wayPoint.Location.Y << ", " << wayPoint.Location.Z << "]" << std::endl;
    }
    std::cout << "-----------------------------------------------------------" << std::endl;

    return status;
}

bool Navigation::sampleRandomTrajectory(AActor* agent_actor, TArray<FNavPathPoint>& path_points)
{
    bool status = false;
    int number_iterations = 0;
    int number_of_way_points = 0;
    float trajectory_length = 0.0;
    float path_criterion = 0.0f;
    float best_path_criterion = 0.0f;
    FNavLocation best_target_location;
    FNavLocation best_init_location;
    FNavLocation target_location;
    FNavLocation init_location;
    FVector2D relative_position_to_target(0.0f, 0.0f);
    FPathFindingQuery nav_query;
    FPathFindingResult collision_free_path;
    path_points.Empty();

    // Get a pointer to the agent's navigation system
    UNavigationSystemV1* nav_sys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(agent_actor->GetWorld());
    ASSERT(nav_sys != nullptr);

    // Get a pointer to the agent's navigation data
    const INavAgentInterface* actor_as_nav_agent = CastChecked<INavAgentInterface>(agent_actor);
    ASSERT(actor_as_nav_agent != nullptr);
    ANavigationData* nav_data = nav_sys->GetNavDataForProps(actor_as_nav_agent->GetNavAgentPropertiesRef(), actor_as_nav_agent->GetNavAgentLocation());
    ASSERT(nav_data != nullptr);

    // Path generation polling to get "interesting" paths in every experiment:
    while (number_iterations < Config::getValue<int>({"SIMULATION_CONTROLLER", "NAVIGATION", "MAX_ITER_REPLAN"})) // Try to generate interesting trajectories with multiple waypoints
    {
        // Get a random initial point:
        ASSERT(nav_sys->GetRandomPoint(init_location, nav_data) == true);

        // Get a random reachable target point, to be reached by the agent from init_location.Location:
        ASSERT(nav_sys->GetRandomReachablePointInRadius(init_location.Location, Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "TARGET_RADIUS"}), target_location));

        // Update relative position between the agent and its new target:
        relative_position_to_target.X = (target_location.Location - init_location.Location).X;
        relative_position_to_target.Y = (target_location.Location - init_location.Location).Y;

        // Update navigation query with the new target:
        nav_query = FPathFindingQuery(agent_actor, *nav_data, init_location.Location, target_location.Location);

        // Genrate a collision-free path between the robot position and the target point:
        collision_free_path = nav_sys->FindPathSync(nav_query, EPathFindingMode::Type::Regular);

        // If path generation is sucessful, make sure that it is not too simple for better training results:
        if (collision_free_path.IsSuccessful() and collision_free_path.Path.IsValid()) {

            // Partial paths are supported. In this case, the agent is expected to move as close as possible to its target
            if (collision_free_path.IsPartial()) {
                std::cout << "Only a partial path could be found by the planner..." << std::endl;
            }

            // Compute a path metric to evaluate its complexity and determine if it can be used for training purposes
            number_of_way_points = collision_free_path.Path->GetPathPoints().Num();
            path_criterion = relative_position_to_target.Size() * Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "PATH_WEIGHT_DIST"}) + number_of_way_points * relative_position_to_target.Size() * Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "PATH_WEIGHT_NUM_WAYPOINTS"});

            if (best_path_criterion <= path_criterion) {
                best_path_criterion = path_criterion;
                best_target_location = target_location;
                best_init_location = init_location;
                path_points.Empty();
                path_points = collision_free_path.Path->GetPathPoints();
                std::cout << "Iteration: " << number_iterations << std::endl;
                std::cout << "Cost: " << best_path_criterion << std::endl;
                std::cout << "Number of way points: " << number_of_way_points << std::endl;
                std::cout << "Target distance: " << relative_position_to_target.Size() / agent_actor->GetWorld()->GetWorldSettings()->WorldToMeters << "m" << std::endl;

                trajectory_length = 0.0;
                for (size_t i = 0; i < number_of_way_points - 1; i++) {
                    trajectory_length += FVector::Dist(path_points[i].Location, path_points[i + 1].Location);
                }
                std::cout << "Path length " << trajectory_length / agent_actor->GetWorld()->GetWorldSettings()->WorldToMeters << "m" << std::endl;
            }
            number_iterations++;
            status = true;
        }
    }

    ASSERT(path_points.Num() > 1);

    // Scaling to meters
    trajectory_length /= agent_actor->GetWorld()->GetWorldSettings()->WorldToMeters;

    std::cout << "Initial position: [" << best_init_location.Location.X << ", " << best_init_location.Location.Y << ", " << best_init_location.Location.Z << "]." << std::endl;
    std::cout << "Reachable position: [" << best_target_location.Location.X << ", " << best_target_location.Location.Y << ", " << best_target_location.Location.Z << "]." << std::endl;
    std::cout << "-----------------------------------------------------------" << std::endl;
    std::cout << "Way points: " << std::endl;
    for (auto wayPoint : path_points) {
        std::cout << "[" << wayPoint.Location.X << ", " << wayPoint.Location.Y << ", " << wayPoint.Location.Z << "]" << std::endl;
    }
    std::cout << "-----------------------------------------------------------" << std::endl;

    return status;
}

bool Navigation::rebuildNavmesh(AActor* agent_actor)
{
    // Get a pointer to the agent's navigation system
    UNavigationSystemV1* nav_sys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(agent_actor->GetWorld());
    ASSERT(nav_sys != nullptr);

    // Get a pointer to the agent's navigation data
    const INavAgentInterface* actor_as_nav_agent = CastChecked<INavAgentInterface>(agent_actor);
    ASSERT(actor_as_nav_agent != nullptr);
    ANavigationData* nav_data = nav_sys->GetNavDataForProps(actor_as_nav_agent->GetNavAgentPropertiesRef(), actor_as_nav_agent->GetNavAgentLocation());
    ASSERT(nav_data != nullptr);

    // Get a pointer to the navigation mesh
    ARecastNavMesh* nav_mesh = Cast<ARecastNavMesh>(nav_data);
    ASSERT(nav_mesh != nullptr);

    // Set the navigation mesh properties:
    nav_mesh->AgentRadius = Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "NAVMESH", "AGENT_RADIUS"});
    nav_mesh->AgentHeight = Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "NAVMESH", "AGENT_HEIGHT"});
    nav_mesh->CellSize = Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "NAVMESH", "CELL_SIZE"});
    nav_mesh->CellHeight = Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "NAVMESH", "CELL_HEIGHT"});
    nav_mesh->AgentMaxSlope = Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "NAVMESH", "AGENT_MAX_SLOPE"});
    nav_mesh->AgentMaxStepHeight = Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "NAVMESH", "AGENT_MAX_STEP_HEIGHT"});
    nav_mesh->MergeRegionSize = Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "NAVMESH", "MERGE_REGION_SIZE"});
    nav_mesh->MinRegionArea = Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "NAVMESH", "MIN_REGION_AREA"}); // ignore region that are too small
    nav_mesh->MaxSimplificationError = Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "NAVMESH", "MAX_SIMPLIFINCATION_ERROR"});

    // Bounding box around the appartment (in the world coordinate system)
    FBox environment_bounds = getWorldBoundingBox();

    // Dynamic update navMesh location and size:
    ANavMeshBoundsVolume* nav_meshbounds_volume = nullptr;
    for (TActorIterator<ANavMeshBoundsVolume> it(agent_actor->GetWorld()); it; ++it) {
        nav_meshbounds_volume = *it;
    }
    ASSERT(nav_meshbounds_volume != nullptr);

    nav_meshbounds_volume->GetRootComponent()->SetMobility(EComponentMobility::Movable);  // Hack
    nav_meshbounds_volume->SetActorLocation(environment_bounds.GetCenter(), false);       // Place the navmesh at the center of the map
    nav_meshbounds_volume->SetActorRelativeScale3D(environment_bounds.GetSize() / 200.f); // Rescale the navmesh so it matches the whole world
    nav_meshbounds_volume->GetRootComponent()->UpdateBounds();
    nav_sys->OnNavigationBoundsUpdated(nav_meshbounds_volume);
    nav_meshbounds_volume->GetRootComponent()->SetMobility(EComponentMobility::Static);

    nav_sys->Build(); // Rebuild NavMesh, required for update AgentRadius

    return true;
}

void Navigation::traceGround(AActor* agent_actor, FVector& spawn_position, FRotator& spawn_rotator, const FVector& box_half_size)
{
    FVector startLoc = spawn_position + FVector(0, 0, 100);
    FVector endLoc = spawn_position + FVector(0, 0, -1000);

    FCollisionQueryParams collisionParams(FName(TEXT("trace2ground")), true, agent_actor);
    FHitResult hit(ForceInit);

    if (UKismetSystemLibrary::BoxTraceSingle(agent_actor->GetWorld(), startLoc, endLoc, box_half_size, spawn_rotator, ETraceTypeQuery::TraceTypeQuery1, false, TArray<AActor*>(), EDrawDebugTrace::Type::ForDuration, hit, true)) {
        spawn_position = hit.Location;
    }
}

FBox Navigation::getWorldBoundingBox(AActor* agent_actor, bool scale_ceiling)
{
    FBox box(ForceInit);
    for (TActorIterator<AActor> it(agent_actor->GetWorld()); it; ++it) {
        if (it->ActorHasTag("architecture") || it->ActorHasTag("furniture")) {
            box += it->GetComponentsBoundingBox(false, true);
        }
    }
    // Remove ceiling
    return !scale_ceiling ? box : box.ExpandBy(box.GetSize() * 0.1f).ShiftBy(FVector(0, 0, -0.3f * box.GetSize().Z));
}

float Navigation::computeTrajectoryLength(AActor* agent_actor, const TArray<FNavPathPoint>& path_points)
{
    ASSERT(path_points.Num() > 1);
    
    float trajectory_length = 0.0;
    for (size_t i = 0; i < number_of_way_points - 1; i++) {
        trajectory_length += FVector::Dist(path_points[i].Location, path_points[i + 1].Location);
    }

    // Scaling to meters
    trajectory_length /= agent_actor->GetWorld()->GetWorldSettings()->WorldToMeters;

    return trajectory_length;
}