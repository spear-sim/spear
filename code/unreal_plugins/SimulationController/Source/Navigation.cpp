#include <Navigation.h>

Navigation::Navigation(AActor* agent_actor) : agent_actor_(agent_actor)
{
    std::cout << "############################  " << __FILE__ << " --> Line: " << __LINE__ << "  ############################" << std::endl;

    // Initialize navigation:
    nav_sys_ = nullptr;
    nav_data_ = nullptr;
    nav_mesh_ = nullptr;

    rebuild();

    initial_position_ = agent_actor_->GetActorLocation(); // Initial position of the agent

    nav_query_ = FPathFindingQuery(agent_actor_, *nav_data_, initial_position_, FVector(0.0f, 0.0f, 0.0f));

    // Set the path query such that case no path to the target can be found, a path that brings the agent as close as possible to the target can still be generated
    nav_query_.SetAllowPartialPaths(true);

    // std::string id = Config::getValue<std::string>({"INTERIORSIM", "MAP_ID"});
    //
    // if (id == "/Game/Maps/Map_237081640"){
    //     execution_counter_ = 0;
    // }
    // else if (id == "/Game/Maps/Map_237081739"){
    //     execution_counter_ = 100;
    // }
    // else if (id == "/Game/Maps/Map_239748000"){
    //     execution_counter_ = 200;
    // }
    // else if (id == "/Game/Maps/Map_239784016"){
    //     execution_counter_ = 300;
    // }
    // else if (id == "/Game/Maps/Map_239784069"){
    //     execution_counter_ = 400;
    // }
    // else {
    //         ASSERT(false);
    // }
}

Navigation::~Navigation()
{
}

void Navigation::reset()
{
    // rebuild();

    // initial_position_ = agent_actor_->GetActorLocation(); // Initial position of the agent

    // nav_query_ = FPathFindingQuery(*agent_actor_, *nav_data_, initial_position_, FVector(0.0f, 0.0f, 0.0f));

    // // Set the path query such that case no path to the target can be found, a path that brings the agent as close as possible to the target can still be generated
    // nav_query_.SetAllowPartialPaths(true);
}

FVector Navigation::generateRandomInitialPosition()
{
    // Finds random point in navigable space (true if any location found, false otherwise):
    FNavLocation result_location;
    ASSERT(nav_sys_->GetRandomPoint(result_location, nav_data_) == true);

    return result_location.Location;
}

void Navigation::setInitialPosition(const FVector& initial_position)
{
    initial_position_ = initial_position;
}

void Navigation::setGoalPosition(const FVector& goal_position)
{
    target_location_.Location = goal_position;
}


void Navigation::generateTrajectoryToRandomTarget()
{
    int number_iterations = 0;
    int number_of_way_points = 0;
    float path_criterion = 0.0f;
    float best_path_criterion = 0.0f;
    FNavLocation best_target_location;
    FVector2D relative_position_to_target(0.0f, 0.0f);

    // Path generation polling to get "interesting" paths in every experiment:
    float trajLength = 0.0;
    while (number_iterations < Config::getValue<int>({"SIMULATION_CONTROLLER", "NAVIGATION", "MAX_ITER_REPLAN"})) // Try to generate interesting trajectories with multiple waypoints
    {
        // Get a random target point, to be reached by the agent:
        ASSERT(nav_sys_->GetRandomReachablePointInRadius(initial_position_, Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "TARGET_RADIUS"}), target_location_));

        // Update relative position between the agent and its new target:
        relative_position_to_target.X = (target_location_.Location - initial_position_).X;
        relative_position_to_target.Y = (target_location_.Location - initial_position_).Y;

        // Update navigation query with the new target:
        nav_query_ = FPathFindingQuery(agent_actor_, *nav_data_, initial_position_, target_location_.Location);

        // Genrate a collision-free path between the robot position and the target point:
        FPathFindingResult collision_free_path = nav_sys_->FindPathSync(nav_query_, EPathFindingMode::Type::Regular);

        // If path generation is sucessful, analyze the obtained path (it should not be too simple):
        if (collision_free_path.IsSuccessful() and collision_free_path.Path.IsValid()) {

            if (collision_free_path.IsPartial()) {
                std::cout << "Only a partial path could be found by the planner..." << std::endl;
            }

            number_of_way_points = collision_free_path.Path->GetPathPoints().Num();
            path_criterion = relative_position_to_target.Size() * Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "PATH_WEIGHT_DIST"}) + number_of_way_points * relative_position_to_target.Size() * Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "PATH_WEIGHT_NUM_WAYPOINTS"});

            if (best_path_criterion <= path_criterion) {
                best_path_criterion = path_criterion;
                best_target_location = target_location_;
                path_points_.Empty();
                path_points_ = collision_free_path.Path->GetPathPoints();
                std::cout << "Iteration: " << number_iterations << std::endl;
                std::cout << "Cost: " << best_path_criterion << std::endl;
                std::cout << "Number of way points: " << number_of_way_points << std::endl;
                std::cout << "Target distance: " << relative_position_to_target.Size() * 0.01 << "m" << std::endl;

                trajLength = 0.0;
                for (size_t i = 0; i < number_of_way_points - 1; i++) {
                    trajLength += FVector::Dist(path_points_[i].Location, path_points_[i + 1].Location);
                }
                std::cout << "Path length " << trajLength * 0.01 << "m" << std::endl;
            }
            number_iterations++;
        }
    }

    ASSERT(path_points_.Num() > 1);

    target_location_ = best_target_location;

    trajectory_length_ = trajLength * 0.01;

    std::cout << "Initial position: [" << initial_position_.X << ", " << initial_position_.Y << ", " << initial_position_.Z << "]." << std::endl;
    std::cout << "Reachable position: [" << best_target_location.Location.X << ", " << best_target_location.Location.Y << ", " << best_target_location.Location.Z << "]." << std::endl;
    std::cout << "-----------------------------------------------------------" << std::endl;
    std::cout << "Way points: " << std::endl;
    for (auto wayPoint : path_points_) {
        std::cout << "[" << wayPoint.Location.X << ", " << wayPoint.Location.Y << ", " << wayPoint.Location.Z << "]" << std::endl;
    }
    std::cout << "-----------------------------------------------------------" << std::endl;
    index_path_ = 1; // Path point 0 is the initial robot position. getCurrentPathPoint() should therefore return the next point.
}

void Navigation::generateTrajectoryToPredefinedTarget()
{
    int number_iterations = 0;
    int number_of_way_points = 0;
    float path_criterion = 0.0f;
    float best_path_criterion = 0.0f;
    FNavLocation best_target_location;
    FVector2D relative_position_to_target(0.0f, 0.0f);

    // DIRTY HACK for neurips:
    FVector pos; // = setPredefinedGoalPosition();

    // Path generation polling to get "interesting" paths in every experiment:
    float trajLength = 0.0;
    while (number_iterations < Config::getValue<int>({"SIMULATION_CONTROLLER", "NAVIGATION", "MAX_ITER_REPLAN"})) // Try to generate interesting trajectories with multiple waypoints
    {
        // Update relative position between the agent and its new target:
        relative_position_to_target.X = (target_location_.Location - initial_position_).X;
        relative_position_to_target.Y = (target_location_.Location - initial_position_).Y;

        // Update navigation query with the new target:
        nav_query_ = FPathFindingQuery(agent_actor_, *nav_data_, initial_position_, target_location_.Location);

        // Genrate a collision-free path between the robot position and the target point:
        FPathFindingResult collision_free_path = nav_sys_->FindPathSync(nav_query_, EPathFindingMode::Type::Regular);

        // If path generation is sucessful, analyze the obtained path (it should not be too simple):
        if (collision_free_path.IsSuccessful() and collision_free_path.Path.IsValid()) {

            if (collision_free_path.IsPartial()) {
                std::cout << "Only a partial path could be found by the planner..." << std::endl;
            }

            number_of_way_points = collision_free_path.Path->GetPathPoints().Num();
            path_criterion = relative_position_to_target.Size() * Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "PATH_WEIGHT_DIST"}) + number_of_way_points * relative_position_to_target.Size() * Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "PATH_WEIGHT_NUM_WAYPOINTS"});

            if (best_path_criterion <= path_criterion) {
                best_path_criterion = path_criterion;
                best_target_location = target_location_;
                path_points_.Empty();
                path_points_ = collision_free_path.Path->GetPathPoints();
                std::cout << "Iteration: " << number_iterations << std::endl;
                std::cout << "Cost: " << best_path_criterion << std::endl;
                std::cout << "Number of way points: " << number_of_way_points << std::endl;
                std::cout << "Target distance: " << relative_position_to_target.Size() * 0.01 << "m" << std::endl;

                trajLength = 0.0;
                for (size_t i = 0; i < number_of_way_points - 1; i++) {
                    trajLength += FVector::Dist(path_points_[i].Location, path_points_[i + 1].Location);
                }
                std::cout << "Path length " << trajLength * 0.01 << "m" << std::endl;
            }
        }
        number_iterations++;
    }

    ASSERT(path_points_.Num() > 1);

    target_location_ = best_target_location;

    trajectory_length_ = trajLength * 0.01;

    std::cout << "Initial position: [" << initial_position_.X << ", " << initial_position_.Y << ", " << initial_position_.Z << "]." << std::endl;
    std::cout << "Reachable position: [" << target_location_.Location.X << ", " << target_location_.Location.Y << ", " << target_location_.Location.Z << "]." << std::endl;
    std::cout << "-----------------------------------------------------------" << std::endl;
    std::cout << "Way points: " << std::endl;
    for (auto wayPoint : path_points_) {
        std::cout << "[" << wayPoint.Location.X << ", " << wayPoint.Location.Y << ", " << wayPoint.Location.Z << "]" << std::endl;
    }
    std::cout << "-----------------------------------------------------------" << std::endl;
    index_path_ = 1; // Path point 0 is the initial robot position. getCurrentPathPoint() should therefore return the next point.
    execution_counter_++;
}

FVector2D Navigation::getPathPoint(size_t index)
{
    ASSERT(path_points_.Num() != 0);
    ASSERT(index < path_points_.Num());
    return FVector2D(path_points_[index].Location.X, path_points_[index].Location.Y);
}

FVector2D Navigation::getCurrentPathPoint()
{
    ASSERT(path_points_.Num() != 0);
    ASSERT(index_path_ < path_points_.Num());
    return FVector2D(path_points_[index_path_].Location.X, path_points_[index_path_].Location.Y);
}

FVector2D Navigation::update()
{
    const FVector agent_current_location = agent_actor_->GetActorLocation();
    FVector2D relative_position_to_goal = getCurrentPathPoint() - FVector2D(agent_current_location.X, agent_current_location.Y);

    target_reached_ = false;

    // If a waypoint is reached
    if ((relative_position_to_goal.Size() * 0.01) < Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "ACCEPTANCE_RADIUS"})) {

        if (index_path_ < path_points_.Num() - 1) { // Move to the next waypoint
            std::cout << "######## Reached waypoint " << index_path_ << " over " << path_points_.Num() - 1 << " ########" << std::endl;
            index_path_++;
        }
        else { // We reached the final target
            std::cout << "############ Reached the target location ! ############" << std::endl;
            target_reached_ = true;
        }
    }

    return getCurrentPathPoint();
}

bool Navigation::rebuild()
{
    nav_sys_ = FNavigationSystem::GetCurrent<UNavigationSystemV1>(agent_actor_->GetWorld());
    ASSERT(nav_sys_ != nullptr);

    const INavAgentInterface* actor_as_nav_agent = CastChecked<INavAgentInterface>(agent_actor_);
    ASSERT(actor_as_nav_agent != nullptr);

    // FNavigationSystem::ECreateIfEmpty CreateNewIfNoneFound;
    // nav_data_ = nav_sys_->GetMainNavData(CreateNewIfNoneFound);
    nav_data_ = nav_sys_->GetNavDataForProps(actor_as_nav_agent->GetNavAgentPropertiesRef(), actor_as_nav_agent->GetNavAgentLocation());
    ASSERT(nav_data_ != nullptr);

    nav_mesh_ = Cast<ARecastNavMesh>(nav_data_);
    ASSERT(nav_mesh_ != nullptr);

    // Set the NavMesh properties:
    nav_mesh_->AgentRadius = Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "NAVMESH", "AGENT_RADIUS"});
    nav_mesh_->AgentHeight = Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "NAVMESH", "AGENT_HEIGHT"});
    nav_mesh_->CellSize = Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "NAVMESH", "CELL_SIZE"});
    nav_mesh_->CellHeight = Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "NAVMESH", "CELL_HEIGHT"});
    nav_mesh_->AgentMaxSlope = Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "NAVMESH", "AGENT_MAX_SLOPE"});
    nav_mesh_->AgentMaxStepHeight = Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "NAVMESH", "AGENT_MAX_STEP_HEIGHT"});
    nav_mesh_->MergeRegionSize = Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "NAVMESH", "MERGE_REGION_SIZE"});
    nav_mesh_->MinRegionArea = Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "NAVMESH", "MIN_REGION_AREA"}); // ignore region that are too small
    nav_mesh_->MaxSimplificationError = Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "NAVMESH", "MAX_SIMPLIFINCATION_ERROR"});

    // Dynamic update navMesh location and size:
    FBox environment_bounds = getWorldBoundingBox(); // Bounding box around the appartment (in the world coordinate system).
    std::cout << "############################  " << __FILE__ << " --> Line: " << __LINE__ << "  ############################" << std::endl;

    ANavMeshBoundsVolume* nav_mesh_bounds_volume = nullptr;
    for (TActorIterator<ANavMeshBoundsVolume> it(agent_actor_->GetWorld()); it; ++it) {
        nav_mesh_bounds_volume = *it;
    }
    ASSERT(nav_mesh_bounds_volume != nullptr);

    std::cout << "############################  " << __FILE__ << " --> Line: " << __LINE__ << "  ############################" << std::endl;

    nav_mesh_bounds_volume->GetRootComponent()->UpdateBounds();
    std::cout << "############################  " << __FILE__ << " --> Line: " << __LINE__ << "  ############################" << std::endl;

    nav_mesh_bounds_volume->SetActorLocation(environment_bounds.GetCenter(), false); // Place the navmesh at the center of the map
    std::cout << "############################  " << __FILE__ << " --> Line: " << __LINE__ << "  ############################" << std::endl;

    nav_mesh_bounds_volume->SetActorRelativeScale3D(environment_bounds.GetSize() / 200.f); // Rescale the navmesh so it matches the whole world
    std::cout << "############################  " << __FILE__ << " --> Line: " << __LINE__ << "  ############################" << std::endl;

    nav_mesh_bounds_volume->GetRootComponent()->UpdateBounds();
    std::cout << "############################  " << __FILE__ << " --> Line: " << __LINE__ << "  ############################" << std::endl;

    if (Config::getValue<bool>({"SIMULATION_CONTROLLER", "NAVIGATION", "NAVMESH", "USE_STATIC_NAVMESH"})) {
        nav_mesh_bounds_volume->GetRootComponent()->SetMobility(EComponentMobility::Static);
    }
    else { // A dynmic navmesh will account for changes occuring in the environment at runtime. But this is more computationally intensive...
        nav_mesh_bounds_volume->GetRootComponent()->SetMobility(EComponentMobility::Movable);
    }
    std::cout << "############################  " << __FILE__ << " --> Line: " << __LINE__ << "  ############################" << std::endl;

    nav_sys_->OnNavigationBoundsUpdated(nav_mesh_bounds_volume);
    nav_sys_->Build(); // Rebuild NavMesh, required for update AgentRadius
    std::cout << "############################  " << __FILE__ << " --> Line: " << __LINE__ << "  ############################" << std::endl;

    return true;
}

void Navigation::traceGround(FVector& spawn_position, FRotator& spawn_rotator, const FVector& box_half_size)
{
    FVector startLoc = spawn_position + FVector(0, 0, 100);
    FVector endLoc = spawn_position + FVector(0, 0, -1000);

    FCollisionQueryParams collisionParams(FName(TEXT("trace2ground")), true, agent_actor_);
    FHitResult hit(ForceInit);

    if (UKismetSystemLibrary::BoxTraceSingle(agent_actor_->GetWorld(), startLoc, endLoc, box_half_size, spawn_rotator, ETraceTypeQuery::TraceTypeQuery1, false, TArray<AActor*>(), EDrawDebugTrace::Type::ForDuration, hit, true)) {
        spawn_position = hit.Location;
    }
}

FBox Navigation::getWorldBoundingBox(bool scale_ceiling)
{
    FBox box(ForceInit);
    for (TActorIterator<AActor> it(agent_actor_->GetWorld()); it; ++it) {
        if (it->ActorHasTag("architecture") || it->ActorHasTag("furniture")) {
            box += it->GetComponentsBoundingBox(false, true);
        }
    }
    // Remove ceiling
    return !scale_ceiling ? box : box.ExpandBy(box.GetSize() * 0.1f).ShiftBy(FVector(0, 0, -0.3f * box.GetSize().Z));
}
