#include <Navigation.h>

FVector Navigation::generateRandomNavigablePoint(AActor* agent_actor)
{
    // Get a pointer to the agent's navigation system
    UNavigationSystemV1* nav_sys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(agent_actor->GetWorld());
    ASSERT(nav_sys != nullptr);

    // Get a pointer to the agent's navigation data
    const INavAgentInterface* actor_as_nav_agent = CastChecked<INavAgentInterface>(agent_actor);
    ASSERT(actor_as_nav_agent != nullptr);
    ANavigationData* nav_data = nav_sys_->GetNavDataForProps(actor_as_nav_agent->GetNavAgentPropertiesRef(), actor_as_nav_agent->GetNavAgentLocation());
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

    // Finds a random target point, reachable by the agent from agent_initial_position_
    FNavLocation target_location;
    ASSERT(nav_sys->GetRandomReachablePointInRadius(agent_actor->GetActorLocation(), Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "TARGET_RADIUS"}), target_location));
    
    return target_location.Location;
}

FVector Navigation::generateRandomReachableTargetPoint(AActor* agent_actor, float radius)
{
    // Get a pointer to the agent's navigation system
    UNavigationSystemV1* nav_sys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(agent_actor->GetWorld());
    ASSERT(nav_sys != nullptr);

    // Finds a random target point, reachable by the agent from agent_initial_position_
    FNavLocation target_location;
    ASSERT(nav_sys->GetRandomReachablePointInRadius(agent_actor->GetActorLocation(), radius, target_location));
    
    return target_location.Location;
}


bool Navigation::generateTrajectoryToTarget(AActor* agent_actor, const FVector &initial_point, const FVector &target_point, TArray<FNavPathPoint> &path_points)
{
    // Get a pointer to the agent's navigation system
    UNavigationSystemV1* nav_sys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(agent_actor->GetWorld());
    ASSERT(nav_sys != nullptr);

    // Get a pointer to the agent's navigation data
    const INavAgentInterface* actor_as_nav_agent = CastChecked<INavAgentInterface>(agent_actor);
    ASSERT(actor_as_nav_agent != nullptr);
    ANavigationData* nav_data = nav_sys_->GetNavDataForProps(actor_as_nav_agent->GetNavAgentPropertiesRef(), actor_as_nav_agent->GetNavAgentLocation());
    ASSERT(nav_data != nullptr);




    return true;
}

bool Navigation::generateTrajectoryToRandomTarget(AActor* agent_actor, const FVector &initial_point, TArray<FNavPathPoint> &path_points)
{
    // Get a pointer to the agent's navigation system
    UNavigationSystemV1* nav_sys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(agent_actor->GetWorld());
    ASSERT(nav_sys != nullptr);

    // Get a pointer to the agent's navigation data
    const INavAgentInterface* actor_as_nav_agent = CastChecked<INavAgentInterface>(agent_actor);
    ASSERT(actor_as_nav_agent != nullptr);
    ANavigationData* nav_data = nav_sys_->GetNavDataForProps(actor_as_nav_agent->GetNavAgentPropertiesRef(), actor_as_nav_agent->GetNavAgentLocation());
    ASSERT(nav_data != nullptr);



    return true;
}

bool Navigation::generateRandomTrajectory(AActor* agent_actor, TArray<FNavPathPoint> &path_points)
{
    // Get a pointer to the agent's navigation system
    UNavigationSystemV1* nav_sys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(agent_actor->GetWorld());
    ASSERT(nav_sys != nullptr);

    // Get a pointer to the agent's navigation data
    const INavAgentInterface* actor_as_nav_agent = CastChecked<INavAgentInterface>(agent_actor);
    ASSERT(actor_as_nav_agent != nullptr);
    ANavigationData* nav_data = nav_sys_->GetNavDataForProps(actor_as_nav_agent->GetNavAgentPropertiesRef(), actor_as_nav_agent->GetNavAgentLocation());
    ASSERT(nav_data != nullptr);



    return true;
}

bool Navigation::sampleTrajectoryToRandomTarget(AActor* agent_actor, const FVector &initial_point, TArray<FNavPathPoint> &path_points)
{
    // Get a pointer to the agent's navigation system
    UNavigationSystemV1* nav_sys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(agent_actor->GetWorld());
    ASSERT(nav_sys != nullptr);

    // Get a pointer to the agent's navigation data
    const INavAgentInterface* actor_as_nav_agent = CastChecked<INavAgentInterface>(agent_actor);
    ASSERT(actor_as_nav_agent != nullptr);
    ANavigationData* nav_data = nav_sys_->GetNavDataForProps(actor_as_nav_agent->GetNavAgentPropertiesRef(), actor_as_nav_agent->GetNavAgentLocation());
    ASSERT(nav_data != nullptr);



    return true;
}

bool Navigation::sampleRandomTrajectory(AActor* agent_actor, TArray<FNavPathPoint> &path_points)
{
    // Get a pointer to the agent's navigation system
    UNavigationSystemV1* nav_sys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(agent_actor->GetWorld());
    ASSERT(nav_sys != nullptr);

    // Get a pointer to the agent's navigation data
    const INavAgentInterface* actor_as_nav_agent = CastChecked<INavAgentInterface>(agent_actor);
    ASSERT(actor_as_nav_agent != nullptr);
    ANavigationData* nav_data = nav_sys_->GetNavDataForProps(actor_as_nav_agent->GetNavAgentPropertiesRef(), actor_as_nav_agent->GetNavAgentLocation());
    ASSERT(nav_data != nullptr);



    return true;
}







Navigation::Navigation(AActor* agent_actor)
{
    // Initialize navigation:
    agent_actor_ = agent_actor;
    rebuild();

    Navigation::initial_position_ = agent_actor_->GetActorLocation(); // Initial position of the agent

    nav_query_ = FPathFindingQuery(agent_actor_, *nav_data_, Navigation::initial_position_, FVector(0.0f, 0.0f, 0.0f));

    // Set the path query such that case no path to the target can be found, a path that brings the agent as close as possible to the target can still be generated
    nav_query_.SetAllowPartialPaths(true);
}

Navigation::~Navigation()
{
    // ASSERT(nav_sys_);
    // nav_sys_ = nullptr;

    // ASSERT(nav_data_);
    // nav_data_ = nullptr;

    // ASSERT(nav_mesh);
    // nav_mesh = nullptr;

    // ASSERT(agent_actor_);
    // agent_actor_ = nullptr;
}

void Navigation::reset()
{
    // rebuild();

    // Navigation::initial_position_ = agent_actor_->GetActorLocation(); // Initial position of the agent

    // nav_query_ = FPathFindingQuery(*agent_actor_, *nav_data_, Navigation::initial_position_, FVector(0.0f, 0.0f, 0.0f));

    // // Set the path query such that case no path to the target can be found, a path that brings the agent as close as possible to the target can still be generated
    // nav_query_.SetAllowPartialPaths(true);
}

bool Navigation::rebuildNavmesh(UNavigationSystemV1* nav_sys, ARecastNavMesh* nav_mesh)
{
    ASSERT(nav_sys != nullptr);
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
    for (TActorIterator<ANavMeshBoundsVolume> it(agent_actor_->GetWorld()); it; ++it) {
        nav_meshbounds_volume = *it;
    }
    ASSERT(nav_meshbounds_volume != nullptr);

    nav_meshbounds_volume->GetRootComponent()->SetMobility(EComponentMobility::Movable); // Hack
    nav_meshbounds_volume->SetActorLocation(environment_bounds.GetCenter(), false); // Place the navmesh at the center of the map
    nav_meshbounds_volume->SetActorRelativeScale3D(environment_bounds.GetSize() / 200.f); // Rescale the navmesh so it matches the whole world
    nav_meshbounds_volume->GetRootComponent()->UpdateBounds();
    nav_sys->OnNavigationBoundsUpdated(nav_meshbounds_volume);
    nav_meshbounds_volume->GetRootComponent()->SetMobility(EComponentMobility::Static);
    
    nav_sys->Build(); // Rebuild NavMesh, required for update AgentRadius
    
    return true;
}



FVector2D Navigation::update(const FVector &agent_current_location, const FVector &target_current_location)
{
    //const FVector agent_current_location = agent_actor_->GetActorLocation();
    //FVector2D relative_position_to_goal = getCurrentPathPoint() - FVector2D(agent_current_location.X, agent_current_location.Y);

    FVector2D relative_position_to_goal = FVector2D(target_current_location.X, target_current_location.Y); - FVector2D(agent_current_location.X, agent_current_location.Y);

    Navigation::target_reached_ = false;

    // If a waypoint is reached
    if ((relative_position_to_goal.Size() * 0.01) < Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "ACCEPTANCE_RADIUS"})) {

        if (index_path_ < path_points_.Num() - 1) { // Move to the next waypoint
            std::cout << "######## Reached waypoint " << index_path_ << " over " << path_points_.Num() - 1 << " ########" << std::endl;
            index_path_++;
        }
        else { // We reached the final target
            std::cout << "############ Reached the target location ! ############" << std::endl;
            Navigation::target_reached_ = true;
        }
    }

    return getCurrentPathPoint();
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