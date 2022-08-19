#include <ImitationLearningTask.h>

#include <algorithm>
#include <iostream>

#include <ActorHitEvent.h>
#include <Assert.h>
#include <Box.h>
#include <Config.h>

#include <EngineUtils.h>
#include <UObject/UObjectGlobals.h>

ImitationLearningTask::ImitationLearningTask(UWorld* world)
{
    std::vector<std::string> obstacle_ignore_actor_names = Config::getValue<std::vector<std::string>>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "OBSTACLE_IGNORE_ACTOR_NAMES"});

    for (TActorIterator<AActor> actor_itr(world, AActor::StaticClass()); actor_itr; ++actor_itr) {

        std::string actor_name = TCHAR_TO_UTF8(*(*actor_itr)->GetName());

        if (actor_name == Config::getValue<std::string>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "AGENT_ACTOR_NAME"})) {
            ASSERT(!agent_actor_);
            agent_actor_ = *actor_itr;
            ASSERT(agent_actor_);
        } else if (actor_name == Config::getValue<std::string>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "GOAL_ACTOR_NAME"}) and Config::getValue<std::string>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "GOAL_ACTOR_NAME"}) != "") {
            ASSERT(!goal_actor_);
            goal_actor_ = *actor_itr;
            ASSERT(goal_actor_);
        } else if (std::find(obstacle_ignore_actor_names.begin(), obstacle_ignore_actor_names.end(), actor_name) != obstacle_ignore_actor_names.end()) {
            obstacle_ignore_actors_.emplace_back(*actor_itr);
            std::cout << "actor_name: " << actor_name << std::endl;
        }
    }

    if (!goal_actor_ and Config::getValue<std::string>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "GOAL_ACTOR_NAME"}) == "") {
        ASSERT(!goal_actor_);
        FActorSpawnParameters goal_spawn_params;
        std::string goal_name = "GoalActor";
        goal_spawn_params.Name = FName(goal_name.c_str());
        goal_spawn_params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;
        goal_actor_ = world->SpawnActor<AActor>(AActor::StaticClass(), FVector::ZeroVector, FRotator::ZeroRotator, goal_spawn_params);
        USceneComponent* SceneComponent = NewObject<USceneComponent>(goal_actor_);
	    SceneComponent->SetMobility(EComponentMobility::Movable);
        goal_actor_->SetRootComponent(SceneComponent);
        ASSERT(goal_actor_);
    }

    // ASSERT(obstacle_ignore_actors_.size() == obstacle_ignore_actor_names.size());

    // Read config value for random stream initialization
    random_stream_.Initialize(Config::getValue<int>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "RANDOM_SEED"}));

    new_object_parent_actor_ = world->SpawnActor<AActor>();
    ASSERT(new_object_parent_actor_);

    // Create and initialize actor hit handler
    actor_hit_event_ = NewObject<UActorHitEvent>(new_object_parent_actor_, TEXT("ActorHitEvent"));
    ASSERT(actor_hit_event_);

    actor_hit_event_->RegisterComponent();
    actor_hit_event_->subscribeToActor(agent_actor_);
    actor_hit_event_delegate_handle_ = actor_hit_event_->delegate_.AddRaw(this, &ImitationLearningTask::actorHitEventHandler);

    // Agent Navigation:
    // Get a pointer to the navigation system
    nav_sys_ = FNavigationSystem::GetCurrent<UNavigationSystemV1>(world);
    ASSERT(nav_sys_ != nullptr);

    // Get a pointer to the agent's navigation data
    const INavAgentInterface* actor_as_nav_agent = CastChecked<INavAgentInterface>(agent_actor_);
    ASSERT(actor_as_nav_agent != nullptr);
    nav_data_ = nav_sys_->GetNavDataForProps(actor_as_nav_agent->GetNavAgentPropertiesRef(), actor_as_nav_agent->GetNavAgentLocation());
    ASSERT(nav_data_ != nullptr);

    // Get a pointer to the navigation mesh
    nav_mesh_ = Cast<ARecastNavMesh>(nav_data_);
    ASSERT(nav_mesh_ != nullptr);

    // Environment scaling factor
    world_to_meters_ = agent_actor_->GetWorld()->GetWorldSettings()->WorldToMeters;

    // Rebuild navigation mesh with the desired properties before executing trajectory planning
    buildNavMesh();
}

ImitationLearningTask::~ImitationLearningTask()
{
    ASSERT(actor_hit_event_);
    actor_hit_event_->delegate_.Remove(actor_hit_event_delegate_handle_);
    actor_hit_event_delegate_handle_.Reset();
    actor_hit_event_->unsubscribeFromActor(agent_actor_);
    actor_hit_event_->DestroyComponent();
    actor_hit_event_ = nullptr;

    ASSERT(new_object_parent_actor_);
    new_object_parent_actor_->Destroy();
    new_object_parent_actor_ = nullptr;

    random_stream_.Reset();

    ASSERT(goal_actor_);
    goal_actor_ = nullptr;

    ASSERT(agent_actor_);
    agent_actor_ = nullptr;

    obstacle_ignore_actors_.clear();
}

void ImitationLearningTask::beginFrame()
{
    // Reset hit states
    hit_goal_ = false;
    hit_obstacle_ = false;
}

void ImitationLearningTask::endFrame()
{
}

float ImitationLearningTask::getReward() const
{
    float reward;

    if (hit_goal_) {
        reward = Config::getValue<float>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "REWARD", "HIT_GOAL"});
    } else if (hit_obstacle_) {
        reward = Config::getValue<float>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "REWARD", "HIT_OBSTACLE"});
    } else {
        const FVector agent_to_goal = goal_actor_->GetActorLocation() - agent_actor_->GetActorLocation();
        reward = -agent_to_goal.Size() * Config::getValue<float>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "REWARD", "DISTANCE_TO_GOAL_SCALE"});
    }
    return reward;
}

bool ImitationLearningTask::isEpisodeDone() const
{
    return hit_goal_ or hit_obstacle_;
}

std::map<std::string, Box> ImitationLearningTask::getStepInfoSpace() const
{
    std::map<std::string, Box> step_info_space;
    Box box;

    box.low = 0;
    box.high = 1;
    box.shape = {1};
    box.dtype = DataType::Boolean;
    step_info_space["hit_goal"] = std::move(box);

    box.low = 0;
    box.high = 1;
    box.shape = {1};
    box.dtype = DataType::Boolean;
    step_info_space["hit_obstacle"] = std::move(box);

    return step_info_space;
}

std::map<std::string, std::vector<uint8_t>> ImitationLearningTask::getStepInfo() const
{
    std::map<std::string, std::vector<uint8_t>> step_info;

    step_info["hit_goal"] = std::vector<uint8_t>{hit_goal_};
    step_info["hit_obstacle"] = std::vector<uint8_t>{hit_obstacle_};

    return step_info;
}

void ImitationLearningTask::reset()
{
    ASSERT(nav_sys_ != nullptr);
    ASSERT(nav_data_ != nullptr);

    constexpr bool sweep = false;                    // Whether we sweep to the destination location, triggering overlaps along the way and stopping short of the target if blocked by something.
    constexpr FHitResult* hit_result_info = nullptr; // The hit result from the move if swept.

    if (Config::getValue<bool>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "RANDOM_SPAWN_TRAJ"})) {

        // Trajectory planning:
        getPositionsFromSamplingCandidateTrajectories();

        // Set agent and goal positions:
        agent_actor_->SetActorLocation(agent_initial_position_.at(0), sweep, hit_result_info, ETeleportType::ResetPhysics);
        goal_actor_->SetActorLocation(agent_goal_position_.at(0), sweep, hit_result_info, ETeleportType::ResetPhysics);
    } else {

        // Predefined initial position:
        getPositionsFromFile();

        // Trajectory planning:
        generateTrajectoryToTarget();

        // Set agent and goal positions:
        agent_actor_->SetActorLocation(agent_initial_position_.at(trajectory_index_), sweep, hit_result_info, ETeleportType::ResetPhysics);
        goal_actor_->SetActorLocation(agent_goal_position_.at(trajectory_index_), sweep, hit_result_info, ETeleportType::ResetPhysics);

        // Iterate in the list of initial/final positions:
        trajectory_index_++;
    }
}

bool ImitationLearningTask::isReady() const
{
    return true;
}

void ImitationLearningTask::actorHitEventHandler(AActor* self_actor, AActor* other_actor, FVector normal_impulse, const FHitResult& hit)
{
    ASSERT(self_actor == agent_actor_);

    if (other_actor == goal_actor_) {
        hit_goal_ = true;
    } else if (std::find(obstacle_ignore_actors_.begin(), obstacle_ignore_actors_.end(), other_actor) == obstacle_ignore_actors_.end()) {
        hit_obstacle_ = true;
    }
}

void ImitationLearningTask::buildNavMesh()
{
    ASSERT(nav_sys_ != nullptr);
    ASSERT(nav_mesh_ != nullptr);

    // Set the navigation mesh properties:
    nav_mesh_->AgentRadius = Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "NAVMESH", "AGENT_RADIUS"});
    nav_mesh_->AgentHeight = Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "NAVMESH", "AGENT_HEIGHT"});
    nav_mesh_->CellSize = Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "NAVMESH", "CELL_SIZE"});
    nav_mesh_->CellHeight = Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "NAVMESH", "CELL_HEIGHT"});
    nav_mesh_->AgentMaxSlope = Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "NAVMESH", "AGENT_MAX_SLOPE"});
    nav_mesh_->AgentMaxStepHeight = Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "NAVMESH", "AGENT_MAX_STEP_HEIGHT"});
    nav_mesh_->MergeRegionSize = Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "NAVMESH", "MERGE_REGION_SIZE"});
    nav_mesh_->MinRegionArea = Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "NAVMESH", "MIN_REGION_AREA"}); // ignore region that are too small
    nav_mesh_->MaxSimplificationError = Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "NAVMESH", "MAX_SIMPLIFINCATION_ERROR"});

    // Bounding box around the appartment (in the world coordinate system)
    FBox environment_bounds = getWorldBoundingBox();

    // Dynamic update navMesh location and size:
    ANavMeshBoundsVolume* nav_meshbounds_volume = nullptr;
    
    for (TActorIterator<ANavMeshBoundsVolume> it(agent_actor_->GetWorld()); it; ++it) {
        nav_meshbounds_volume = *it;
    }
    ASSERT(nav_meshbounds_volume != nullptr);

    nav_meshbounds_volume->GetRootComponent()->SetMobility(EComponentMobility::Movable); // Hack
    nav_meshbounds_volume->SetActorLocation(environment_bounds.GetCenter(), false);      // Place the navmesh at the center of the map
    std::cout << "environment_bounds: X: " << environment_bounds.GetCenter().X << ", Y: " << environment_bounds.GetCenter().Y << ", Z: " << environment_bounds.GetCenter().Z << std::endl;
    nav_meshbounds_volume->SetActorRelativeScale3D(environment_bounds.GetSize() / 200.f); // Rescale the navmesh so it matches the whole world
    std::cout << "environment_bounds.GetSize(): X: " << environment_bounds.GetSize().X << ", Y: " << environment_bounds.GetSize().Y << ", Z: " << environment_bounds.GetSize().Z << std::endl;
    nav_meshbounds_volume->GetRootComponent()->UpdateBounds();
    nav_sys_->OnNavigationBoundsUpdated(nav_meshbounds_volume);
    nav_meshbounds_volume->GetRootComponent()->SetMobility(EComponentMobility::Static);
    nav_sys_->Build(); // Rebuild NavMesh, required for update AgentRadius
}

void ImitationLearningTask::getPositionsFromFile()
{
    number_start_goal_pairs_ = 0;
    std::string line;
    std::string field;
    float value = 0.0f;
    int column_index = 0;
    FVector init, goal;

    // Create an input filestream
    std::ifstream trajectory_pairs_file(Config::getValue<std::string>({"SIMULATION_CONTROLLER", "NAVIGATION", "PATH_TRAJECTORY_PAIRS"}));
    ASSERT(trajectory_pairs_file.is_open());

    // Read file data, line by line in the format "init.X, init.Y, init.Z, goal.X, goal.Y, goal.Z"
    std::getline(trajectory_pairs_file, line); // The first line contains the names of the fields

    while (std::getline(trajectory_pairs_file, line)) {

        // Extract coordinates
        std::istringstream line_stream(line);
        while (std::getline(line_stream, field, ',')) {

            value = std::stof(field);

            switch (column_index) {
            case 0:
                init.X = value;
                break;

            case 1:
                init.Y = value;
                break;

            case 2:
                init.Z = value;
                break;

            case 3:
                goal.X = value;
                break;

            case 4:
                goal.Y = value;
                break;

            case 5:
                goal.Z = value;
                break;

            default:
                ASSERT(false);
                break;
            }

            // Iterate in the columns
            column_index++;
        }
        ASSERT(column_index == 6);
        number_start_goal_pairs_++;

        agent_initial_position_.push_back(init);
        agent_goal_position_.push_back(goal);
        column_index = 0;
    }

    // Close file
    trajectory_pairs_file.close();

    initial_point_generated_ = true;
    target_point_generated_ = true;
}

bool ImitationLearningTask::generateTrajectoryToTarget()
{
    bool status = false;
    int number_of_way_points = 0;
    FNavLocation target_location;
    FVector2D relative_position_to_target(0.0f, 0.0f);
    FPathFindingQuery nav_query;
    FPathFindingResult collision_free_path;
    path_points_.Empty();

    // Sanity checks
    ASSERT(nav_data_ != nullptr);
    ASSERT(nav_sys_ != nullptr);
    ASSERT(initial_point_generated_ and target_point_generated_);
    ASSERT(trajectory_index_ < number_start_goal_pairs_);

    // Update relative position between the agent and its new target:
    relative_position_to_target.X = (agent_goal_position_.at(trajectory_index_) - agent_initial_position_.at(trajectory_index_)).X;
    relative_position_to_target.Y = (agent_goal_position_.at(trajectory_index_) - agent_initial_position_.at(trajectory_index_)).Y;

    // Update navigation query with the new target:
    nav_query = FPathFindingQuery(agent_actor_, *nav_data_, agent_initial_position_.at(trajectory_index_), agent_goal_position_.at(trajectory_index_));

    // Genrate a collision-free path between the robot position and the target point:
    collision_free_path = nav_sys_->FindPathSync(nav_query, EPathFindingMode::Type::Regular);

    // If path generation is sucessful, analyze the obtained path (it should not be too simple):
    if (collision_free_path.IsSuccessful() and collision_free_path.Path.IsValid()) {

        if (collision_free_path.IsPartial()) {
            std::cout << "Only a partial path could be found by the planner..." << std::endl;
        }

        number_of_way_points = collision_free_path.Path->GetPathPoints().Num();

        path_points_ = collision_free_path.Path->GetPathPoints();

        std::cout << "Number of way points: " << number_of_way_points << std::endl;
        std::cout << "Target distance: " << relative_position_to_target.Size() / world_to_meters_ << "m" << std::endl;

        trajectory_length_ = 0.0;

        for (size_t i = 0; i < number_of_way_points - 1; i++) {
            trajectory_length_ += FVector::Dist(path_points_[i].Location, path_points_[i + 1].Location);
        }

        // Scaling to meters
        trajectory_length_ /= world_to_meters_;

        // Update status
        status = true;

        std::cout << "Path length " << trajectory_length_ << "m" << std::endl;
    }

    ASSERT(path_points_.Num() > 1);

    std::cout << "Initial position: [" << agent_initial_position_.at(trajectory_index_).X << ", " << agent_initial_position_.at(trajectory_index_).Y << ", " << agent_initial_position_.at(trajectory_index_).Z << "]." << std::endl;
    std::cout << "Reachable position: [" << agent_goal_position_.at(trajectory_index_).X << ", " << agent_goal_position_.at(trajectory_index_).Y << ", " << agent_goal_position_.at(trajectory_index_).Z << "]." << std::endl;
    std::cout << "-----------------------------------------------------------" << std::endl;
    std::cout << "Way points: " << std::endl;

    for (auto wayPoint : path_points_) {
        std::cout << "[" << wayPoint.Location.X << ", " << wayPoint.Location.Y << ", " << wayPoint.Location.Z << "]" << std::endl;
    }

    std::cout << "-----------------------------------------------------------" << std::endl;

    return status;
}

bool ImitationLearningTask::getPositionsFromSamplingCandidateTrajectories()
{
    agent_initial_position_.resize(1);
    agent_goal_position_.resize(1);
    number_start_goal_pairs_ = 1;
    bool status = false;
    int number_iterations = 0;
    int number_of_way_points = 0;
    float path_criterion = 0.0f;
    float best_path_criterion = 0.0f;
    FNavLocation best_target_location;
    FNavLocation best_init_location;
    FNavLocation target_location;
    FNavLocation init_location;
    FVector2D relative_position_to_target(0.0f, 0.0f);
    FPathFindingQuery nav_query;
    FPathFindingResult collision_free_path;
    path_points_.Empty();

    // Sanity checks
    ASSERT(nav_data_ != nullptr);
    ASSERT(nav_sys_ != nullptr);

    // Path generation polling to get "interesting" paths in every experiment:
    while (number_iterations < Config::getValue<int>({"SIMULATION_CONTROLLER", "NAVIGATION", "MAX_ITER_REPLAN"})) // Try to generate interesting trajectories with multiple waypoints
    {
        // Get a random initial point:
        ASSERT(nav_sys_->GetRandomPoint(init_location, nav_data_) == true);

        // Get a random reachable target point, to be reached by the agent from init_location.Location:
        ASSERT(nav_sys_->GetRandomReachablePointInRadius(init_location.Location, Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "TARGET_RADIUS"}), target_location));

        // Update relative position between the agent and its new target:
        relative_position_to_target.X = (target_location.Location - init_location.Location).X;
        relative_position_to_target.Y = (target_location.Location - init_location.Location).Y;

        // Update navigation query with the new target:
        nav_query = FPathFindingQuery(agent_actor_, *nav_data_, init_location.Location, target_location.Location);

        // Genrate a collision-free path between the robot position and the target point:
        collision_free_path = nav_sys_->FindPathSync(nav_query, EPathFindingMode::Type::Regular);

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
                path_points_.Empty();
                path_points_ = collision_free_path.Path->GetPathPoints();
                std::cout << "Iteration: " << number_iterations << std::endl;
                std::cout << "Cost: " << best_path_criterion << std::endl;
                std::cout << "Number of way points: " << number_of_way_points << std::endl;
                std::cout << "Target distance: " << relative_position_to_target.Size() / world_to_meters_ << "m" << std::endl;

                trajectory_length_ = 0.0;
                for (size_t i = 0; i < number_of_way_points - 1; i++) {
                    trajectory_length_ += FVector::Dist(path_points_[i].Location, path_points_[i + 1].Location);
                }
                std::cout << "Path length " << trajectory_length_ / world_to_meters_ << "m" << std::endl;
            }

            number_iterations++;

            if (number_iterations % 1000 == 0) {
                std::cout << ".";
            }

            status = true;
        }
    }
    std::cout << "" << std::endl;

    ASSERT(path_points_.Num() > 1);

    // Update the goal position
    agent_initial_position_.at(0) = best_init_location.Location;
    initial_point_generated_ = true;

    agent_goal_position_.at(0) = best_target_location.Location;
    target_point_generated_ = true;

    // Scaling to meters
    trajectory_length_ /= world_to_meters_;

    std::cout << "Initial position: [" << agent_initial_position_.at(0).X << ", " << agent_initial_position_.at(0).Y << ", " << agent_initial_position_.at(0).Z << "]." << std::endl;
    std::cout << "Reachable position: [" << agent_goal_position_.at(0).X << ", " << agent_goal_position_.at(0).Y << ", " << agent_goal_position_.at(0).Z << "]." << std::endl;
    std::cout << "-----------------------------------------------------------" << std::endl;
    std::cout << "Way points: " << std::endl;

    for (auto wayPoint : path_points_) {
        std::cout << "[" << wayPoint.Location.X << ", " << wayPoint.Location.Y << ", " << wayPoint.Location.Z << "]" << std::endl;
    }

    std::cout << "-----------------------------------------------------------" << std::endl;

    return status;
}

FBox ImitationLearningTask::getWorldBoundingBox(bool scale_ceiling)
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