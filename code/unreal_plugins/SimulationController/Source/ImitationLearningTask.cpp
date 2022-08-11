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
        }
        else if (actor_name == Config::getValue<std::string>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "GOAL_ACTOR_NAME"}) and Config::getValue<std::string>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "GOAL_ACTOR_NAME"}) != "") {
            ASSERT(!goal_actor_);
            goal_actor_ = *actor_itr;
            ;
            ASSERT(goal_actor_);
        }
        else if (std::find(obstacle_ignore_actor_names.begin(), obstacle_ignore_actor_names.end(), actor_name) != obstacle_ignore_actor_names.end()) {
            obstacle_ignore_actors_.emplace_back(*actor_itr);
            std::cout << "actor_name: " << actor_name << std::endl;
        }
    }

    if (Config::getValue<std::string>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "GOAL_ACTOR_NAME"}) == "") {
        ASSERT(!goal_actor_);
        goal_actor_ = world->SpawnActor<AActor>();
        ASSERT(goal_actor_);
    }

    // ASSERT(obstacle_ignore_actors_.size() == obstacle_ignore_actor_names.size());

    // read config value for random stream initialization
    random_stream_.Initialize(Config::getValue<int>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "RANDOM_SEED"}));

    new_object_parent_actor_ = world->SpawnActor<AActor>();
    ASSERT(new_object_parent_actor_);

    // create and initialize actor hit handler
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
    // reset hit states
    hit_goal_ = false;
    hit_obstacle_ = false;
}

void ImitationLearningTask::endFrame()
{
    APawn* vehicle_pawn = dynamic_cast<APawn*>(agent_actor_);
    ASSERT(vehicle_pawn);

    // TODO: unclean...
    if (goalReached()) {
        hit_goal_ = true;
    }
}

float ImitationLearningTask::getReward() const
{
    float reward;

    if (hit_goal_) {
        reward = Config::getValue<float>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "REWARD", "HIT_GOAL"});
    }
    else if (hit_obstacle_) {
        reward = Config::getValue<float>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "REWARD", "HIT_OBSTACLE"});
    }
    else {
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
    FVector agent_initial_position(0), agent_goal_position(0);
    
    if (Config::getValue<bool>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "RANDOM_SPAWN_TRAJ"})) {

        // Random initial position:
        FNavLocation agent_location;
        ASSERT(nav_sys_->GetRandomPoint(agent_location, nav_data_) == true);
        agent_initial_position = agent_location.Location;

        // Trajectory planning:
        generateTrajectoryToRandomTarget();
    }
    else {
        // Predefined initial position:
        agent_actor_->SetActorLocation(agent_position);

        // Trajectory planning:
        generateTrajectoryToTarget(goal_position);
    }

    agent_actor_->SetActorLocation(agent_initial_position);
    goal_actor_->SetActorLocation(agent_goal_position);
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
    }
    else if (std::find(obstacle_ignore_actors_.begin(), obstacle_ignore_actors_.end(), other_actor) == obstacle_ignore_actors_.end()) {
        hit_obstacle_ = true;
    }
}

void ImitationLearningTask::updateInitialPositionFromParameterFile()
{
    // TODO: import from csv
    initial_point_generated_ = true;
}

void ImitationLearningTask::updateTargetPositionFromParameterFile()
{
    // TODO: import from csv
    target_point_generated_ = true;
}

FVector ImitationLearningTask::generateRandomNavigablePoint()
{
    // Sanity checks
    ASSERT(nav_data_ != nullptr);
    ASSERT(nav_sys_ != nullptr);

    // Generate a random point in navigable space
    FNavLocation result_location;
    ASSERT(nav_sys_->GetRandomPoint(result_location, nav_data_) == true);
    return result_location.Location;
}

void ImitationLearningTask::generateRandomReachableTargetPoint()
{
    // Sanity check
    ASSERT(nav_sys_ != nullptr);

    // Finds a random target point, reachable by the agent from agent_initial_position_:
    FNavLocation target_location;
    ASSERT(nav_sys_->GetRandomReachablePointInRadius(agent_initial_position_, Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "TARGET_RADIUS"}), target_location));    
    agent_goal_position_ = target_location.Location;
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

    // Update relative position between the agent and its new target:
    relative_position_to_target.X = (agent_goal_position_ - agent_initial_position_).X;
    relative_position_to_target.Y = (agent_goal_position_ - agent_initial_position_).Y;

    // Update navigation query with the new target:
    nav_query = FPathFindingQuery(agent_actor_, *nav_data_, agent_initial_position_, agent_goal_position_);

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

    std::cout << "Initial position: [" << agent_initial_position_.X << ", " << agent_initial_position_.Y << ", " << agent_initial_position_.Z << "]." << std::endl;
    std::cout << "Reachable position: [" << agent_goal_position_.X << ", " << agent_goal_position_.Y << ", " << agent_goal_position_.Z << "]." << std::endl;
    std::cout << "-----------------------------------------------------------" << std::endl;
    std::cout << "Way points: " << std::endl;
    for (auto wayPoint : path_points_) {
        std::cout << "[" << wayPoint.Location.X << ", " << wayPoint.Location.Y << ", " << wayPoint.Location.Z << "]" << std::endl;
    }
    std::cout << "-----------------------------------------------------------" << std::endl;

    return status;
}

bool ImitationLearningTask::generateTrajectoryToRandomTarget()
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

    // Get a random reachable target point, to be reached by the agent from agent_initial_position_:
    ASSERT(initial_point_generated_);
    ASSERT(nav_sys_->GetRandomReachablePointInRadius(agent_initial_position_, Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "TARGET_RADIUS"}), target_location));
    agent_goal_position_ = target_location.Location;
    target_point_generated_ = true;

    // Update relative position between the agent and its new target:
    relative_position_to_target.X = (agent_goal_position_ - agent_initial_position_).X;
    relative_position_to_target.Y = (agent_goal_position_ - agent_initial_position_).Y;

    // Update navigation query with the new target:
    nav_query = FPathFindingQuery(agent_actor_, *nav_data_, agent_initial_position_, agent_goal_position_);

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

    std::cout << "Initial position: [" << agent_initial_position_.X << ", " << agent_initial_position_.Y << ", " << agent_initial_position_.Z << "]." << std::endl;
    std::cout << "Reachable position: [" << agent_goal_position_.X << ", " << agent_goal_position_.Y << ", " << agent_goal_position_.Z << "]." << std::endl;
    std::cout << "-----------------------------------------------------------" << std::endl;
    std::cout << "Way points: " << std::endl;
    for (auto wayPoint : path_points_) {
        std::cout << "[" << wayPoint.Location.X << ", " << wayPoint.Location.Y << ", " << wayPoint.Location.Z << "]" << std::endl;
    }
    std::cout << "-----------------------------------------------------------" << std::endl;

    return status;
}

bool ImitationLearningTask::generateRandomTrajectory()
{
    bool status = false;
    int number_of_way_points = 0;
    FNavLocation init_location;
    FNavLocation target_location;
    FVector2D relative_position_to_target(0.0f, 0.0f);
    FPathFindingQuery nav_query;
    FPathFindingResult collision_free_path;
    path_points_.Empty();

    // Sanity checks
    ASSERT(nav_data_ != nullptr);
    ASSERT(nav_sys_ != nullptr);

    // Get a random initial point:
    ASSERT(nav_sys_->GetRandomPoint(init_location, nav_data_) == true);
    agent_initial_position_ = init_location.Location;
    initial_point_generated_ = true;

    // Get a random reachable target point, to be reached by the agent from agent_initial_position_:
    ASSERT(nav_sys_->GetRandomReachablePointInRadius(agent_initial_position_, Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "TARGET_RADIUS"}), target_location));
    agent_goal_position_ = target_location.Location;
    target_point_generated_ = true;

    // Update relative position between the agent and its new target:
    relative_position_to_target.X = (agent_goal_position_ - agent_initial_position_).X;
    relative_position_to_target.Y = (agent_goal_position_ - agent_initial_position_).Y;

    // Update navigation query with the new target:
    nav_query = FPathFindingQuery(agent_actor_, *nav_data_, agent_initial_position_, agent_goal_position_);

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

    std::cout << "Initial position: [" << agent_initial_position_.X << ", " << agent_initial_position_.Y << ", " << agent_initial_position_.Z << "]." << std::endl;
    std::cout << "Reachable position: [" << agent_goal_position_.X << ", " << agent_goal_position_.Y << ", " << agent_goal_position_.Z << "]." << std::endl;
    std::cout << "-----------------------------------------------------------" << std::endl;
    std::cout << "Way points: " << std::endl;
    for (auto wayPoint : path_points_) {
        std::cout << "[" << wayPoint.Location.X << ", " << wayPoint.Location.Y << ", " << wayPoint.Location.Z << "]" << std::endl;
    }
    std::cout << "-----------------------------------------------------------" << std::endl;

    return status;
}

bool ImitationLearningTask::sampleTrajectoryToRandomTarget()
{
    bool status = false;
    int number_iterations = 0;
    int number_of_way_points = 0;
    float path_criterion = 0.0f;
    float best_path_criterion = 0.0f;
    FNavLocation best_target_location;
    FNavLocation target_location;
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
        // Get a random reachable target point, to be reached by the agent from agent_initial_position_:
        ASSERT(nav_sys_->GetRandomReachablePointInRadius(agent_initial_position_, Config::getValue<float>({"SIMULATION_CONTROLLER", "NAVIGATION", "TARGET_RADIUS"}), target_location));

        // Update relative position between the agent and its new target:
        relative_position_to_target.X = (target_location.Location - agent_initial_position_).X;
        relative_position_to_target.Y = (target_location.Location - agent_initial_position_).Y;

        // Update navigation query with the new target:
        nav_query = FPathFindingQuery(agent_actor_, *nav_data_, agent_initial_position_, target_location.Location);

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
                std::cout << "Path length " << trajectory_length_/world_to_meters_ << "m" << std::endl;
            }
            number_iterations++;
            status = true;
        }
    }

    ASSERT(path_points_.Num() > 1);

    // Update the goal position
    agent_goal_position_ = best_target_location.Location;
    target_point_generated_ = true;

    // Scaling to meters
    trajectory_length_ /= world_to_meters_;

    std::cout << "Initial position: [" << agent_initial_position_.X << ", " << agent_initial_position_.Y << ", " << agent_initial_position_.Z << "]." << std::endl;
    std::cout << "Reachable position: [" << agent_goal_position_.X << ", " << agent_goal_position_.Y << ", " << agent_goal_position_.Z << "]." << std::endl;
    std::cout << "-----------------------------------------------------------" << std::endl;
    std::cout << "Way points: " << std::endl;
    for (auto wayPoint : path_points_) {
        std::cout << "[" << wayPoint.Location.X << ", " << wayPoint.Location.Y << ", " << wayPoint.Location.Z << "]" << std::endl;
    }
    std::cout << "-----------------------------------------------------------" << std::endl;

    return status;
}

bool ImitationLearningTask::sampleRandomTrajectory()
{
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
                std::cout << "Path length " << trajectory_length_/world_to_meters_ << "m" << std::endl;
            }
            number_iterations++;
            status = true;
        }
    }

    ASSERT(path_points_.Num() > 1);

    // Update the goal position
    agent_initial_position_ = best_init_location.Location;
    initial_point_generated_ = true;

    agent_goal_position_ = best_target_location.Location;
    target_point_generated_ = true;

    // Scaling to meters
    trajectory_length_ /= world_to_meters_;

    std::cout << "Initial position: [" << agent_initial_position_.X << ", " << agent_initial_position_.Y << ", " << agent_initial_position_.Z << "]." << std::endl;
    std::cout << "Reachable position: [" << agent_goal_position_.X << ", " << agent_goal_position_.Y << ", " << agent_goal_position_.Z << "]." << std::endl;
    std::cout << "-----------------------------------------------------------" << std::endl;
    std::cout << "Way points: " << std::endl;
    for (auto wayPoint : path_points_) {
        std::cout << "[" << wayPoint.Location.X << ", " << wayPoint.Location.Y << ", " << wayPoint.Location.Z << "]" << std::endl;
    }
    std::cout << "-----------------------------------------------------------" << std::endl;

    return status;
}
