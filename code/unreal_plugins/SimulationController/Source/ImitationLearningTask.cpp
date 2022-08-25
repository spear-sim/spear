#include "ImitationLearningTask.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

#include <EngineUtils.h>
#include <UObject/UObjectGlobals.h>

#include "ActorHitEvent.h"
#include "Assert.h"
#include "Box.h"
#include "Config.h"

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

    if (Config::getValue<std::string>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "GOAL_ACTOR_NAME"}) != "") {
        ASSERT(goal_actor_);
    } else {
        FActorSpawnParameters goal_spawn_params;
        goal_spawn_params.Name = FName("GoalActor");
        goal_spawn_params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;
        goal_actor_ = world->SpawnActor<AActor>(AActor::StaticClass(), FVector::ZeroVector, FRotator::ZeroRotator, goal_spawn_params);
        USceneComponent* SceneComponent = NewObject<USceneComponent>(goal_actor_);
        SceneComponent->SetMobility(EComponentMobility::Movable);
        goal_actor_->SetRootComponent(SceneComponent);
        ASSERT(goal_actor_);
    }

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
    ASSERT(nav_sys_);

    // Get a pointer to the agent's navigation data
    const INavAgentInterface* actor_as_nav_agent = CastChecked<INavAgentInterface>(agent_actor_);
    ASSERT(actor_as_nav_agent);
    nav_data_ = nav_sys_->GetNavDataForProps(actor_as_nav_agent->GetNavAgentPropertiesRef(), actor_as_nav_agent->GetNavAgentLocation());
    ASSERT(nav_data_);

    // If the start/goal positions are not randomly generated 
    if (not Config::getValue<bool>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "RANDOM_SPAWN_TRAJ"})) {
        getPositionsFromFile();
    } 
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
    return -std::numeric_limits<float>::infinity();
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
    constexpr bool sweep = false;                    // Whether we sweep to the destination location, triggering overlaps along the way and stopping short of the target if blocked by something.
    constexpr FHitResult* hit_result_info = nullptr; // The hit result from the move if swept.

    if (Config::getValue<bool>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "RANDOM_SPAWN_TRAJ"})) {

        // Trajectory planning between randomly sampled initial/final positions:
        getPositionsFromTrajectorySampling();

        // Reset trajectory index:
        position_index_ = 0;
    } 

    // Set agent and goal positions:
    agent_actor_->SetActorLocation(agent_initial_position_.at(position_index_), sweep, hit_result_info, ETeleportType::ResetPhysics);
    goal_actor_->SetActorLocation(agent_goal_position_.at(position_index_), sweep, hit_result_info, ETeleportType::ResetPhysics);
    
    if (position_index_ < agent_goal_position_.size()-1){ 
        position_index_++; // Move to the next pair of points (if available)
    }  else { 
        position_index_ = 0; // wrap around
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

void ImitationLearningTask::getPositionsFromFile()
{
    std::string line;
    std::string token;
    FVector init, goal;
    int episodes;

    // Create an input filestream 
    std::ifstream fs(Config::getValue<std::string>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "TRAJECTORY_FILE"})); 
    ASSERT(fs.is_open());

    // Read file data, line by line in the format "init.X, init.Y, init.Z, goal.X, goal.Y, goal.Z, episodes"
    std::getline(fs, line); // get csv header
    while (std::getline(fs, line)) {
        std::istringstream ss(line);
        std::getline(ss, token, ','); ASSERT(ss); init.X = std::stof(token);
        std::getline(ss, token, ','); ASSERT(ss); init.Y = std::stof(token);
        std::getline(ss, token, ','); ASSERT(ss); init.Z = std::stof(token);
        std::getline(ss, token, ','); ASSERT(ss); goal.X = std::stof(token);
        std::getline(ss, token, ','); ASSERT(ss); goal.Y = std::stof(token);
        std::getline(ss, token, ','); ASSERT(ss); goal.Z = std::stof(token);
        agent_initial_position_.push_back(init);
        agent_goal_position_.push_back(goal);
    }

    // Close file
    fs.close();

    // Initialize trajectory index
    position_index_ = 0;
}

void ImitationLearningTask::getPositionsFromTrajectorySampling()
{
    agent_initial_position_.resize(1);
    agent_goal_position_.resize(1);
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
    TArray<FNavPathPoint> path_points;

    // Sanity checks
    ASSERT(nav_data_);
    ASSERT(nav_sys_);

    // Path generation polling to get "interesting" paths in every experiment:
    while (number_iterations < Config::getValue<int>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "MAX_ITER_REPLAN"})) { // Try to generate interesting trajectories with multiple waypoints

        // Get a random initial point:
        ASSERT(nav_sys_->GetRandomPoint(init_location, nav_data_));

        // Get a random reachable target point, to be reached by the agent from init_location.Location:
        ASSERT(nav_sys_->GetRandomReachablePointInRadius(init_location.Location, Config::getValue<float>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "TARGET_RADIUS"}), target_location));

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
            path_criterion = relative_position_to_target.Size() * Config::getValue<float>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "PATH_WEIGHT_DIST"}) + number_of_way_points * relative_position_to_target.Size() * Config::getValue<float>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "PATH_WEIGHT_NUM_WAYPOINTS"});

            if (best_path_criterion <= path_criterion) {
                best_path_criterion = path_criterion;
                best_target_location = target_location;
                best_init_location = init_location;
                path_points.Empty();
                path_points = collision_free_path.Path->GetPathPoints();
                std::cout << "Iteration: " << number_iterations << std::endl;
                std::cout << "Cost: " << best_path_criterion << std::endl;
                std::cout << "Number of way points: " << number_of_way_points << std::endl;
                std::cout << "Target distance: " << relative_position_to_target.Size() / agent_actor_->GetWorld()->GetWorldSettings()->WorldToMeters << "m" << std::endl;

                float trajectory_length = 0.0;
                for (size_t i = 0; i < number_of_way_points - 1; i++) {
                    trajectory_length += FVector::Dist(path_points[i].Location, path_points[i + 1].Location);
                }
                std::cout << "Path length " << trajectory_length / agent_actor_->GetWorld()->GetWorldSettings()->WorldToMeters << "m" << std::endl;
            }

            number_iterations++;

            if (number_iterations % 1000 == 0) {
                std::cout << ".";
            }

        }
    }
    std::cout << "" << std::endl;

    ASSERT(path_points.Num() > 1);

    // Update positions
    agent_initial_position_.at(0) = best_init_location.Location;
    agent_goal_position_.at(0) = best_target_location.Location;

    std::cout << "Initial position: [" << agent_initial_position_.at(0).X << ", " << agent_initial_position_.at(0).Y << ", " << agent_initial_position_.at(0).Z << "]." << std::endl;
    std::cout << "Reachable position: [" << agent_goal_position_.at(0).X << ", " << agent_goal_position_.at(0).Y << ", " << agent_goal_position_.at(0).Z << "]." << std::endl;
    std::cout << "-----------------------------------------------------------" << std::endl;
    std::cout << "Way points: " << std::endl;

    for (auto wayPoint : path_points) {
        std::cout << "[" << wayPoint.Location.X << ", " << wayPoint.Location.Y << ", " << wayPoint.Location.Z << "]" << std::endl;
    }

    std::cout << "-----------------------------------------------------------" << std::endl;
}
