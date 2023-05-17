//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SimulationController/ImitationLearningTask.h"

#include <fstream>
#include <map>
#include <string>
#include <vector>

#include <AI/Navigation/NavigationTypes.h>
#include <AI/NavigationSystemBase.h>
#include <Components/SceneComponent.h>
#include <Delegates/IDelegateInstance.h>
#include <DrawDebugHelpers.h>
#include <Engine/EngineTypes.h>
#include <Engine/World.h>
#include <GameFramework/Actor.h>
#include <Math/Rotator.h>
#include <Math/Vector.h>
#include <NavigationData.h>
#include <NavigationSystem.h>
#include <NavigationSystemTypes.h>
#include <NavMesh/RecastNavMesh.h>

#include "CoreUtils/ArrayDesc.h"
#include "CoreUtils/Assert.h"
#include "CoreUtils/Config.h"
#include "CoreUtils/Std.h"
#include "CoreUtils/Unreal.h"
#include "SimulationController/ActorHitEventComponent.h"

ImitationLearningTask::ImitationLearningTask(UWorld* world)
{
    FActorSpawnParameters actor_spawn_params;
    actor_spawn_params.Name = Unreal::toFName(Config::get<std::string>("SIMULATION_CONTROLLER.IMITATION_LEARNING_TASK.GOAL_ACTOR_NAME"));
    actor_spawn_params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    goal_actor_ = world->SpawnActor<AActor>(FVector::ZeroVector, FRotator::ZeroRotator, actor_spawn_params);
    SP_ASSERT(goal_actor_);

    auto scene_component = NewObject<USceneComponent>(goal_actor_);
    scene_component->SetMobility(EComponentMobility::Movable);
    goal_actor_->SetRootComponent(scene_component);

    parent_actor_ = world->SpawnActor<AActor>();
    SP_ASSERT(parent_actor_);

    // Create UActorHitEvent but don't subscribe to any actors yet
    actor_hit_event_component_ = NewObject<UActorHitEventComponent>(parent_actor_);
    SP_ASSERT(actor_hit_event_component_);
    actor_hit_event_component_->RegisterComponent();
    actor_hit_event_handle_ = actor_hit_event_component_->delegate_.AddRaw(this, &ImitationLearningTask::actorHitEventHandler);

    // If the start/goal positions are not randomly generated, get them from a file
    if (!Config::get<bool>("SIMULATION_CONTROLLER.IMITATION_LEARNING_TASK.GET_POSITIONS_FROM_TRAJECTORY_SAMPLING")) {
        getPositionsFromFile();
    }
}

ImitationLearningTask::~ImitationLearningTask()
{
    clearPositions();
    
    SP_ASSERT(actor_hit_event_component_);
    actor_hit_event_component_->delegate_.Remove(actor_hit_event_handle_);
    actor_hit_event_handle_.Reset();
    actor_hit_event_component_->DestroyComponent();
    actor_hit_event_component_ = nullptr;

    SP_ASSERT(parent_actor_);
    parent_actor_->Destroy();
    parent_actor_ = nullptr;

    SP_ASSERT(goal_actor_);
    goal_actor_->Destroy();
    goal_actor_ = nullptr;
}

void ImitationLearningTask::findObjectReferences(UWorld* world)
{
    agent_actor_ = Unreal::findActorByName(world, Config::get<std::string>("SIMULATION_CONTROLLER.IMITATION_LEARNING_TASK.AGENT_ACTOR_NAME"));
    SP_ASSERT(agent_actor_);

    bool return_null_if_not_found = false;
    obstacle_ignore_actors_ = Unreal::findActorsByName(
        world, Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.IMITATION_LEARNING_TASK.OBSTACLE_IGNORE_ACTOR_NAMES"), return_null_if_not_found);

    // Subscribe to the agent actor now that we have obtained a reference to it
    actor_hit_event_component_->subscribeToActor(agent_actor_);

    nav_sys_ = FNavigationSystem::GetCurrent<UNavigationSystemV1>(world);
    SP_ASSERT(nav_sys_);

    FNavAgentProperties agent_properties;
    agent_properties.AgentHeight     = Config::get<float>("SIMULATION_CONTROLLER.IMITATION_LEARNING_TASK.NAVMESH.AGENT_HEIGHT");
    agent_properties.AgentRadius     = Config::get<float>("SIMULATION_CONTROLLER.IMITATION_LEARNING_TASK.NAVMESH.AGENT_RADIUS");
    agent_properties.AgentStepHeight = Config::get<float>("SIMULATION_CONTROLLER.IMITATION_LEARNING_TASK.NAVMESH.AGENT_MAX_STEP_HEIGHT");

    ANavigationData* nav_data = nav_sys_->GetNavDataForProps(agent_properties);
    SP_ASSERT(nav_data);

    nav_mesh_ = dynamic_cast<ARecastNavMesh*>(nav_data);
    SP_ASSERT(nav_mesh_);
}

void ImitationLearningTask::cleanUpObjectReferences()
{
    SP_ASSERT(nav_mesh_);
    nav_mesh_ = nullptr;

    SP_ASSERT(nav_sys_);
    nav_sys_ = nullptr;

    SP_ASSERT(actor_hit_event_component_);
    actor_hit_event_component_->unsubscribeFromActor(agent_actor_);

    obstacle_ignore_actors_.clear();

    SP_ASSERT(agent_actor_);
    agent_actor_ = nullptr;
}

void ImitationLearningTask::beginFrame()
{
    hit_goal_ = false;
    hit_obstacle_ = false;
}

void ImitationLearningTask::endFrame() {}

float ImitationLearningTask::getReward() const
{
    return -std::numeric_limits<float>::infinity();
}

bool ImitationLearningTask::isEpisodeDone() const
{
    return hit_goal_ || hit_obstacle_;
}

std::map<std::string, ArrayDesc> ImitationLearningTask::getStepInfoSpace() const
{
    std::map<std::string, ArrayDesc> step_info_space;
    ArrayDesc array_desc;

    array_desc.low_ = 0.0;
    array_desc.high_ = 1.0;
    array_desc.shape_ = {1};
    array_desc.datatype_ = DataType::UInteger8;
    step_info_space["hit_goal"] = std::move(array_desc);

    array_desc.low_ = 0.0;
    array_desc.high_ = 1.0;
    array_desc.shape_ = {1};
    array_desc.datatype_ = DataType::UInteger8;
    step_info_space["hit_obstacle"] = std::move(array_desc);

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
    // If we are generating positions via trajectory sampling, then update agent_initial_positions_
    // and agent_goal_positions_ to store the results from one round of trajectory sampling, and
    // reset position_index_ to 0.
    if (Config::get<bool>("SIMULATION_CONTROLLER.IMITATION_LEARNING_TASK.GET_POSITIONS_FROM_TRAJECTORY_SAMPLING")) {
        getPositionsFromTrajectorySampling();
    }

    // Set agent and goal positions
    bool sweep = false;
    FHitResult* hit_result = nullptr;
    agent_actor_->SetActorLocationAndRotation(
        agent_initial_positions_.at(position_index_), FRotator::ZeroRotator, sweep, hit_result, ETeleportType::TeleportPhysics);
    goal_actor_->SetActorLocationAndRotation(
        agent_goal_positions_.at(position_index_), FRotator::ZeroRotator, sweep, hit_result, ETeleportType::TeleportPhysics);

    // Increment position_index_
    if (position_index_ < agent_goal_positions_.size() - 1) { 
        position_index_++;
    }  else {
        position_index_ = 0;
    }
}

bool ImitationLearningTask::isReady() const
{
    return true;
}

void ImitationLearningTask::actorHitEventHandler(AActor* self_actor, AActor* other_actor, FVector normal_impulse, const FHitResult& hit_result)
{
    SP_ASSERT(self_actor == agent_actor_);

    if (other_actor == goal_actor_) {
        hit_goal_ = true;
    } else if (!Std::contains(obstacle_ignore_actors_, other_actor)) {
        hit_obstacle_ = true;
    }
}

void ImitationLearningTask::getPositionsFromFile()
{
    agent_initial_positions_.clear();
    agent_goal_positions_.clear();
    position_index_ = -1;

    // Create an input filestream 
    std::ifstream fs(Config::get<std::string>("SIMULATION_CONTROLLER.IMITATION_LEARNING_TASK.POSITIONS_FILE")); 
    SP_ASSERT(fs.is_open());

    // Read file data, line-by-line in the format:
    // scene_id, init_pos_x_cms, init_pos_y_cms, init_pos_z_cms, goal_pos_x_cms, goal_pos_y_cms, goal_pos_z_cms
    std::string line;
    std::getline(fs, line); // header
    while (std::getline(fs, line)) {        
        std::vector<std::string> tokens = Std::tokenize(line, ",");
        SP_ASSERT(tokens.size() == 7);
        std::string scene_id = tokens.at(0);
        FVector init(std::stod(tokens.at(1)), std::stod(tokens.at(2)), std::stod(tokens.at(3)));
        FVector goal(std::stod(tokens.at(4)), std::stod(tokens.at(5)), std::stod(tokens.at(6)));

        // If the scene id matches the currently opened map, then add to our list of positions
        if(scene_id == Config::get<std::string>("SIMULATION_CONTROLLER.SCENE_ID")) {
            agent_initial_positions_.push_back(init);
            agent_goal_positions_.push_back(goal);
        }
    }

    // Close file
    fs.close();

    position_index_ = 0;
}

void ImitationLearningTask::getPositionsFromTrajectorySampling()
{
    agent_initial_positions_.clear();
    agent_goal_positions_.clear();
    position_index_ = -1;

    float best_path_score = 0.0f;
    FNavLocation best_init_location;
    FNavLocation best_goal_location;
    TArray<FNavPathPoint> best_path_points;

    // Trajectory sampling to get an interesting path
    for (int i = 0; i < Config::get<int>("SIMULATION_CONTROLLER.IMITATION_LEARNING_TASK.TRAJECTORY_SAMPLING_MAX_ITERS"); i++) {

        FNavLocation init_location;
        FNavLocation goal_location;
        
        // Get a random initial point
        init_location = nav_mesh_->GetRandomPoint();

        // Get a random reachable goal point, to be reached by the agent from init_location.Location
        bool found = nav_mesh_->GetRandomReachablePointInRadius(
            init_location.Location, Config::get<float>("SIMULATION_CONTROLLER.IMITATION_LEARNING_TASK.TRAJECTORY_SAMPLING_SEARCH_RADIUS"), goal_location);
        SP_ASSERT(found);

        // Update navigation query with the new goal
        FPathFindingQuery nav_query = FPathFindingQuery(agent_actor_, *nav_mesh_, init_location.Location, goal_location.Location);

        // Generate a collision-free path between the initial position and the goal position
        FPathFindingResult path = nav_sys_->FindPathSync(nav_query, EPathFindingMode::Type::Regular);

        // If path finding is sucessful, make sure that it is not too simple
        if (path.IsSuccessful() && path.Path.IsValid()) {

            // Debug output
            if (path.IsPartial()) {
                SP_LOG("Only a partial path could be found...");
            }

            // Compute a path score to evaluate its complexity
            int num_waypoints = path.Path->GetPathPoints().Num();
            FVector2D relative_position_to_goal((goal_location.Location - init_location.Location).X, (goal_location.Location - init_location.Location).Y);
            float path_score = relative_position_to_goal.Size() * num_waypoints;

            // If the path_score is the best we've seen so far, update best_init_location and best_goal_location
            if (best_path_score <= path_score) {
                best_path_score = path_score;
                best_init_location = init_location;
                best_goal_location = goal_location;
                best_path_points = path.Path->GetPathPoints();

                // Debug output
                if (Config::get<bool>("SIMULATION_CONTROLLER.IMITATION_LEARNING_TASK.TRAJECTORY_SAMPLING_DEBUG_RENDER")) {
                    float trajectory_length = 0.0f;
                    for (int j = 0; j < num_waypoints - 1; j++) {
                        trajectory_length += FVector::Dist(best_path_points[j].Location, best_path_points[j + 1].Location);
                    }

                    float world_to_meters = agent_actor_->GetWorld()->GetWorldSettings()->WorldToMeters;

                    SP_LOG("Iteration:           ", i);
                    SP_LOG("Score:               ", best_path_score);
                    SP_LOG("Number of waypoints: ", num_waypoints);
                    SP_LOG("Goal distance:       ", relative_position_to_goal.Size() / world_to_meters, "m");
                    SP_LOG("Path length:         ", trajectory_length / world_to_meters, "m");
                }
            }
        }
    }

    SP_ASSERT(best_path_points.Num() > 1);

    // Update positions
    agent_initial_positions_.push_back(best_init_location.Location);
    agent_goal_positions_.push_back(best_goal_location.Location);
    position_index_ = 0;

    // Debug output
    if (Config::get<bool>("SIMULATION_CONTROLLER.IMITATION_LEARNING_TASK.TRAJECTORY_SAMPLING_DEBUG_RENDER")) {
        SP_LOG("Initial position: ", agent_initial_positions_.at(0).X, ", ", agent_initial_positions_.at(0).Y, ", ", agent_initial_positions_.at(0).Z);
        SP_LOG("Goal position:    ", agent_goal_positions_.at(0).X,    ", ", agent_goal_positions_.at(0).Y,    ", ", agent_goal_positions_.at(0).Z);
        SP_LOG("Waypoints:");
        for (int i = 1; i < best_path_points.Num(); i++) {
            SP_LOG("    ", best_path_points[i].Location.X, ", ", best_path_points[i].Location.Y, ", ", best_path_points[i].Location.Z);
        }

        for (int i = 0; i < best_path_points.Num(); i++) {
            DrawDebugPoint(agent_actor_->GetWorld(), best_path_points[i].Location, 20.0f, FColor(25, 116, 210), false, 10.0f, 0);
        }

        for (int i = 0; i < best_path_points.Num() - 1; i++) {
            DrawDebugLine(agent_actor_->GetWorld(), best_path_points[i].Location, best_path_points[i+1].Location, FColor(25, 116, 210), false, 10.0f, 0, 0.15f);
        }
    }
}

void ImitationLearningTask::clearPositions()
{
    agent_initial_positions_.clear();
    agent_goal_positions_.clear();
    position_index_ = -1;
}
