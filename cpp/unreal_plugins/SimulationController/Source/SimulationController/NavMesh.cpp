//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SimulationController/NavMesh.h"

#include <vector>

#include <AI/NavDataGenerator.h>
#include <AI/Navigation/NavigationTypes.h>
#include <NavigationData.h>
#include <NavigationSystem.h>
#include <NavMesh/RecastNavMesh.h>

#include "CoreUtils/Config.h"
#include "CoreUtils/Log.h"
#include "CoreUtils/Unreal.h"

void NavMesh::findObjectReferences(UWorld* world)
{
    SP_ASSERT(world);
    world_ = world;

    auto navigation_system_v1_ = FNavigationSystem::GetCurrent<UNavigationSystemV1>(world_);
    ASSERT(navigation_system_v1_);

    FNavAgentProperties agent_properties;
    agent_properties.AgentHeight     = Config::get<float>("SIMULATION_CONTROLLER.NAVMESH.AGENT_HEIGHT");
    agent_properties.AgentRadius     = Config::get<float>("SIMULATION_CONTROLLER.NAVMESH.AGENT_RADIUS");
    agent_properties.AgentStepHeight = Config::get<float>("SIMULATION_CONTROLLER.NAVMESH.AGENT_MAX_STEP_HEIGHT");

    ANavigationData* navigation_data = navigation_system_v1_->GetNavDataForProps(agent_properties);
    ASSERT(navigation_data);

    recast_nav_mesh_ = dynamic_cast<ARecastNavMesh*>(navigation_data);
    ASSERT(recast_nav_mesh_);

    recast_nav_mesh_->TilePoolSize           = Config::get<float>("SIMULATION_CONTROLLER.NAVMESH.TILE_POOL_SIZE");
    recast_nav_mesh_->TileSizeUU             = Config::get<float>("SIMULATION_CONTROLLER.NAVMESH.TILE_SIZE_UU");
    recast_nav_mesh_->AgentRadius            = Config::get<float>("SIMULATION_CONTROLLER.NAVMESH.AGENT_RADIUS");
    recast_nav_mesh_->AgentHeight            = Config::get<float>("SIMULATION_CONTROLLER.NAVMESH.AGENT_HEIGHT");
    recast_nav_mesh_->AgentMaxSlope          = Config::get<float>("SIMULATION_CONTROLLER.NAVMESH.AGENT_MAX_SLOPE");
    recast_nav_mesh_->AgentMaxStepHeight     = Config::get<float>("SIMULATION_CONTROLLER.NAVMESH.AGENT_MAX_STEP_HEIGHT");
    recast_nav_mesh_->MinRegionArea          = Config::get<float>("SIMULATION_CONTROLLER.NAVMESH.MIN_REGION_AREA");
    recast_nav_mesh_->MergeRegionSize        = Config::get<float>("SIMULATION_CONTROLLER.NAVMESH.MERGE_REGION_SIZE");
    recast_nav_mesh_->MaxSimplificationError = Config::get<float>("SIMULATION_CONTROLLER.NAVMESH.MAX_SIMPLIFICATION_ERROR");

    auto cell_size   = Config::get<float>("SIMULATION_CONTROLLER.NAVMESH.CELL_SIZE");
    auto cell_height = Config::get<float>("SIMULATION_CONTROLLER.NAVMESH.CELL_HEIGHT");

    recast_nav_mesh_->NavMeshResolutionParams[static_cast<uint8>(ENavigationDataResolution::Low)].CellSize       = cell_size;
    recast_nav_mesh_->NavMeshResolutionParams[static_cast<uint8>(ENavigationDataResolution::Low)].CellHeight     = cell_height;
    recast_nav_mesh_->NavMeshResolutionParams[static_cast<uint8>(ENavigationDataResolution::Default)].CellSize   = cell_size;
    recast_nav_mesh_->NavMeshResolutionParams[static_cast<uint8>(ENavigationDataResolution::Default)].CellHeight = cell_height;
    recast_nav_mesh_->NavMeshResolutionParams[static_cast<uint8>(ENavigationDataResolution::High)].CellSize      = cell_size;
    recast_nav_mesh_->NavMeshResolutionParams[static_cast<uint8>(ENavigationDataResolution::High)].CellHeight    = cell_height;

    SP_LOG("Building navigation mesh...");

    navigation_system_v1_->Build();

    // We need to wrap this call with guards because ExportNavigationData(...) is only implemented in non-shipping builds, see:
    //     Engine/Source/Runtime/Engine/Public/AI/NavDataGenerator.h
    //     Engine/Source/Runtime/NavigationSystem/Public/NavMesh/RecastNavMeshGenerator.h
    //     Engine/Source/Runtime/NavigationSystem/Private/NavMesh/RecastNavMeshGenerator.cpp
    #if !(UE_BUILD_SHIPPING || UE_BUILD_TEST)
        auto debug_navigation_data_file = Config::get<std::string>("SIMULATION_CONTROLLER.NAVMESH.DEBUG_NAVIGATION_DATA_FILE");
        if (debug_navigation_data_file != "") {
            recast_nav_mesh_->GetGenerator()->ExportNavigationData(Unreal::toFString(debug_navigation_data_file));
        }
    #endif
}

void NavMesh::cleanUpObjectReferences()
{
    recast_nav_mesh_ = nullptr;
    navigation_system_v1_ = nullptr;
    world_ = nullptr;
}

std::vector<uint8_t> NavMesh::getRandomPoints(int num_points)
{
    std::vector<double> points;
    for (int i = 0; i < num_points; i++) {
        FVector point = recast_nav_mesh_->GetRandomPoint().Location;
        points.push_back(point.X);
        points.push_back(point.Y);
        points.push_back(point.Z);
    }
    return Std::reinterpretAs<uint8_t>(points);
}

std::vector<uint8_t> NavMesh::getTrajectoryBetweenTwoPoints(const std::vector<float>& start_point, const std::vector<float>& end_point)
{
    std::vector<float> trajectory;

    SP_ASSERT(start_point.size() == 3);
    SP_ASSERT(end_point.size() == 3);

    FVector start_point_fvector{start_point.at(0), start_point.at(1), start_point.at(2)};
    FVector end_point_fvector{end_point.at(0), end_point.at(1), end_point.at(2)};

    // Update navigation query with the new agent position and goal position
    // TODO: check if we need to store reference to world because this is the only place it is being used.
    //       Can we just use recast_nav_mesh_ or navigation_system_v1_ as the owner object instead of world_?
    FPathFindingQuery nav_query = FPathFindingQuery(world_, *recast_nav_mesh_, start_point_fvector, end_point_fvector);

    // Generate a collision-free path between the agent position and the goal position
    FPathFindingResult path = navigation_system_v1_->FindPathSync(nav_query, EPathFindingMode::Type::Regular);

    // Ensure that path generation process was successful and that the generated path is valid
    ASSERT(path.IsSuccessful());
    ASSERT(path.Path.IsValid());

    // Update trajectory
    TArray<FNavPathPoint> path_points = path.Path->GetPathPoints();
    ASSERT(path_points.Num() > 1); // There should be at least a starting point and a goal point

    for (auto& path_point : path_points) {
        trajectory.push_back(path_point.Location.X);
        trajectory.push_back(path_point.Location.Y);
        trajectory.push_back(path_point.Location.Z);
    }

    // Debug output
    if (path.IsPartial()) {
        SP_LOG("Only a partial path could be found...");
    }

    int num_waypoints = path.Path->GetPathPoints().Num();
    float trajectory_length = 0.0f;
    for (int i = 0; i < num_waypoints - 1; i++) {
        trajectory_length += FVector::Dist(path_points[i].Location, path_points[i + 1].Location);
    }
    trajectory_length /= world_->GetWorldSettings()->WorldToMeters;
    FVector2D relative_position_to_goal(
        (end_point_fvector - start_point_fvector).X, (end_point_fvector - start_point_fvector).Y);

    SP_LOG("Number of waypoints: ", num_waypoints);
    SP_LOG("Goal distance: ", relative_position_to_goal.Size() / world_->GetWorldSettings()->WorldToMeters, "m");
    SP_LOG("Path length: ", trajectory_length, "m");
    SP_LOG("Initial position: [",
        start_point_fvector.X, ", ", start_point_fvector.Y, ", ", start_point_fvector.Z, "].");
    SP_LOG("Goal position: [",
        end_point_fvector.X, ", ", end_point_fvector.Y, ", ", end_point_fvector.Z, "].");
    SP_LOG("----------------------");
    SP_LOG("Waypoints: ");
    for (auto& point : path_points) {
        SP_LOG("[", point.Location.X, ", ", point.Location.Y, ", ", point.Location.Z, "]");
    }
    SP_LOG("----------------------");

    return Std::reinterpretAs<uint8_t>(trajectory);
}

std::vector<uint8_t> NavMesh::getReachablePoints()
{

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
        SP_LOG("Goal position:    ", agent_goal_positions_.at(0).X, ", ", agent_goal_positions_.at(0).Y, ", ", agent_goal_positions_.at(0).Z);
        SP_LOG("Waypoints:");
        for (int i = 1; i < best_path_points.Num(); i++) {
            SP_LOG("    ", best_path_points[i].Location.X, ", ", best_path_points[i].Location.Y, ", ", best_path_points[i].Location.Z);
        }

        for (int i = 0; i < best_path_points.Num(); i++) {
            DrawDebugPoint(agent_actor_->GetWorld(), best_path_points[i].Location, 20.0f, FColor(25, 116, 210), false, 10.0f, 0);
        }

        for (int i = 0; i < best_path_points.Num() - 1; i++) {
            DrawDebugLine(agent_actor_->GetWorld(), best_path_points[i].Location, best_path_points[i + 1].Location, FColor(25, 116, 210), false, 10.0f, 0, 0.15f);
        }
    }
}