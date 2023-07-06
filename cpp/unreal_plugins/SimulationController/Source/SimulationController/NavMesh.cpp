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

#include "CoreUtils/Assert.h"
#include "CoreUtils/Config.h"
#include "CoreUtils/Log.h"
#include "CoreUtils/Unreal.h"

void NavMesh::findObjectReferences(UWorld* world)
{
    SP_ASSERT(world);
    world_ = world;

    navigation_system_v1_ = FNavigationSystem::GetCurrent<UNavigationSystemV1>(world_);
    SP_ASSERT(navigation_system_v1_);

    FNavAgentProperties agent_properties;
    agent_properties.AgentHeight     = Config::get<float>("SIMULATION_CONTROLLER.NAVMESH.AGENT_HEIGHT");
    agent_properties.AgentRadius     = Config::get<float>("SIMULATION_CONTROLLER.NAVMESH.AGENT_RADIUS");
    agent_properties.AgentStepHeight = Config::get<float>("SIMULATION_CONTROLLER.NAVMESH.AGENT_MAX_STEP_HEIGHT");

    ANavigationData* navigation_data = navigation_system_v1_->GetNavDataForProps(agent_properties);
    SP_ASSERT(navigation_data);

    recast_nav_mesh_ = dynamic_cast<ARecastNavMesh*>(navigation_data);
    SP_ASSERT(recast_nav_mesh_);

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

std::vector<uint8_t> NavMesh::getReachablePoints(const std::vector<std::vector<float>>& start_points)
{
    std::vector<double> end_points;
    for (auto& start_point : start_points) {

        SP_ASSERT(start_point.size()==3);

        FVector start_point_fvector{start_point.at(0), start_point.at(1), start_point.at(2)};
        FNavLocation end_location;

        bool found = recast_nav_mesh_->GetRandomReachablePointInRadius(
            start_point_fvector, Config::get<float>("SIMULATION_CONTROLLER.NAVMESH.TRAJECTORY_SAMPLING_SEARCH_RADIUS"), end_location);
        SP_ASSERT(found);

        end_points.push_back(end_location.Location.X);
        end_points.push_back(end_location.Location.Y);
        end_points.push_back(end_location.Location.Z);
    }
    return Std::reinterpretAs<uint8_t>(end_points);
}

std::vector<std::vector<uint8_t>> NavMesh::getTrajectories(const std::vector<std::vector<float>>& start_points, const std::vector<std::vector<float>>& end_points)
{
    SP_ASSERT(start_points.size() == end_points.size());

    std::vector<std::vector<uint8_t>> trajectories;

    for (int i = 0; i < start_points.size(); ++i) {
        SP_ASSERT(start_points.at(i).size() == 3);
        SP_ASSERT(end_points.at(i).size() == 3);

        FVector start_point_fvector{start_points.at(i).at(0), start_points.at(i).at(1), start_points.at(i).at(2)};
        FVector end_point_fvector{end_points.at(i).at(0), end_points.at(i).at(1), end_points.at(i).at(2)};

        // Update navigation query with the start and end location
        FPathFindingQuery nav_query = FPathFindingQuery(world_, *recast_nav_mesh_, start_point_fvector, end_point_fvector);

        // Generate a collision-free path between the start location and the end location
        FPathFindingResult path = navigation_system_v1_->FindPathSync(nav_query, EPathFindingMode::Type::Regular);

        // Ensure that path generation process was successful and that the generated path is valid
        SP_ASSERT(path.IsSuccessful());
        SP_ASSERT(path.Path.IsValid());

        // Update trajectory with the waypoints
        TArray<FNavPathPoint> path_points = path.Path->GetPathPoints();
        SP_ASSERT(path_points.Num() > 1);

        std::vector<double> trajectory;
        for (auto& path_point : path_points) {
            trajectory.push_back(path_point.Location.X);
            trajectory.push_back(path_point.Location.Y);
            trajectory.push_back(path_point.Location.Z);
        }

        trajectories.push_back(Std::reinterpretAs<uint8_t>(trajectory));

        // Debug drawings
        if (Config::get<bool>("SIMULATION_CONTROLLER.NAVMESH.TRAJECTORY_SAMPLING_DEBUG_RENDER")) {
            for (int j = 1; j < path_points.Num(); ++j) {
                DrawDebugPoint(world_, path_points[j].Location, 20.0f, FColor(25, 116, 210), false, 10.0f, 0);
                DrawDebugLine(world_, path_points[j - 1].Location, path_points[j].Location, FColor(25, 116, 210), false, 10.0f, 0, 0.15f);
            }
        }
    }

    return trajectories;
}
