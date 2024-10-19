//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpServices/Legacy/NavMesh.h"

#include <stdint.h> // uint8_t
#include <string>
#include <vector>

#include <AI/NavDataGenerator.h>           // FNavDataGenerator::ExportNavigationData
#include <AI/Navigation/NavigationTypes.h> // FNavAgentProperties, FNavLocation
#include <Containers/Array.h>
#include <Misc/Build.h>                    // UE_BUILD_SHIPPING, UE_BUILD_TEST
#include <NavigationData.h>                // FPathFindingResult
#include <NavigationSystem.h>
#include <NavigationSystemTypes.h>         // FPathFindingQuery
#include <NavMesh/RecastNavMesh.h>
#include <Templates/Casts.h>

#include "SpCore/Assert.h"
#include "SpCore/Config.h"
#include "SpCore/Log.h"
#include "SpCore/Unreal.h"

void NavMesh::findObjectReferences(UWorld* world)
{
    SP_ASSERT(world);
    world_ = world;

    navigation_system_v1_ = FNavigationSystem::GetCurrent<UNavigationSystemV1>(world_);
    SP_ASSERT(navigation_system_v1_);

    FNavAgentProperties agent_properties;
    if (Config::isInitialized()) {
        agent_properties.AgentHeight = Config::get<float>("SP_SERVICES.LEGACY.NAVMESH.AGENT_HEIGHT");
        agent_properties.AgentRadius = Config::get<float>("SP_SERVICES.LEGACY.NAVMESH.AGENT_RADIUS");
        agent_properties.AgentStepHeight = Config::get<float>("SP_SERVICES.LEGACY.NAVMESH.AGENT_MAX_STEP_HEIGHT");
    } else {
        agent_properties.AgentHeight = 200.0;
        agent_properties.AgentRadius = 50.0;
        agent_properties.AgentStepHeight = 1.0;
    }

    ANavigationData* navigation_data = navigation_system_v1_->GetNavDataForProps(agent_properties);
    SP_ASSERT(navigation_data);

    recast_nav_mesh_ = Cast<ARecastNavMesh>(navigation_data); // no RTTI available, so use Cast instead of dynamic_cast
    SP_ASSERT(recast_nav_mesh_);

    float cell_size = 1.0;
    float cell_height = 1.0;

    if (Config::isInitialized()) {
        recast_nav_mesh_->TilePoolSize = Config::get<float>("SP_SERVICES.LEGACY.NAVMESH.TILE_POOL_SIZE");
        recast_nav_mesh_->TileSizeUU = Config::get<float>("SP_SERVICES.LEGACY.NAVMESH.TILE_SIZE_UU");
        recast_nav_mesh_->AgentRadius = Config::get<float>("SP_SERVICES.LEGACY.NAVMESH.AGENT_RADIUS");
        recast_nav_mesh_->AgentHeight = Config::get<float>("SP_SERVICES.LEGACY.NAVMESH.AGENT_HEIGHT");
        recast_nav_mesh_->AgentMaxSlope = Config::get<float>("SP_SERVICES.LEGACY.NAVMESH.AGENT_MAX_SLOPE");
        // recast_nav_mesh_->AgentMaxStepHeight = Config::get<float>("SP_SERVICES.LEGACY.NAVMESH.AGENT_MAX_STEP_HEIGHT"); // TODO: not available in UE 5.4
        recast_nav_mesh_->MinRegionArea = Config::get<float>("SP_SERVICES.LEGACY.NAVMESH.MIN_REGION_AREA");
        recast_nav_mesh_->MergeRegionSize = Config::get<float>("SP_SERVICES.LEGACY.NAVMESH.MERGE_REGION_SIZE");
        recast_nav_mesh_->MaxSimplificationError = Config::get<float>("SP_SERVICES.LEGACY.NAVMESH.MAX_SIMPLIFICATION_ERROR");

        cell_size = Config::get<float>("SP_SERVICES.LEGACY.NAVMESH.CELL_SIZE");
        cell_height = Config::get<float>("SP_SERVICES.LEGACY.NAVMESH.CELL_HEIGHT");
    } else {
        recast_nav_mesh_->TilePoolSize = 1024;
        recast_nav_mesh_->TileSizeUU = 1000.0;
        recast_nav_mesh_->AgentRadius = 50.0;
        recast_nav_mesh_->AgentHeight = 200.0;
        recast_nav_mesh_->AgentMaxSlope = 1.0;
        // recast_nav_mesh_->AgentMaxStepHeight = 1.0; // TODO: not available in UE 5.4
        recast_nav_mesh_->MinRegionArea = 100.0;
        recast_nav_mesh_->MergeRegionSize = 400.0;
        recast_nav_mesh_->MaxSimplificationError = 1.3;
    }

    recast_nav_mesh_->NavMeshResolutionParams[static_cast<uint8>(ENavigationDataResolution::Low)].CellSize = cell_size;
    recast_nav_mesh_->NavMeshResolutionParams[static_cast<uint8>(ENavigationDataResolution::Low)].CellHeight = cell_height;
    recast_nav_mesh_->NavMeshResolutionParams[static_cast<uint8>(ENavigationDataResolution::Default)].CellSize = cell_size;
    recast_nav_mesh_->NavMeshResolutionParams[static_cast<uint8>(ENavigationDataResolution::Default)].CellHeight = cell_height;
    recast_nav_mesh_->NavMeshResolutionParams[static_cast<uint8>(ENavigationDataResolution::High)].CellSize = cell_size;
    recast_nav_mesh_->NavMeshResolutionParams[static_cast<uint8>(ENavigationDataResolution::High)].CellHeight = cell_height;

    SP_LOG("Building navigation mesh...");

    navigation_system_v1_->Build();

    // We need to wrap this call with guards because ExportNavigationData(...) is only implemented in non-shipping builds, see:
    //     Engine/Source/Runtime/Engine/Public/AI/NavDataGenerator.h
    //     Engine/Source/Runtime/NavigationSystem/Public/NavMesh/RecastNavMeshGenerator.h
    //     Engine/Source/Runtime/NavigationSystem/Private/NavMesh/RecastNavMeshGenerator.cpp
#if !(UE_BUILD_SHIPPING || UE_BUILD_TEST)

    std::string debug_navigation_data_file = "";

    if (Config::isInitialized()) {
        debug_navigation_data_file = Config::get<std::string>("SP_SERVICES.LEGACY.NAVMESH.DEBUG_NAVIGATION_DATA_FILE");
    }

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

std::vector<double> NavMesh::getRandomPoints(int num_points)
{
    std::vector<double> points;

    for (int i = 0; i < num_points; i++) {
        FVector point = recast_nav_mesh_->GetRandomPoint().Location;
        points.push_back(point.X);
        points.push_back(point.Y);
        points.push_back(point.Z);
    }

    return points;
}

std::vector<double> NavMesh::getRandomReachablePointsInRadius(const std::vector<double>& reference_points, float radius)
{
    SP_ASSERT(reference_points.size() % 3 == 0);

    std::vector<double> reachable_points;

    for (int i = 0; i < reference_points.size(); i += 3) {

        FVector reference_point = {reference_points.at(i), reference_points.at(i + 1), reference_points.at(i + 2)};
        FNavLocation nav_location;

        bool found = recast_nav_mesh_->GetRandomReachablePointInRadius(reference_point, radius, nav_location);
        SP_ASSERT(found);

        reachable_points.push_back(nav_location.Location.X);
        reachable_points.push_back(nav_location.Location.Y);
        reachable_points.push_back(nav_location.Location.Z);
    }

    return reachable_points;
}

std::vector<std::vector<double>> NavMesh::getPaths(const std::vector<double>& initial_points, const std::vector<double>& goal_points)
{
    SP_ASSERT(initial_points.size() == goal_points.size());
    SP_ASSERT(initial_points.size() % 3 == 0);

    std::vector<std::vector<double>> paths;

    for (int i = 0; i < initial_points.size(); i += 3) {
        FVector initial_point = {initial_points.at(i), initial_points.at(i + 1), initial_points.at(i + 2)};
        FVector goal_point = {goal_points.at(i), goal_points.at(i + 1), goal_points.at(i + 2)};
        FPathFindingQuery path_finding_query = FPathFindingQuery(world_, *recast_nav_mesh_, initial_point, goal_point);
        FPathFindingResult path_finding_result = navigation_system_v1_->FindPathSync(path_finding_query, EPathFindingMode::Type::Regular);
        SP_ASSERT(path_finding_result.IsSuccessful());
        SP_ASSERT(path_finding_result.Path.IsValid());

        TArray<FNavPathPoint> nav_path_points = path_finding_result.Path->GetPathPoints();
        SP_ASSERT(nav_path_points.Num() >= 2);

        std::vector<double> path;
        for (auto& point : nav_path_points) {
            path.push_back(point.Location.X);
            path.push_back(point.Location.Y);
            path.push_back(point.Location.Z);
        }
        paths.push_back(path);
    }

    return paths;
}
