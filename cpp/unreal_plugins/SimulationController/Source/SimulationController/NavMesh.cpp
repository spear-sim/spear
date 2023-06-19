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
#include "CoreUtils/Unreal.h"

NavMesh::NavMesh(UWorld* world) {}

NavMesh::~NavMesh() {}

void NavMesh::findObjectReferences(UWorld* world)
{
    auto navigation_system_v1 = FNavigationSystem::GetCurrent<UNavigationSystemV1>(world);
    ASSERT(navigation_system_v1);

    FNavAgentProperties agent_properties;
    agent_properties.AgentHeight     = Config::get<float>("SIMULATION_CONTROLLER.NAVMESH.AGENT_HEIGHT");
    agent_properties.AgentRadius     = Config::get<float>("SIMULATION_CONTROLLER.NAVMESH.AGENT_RADIUS");
    agent_properties.AgentStepHeight = Config::get<float>("SIMULATION_CONTROLLER.NAVMESH.AGENT_MAX_STEP_HEIGHT");

    ANavigationData* navigation_data = navigation_system_v1->GetNavDataForProps(agent_properties);
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

    SP_LOG("Before navigation_system_v1->Build()...");

    navigation_system_v1->Build();

    SP_LOG("After navigation_system_v1->Build()...");

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
