//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpComponents/SpNavMeshActor.h"

#include <Containers/Array.h>
#include <GameFramework/Actor.h>
#include <HAL/Platform.h>                  // uint64
#include <AI/Navigation/NavigationTypes.h> // FNavAgentProperties, FNavLocation
#include <Containers/Array.h>
#include <NavigationData.h> // FPathFindingResult
#include <NavigationSystem.h>
#include <NavigationSystemTypes.h> // FPathFindingQuery
#include <NavMesh/RecastNavMesh.h>

#include "SpCore/Assert.h"
#include "SpCore/Log.h"
#include "SpCore/Unreal.h"

ASpNavMeshActor::ASpNavMeshActor()
{
    SP_LOG_CURRENT_FUNCTION();

    PrimaryActorTick.bCanEverTick        = true;
    PrimaryActorTick.bTickEvenWhenPaused = false;
    PrimaryActorTick.TickGroup           = ETickingGroup::TG_PrePhysics;
}

ASpNavMeshActor::~ASpNavMeshActor()
{
    SP_LOG_CURRENT_FUNCTION();
}

void ASpNavMeshActor::Tick(float delta_time)
{
    AActor::Tick(delta_time);
}

void ASpNavMeshActor::setup(float agent_height, float agent_radius)
{
    SP_ASSERT(GetWorld());

    navigation_system_v1_ = FNavigationSystem::GetCurrent<UNavigationSystemV1>(GetWorld());
    SP_ASSERT(navigation_system_v1_);

    FNavAgentProperties agent_properties;
    agent_properties.AgentHeight     = agent_height;
    agent_properties.AgentRadius     = agent_radius;
    agent_properties.AgentStepHeight = 1.0;

    ANavigationData* navigation_data = navigation_system_v1_->GetNavDataForProps(agent_properties);
    SP_ASSERT(navigation_data);

    recast_nav_mesh_ = Cast<ARecastNavMesh>(navigation_data); // no RTTI available, so use Cast instead of dynamic_cast
    SP_ASSERT(recast_nav_mesh_);

    float cell_size   = 1.0;
    float cell_height = 1.0;

    recast_nav_mesh_->TilePoolSize           = 1024;
    recast_nav_mesh_->TileSizeUU             = 1000.0;
    recast_nav_mesh_->AgentRadius            = agent_radius;
    recast_nav_mesh_->AgentHeight            = agent_height;
    recast_nav_mesh_->AgentMaxSlope          = 1.0;
    recast_nav_mesh_->AgentMaxStepHeight     = 1.0;
    recast_nav_mesh_->MinRegionArea          = 100.0;
    recast_nav_mesh_->MergeRegionSize        = 400.0;
    recast_nav_mesh_->MaxSimplificationError = 1.3;

    recast_nav_mesh_->NavMeshResolutionParams[static_cast<uint8>(ENavigationDataResolution::Low)].CellSize       = cell_size;
    recast_nav_mesh_->NavMeshResolutionParams[static_cast<uint8>(ENavigationDataResolution::Low)].CellHeight     = cell_height;
    recast_nav_mesh_->NavMeshResolutionParams[static_cast<uint8>(ENavigationDataResolution::Default)].CellSize   = cell_size;
    recast_nav_mesh_->NavMeshResolutionParams[static_cast<uint8>(ENavigationDataResolution::Default)].CellHeight = cell_height;
    recast_nav_mesh_->NavMeshResolutionParams[static_cast<uint8>(ENavigationDataResolution::High)].CellSize      = cell_size;
    recast_nav_mesh_->NavMeshResolutionParams[static_cast<uint8>(ENavigationDataResolution::High)].CellHeight    = cell_height;

    SP_LOG("Building navigation mesh...");

    navigation_system_v1_->Build();
}

TArray<FVector> ASpNavMeshActor::getRandomPoints(int num_points)
{
    TArray<FVector> points;
    for (int i = 0; i < num_points; i++) {
        FVector point = recast_nav_mesh_->GetRandomPoint().Location;
        points.Add(point);
    }
    return points;
}

TArray<FVector> ASpNavMeshActor::getPaths(const FVector& initial_point, const FVector& goal_point)
{
    FPathFindingQuery path_finding_query   = FPathFindingQuery(GetWorld(), *recast_nav_mesh_, initial_point, goal_point);
    FPathFindingResult path_finding_result = navigation_system_v1_->FindPathSync(path_finding_query, EPathFindingMode::Type::Regular);
    SP_ASSERT(path_finding_result.IsSuccessful());
    SP_ASSERT(path_finding_result.Path.IsValid());
    if (path_finding_result.IsPartial()) {
        SP_LOG("partial result");
    }

    TArray<FNavPathPoint> nav_path_points = path_finding_result.Path->GetPathPoints();
    SP_ASSERT(nav_path_points.Num() >= 2);

    TArray<FVector> path;
    for (auto& point : nav_path_points) {
        path.Add(point.Location);
    }
    return path;
}
