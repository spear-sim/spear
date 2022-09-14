// Fill out your copyright notice in the Description page of Project Settings.


#include "DummyActor.h"

#include <AI/NavDataGenerator.h>
#include <Engine/EngineTypes.h>
#include <Engine/World.h>
#include <EngineUtils.h>
#include <NavigationSystem.h>
#include <NavMesh/RecastNavMesh.h>
#include <NavMesh/NavMeshBoundsVolume.h>

#include <fstream>
#include <iostream>

// Sets default values
ADummyActor::ADummyActor()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

ADummyActor::~ADummyActor()
{}

// Called when the game starts or when spawned
void ADummyActor::BeginPlay()
{
    //world_begin_play_delegate_handle_ = GetWorld()->OnWorldBeginPlay.AddUObject(this, &ADummyActor::worldBeginPlayEventHandler);

	Super::BeginPlay();

    rebuildNavSystem();

    std::ofstream myfile;
    std::string file = "/Users/mroberts/Downloads/poses_for_debug.csv";
    //UE_LOG(LogTemp, Warning, TEXT("printing filename for storing debug poses %s"), UTF8_TO_TCHAR(file.c_str()));
    myfile.open(file);
    myfile << "pos_x_cm,pos_y_cm,pos_z_cm\n";
    for (size_t i = 0u; i < 1000000u; ++i) {
        FVector random_position = nav_mesh_->GetRandomPoint().Location;
        myfile << random_position.X << "," << random_position.Y << "," << random_position.Z << "\n";

        if (i < 1000) {
            std::cout << random_position.X << "," << random_position.Y << "," << random_position.Z << std::endl;
        }
    }
    myfile.close();
}

// Called every frame
void ADummyActor::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);
}

void ADummyActor::rebuildNavSystem()
{
    UNavigationSystemV1* nav_sys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(GetWorld());
    check(nav_sys);

    // FNavAgentProperties agent_properties;
    // agent_properties.AgentHeight = 1000.0f;
    // agent_properties.AgentRadius = 5.0f;
    // agent_properties.AgentStepHeight = 1.0f;

    // ANavigationData* nav_data = nav_sys->GetNavDataForProps(agent_properties);
    auto nav_data = nav_sys->GetMainNavData();
    check(nav_data);
    
    nav_mesh_ = Cast<ARecastNavMesh>(nav_data);
    check(nav_mesh_);

    ANavMeshBoundsVolume* nav_mesh_bounds = nullptr;
    for (TActorIterator<ANavMeshBoundsVolume> it(GetWorld()); it; ++it) {
        nav_mesh_bounds = *it;
    }
    check(nav_mesh_bounds);

    // Set the NavMesh properties:
    // nav_mesh_->CellSize = 1.0f;
    // nav_mesh_->CellHeight = 1.0f;
    // nav_mesh_->MergeRegionSize = 0.0f;
    // nav_mesh_->MinRegionArea = 100.0f;
    // nav_mesh_->AgentMaxStepHeight = 1.0f;
    // nav_mesh_->AgentMaxSlope = 1.0f;
    // nav_mesh_->TileSizeUU = 300.0f;
    // nav_mesh_->AgentRadius = 5.0f;
    // nav_mesh_->AgentHeight = 500.0f;
    // Dynamic update navMesh location and size

    nav_mesh_->AgentHeight                             = 1000.0f;
    nav_mesh_->AgentMaxSlope                           = 1.0f;
    nav_mesh_->AgentMaxStepHeight                      = 1.0f;
    nav_mesh_->AgentRadius                             = 5.0f;
    // nav_mesh_->bDrawClusters                           = true;
    // nav_mesh_->bDrawOctree                             = true;
    // nav_mesh_->bDrawOctreeDetails                      = true;
    // nav_mesh_->bDrawPathCollidingGeometry              = true;
    // nav_mesh_->bDrawPolyEdges                          = true;
    // nav_mesh_->bDrawPolygonLabels                      = true;
    // nav_mesh_->bDrawTileBounds                         = true;
    // nav_mesh_->bDrawTileLabels                         = true;
    // nav_mesh_->bDrawTriangleEdges                      = true;
    nav_mesh_->CellHeight                              = 10.0f;
    nav_mesh_->CellSize                                = 10.0f;
    // nav_mesh_->DefaultDrawDistance                     = 9999.0f;
    // nav_mesh_->DefaultMaxHierarchicalSearchNodes       = 2048.0;
    // nav_mesh_->DefaultMaxSearchNodes                   = 2048.0f;
    // nav_mesh_->DrawOffset                              = 10.0f;
    nav_mesh_->MergeRegionSize                         = 0.0f;
    nav_mesh_->MinRegionArea                           = 100.0f;
    // nav_mesh_->PolyRefNavPolyBits                      = 32;
    // nav_mesh_->PolyRefSaltBits                         = 12;
    // nav_mesh_->PolyRefTileBits                         = 20;
    // nav_mesh_->RegionChunkSplits                       = 2;
    // nav_mesh_->TileNumberHardLimit                     = 1048576; // 1024*1024*1024;
    // nav_mesh_->TilePoolSize                            = 1024;
    // nav_mesh_->TileSizeUU                              = 500.0f;
    // nav_mesh_->VerticalDeviationFromGroundCompensation = 0.0f;

    std::cout << "nav_mesh_->AgentHeight                             = " << nav_mesh_->AgentHeight                             << std::endl;
    std::cout << "nav_mesh_->AgentMaxSlope                           = " << nav_mesh_->AgentMaxSlope                           << std::endl;
    std::cout << "nav_mesh_->AgentMaxStepHeight                      = " << nav_mesh_->AgentMaxStepHeight                      << std::endl;
    std::cout << "nav_mesh_->AgentRadius                             = " << nav_mesh_->AgentRadius                             << std::endl;
    std::cout << "nav_mesh_->bAllowNavLinkAsPathEnd                  = " << nav_mesh_->bAllowNavLinkAsPathEnd                  << std::endl;
    std::cout << "nav_mesh_->bDistinctlyDrawTilesBeingBuilt          = " << nav_mesh_->bDistinctlyDrawTilesBeingBuilt          << std::endl;
    std::cout << "nav_mesh_->bDoFullyAsyncNavDataGathering           = " << nav_mesh_->bDoFullyAsyncNavDataGathering           << std::endl;
    std::cout << "nav_mesh_->bDrawClusters                           = " << nav_mesh_->bDrawClusters                           << std::endl;
    std::cout << "nav_mesh_->bDrawDefaultPolygonCost                 = " << nav_mesh_->bDrawDefaultPolygonCost                 << std::endl;
    std::cout << "nav_mesh_->bDrawFailedNavLinks                     = " << nav_mesh_->bDrawFailedNavLinks                     << std::endl;
    std::cout << "nav_mesh_->bDrawFilledPolys                        = " << nav_mesh_->bDrawFilledPolys                        << std::endl;
    std::cout << "nav_mesh_->bDrawLabelsOnPathNodes                  = " << nav_mesh_->bDrawLabelsOnPathNodes                  << std::endl;
    std::cout << "nav_mesh_->bDrawMarkedForbiddenPolys               = " << nav_mesh_->bDrawMarkedForbiddenPolys               << std::endl;
    std::cout << "nav_mesh_->bDrawNavLinks                           = " << nav_mesh_->bDrawNavLinks                           << std::endl;
    std::cout << "nav_mesh_->bDrawNavMeshEdges                       = " << nav_mesh_->bDrawNavMeshEdges                       << std::endl;
    std::cout << "nav_mesh_->bDrawOctree                             = " << nav_mesh_->bDrawOctree                             << std::endl;
    std::cout << "nav_mesh_->bDrawOctreeDetails                      = " << nav_mesh_->bDrawOctreeDetails                      << std::endl;
    std::cout << "nav_mesh_->bDrawPathCollidingGeometry              = " << nav_mesh_->bDrawPathCollidingGeometry              << std::endl;
    std::cout << "nav_mesh_->bDrawPolyEdges                          = " << nav_mesh_->bDrawPolyEdges                          << std::endl;
    std::cout << "nav_mesh_->bDrawPolygonLabels                      = " << nav_mesh_->bDrawPolygonLabels                      << std::endl;
    std::cout << "nav_mesh_->bDrawTileBounds                         = " << nav_mesh_->bDrawTileBounds                         << std::endl;
    std::cout << "nav_mesh_->bDrawTileLabels                         = " << nav_mesh_->bDrawTileLabels                         << std::endl;
    std::cout << "nav_mesh_->bDrawTriangleEdges                      = " << nav_mesh_->bDrawTriangleEdges                      << std::endl;
    std::cout << "nav_mesh_->bFilterLowSpanFromTileCache             = " << nav_mesh_->bFilterLowSpanFromTileCache             << std::endl;
    std::cout << "nav_mesh_->bFilterLowSpanSequences                 = " << nav_mesh_->bFilterLowSpanSequences                 << std::endl;
    std::cout << "nav_mesh_->bFixedTilePoolSize                      = " << nav_mesh_->bFixedTilePoolSize                      << std::endl;
    std::cout << "nav_mesh_->bMarkLowHeightAreas                     = " << nav_mesh_->bMarkLowHeightAreas                     << std::endl;
    std::cout << "nav_mesh_->bPerformVoxelFiltering                  = " << nav_mesh_->bPerformVoxelFiltering                  << std::endl;
    std::cout << "nav_mesh_->bSortNavigationAreasByCost              = " << nav_mesh_->bSortNavigationAreasByCost              << std::endl;
    std::cout << "nav_mesh_->bStoreEmptyTileLayers                   = " << nav_mesh_->bStoreEmptyTileLayers                   << std::endl;
    std::cout << "nav_mesh_->bUseBetterOffsetsFromCorners            = " << nav_mesh_->bUseBetterOffsetsFromCorners            << std::endl;
    std::cout << "nav_mesh_->bUseExtraTopCellWhenMarkingAreas        = " << nav_mesh_->bUseExtraTopCellWhenMarkingAreas        << std::endl;
    std::cout << "nav_mesh_->bUseVirtualFilters                      = " << nav_mesh_->bUseVirtualFilters                      << std::endl;
    std::cout << "nav_mesh_->CellHeight                              = " << nav_mesh_->CellHeight                              << std::endl;
    std::cout << "nav_mesh_->CellSize                                = " << nav_mesh_->CellSize                                << std::endl;
    std::cout << "nav_mesh_->DefaultDrawDistance                     = " << nav_mesh_->DefaultDrawDistance                     << std::endl;
    std::cout << "nav_mesh_->DefaultMaxHierarchicalSearchNodes       = " << nav_mesh_->DefaultMaxHierarchicalSearchNodes       << std::endl;
    std::cout << "nav_mesh_->DefaultMaxSearchNodes                   = " << nav_mesh_->DefaultMaxSearchNodes                   << std::endl;
    std::cout << "nav_mesh_->DrawOffset                              = " << nav_mesh_->DrawOffset                              << std::endl;
    std::cout << "nav_mesh_->HeuristicScale                          = " << nav_mesh_->HeuristicScale                          << std::endl;
    std::cout << "nav_mesh_->LayerChunkSplits                        = " << nav_mesh_->LayerChunkSplits                        << std::endl;
    std::cout << "nav_mesh_->MaxSimplificationError                  = " << nav_mesh_->MaxSimplificationError                  << std::endl;
    std::cout << "nav_mesh_->MaxSimultaneousTileGenerationJobsCount  = " << nav_mesh_->MaxSimultaneousTileGenerationJobsCount  << std::endl;
    std::cout << "nav_mesh_->MergeRegionSize                         = " << nav_mesh_->MergeRegionSize                         << std::endl;
    std::cout << "nav_mesh_->MinRegionArea                           = " << nav_mesh_->MinRegionArea                           << std::endl;
    std::cout << "nav_mesh_->NavMeshOriginOffset                     = " << nav_mesh_->NavMeshOriginOffset.X << " " << nav_mesh_->NavMeshOriginOffset.Y << " " << nav_mesh_->NavMeshOriginOffset.Z << std::endl;
    std::cout << "nav_mesh_->PolyRefNavPolyBits                      = " << nav_mesh_->PolyRefNavPolyBits                      << std::endl;
    std::cout << "nav_mesh_->PolyRefSaltBits                         = " << nav_mesh_->PolyRefSaltBits                         << std::endl;
    std::cout << "nav_mesh_->PolyRefTileBits                         = " << nav_mesh_->PolyRefTileBits                         << std::endl;
    std::cout << "nav_mesh_->RegionChunkSplits                       = " << nav_mesh_->RegionChunkSplits                       << std::endl;
    std::cout << "nav_mesh_->TileNumberHardLimit                     = " << nav_mesh_->TileNumberHardLimit                     << std::endl;
    std::cout << "nav_mesh_->TilePoolSize                            = " << nav_mesh_->TilePoolSize                            << std::endl;
    std::cout << "nav_mesh_->TileSizeUU                              = " << nav_mesh_->TileSizeUU                              << std::endl;
    std::cout << "nav_mesh_->VerticalDeviationFromGroundCompensation = " << nav_mesh_->VerticalDeviationFromGroundCompensation << std::endl;

    nav_mesh_bounds->GetRootComponent()->SetMobility(EComponentMobility::Movable);

    // FBox worldBox = getWorldBoundingBox(false);

    // std::cout << "worldbox center (X,Y,Z) : (" << worldBox.GetCenter().X << worldBox.GetCenter().Y << worldBox.GetCenter().Z << ")" << std::endl;
    // UE_LOG(LogTemp, Warning, TEXT("worldbox center (X,Y,Z) : (%f, %f, %f)"), worldBox.GetCenter().X, worldBox.GetCenter().Y, worldBox.GetCenter().Z);

    // std::cout << worldBox.GetCenter().X << " " << worldBox.GetCenter().Y << " "  << worldBox.GetCenter().Z  << " " << std::endl;
    // std::cout << nav_mesh_bounds->GetActorLocation().X << " " << nav_mesh_bounds->GetActorLocation().Y << " "  << nav_mesh_bounds->GetActorLocation().Z  << " " << std::endl;

    // std::cout << worldBox.GetSize().X << " " << worldBox.GetSize().Y << " "  << worldBox.GetSize().Z  << " " << std::endl;
    // std::cout << nav_mesh_bounds->GetActorRelativeScale3D().X << " " << nav_mesh_bounds->GetActorRelativeScale3D().Y << " "  << nav_mesh_bounds->GetActorRelativeScale3D().Z  << " " << std::endl;

    // nav_mesh_bounds->SetActorLocation(worldBox.GetCenter(), false);          // Place the navmesh at the center of the map
    // nav_mesh_bounds->SetActorRelativeScale3D(worldBox.GetSize() / 200.0f);   // Rescale the navmesh

    // nav_mesh_bounds->SetActorLocation(worldBox.GetCenter(), false);          // Place the navmesh at the center of the map
    // nav_mesh_bounds->SetActorRelativeScale3D(worldBox.GetSize() / 200.0f);   // Rescale the navmesh

    nav_mesh_bounds->GetRootComponent()->UpdateBounds();
    nav_sys->OnNavigationBoundsUpdated(nav_mesh_bounds);
    nav_mesh_bounds->GetRootComponent()->SetMobility(EComponentMobility::Static);

    nav_sys->Build(); // Rebuild NavMesh, required for update AgentRadius

    /*
    // spawn dummy navmesh volume bounds; UE bug
    FActorSpawnParameters spawn_params;
    spawn_params.Name = FName("DummyNavMeshBoundsVolume");
    spawn_params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    FVector spawn_location = FVector(0.f, 0.f, nav_mesh_bounds->GetComponentsBoundingBox(true, true).Min.Z - 1000.f);
    UE_LOG(LogTemp, Warning, TEXT("Spawning second navmesh at (%f, %f, %f)"), spawn_location.X, spawn_location.Y, spawn_location.Z);
    dummy_navmesh_bound_volume_ = GetWorld()->SpawnActor<ANavMeshBoundsVolume>(spawn_location, FRotator(0, 0, 0), spawn_params);
    dummy_navmesh_bound_volume_->BrushType = EBrushType::Brush_Add;
    dummy_navmesh_bound_volume_->GetRootComponent()->SetMobility(EComponentMobility::Movable);
    dummy_navmesh_bound_volume_->SetActorLocation(spawn_location, false);
    dummy_navmesh_bound_volume_->SetActorRelativeScale3D(worldBox.GetSize() / 200.0f);
    dummy_navmesh_bound_volume_->GetRootComponent()->UpdateBounds();
    nav_sys->OnNavigationBoundsUpdated(nav_mesh_bounds);
    nav_mesh_bounds->GetRootComponent()->SetMobility(EComponentMobility::Static);
    nav_sys->Build();
    FBox second_box = dummy_navmesh_bound_volume_->GetComponentsBoundingBox(true, true);
    */
    
    //UE_LOG(LogTemp, Warning, TEXT("after second navmesh rescale Min: (%f, %f, %f)"), second_box.Min.X, second_box.Min.Y, second_box.Min.Z);
    //UE_LOG(LogTemp, Warning, TEXT("after second navmesh rescale Max: (%f, %f, %f)"), second_box.Max.X, second_box.Max.Y, second_box.Max.Z);    

    //export navmesh obj file
    nav_mesh_->GetGenerator()->ExportNavigationData(FString("/Users/mroberts/Downloads/"));

    std::cout << "finished generating navmesh obj file" << std::endl;
}

FBox ADummyActor::getWorldBoundingBox(bool alter_height)
{
    FBox box(ForceInit);
    for (TActorIterator<AActor> it(GetWorld()); it; ++it) {
        if (it->ActorHasTag("floor") || it->ActorHasTag("asset")) {
            box += it->GetComponentsBoundingBox(false, true);
        }
    }

    // Remove ceiling
    UE_LOG(LogTemp, Warning, TEXT("Before: Box Min Vector is (%f, %f, %f)"), box.Min.X, box.Min.Y, box.Min.Z);
    UE_LOG(LogTemp, Warning, TEXT("Before: Box Max Vector is (%f, %f, %f)"), box.Max.X, box.Max.Y, box.Max.Z);
    const float height = 100.f;
    UE_LOG(LogTemp, Warning, TEXT("Max Vector used is (%f, %f, %f)"), box.Max.X, box.Max.Y, box.Min.Z + height);

    return alter_height ? FBox(box.Min, FVector(box.Max.X, box.Max.Y, box.Min.Z + height)) : box;
}