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
    std::string file = "E:/new_isim_images/NavMeshDebugProject/poses_for_debug.txt";
    //UE_LOG(LogTemp, Warning, TEXT("printing filename for storing debug poses %s"), UTF8_TO_TCHAR(file.c_str()));
    myfile.open(file);
    myfile << "pos_x_cm,pos_y_cm,pos_z_cm\n";
    for (size_t i = 0u; i < 1000000u; ++i) {
        FVector random_position = nav_mesh_->GetRandomPoint().Location;
        myfile << random_position.X << "," << random_position.Y << "," << random_position.Z << "\n";
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

    FNavAgentProperties agent_properties;
    agent_properties.AgentHeight = 150.f;
    agent_properties.AgentRadius = 5.f;
    agent_properties.AgentStepHeight = 1.f;

    ANavigationData* nav_data = nav_sys->GetNavDataForProps(agent_properties);
    //auto nav_data = nav_sys->GetMainNavData();
    check(nav_data);
    
    nav_mesh_ = Cast<ARecastNavMesh>(nav_data);
    check(nav_mesh_);

    ANavMeshBoundsVolume* nav_mesh_bounds = nullptr;
    for (TActorIterator<ANavMeshBoundsVolume> it(GetWorld()); it; ++it) {
        nav_mesh_bounds = *it;
    }
    check(nav_mesh_bounds);

    // Set the NavMesh properties:
    nav_mesh_->CellSize = 1.0f;
    nav_mesh_->CellHeight = 1.0f;
    nav_mesh_->MergeRegionSize = 0.0f;
    nav_mesh_->MinRegionArea = 100.0f;
    nav_mesh_->AgentMaxStepHeight = 1.0f;
    nav_mesh_->AgentMaxSlope = 1.0f;
    nav_mesh_->TileSizeUU = 300.0f;
    nav_mesh_->AgentRadius = 5.0f;
    nav_mesh_->AgentHeight = 150.0f;
    // Dynamic update navMesh location and size

    nav_mesh_bounds->GetRootComponent()->SetMobility(EComponentMobility::Movable);
    FBox worldBox = getWorldBoundingBox(false);
    // std::cout << "worldbox center (X,Y,Z) : (" << worldBox.GetCenter().X << worldBox.GetCenter().Y << worldBox.GetCenter().Z << ")" << std::endl;
    // UE_LOG(LogTemp, Warning, TEXT("worldbox center (X,Y,Z) : (%f, %f, %f)"), worldBox.GetCenter().X, worldBox.GetCenter().Y, worldBox.GetCenter().Z);
    nav_mesh_bounds->SetActorLocation(worldBox.GetCenter(), false);          // Place the navmesh at the center of the map
    nav_mesh_bounds->SetActorRelativeScale3D(worldBox.GetSize() / 200.0f);   // Rescale the navmesh
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
    nav_mesh_->GetGenerator()->ExportNavigationData(FString("E:/new_isim_images/NavMeshDebugProject/"));
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