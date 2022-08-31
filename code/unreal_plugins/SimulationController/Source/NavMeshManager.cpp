#include "NavMeshManager.h"

#include "Kismet/KismetSystemLibrary.h"
#include "Kismet/GameplayStatics.h"

#include "AI/NavDataGenerator.h"
#include "EngineUtils.h"
#include "NavMesh/RecastNavMesh.h"
#include "NavMesh/NavMeshBoundsVolume.h"
#include "NavigationSystem.h"

#include <Config.h>

void NavMeshManager::exportData(UWorld* world)
{

    UNavigationSystemV1* navSys =
        FNavigationSystem::GetCurrent<UNavigationSystemV1>(world);

    auto navData = navSys->GetMainNavData();
    ARecastNavMesh* navMesh = Cast<ARecastNavMesh>(navData);

    FBox box = getWorldBoundingBox(world);
    TArray<FNavPoly> polys;
    navMesh->GetPolysInBox(box, polys);

    TArray<FString> lines;
    lines.Add(box.GetCenter().ToString());
    for (auto& poly : polys)
    {
        FVector center;
        navMesh->GetPolyCenter(poly.Ref, center);
        TArray<FVector> outVerts;
        if (navMesh->GetPolyVerts(poly.Ref, outVerts))
        {
            FString line = "";
            bool start = true;
            // line.Append(center.ToString());
            for (auto& vert : outVerts)
            {
                if (start)
                {
                    start = false;
                }
                else
                {
                    line.Append(" , ");
                }
                line.Append(vert.ToString());
            }
            lines.Add(line);
        }
    }

    FString fileName = Config::getValue<std::string>({ "SIMULATION_CONTROLLER", "IMAGE_SAMPLING_AGENT_CONTROLLER", "DEBUG_POSES_DIR" }).c_str() + world->GetName() +"/" + world->GetName() + ".csv";
    FFileHelper::SaveStringArrayToFile(
        lines, *fileName, FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM);
}

FBox NavMeshManager::getWorldBoundingBox(UWorld* world, bool bScaleCeiling)
{
    FBox box(ForceInit);
    for (TActorIterator<AActor> it(world); it; ++it)
    {
        if (it->ActorHasTag("architecture") || it->ActorHasTag("furniture"))
        {
            box += it->GetComponentsBoundingBox(false, true);
        }
    }
    // remove ceiling
    return !bScaleCeiling
               ? box
               : box.ExpandBy(box.GetSize() * 0.1f)
                     .ShiftBy(FVector(0, 0, -0.3f * box.GetSize().Z));
}

// check all vert on the same plane, might not guarentee flat?
bool NavMeshManager::isFlat(FVector& center, TArray<FVector>& OutVerts)
{
    for (auto vert : OutVerts)
    {
        if (center.Z != vert.Z)
        {
            return false;
        }
    }
    return true;
}

bool NavMeshManager::navSystemRebuild(UWorld* world, float agentRadius)
{
    UNavigationSystemV1* navSys =
        FNavigationSystem::GetCurrent<UNavigationSystemV1>(world);
    if (!navSys)
    {
        return false;
    }
    auto navData = navSys->GetMainNavData();
    ARecastNavMesh* navMesh = Cast<ARecastNavMesh>(navData);
    if (!navMesh)
    {
        return false;
    }

    ANavMeshBoundsVolume* navmeshBounds = nullptr;
    for (TActorIterator<ANavMeshBoundsVolume> it(world); it; ++it)
    {
        navmeshBounds = *it;
    }
    if (navmeshBounds == nullptr)
    {
        return false;
    }

    navMesh->AgentRadius = agentRadius;
    navMesh->AgentHeight = 100;
    navMesh->CellSize = 10;
    navMesh->CellHeight = 10;
    navMesh->AgentMaxSlope = 0.1f;
    navMesh->AgentMaxStepHeight = 0.1f;
    navMesh->MergeRegionSize = 0;
    // ignore region that are too small
    navMesh->MinRegionArea = 400;
    // navMesh->MaxSimplificationError = 1.3;

    // dynamic update navMesh location and size
    navmeshBounds->GetRootComponent()->SetMobility(EComponentMobility::Movable);

    FBox worldBox = getWorldBoundingBox(world);
    navmeshBounds->SetActorLocation(worldBox.GetCenter(), false);
    navmeshBounds->SetActorRelativeScale3D(worldBox.GetSize() / 200.0f);
    navmeshBounds->SetActorRelativeScale3D(FVector(20, 20, 1.5));
    navmeshBounds->GetRootComponent()->UpdateBounds();
    // NavmeshBounds->SupportedAgents.bSupportsAgent0;
    navSys->OnNavigationBoundsUpdated(navmeshBounds);
    // redo modify frequency change
    navmeshBounds->GetRootComponent()->SetMobility(EComponentMobility::Static);

    // rebuild NavMesh, required for update AgentRadius
    navSys->Build();
    return true;
}
