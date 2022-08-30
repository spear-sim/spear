#pragma once

#include "CoreMinimal.h"

class NavMeshManager
{
public:
    static void exportData(UWorld* world);
    static bool navSystemRebuild(UWorld* world, float agentRadius);

private:
    static FBox getWorldBoundingBox(UWorld* world, bool bScaleCeiling = true);
    static bool isFlat(FVector& center, TArray<FVector>& outVerts);
};
