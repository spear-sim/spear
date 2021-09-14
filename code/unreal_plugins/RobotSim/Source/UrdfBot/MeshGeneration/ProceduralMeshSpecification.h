#pragma once

#include "CoreMinimal.h"

class ProceduralMeshSpecification
{
    public:
        TArray<FVector> Verticies;
        TArray<int32> Triangles;
        bool ParseSuccessful;
};
