//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Containers/Array.h>
#include <GameFramework/Actor.h>
#include <HAL/Platform.h> // uint64
#include <Math/Vector.h>
#include <UObject/ObjectMacros.h> // GENERATED_BODY, UCLASS, UFUNCTION, UPROPERTY

#include "SpNavMeshActor.generated.h"

class ARecastNavMesh;
class UNavigationSystemV1;

UCLASS(ClassGroup = "SPEAR", HideCategories = (Rendering, Replication, Collision, HLOD, Physics, Networking, Input, Actor, Cooking))
class ASpNavMeshActor : public AActor
{
    GENERATED_BODY()
public:
    ASpNavMeshActor();
    ~ASpNavMeshActor();

    // AActor interface
    void Tick(float delta_time) override;

    UFUNCTION()
    void setup(float agent_height, float agent_radius);

    UFUNCTION()
    TArray<FVector> getRandomPoints(int num_points);

    UFUNCTION()
    TArray<FVector> getPaths(const FVector& initial_point, const FVector& goal_point);

private:
    UNavigationSystemV1* navigation_system_v1_ = nullptr;
    ARecastNavMesh* recast_nav_mesh_           = nullptr;
};
