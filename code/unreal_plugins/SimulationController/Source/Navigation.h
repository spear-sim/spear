#pragma once

#include <algorithm>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include <Engine/EngineTypes.h>
#include <NavMesh/NavMeshBoundsVolume.h>
#include <NavMesh/RecastNavMesh.h>
#include <NavigationSystem.h>

class Navigation
{
public:
    Navigation(/* args */);
    virtual ~Navigation();

    void generateReachableGoal();

    void generateTrajectory();

private:

    UNavigationSystemV1* navSys_;
    ANavigationData* navData_;

    int numberOfWayPoints_ = 0;
    int numIter_ = 0;
    float pathCriterion_ = 0.f;
    FVector initialPosition_;
    FNavLocation bestTargetLocation_;

    // An array containing the different waypoints to be followed by the agent:
    static TArray<FNavPathPoint> pathPoints_;

    // The path point begin considered by the PID controller:
    static FVector2D currentPathPoint_;
    static FNavLocation targetLocation_;

    // Index of the considered path point (starts at one since 0 is the initial position):
    static unsigned int indexPath_ = 1;
};
