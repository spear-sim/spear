#pragma once

#include <CoreMinimal.h>
#include <Engine/EngineTypes.h>
#include <Kismet/KismetSystemLibrary.h>
#include <NavMesh/NavMeshBoundsVolume.h>
#include <NavMesh/RecastNavMesh.h>
#include <NavigationSystem.h>

#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Assert.h>
#include <Config.h>

struct Navigation {

    // Generate a random point in the navigable space of agent_actor
    static FVector generateRandomNavigablePoint(AActor* agent_actor);

    // Generate a random target position reachable by the agent from its initial position
    static FVector generateRandomReachableTargetPoint(AActor* agent_actor);

    // Generate a random target position reachable by the agent from its initial position in a given radius
    static FVector generateRandomReachableTargetPoint(AActor* agent_actor, float radius);

    // Generate a collision-free trajectory between an initial and a target location.
    // Returns true if successful.
    static bool generateTrajectoryToTarget(AActor* agent_actor, const FVector &initial_point, const FVector &target_point, TArray<FNavPathPoint> &path_points);

    // From a given initial position, generate a random reachable target point as well as a collision-free trajectory between them.
    // Returns true if successful.
    static bool generateTrajectoryToRandomTarget(AActor* agent_actor, const FVector &initial_point, TArray<FNavPathPoint> &path_points);

    // Generate a pair of random (initial point - reachable target point) as well as a collision-free trajectory between them.
    // Returns true if successful.
    static bool generateRandomTrajectory(AActor* agent_actor, TArray<FNavPathPoint> &path_points);

    // From a given initial position, generate a random reachable target point as well as a collision-free trajectory between them.
    // Multiple target points as well as the trajectories between the initial and target points are generated and evaluated. 
    // Only the best pair of target point / trajectory is kept.
    // Returns true if successful.
    static bool sampleTrajectoryToRandomTarget(AActor* agent_actor, const FVector &initial_point, TArray<FNavPathPoint> &path_points);

    // Generate a pair of random (initial point - reachable target point) as well as a collision-free trajectory between them.
    // Multiple pairs of (initial point - reachable target point) as well as trajectories between them are generated and evaluated. 
    // Only the best pair is kept.
    // Returns true if successful.
    static bool sampleRandomTrajectory(AActor* agent_actor, TArray<FNavPathPoint> &path_points);

    // Returns a first order approximation of the trajectory length.
    static float computeTrajectoryLength(AActor* agent_actor, const TArray<FNavPathPoint> &path_points);

    // Rebuild the navigation mesh with, with the properties stored in the .yaml parameter files
    static bool rebuildNavmesh(AActor* agent_actor);

    // Get the World bounding box dimensions
    static FBox getWorldBoundingBox(AActor* agent_actor, bool scale_ceiling = true);

    // Ensure the agent spawns on the ground surface
    static void traceGround(AActor* agent_actor, FVector& spawn_position, FRotator& spawn_rotator, const FVector& box_half_size);
};
