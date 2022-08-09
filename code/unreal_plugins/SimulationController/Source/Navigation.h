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

class Navigation {
public:
    // Build the navigation object
    Navigation(AActor* agent_actor);

    // Destroy the navigation object
    virtual ~Navigation();

    // Generate an random initial position for the agent
    static FVector generateRandomInitialPosition();

    // Set the initial agent position from a data file
    static void setInitialPosition(const FVector& initial_position);

    // Set the goal position from a data file
    static void setGoalPosition(const FVector& goal_position);

    // From the generated initial position, generate a random reachale target point and a collision-free trajectory between them.
    static void generateTrajectoryToRandomTarget();

    // From the generated initial and final positions, generate a collision-free trajectory between them.
    static void generateTrajectoryToPredefinedTarget();

    // Reset the navigation object. This allows regenerating a navmesh and changing its properties.
    static void reset();

    // Get the trajectory point at a given index
    static FVector2D getPathPoint(size_t index);

    // Get the current trajectory point (does not update the waypoint based on the agent location)
    static FVector2D getCurrentPathPoint();

    // Returns the updated waypoint based on the agent location
    static FVector2D update();

    // Returns a first order appromination of the desired trajectory length
    static inline float getTrajectoryLength()
    {
        return Navigation::trajectory_length_;
    }

    // Get the goal position
    static inline FVector getGoal()
    {
        return FVector(Navigation::path_points_.Last().Location.X, Navigation::path_points_.Last().Location.Y, Navigation::path_points_.Last().Location.Z);
    }

    // Returns true if the goal has been reached (with a tolerance SIMULATION_CONTROLLER.NAVIGATION.ACCEPTANCE_RADIUS)
    static inline bool goalReached()
    {
        return target_reached_;
    }

private:
    
    // Rebuild the navigation mesh with, with the properties stored in the .yaml parameter files
    static bool rebuild();

    // Get the World bounding box dimensions
    static FBox getWorldBoundingBox(bool scale_ceiling = true);

    // Ensure the agent spawns on the ground surface
    static void traceGround(FVector& spawn_position, FRotator& spawn_rotator, const FVector& box_half_size);

    static UNavigationSystemV1* nav_sys_;
    static ANavigationData* nav_data_;
    static ARecastNavMesh* nav_mesh_;
    static AActor* agent_actor_;

    static int number_iterations_;
    static FNavLocation best_target_location_;
    static FPathFindingQuery nav_query_;

    static FVector initial_position_;

    // An array containing the different waypoints to be followed by the agent:
    static TArray<FNavPathPoint> path_points_;

    // The path point begin considered by the PID controller:
    static FNavLocation target_location_;

    // Index of the considered path point:
    static size_t index_path_;

    static bool target_reached_;

    static float trajectory_length_;
};
