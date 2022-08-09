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
    // Delete the copy constructor in singleton class
    Navigation(const Navigation&) = delete;

    // Access the singleton navigation object
    static Navigation& Singleton(AActor* agent_actor)
    {
        static Navigation navInstance(agent_actor);
        return navInstance;
    }

    // Destroy the singleton navigation object
    virtual ~Navigation();

    // Generate an random initial position for the agent
    FVector generateRandomInitialPosition();

    // se the yaml parameter system to store a starting point which can be changed at every run in python (hack to be removed)
    FVector getPredefinedInitialPosition();

    // Use the yaml parameter system to store a goal point which can be changed at every run in python (hack to be removed)
    FVector getPredefinedGoalPosition();

    // From the generated initial position, generate a random reachale target point and a collision-free trajectory between them.
    void generateTrajectoryToRandomTarget();

    // From the generated initial and final positions, generate a collision-free trajectory between them.
    void generateTrajectoryToPredefinedTarget();

    // Reset the navigation object. This allows regenerating a navmesh and changing its properties.
    void reset();

    // Get the trajectory point at a given index
    FVector2D getPathPoint(size_t index);

    // Get the current trajectory point (does not update the waypoint based on the agent location)
    FVector2D getCurrentPathPoint();

    // Returns the updated waypoint based on the agent location
    FVector2D update();

    // Returns a first order appromination of the desired trajectory length
    inline float getTrajectoryLength()
    {
        return trajectory_length_;
    }

    // Get the goal position
    inline FVector getGoal()
    {
        return FVector(path_points_.Last().Location.X, path_points_.Last().Location.Y, path_points_.Last().Location.Z);
    }

    // Returns true if the goal has been reached (with a tolerance SIMULATION_CONTROLLER.NAVIGATION.ACCEPTANCE_RADIUS)
    inline bool goalReached()
    {
        return target_reached_;
    }

    // DIRTY hack for neurips
    inline void iterateIndex()
    {
        execution_counter_++;
    }

private:
    // Private constructor for the singleton design template
    Navigation(AActor* agent_actor);

    //
    bool rebuild();

    // Get the World Bounding Box object
    FBox getWorldBoundingBox(bool scale_ceiling = true);

    // Ensure the agent spawns on the ground surface
    void traceGround(FVector& spawn_position, FRotator& spawn_rotator, const FVector& box_half_size);

    UNavigationSystemV1* nav_sys_;
    ANavigationData* nav_data_;
    ARecastNavMesh* nav_mesh_;
    AActor* agent_actor_ = nullptr;

    int number_iterations_ = 0;
    FVector initial_position_;
    FNavLocation best_target_location_;
    FPathFindingQuery nav_query_;

    // An array containing the different waypoints to be followed by the agent:
    TArray<FNavPathPoint> path_points_;

    // The path point begin considered by the PID controller:
    FNavLocation target_location_;

    // Index of the considered path point:
    size_t index_path_;

    bool target_reached_ = false;

    int execution_counter_ = 0;

    float trajectory_length_ = 0.0;
};
