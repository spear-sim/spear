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
    /**
     * @brief Delete the copy constructor in singleton class
     *
     */
    Navigation(const Navigation&) = delete;

    /**
     * @brief Access the singleton navigation object
     *
     * @param agent_actor
     * @return Navigation&
     */
    static Navigation& Singleton(AActor* agent_actor)
    {
        static Navigation navInstance(agent_actor);
        return navInstance;
    }

    /**
     * @brief Destroy the singleton navigation object
     *
     */
    virtual ~Navigation();

    /**
     * @brief Generate an random initial position for the agent
     *
     * @return FVector
     */
    FVector generateRandomInitialPosition();

    /**
     * @brief Use the yaml parameter system to store a starting point which can be changed at every run in python (hack to be removed)
     *
     * @return FVector
     */
    FVector getPredefinedInitialPosition();

    /**
     * @brief Use the yaml parameter system to store a goal point which can be changed at every run in python (hack to be removed)
     *
     * @return FVector
     */
    FVector getPredefinedGoalPosition();

    /**
     * @brief From the generated initial position, generate a random reachale target point and a collision-free trajectory between them.
     *
     */
    void generateTrajectoryToRandomTarget();

    /**
     * @brief From the generated initial and final positions, generate a collision-free trajectory between them.
     *
     */
    void generateTrajectoryToPredefinedTarget();

    /**
     * @brief Reset the navigation object. This allows regenerating a navmesh and changing its properties.
     *
     */
    void reset();

    /**
     * @brief Get the trajectory point at a given index
     *
     * @param index
     * @return FVector2D
     */
    FVector2D getPathPoint(size_t index);

    /**
     * @brief Get the current trajectory point (does not update the waypoint based on the agent location)
     *
     * @return FVector2D
     */
    FVector2D getCurrentPathPoint();

    /**
     * @brief Returns the updated waypoint based on the agent location
     *
     * @param relative_position_to_goal
     * @return FVector2D
     */
    FVector2D update();

    /**
     * @brief Returns a first order appromination of the desired trajectory length
     *
     * @return float
     */
    inline float getTrajectoryLength()
    {
        return trajectory_length_;
    }

    /**
     * @brief Get the goal position
     *
     * @param relative_position_to_goal
     * @return FVector2D
     */
    inline FVector getGoal()
    {
        return FVector(path_points_.Last().Location.X, path_points_.Last().Location.Y, path_points_.Last().Location.Z);
    }

    /**
     * @brief Returns true if the goal has been reached (with a tolerance SIMULATION_CONTROLLER.NAVIGATION.ACCEPTANCE_RADIUS)
     *
     * @return true
     * @return false
     */
    inline bool goalReached()
    {
        return target_reached_;
    }

    /**
     * @brief DIRTY hack for neurips
     *
     */
    inline void iterateIndex()
    {
        execution_counter_++;
    }

private:
    /**
     * @brief Private constructor for the singleton design template
     *
     */
    Navigation(AActor* agent_actor);

    /**
     * @brief
     *
     * @return true
     * @return false
     */
    bool rebuild();

    /**
     * @brief Get the World Bounding Box object
     *
     * @param scale_ceiling
     * @return FBox
     */
    FBox getWorldBoundingBox(bool scale_ceiling = true);

    /**
     * @brief Ensure the agent spawns on the ground surface
     *
     * @param spawn_position
     * @param spawn_rotator
     * @param box_half_size
     */
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
