#pragma once

#include <CoreMinimal.h>
#include <Engine/EngineTypes.h>
#include <NavMesh/NavMeshBoundsVolume.h>
#include <NavMesh/RecastNavMesh.h>
#include <NavigationSystem.h>
#include "Kismet/KismetSystemLibrary.h"

#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "Assert.h"
#include "Config.h"

class Navigation
{
public:
    /**
     * @brief Delete the copy constructor in singleton class
     *
     */
    Navigation(const Navigation&) = delete;

    /**
     * @brief Access the singleton navigation object
     *
     * @param pawnAgent
     * @return Navigation&
     */
    static Navigation& Singleton(APawn* pawnAgent)
    {
        static Navigation navInstance(pawnAgent);
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
    void resetNavigation();

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
    FVector2D updateNavigation();

    /**
     * @brief Get the goal position 
     *
     * @param relative_position_to_goal
     * @return FVector2D
     */
    inline FVector getGoal()
    {
        return FVector(pathPoints_.Last().Location.X, pathPoints_.Last().Location.Y, pathPoints_.Last().Location.Z);
    }

    /**
     * @brief Returns true if the goal has been reached (with a tolerance SIMULATION_CONTROLLER.NAVIGATION.ACCEPTANCE_RADIUS)
     *
     * @return true
     * @return false
     */
    inline bool goalReached()
    {
        return targetReached_;
    }

    /**
     * @brief DIRTY hack for neurips
     * 
     */
    inline void iterateIndex()
    {
        executionCounter_++;
    }

private:
    /**
     * @brief Private constructor for the singleton design template
     *
     */
    Navigation(APawn* pawnAgent);

    /**
     * @brief
     *
     * @return true
     * @return false
     */
    bool navSystemRebuild();

    /**
     * @brief Get the World Bounding Box object
     *
     * @param bScaleCeiling
     * @return FBox
     */
    FBox GetWorldBoundingBox(bool bScaleCeiling = true);

    /**
     * @brief Ensure the agent spawns on the ground surface 
     *
     * @param spawnPosition
     * @param spawnRotator
     * @param boxHalfSize
     */
    void traceGround(FVector& spawnPosition, FRotator& spawnRotator, const FVector& boxHalfSize);

    UNavigationSystemV1* navSys_;
    ANavigationData* navData_;
    ARecastNavMesh* navMesh_;
    ANavMeshBoundsVolume* navmeshBounds_;
    const APawn* pawnAgent_;

    int numberOfWayPoints_ = 0;
    int numIter_ = 0;
    float pathCriterion_ = 0.f;
    FVector initialPosition_;
    FNavLocation bestTargetLocation_;
    FPathFindingQuery navQuery_;

    // An array containing the different waypoints to be followed by the agent:
    TArray<FNavPathPoint> pathPoints_;

    // The path point begin considered by the PID controller:
    FNavLocation targetLocation_;

    // Index of the considered path point:
    size_t indexPath_;

    bool targetReached_ = false;

    int executionCounter_ = 0;
};
