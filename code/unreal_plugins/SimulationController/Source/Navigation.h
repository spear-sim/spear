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
     * @brief
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
     * @brief Destroy the Navigation object
     *
     */
    virtual ~Navigation();

    /**
     * @brief
     *
     * @return FVector
     */
    FVector generateRandomInitialPosition();

    /**
     * @brief
     *
     */
    void generateTrajectory();

    /**
     * @brief 
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
     * @brief Get the current trajectory point
     *
     * @return FVector2D
     */
    FVector2D getCurrentPathPoint();

    /**
     * @brief Get the next trajectory point
     *
     * @return FVector2D
     */
    FVector2D getNextPathPoint();

    /**
     * @brief Returns the updated waypoint based on the agent location
     *
     * @param relative_position_to_goal
     * @return FVector2D
     */
    FVector2D updateNavigation();

    /**
     * @brief Returns the updated waypoint based on the agent location
     *
     * @param relative_position_to_goal
     * @return FVector2D
     */
    inline FVector getGoal()
    {
        return FVector(pathPoints_.Last().Location.X, pathPoints_.Last().Location.Y, pathPoints_.Last().Location.Z);
    }

    /**
     * @brief
     *
     * @return true
     * @return false
     */
    inline bool goalReached()
    {
        return targetReached_;
    }

private:
    /**
     * @brief Private Singleton constructor
     *
     */
    Navigation(APawn* pawnAgent);

    /**
     * @brief
     *
     * @param AgentRadius
     * @return true
     * @return false
     */
    bool navSystemRebuild(float AgentRadius);

    /**
     * @brief Get the World Bounding Box object
     *
     * @param bScaleCeiling
     * @return FBox
     */
    FBox GetWorldBoundingBox(bool bScaleCeiling = true);

    /**
     * @brief
     *
     * @param spawnPosition
     * @param spawnRotator
     * @param boxHalfSize
     */
    void traceGround(FVector& spawnPosition, FRotator& spawnRotator, const FVector& boxHalfSize);

    UNavigationSystemV1* navSys_;
    INavigationDataInterface* navDataInterface_;
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
};
