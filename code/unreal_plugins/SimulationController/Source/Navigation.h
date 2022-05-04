#pragma once

#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <iostream>

#include <Engine/EngineTypes.h>
#include <NavMesh/NavMeshBoundsVolume.h>
#include <NavMesh/RecastNavMesh.h>
#include <NavigationSystem.h>

#include "Assert.h"
#include "Config.h"

class Navigation {
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

    UNavigationSystemV1* navSys_;
    ANavigationData* navData_;
    ARecastNavMesh* navMesh_;
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
