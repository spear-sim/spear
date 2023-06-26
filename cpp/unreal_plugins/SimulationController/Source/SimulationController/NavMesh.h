//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <vector>

class ARecastNavMesh;
class UNavigationSystemV1;
class UWorld;

class NavMesh
{
public:    
    NavMesh() = default;
    ~NavMesh() = default;

    void findObjectReferences(UWorld* world);
    void cleanUpObjectReferences();

    std::vector<uint8_t> getRandomPoints(int num_points);
    std::vector<uint8_t> getTrajectoryBetweenTwoPoints(const std::vector<float>& start_point, const std::vector<float>& end_point);

    // Generate a pair of random (initial point, reachable goal point) as well as a collision-free trajectory between them.
    // Multiple pairs of (initial point, reachable target point) as well as trajectories between them are generated and evaluated.
    // Only the best pair is kept.
    void getPositionsFromTrajectorySampling();

private:
    ARecastNavMesh* recast_nav_mesh_ = nullptr;
    UNavigationSystemV1* navigation_system_v1_ = nullptr;
    UWorld* world_ = nullptr;
};
