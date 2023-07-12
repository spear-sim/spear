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

    std::vector<double> getRandomPoints(int num_points);
    std::vector<double> getRandomReachablePointsInRadius(const std::vector<double>& initial_points, float radius);
    std::vector<std::vector<double>> getPaths(const std::vector<double>& initial_points, const std::vector<double>& goal_points);
private:
    UWorld* world_ = nullptr;
    UNavigationSystemV1* navigation_system_v1_ = nullptr;
    ARecastNavMesh* recast_nav_mesh_ = nullptr;
};
