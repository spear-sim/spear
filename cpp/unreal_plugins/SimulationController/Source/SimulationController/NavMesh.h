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
    std::vector<uint8_t> getReachablePoints(const std::vector<std::vector<float>>& start_points);
    std::vector<std::vector<uint8_t>> getTrajectories(const std::vector<std::vector<float>>& start_points, const std::vector<std::vector<float>>& end_points);
private:
    ARecastNavMesh* recast_nav_mesh_ = nullptr;
    UNavigationSystemV1* navigation_system_v1_ = nullptr;
    UWorld* world_ = nullptr;
};
