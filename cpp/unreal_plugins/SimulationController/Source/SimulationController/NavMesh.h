//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <vector>

class ARecastNavMesh;
class UWorld;

class NavMesh
{
public:    
    NavMesh() = default;
    ~NavMesh() = default;

    void findObjectReferences(UWorld* world);
    void cleanUpObjectReferences();

    std::vector<uint8_t> getRandomPoints(int num_points);

private:
    ARecastNavMesh* recast_nav_mesh_ = nullptr;
};
