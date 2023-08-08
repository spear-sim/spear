//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <map>
#include <string>
#include <vector>

class AActor;
class UWorld;

class SceneManager
{
public:    
    SceneManager() = default;
    ~SceneManager() = default;

    void findObjectReferences(UWorld* world);
    void cleanUpObjectReferences();

    void setObjectLocations(std::map<std::string, std::vector<double>> object_locations);
    void setObjectRotations(std::map<std::string, std::vector<double>> object_rotations);
private:
    UWorld* world_ = nullptr;
    std::map<std::string, AActor*> all_actors_name_map_;
};
