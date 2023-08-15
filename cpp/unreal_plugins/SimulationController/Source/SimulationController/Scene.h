//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <map>
#include <string>
#include <vector>

class AActor;
class UWorld;

class Scene
{
public:    
    Scene() = default;
    ~Scene() = default;

    void findObjectReferences(UWorld* world);
    void cleanUpObjectReferences();

    void setObjectLocations(std::map<std::string, std::vector<uint8_t>> object_locations);
    void setObjectRotations(std::map<std::string, std::vector<uint8_t>> object_rotations);

    std::vector<std::string> getAllObjectNames();
    std::vector<uint8_t> getObjectLocations(std::vector<std::string> object_names);
    std::vector<uint8_t> getObjectRotations(std::vector<std::string> object_names);
    std::map<std::string, std::vector<uint8_t>> getAllObjectLocations();
    std::map<std::string, std::vector<uint8_t>> getAllObjectRotations();
private:
    UWorld* world_ = nullptr;
    std::map<std::string, AActor*> all_actors_name_ref_map_;
};
