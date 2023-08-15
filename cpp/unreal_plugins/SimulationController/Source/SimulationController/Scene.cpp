//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SimulationController/Scene.h"

#include <map>
#include <string>
#include <vector>

#include <GameFramework/Actor.h>

#include "CoreUtils/Assert.h"
#include "CoreUtils/Unreal.h"

void Scene::findObjectReferences(UWorld* world)
{
    SP_ASSERT(world);
    world_ = world;

    all_actors_name_ref_map_ = Unreal::findActorsByTagAllAsMap(world_, {});
    SP_ASSERT(!all_actors_name_ref_map_.empty());
}

void Scene::cleanUpObjectReferences()
{
    all_actors_name_ref_map_.clear();
    world_ = nullptr;
}

void Scene::setObjectLocations(std::map<std::string, std::vector<uint8_t>> object_locations)
{
    for (auto& object_location: object_locations) {
        SP_ASSERT(all_actors_name_ref_map_.count(object_location.first));

        std::vector<double> location = Std::reinterpretAs<double>(object_location.second);
        SP_ASSERT(location.size() % 3 == 0);
        
        FVector location_fvector = {location.at(0), location.at(1), location.at(2)};
        bool sweep = false;
        FHitResult* hit_result = nullptr;
        bool success = all_actors_name_ref_map_.at(object_location.first)->SetActorLocation(location_fvector, sweep, hit_result, ETeleportType::ResetPhysics);
        SP_ASSERT(success);
    }
}

void Scene::setObjectRotations(std::map<std::string, std::vector<uint8_t>> object_rotations)
{
    for (auto& object_rotation: object_rotations) {
        SP_ASSERT(all_actors_name_ref_map_.count(object_rotation.first));
        
        std::vector<double> rotation = Std::reinterpretAs<double>(object_rotation.second);
        SP_ASSERT(rotation.size() % 3 == 0);
        
        FRotator rotation_frotator = {rotation.at(0), rotation.at(1), rotation.at(2)};
        bool success = all_actors_name_ref_map_.at(object_rotation.first)->SetActorRotation(rotation_frotator, ETeleportType::ResetPhysics);
        SP_ASSERT(success);
    }
}

std::vector<std::string> Scene::getAllObjectNames()
{
    std::vector<std::string> object_names;
    for(auto& element: all_actors_name_ref_map_) {
        object_names.push_back(element.first);
    }
    return object_names;
}

std::vector<uint8_t> Scene::getObjectLocations(std::vector<std::string> object_names)
{
    std::vector<uint8_t> object_locations;
    for(auto& object_name: object_names){
        SP_ASSERT(all_actors_name_ref_map_.count(object_name));
        FVector location = all_actors_name_ref_map_.at(object_name)->GetActorLocation();
        std::vector<uint8_t> object_location = Std::reinterpretAs<uint8_t>(std::vector<double>{location.X, location.Y, location.Z});
        object_locations.insert(object_locations.end(), object_location.begin(), object_location.end());
    }
    return object_locations;
}

std::vector<uint8_t> Scene::getObjectRotations(std::vector<std::string> object_names)
{
    std::vector<uint8_t> object_rotations;
    for(auto& object_name: object_names){
        SP_ASSERT(all_actors_name_ref_map_.count(object_name));
        FRotator rotation = all_actors_name_ref_map_.at(object_name)->GetActorRotation();
        std::vector<uint8_t> object_rotation = Std::reinterpretAs<uint8_t>(std::vector<double>{rotation.Pitch, rotation.Yaw, rotation.Roll});
        object_rotations.insert(object_rotations.end(), object_rotation.begin(), object_rotation.end());
    }
    return object_rotations;
}

std::map<std::string, std::vector<uint8_t>> Scene::getAllObjectLocations()
{
    std::map<std::string, std::vector<uint8_t>> object_locations;
    for(auto& element: all_actors_name_ref_map_) {
        FVector location = element.second->GetActorLocation();
        object_locations[element.first] = Std::reinterpretAs<uint8_t>(std::vector<double>{location.X, location.Y, location.Z});
    }
    return object_locations;
}

std::map<std::string, std::vector<uint8_t>> Scene::getAllObjectRotations()
{
    std::map<std::string, std::vector<uint8_t>> object_rotations;
    for(auto& element: all_actors_name_ref_map_) {
        FRotator rotation = element.second->GetActorRotation();
        object_rotations[element.first] = Std::reinterpretAs<uint8_t>(std::vector<double>{rotation.Pitch, rotation.Yaw, rotation.Roll});
    }
    return object_rotations;
}
