//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SimulationController/SceneManager.h"

#include <map>
#include <string>
#include <vector>

#include <GameFramework/Actor.h>

#include "CoreUtils/Assert.h"
#include "CoreUtils/Config.h"
#include "CoreUtils/Log.h"
#include "CoreUtils/Unreal.h"

void SceneManager::findObjectReferences(UWorld* world)
{
    SP_ASSERT(world);
    world_ = world;

    SP_LOG("----------------------------------------------------------------------------------------");
    all_actors_name_map_ = Unreal::findActorsByTagAllAsMap(world_, {});
    for(const auto& actor: all_actors_name_map_) {
        SP_LOG("Name of the actor is ", actor.first);
    }
    SP_LOG("----------------------------------------------------------------------------------------");
}

void SceneManager::cleanUpObjectReferences()
{
    all_actors_name_map_.clear();
    world_ = nullptr;
}

void SceneManager::setObjectLocations(std::map<std::string, std::vector<double>> object_locations)
{

}

void SceneManager::setObjectRotations(std::map<std::string, std::vector<double>> object_rotations)
{

}
