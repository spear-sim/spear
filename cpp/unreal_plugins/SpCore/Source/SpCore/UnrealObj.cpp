//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpCore/UnrealObj.h"

#include <initializer_list>
#include <map>
#include <string>
#include <utility> // std::make_pair
#include <vector>

#include "SpCore/Assert.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"

std::map<std::string, std::string> UnrealObjUtils::getObjectPropertiesAsStrings(std::initializer_list<UnrealObjBase*> unreal_objs)
{
    // convert from initializer_list to vector
    return getObjectPropertiesAsStrings(Std::toVector<UnrealObjBase*>(unreal_objs));
}

std::map<std::string, std::string> UnrealObjUtils::getObjectPropertiesAsStrings(const std::vector<UnrealObjBase*>& unreal_objs)
{
    // convert from vector to map
    SP_ASSERT(!Std::contains(unreal_objs, nullptr));
    auto unreal_obj_map = Std::toMap<std::string, UnrealObjBase*>(
        unreal_objs | std::views::transform([](auto unreal_obj) { return std::make_pair(unreal_obj->getName(), unreal_obj); }));
    return getObjectPropertiesAsStrings(unreal_obj_map);
}

std::map<std::string, std::string> UnrealObjUtils::getObjectPropertiesAsStrings(const std::map<std::string, UnrealObjBase*>& unreal_objs)
{
    // input is a map from names to pointers, output is a map from names to property strings
    return Std::toMap<std::string, std::string>(
        unreal_objs |
        std::views::transform([](auto& pair) {
            auto& [name, unreal_obj] = pair;
            SP_ASSERT(unreal_obj);
            return std::make_pair(name, Unreal::getObjectPropertiesAsString(unreal_obj->getValuePtr(), unreal_obj->getStaticStruct()));
        }));
}

void UnrealObjUtils::setObjectPropertiesFromStrings(std::initializer_list<UnrealObjBase*> unreal_objs, const std::map<std::string, std::string>& strings)
{
    // convert from initializer_list to vector
    setObjectPropertiesFromStrings(Std::toVector<UnrealObjBase*>(unreal_objs), strings);
}

void UnrealObjUtils::setObjectPropertiesFromStrings(const std::vector<UnrealObjBase*>& unreal_objs, const std::map<std::string, std::string>& strings)
{
    // convert from vector to map
    SP_ASSERT(!Std::contains(unreal_objs, nullptr));
    auto unreal_obj_map = Std::toMap<std::string, UnrealObjBase*>(
        unreal_objs | std::views::transform([](auto unreal_obj) { return std::make_pair(unreal_obj->getName(), unreal_obj); }));
    setObjectPropertiesFromStrings(unreal_obj_map, strings);
}

void UnrealObjUtils::setObjectPropertiesFromStrings(const std::map<std::string, UnrealObjBase*>& unreal_objs, const std::map<std::string, std::string>& strings)
{
    for (auto& [name, unreal_obj] : unreal_objs) {
        SP_ASSERT(unreal_obj);
        SP_ASSERT(Std::containsKey(strings, name));
        Unreal::setObjectPropertiesFromString(unreal_obj->getValuePtr(), unreal_obj->getStaticStruct(), strings.at(name));
    }
}
