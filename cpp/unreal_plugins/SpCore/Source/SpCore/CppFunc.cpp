//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpCore/CppFunc.h"

#include <initializer_list>
#include <map>
#include <string>
#include <utility> // std::make_pair
#include <vector>

#include "SpCore/Assert.h"
#include "SpCore/Std.h"

//
// typically called before calling a CppFunc to set args, and from inside a CppFunc to set return values
//

std::map<std::string, CppFuncItem> CppFuncUtils::moveDataToItems(std::initializer_list<CppFuncDataBase*> data_objs)
{
    // convert from initializer_list to vector
    return moveDataToItems(Std::toVector<CppFuncDataBase*>(data_objs));
}

std::map<std::string, CppFuncItem> CppFuncUtils::moveDataToItems(const std::vector<CppFuncDataBase*>& data_objs)
{
    // convert from vector to map
    SP_ASSERT(!Std::contains(data_objs, nullptr));
    auto data_obj_map = Std::toMap<std::string, CppFuncDataBase*>(
        data_objs | std::views::transform([](auto data_obj) { return std::make_pair(data_obj->getName(), data_obj); }));
    return moveDataToItems(data_obj_map);
}

std::map<std::string, CppFuncItem> CppFuncUtils::moveDataToItems(const std::map<std::string, CppFuncDataBase*>& data_objs)
{
    // input is a map from names to pointers, output is a map from names to CppFuncArgs
    std::map<std::string, CppFuncItem> items;
    for (auto& [name, data_obj] : data_objs) {
        CppFuncItem item;
        data_obj->moveDataToItem(item);
        Std::insert(items, name, std::move(item));
    }
    return items;
}

//
// typically called from inside a CppFunc to retrieve args, and after calling a CppFunc to retrieve return values
//

void CppFuncUtils::setViewsFromItems(std::initializer_list<CppFuncViewBase*> view_objs, const std::map<std::string, CppFuncItem>& items)
{
    // convert from initializer_list to vector
    setViewsFromItems(Std::toVector<CppFuncViewBase*>(view_objs), items);
}

void CppFuncUtils::setViewsFromItems(const std::vector<CppFuncViewBase*>& view_objs, const std::map<std::string, CppFuncItem>& items)
{
    // convert from vector to map
    SP_ASSERT(!Std::contains(view_objs, nullptr));
    auto view_obj_map = Std::toMap<std::string, CppFuncViewBase*>(
        view_objs | std::views::transform([](auto view_obj) { return std::make_pair(view_obj->getName(), view_obj); }));
    setViewsFromItems(view_obj_map, items);
}

void CppFuncUtils::setViewsFromItems(const std::map<std::string, CppFuncViewBase*>& view_objs, const std::map<std::string, CppFuncItem>& items)
{
    for (auto& [name, view_obj] : view_objs) {
        SP_ASSERT(Std::containsKey(items, name));
        view_obj->setViewFromItem(items.at(name));
    }
}

//
// useful for transferring an arg or a return value item to a local data object
//

void CppFuncUtils::moveItemsToData(std::initializer_list<CppFuncDataBase*> data_objs, std::map<std::string, CppFuncItem>& items)
{
    // convert from initializer_list to vector
    moveItemsToData(Std::toVector<CppFuncDataBase*>(data_objs), items);
}

void CppFuncUtils::moveItemsToData(const std::vector<CppFuncDataBase*>& data_objs, std::map<std::string, CppFuncItem>& items)
{
    // convert from vector to map
    SP_ASSERT(!Std::contains(data_objs, nullptr));
    auto data_obj_map = Std::toMap<std::string, CppFuncDataBase*>(
        data_objs | std::views::transform([](auto data_obj) { return std::make_pair(data_obj->getName(), data_obj); }));
    moveItemsToData(data_obj_map, items);
}

void CppFuncUtils::moveItemsToData(const std::map<std::string, CppFuncDataBase*>& data_objs, std::map<std::string, CppFuncItem>& items)
{
    for (auto& [name, data_obj] : data_objs) {
        SP_ASSERT(Std::containsKey(items, name));
        data_obj->moveItemToData(items.at(name));
    }
}
