//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpCore/CppFuncData.h"

#include <initializer_list>
#include <map>
#include <span>
#include <string>
#include <utility> // std::make_pair
#include <vector>

#include "SpCore/Assert.h"
#include "SpCore/Std.h"

//
// getArgsFromData(...) is typically called when preparing to call a UCppFuncComponent function
//

std::map<std::string, CppFuncArg> CppFuncDataUtils::getArgsFromData(std::initializer_list<CppFuncDataBase*> data_objs)
{
    // convert from initializer_list to vector
    return getArgsFromData(Std::toVector<CppFuncDataBase*>(data_objs));
}

std::map<std::string, CppFuncArg> CppFuncDataUtils::getArgsFromData(const std::vector<CppFuncDataBase*>& data_objs)
{
    // convert from vector to map
    SP_ASSERT(!Std::contains(data_objs, nullptr));
    auto data_obj_map = Std::toMap<std::string, CppFuncDataBase*>(
        data_objs | std::views::transform([](auto data_obj) { return std::make_pair(data_obj->getName(), data_obj); }));
    return getArgsFromData(data_obj_map);
}

std::map<std::string, CppFuncArg> CppFuncDataUtils::getArgsFromData(const std::map<std::string, CppFuncDataBase*>& data_objs)
{
    // input is a map from names to pointers, output is a map from names to CppFuncArgs
    std::map<std::string, CppFuncArg> args;
    for (auto& [name, data_obj] : data_objs) {
        CppFuncArg arg;
        data_obj->setDataAsTargetFor(arg);
        Std::insert(args, name, std::move(arg));
    }
    return args;
}

//
// setDataFromArgs(...) is typically called from inside a UCppFuncComponent function to retrieve args
//

void CppFuncDataUtils::setDataFromArgs(std::initializer_list<CppFuncDataBase*> data_objs, const std::map<std::string, CppFuncArg>& args)
{
    // convert from initializer_list to vector
    setDataFromArgs(Std::toVector<CppFuncDataBase*>(data_objs), args);
}

void CppFuncDataUtils::setDataFromArgs(const std::vector<CppFuncDataBase*>& data_objs, const std::map<std::string, CppFuncArg>& args)
{
    // convert from span to map
    SP_ASSERT(!Std::contains(data_objs, nullptr));
    auto data_obj_map = Std::toMap<std::string, CppFuncDataBase*>(
        data_objs | std::views::transform([](auto data_obj) { return std::make_pair(data_obj->getName(), data_obj); }));
    setDataFromArgs(data_obj_map, args);
}

void CppFuncDataUtils::setDataFromArgs(const std::map<std::string, CppFuncDataBase*>& data_objs, const std::map<std::string, CppFuncArg>& args)
{
    for (auto& [name, data_obj] : data_objs) {
        SP_ASSERT(Std::containsKey(args, name));
        data_obj->setData(args.at(name));
    }
}

//
// getReturnValuesFromData(...) is typically called from inside a UCppFuncComponent function to set return values
//

std::map<std::string, CppFuncReturnValue> CppFuncDataUtils::getReturnValuesFromData(std::initializer_list<CppFuncDataBase*> data_objs)
{
    // convert from initializer_list to vector
    return getReturnValuesFromData(Std::toVector<CppFuncDataBase*>(data_objs));
}

std::map<std::string, CppFuncReturnValue> CppFuncDataUtils::getReturnValuesFromData(const std::vector<CppFuncDataBase*>& data_objs)
{
    // convert from vector to map
    SP_ASSERT(!Std::contains(data_objs, nullptr));
    auto data_obj_map = Std::toMap<std::string, CppFuncDataBase*>(
        data_objs | std::views::transform([](auto data_obj) { return std::make_pair(data_obj->getName(), data_obj); }));
    return getReturnValuesFromData(data_obj_map);
}

std::map<std::string, CppFuncReturnValue> CppFuncDataUtils::getReturnValuesFromData(const std::map<std::string, CppFuncDataBase*>& data_objs)
{
    // input is a map from names to pointers, output is a map from names to CppFuncReturnValues
    std::map<std::string, CppFuncReturnValue> return_values;
    for (auto& [name, data_obj] : data_objs) {
        CppFuncReturnValue return_value;
        data_obj->moveDataTo(return_value);
        Std::insert(return_values, name, std::move(return_value));
    }
    return return_values;
}

//
// setDataFromReturnValues(...) is typically called after returning from a UCppFuncComponent function
//

void CppFuncDataUtils::setDataFromReturnValues(std::initializer_list<CppFuncDataBase*> data_objs, const std::map<std::string, CppFuncReturnValue>& return_values)
{
    // convert from initializer_list to vector
    setDataFromReturnValues(Std::toVector<CppFuncDataBase*>(data_objs), return_values);
}

void CppFuncDataUtils::setDataFromReturnValues(const std::vector<CppFuncDataBase*>& data_objs, const std::map<std::string, CppFuncReturnValue>& return_values)
{
    // convert from vector to map
    SP_ASSERT(!Std::contains(data_objs, nullptr));
    auto data_obj_map = Std::toMap<std::string, CppFuncDataBase*>(
        data_objs | std::views::transform([](auto data_obj) { return std::make_pair(data_obj->getName(), data_obj); }));
    setDataFromReturnValues(data_obj_map, return_values);
}

void CppFuncDataUtils::setDataFromReturnValues(const std::map<std::string, CppFuncDataBase*>& data_objs, const std::map<std::string, CppFuncReturnValue>& return_values)
{
    for (auto& [name, data_obj] : data_objs) {
        SP_ASSERT(Std::containsKey(return_values, name));
        data_obj->setData(return_values.at(name));
    }
}
