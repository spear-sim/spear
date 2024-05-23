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
// CppFuncItem
//

void CppFuncItem::resolveArg(const CppFuncSharedMemoryView* shared_memory_view)
{
    // validate internal state
    SP_ASSERT(!view_);
    SP_ASSERT(data_type_ != CppFuncDataType::Invalid);

    if (use_shared_memory_) {
        // validate internal state
        SP_ASSERT(data_.empty());
        SP_ASSERT(num_elements_ >= 0);
        SP_ASSERT(data_type_ != CppFuncDataType::Invalid);
        SP_ASSERT(shared_memory_name_ != "");

        // validate shared memory state
        SP_ASSERT(shared_memory_view);
        SP_ASSERT(shared_memory_view->data_);
        SP_ASSERT(shared_memory_view->usage_flags_ & CppFuncSharedMemoryUsageFlags::Arg);

        // validate consistency of internal state and shared memory state
        SP_ASSERT(num_elements_*CppFuncDataTypeUtils::getSizeOf(data_type_) <= shared_memory_view->num_bytes_);

        // update internal state
        view_ = shared_memory_view->data_;

    } else {
        // validate internal state
        SP_ASSERT(num_elements_ == -1);
        SP_ASSERT(shared_memory_name_ == "");

        // validate consistency of internal state
        SP_ASSERT(data_.size() % CppFuncDataTypeUtils::getSizeOf(data_type_) == 0);

        // update internal state
        view_ = data_.data();
        num_elements_ = data_.size() / CppFuncDataTypeUtils::getSizeOf(data_type_);
    }
}

void CppFuncItem::resolveReturnValue(const CppFuncSharedMemoryView* shared_memory_view)
{
    // validate internal state
    SP_ASSERT(view_);
    SP_ASSERT(num_elements_ >= 0);
    SP_ASSERT(data_type_ != CppFuncDataType::Invalid);

    // update internal state
    view_ = nullptr;

    if (use_shared_memory_) {
        // validate internal state
        SP_ASSERT(data_.empty());
        SP_ASSERT(shared_memory_name_ != "");

        // validate shared memory state
        SP_ASSERT(shared_memory_view);
        SP_ASSERT(shared_memory_view->data_);
        SP_ASSERT(shared_memory_view->usage_flags_ & CppFuncSharedMemoryUsageFlags::ReturnValue);

        // validate consistency of internal state and shared memory state
        SP_ASSERT(num_elements_*CppFuncDataTypeUtils::getSizeOf(data_type_) <= shared_memory_view->num_bytes_);

    } else {
        // validate internal state
        SP_ASSERT(shared_memory_name_ == "");

        // validate consistency of internal state
        SP_ASSERT(data_.size() / CppFuncDataTypeUtils::getSizeOf(data_type_) == num_elements_);
        SP_ASSERT(data_.size() % CppFuncDataTypeUtils::getSizeOf(data_type_) == 0);

        // update internal state
        num_elements_ = -1;
    }
}

//
// CppFuncUtils
//

//
// typically called at the beginning of a C++ entry point to prepare items for use (e.g., by resolving pointers to shared memory)
//

void CppFuncUtils::resolveArg(CppFuncItem& arg, const std::map<std::string, CppFuncSharedMemoryView>& shared_memory_views)
{
    if (arg.use_shared_memory_) {
        SP_ASSERT(arg.shared_memory_name_ != "");
        SP_ASSERT(Std::containsKey(shared_memory_views, arg.shared_memory_name_));
        const CppFuncSharedMemoryView* shared_memory_view = &(shared_memory_views.at(arg.shared_memory_name_));
        arg.resolveArg(shared_memory_view);
    } else {
        arg.resolveArg();
    }
}

void CppFuncUtils::resolveArgs(std::vector<CppFuncItem>& args, const std::map<std::string, CppFuncSharedMemoryView>& shared_memory_views)
{
    for (auto& arg : args) {
        resolveArg(arg, shared_memory_views);
    }
}

void CppFuncUtils::resolveArgs(std::map<std::string, CppFuncItem>& args, const std::map<std::string, CppFuncSharedMemoryView>& shared_memory_views)
{
    for (auto& [arg_name, arg] : args) {
        resolveArg(arg, shared_memory_views);
    }
}

//
// typically called at the end of a C++ entry point to prepare items for being returned (e.g., by validating the internal consistency of each item)
//

void CppFuncUtils::resolveReturnValue(CppFuncItem& return_value, const std::map<std::string, CppFuncSharedMemoryView>& shared_memory_views)
{
    if (return_value.use_shared_memory_) {
        SP_ASSERT(return_value.shared_memory_name_ != "");
        SP_ASSERT(Std::containsKey(shared_memory_views, return_value.shared_memory_name_));
        const CppFuncSharedMemoryView* shared_memory_view = &(shared_memory_views.at(return_value.shared_memory_name_));
        return_value.resolveReturnValue(shared_memory_view);
    } else {
        return_value.resolveReturnValue();
    }
}

void CppFuncUtils::resolveReturnValues(std::vector<CppFuncItem>& return_values, const std::map<std::string, CppFuncSharedMemoryView>& shared_memory_views)
{
    for (auto& return_value : return_values) {
        resolveReturnValue(return_value, shared_memory_views);
    }
}

void CppFuncUtils::resolveReturnValues(std::map<std::string, CppFuncItem>& return_values, const std::map<std::string, CppFuncSharedMemoryView>& shared_memory_views)
{
    for (auto& [return_value_name, return_value] : return_values) {
        resolveReturnValue(return_value, shared_memory_views);
    }
}

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
// typically called when transferring an arg or a return value item to a local data object
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
