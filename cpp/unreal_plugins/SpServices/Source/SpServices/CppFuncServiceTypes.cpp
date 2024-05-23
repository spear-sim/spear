//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpServices/CppFuncServiceTypes.h"

#include "SpCore/Assert.h"
#include "SpCore/Std.h"

// The functions below assign directly to base type lvalues from derived type rvalues, and vice versa. These
// operations are valid because each derived type inherits from the base type without defining any new data
// members, so we expect the data layout of each base and derived type to identical.

std::map<std::string, CppFuncServiceSharedMemoryView> CppFuncServiceUtils::toServiceSharedMemoryViews(const std::map<std::string, CppFuncSharedMemoryView>& shared_memory_views)
{
    std::map<std::string, CppFuncServiceSharedMemoryView> service_shared_memory_views;
    for (auto& [shared_memory_view_name, shared_memory_view] : shared_memory_views) {
        SP_ASSERT(shared_memory_view_name != "");
        CppFuncServiceSharedMemoryView service_shared_memory_view = std::move(static_cast<const CppFuncServiceSharedMemoryView&>(shared_memory_view));
        Std::insert(service_shared_memory_views, shared_memory_view_name, service_shared_memory_view);
    }
    return service_shared_memory_views;
}

std::map<std::string, CppFuncItem> CppFuncServiceUtils::moveToItems(const std::map<std::string, CppFuncServiceItem>& service_items)
{
    std::map<std::string, CppFuncItem> items;
    for (auto& [service_item_name, service_item] : service_items) {
        SP_ASSERT(service_item_name != "");
        CppFuncItem item = std::move(static_cast<const CppFuncItem&>(service_item));
        Std::insert(items, service_item_name, std::move(item));
    }
    return items;
}

std::vector<CppFuncItem> CppFuncServiceUtils::moveToItems(const std::vector<CppFuncServiceItem>& service_items)
{
    std::vector<CppFuncItem> items;
    for (auto& service_item : service_items) {
        CppFuncItem item = std::move(static_cast<const CppFuncItem&>(service_item));
        items.push_back(std::move(item));
    }
    return items;
}

std::map<std::string, CppFuncServiceItem> CppFuncServiceUtils::moveToServiceItems(const std::map<std::string, CppFuncItem>& items)
{
    std::map<std::string, CppFuncServiceItem> service_items;
    for (auto& [item_name, item] : items) {
        SP_ASSERT(item_name != "");
        CppFuncServiceItem service_item = std::move(static_cast<const CppFuncServiceItem&>(item));
        Std::insert(service_items, item_name, std::move(service_item));
    }
    return service_items;
}

std::vector<CppFuncServiceItem> CppFuncServiceUtils::moveToServiceItems(const std::vector<CppFuncItem>& items)
{
    std::vector<CppFuncServiceItem> service_items;
    for (auto& item : items) {
        CppFuncServiceItem service_item = std::move(static_cast<const CppFuncServiceItem&>(item));
        service_items.push_back(std::move(service_item));
    }
    return service_items;
}

CppFuncPackage CppFuncServiceUtils::moveToPackage(const CppFuncServicePackage& service_package)
{
    CppFuncPackage package;
    package.items_ = moveToItems(service_package.items_);
    package.unreal_obj_strings_ = std::move(service_package.unreal_obj_strings_);
    package.info_ = std::move(service_package.info_);
    return package;
}

CppFuncServicePackage CppFuncServiceUtils::moveToServicePackage(const CppFuncPackage& package)
{
    CppFuncServicePackage service_package;
    service_package.items_ = moveToServiceItems(package.items_);
    service_package.unreal_obj_strings_ = std::move(package.unreal_obj_strings_);
    service_package.info_ = std::move(package.info_);
    return service_package;
}
