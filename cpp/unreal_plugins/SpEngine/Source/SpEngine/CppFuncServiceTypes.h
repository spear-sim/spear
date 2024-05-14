//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <map>
#include <string>

#include "SpCore/CppFunc.h"
#include "SpCore/Rpclib.h"

// We use MSGPACK macros here to define structs that can be passed into, and returned from, the service entry
// points defined below. There are already similar structs defined in SpCore, but we choose to define separate
// structs here to avoid a dependency on RPCLib in SpCore.

MSGPACK_ADD_ENUM(CppFuncDataType);
MSGPACK_ADD_ENUM(CppFuncSharedMemoryUsageFlags);

struct CppFuncServiceSharedMemoryView : public CppFuncSharedMemoryView
{
    MSGPACK_DEFINE_MAP(id_, num_bytes_, usage_flags_);
};

struct CppFuncServiceItem : public CppFuncItem
{
    MSGPACK_DEFINE_MAP(data_, num_elements_, data_type_, use_shared_memory_, shared_memory_name_);
};

struct CppFuncServicePackage : public CppFuncPackage
{
    std::map<std::string, CppFuncServiceItem> items_;
    MSGPACK_DEFINE_MAP(items_, unreal_obj_strings_, info_);
};

class CppFuncServiceUtils
{
public:
    static std::map<std::string, CppFuncServiceSharedMemoryView> toServiceSharedMemoryViews(const std::map<std::string, CppFuncSharedMemoryView>& shared_memory_views);

    static std::map<std::string, CppFuncItem> moveToItems(const std::map<std::string, CppFuncServiceItem>& service_items);
    static std::vector<CppFuncItem> moveToItems(const std::vector<CppFuncServiceItem>& service_items);

    static std::map<std::string, CppFuncServiceItem> moveToServiceItems(const std::map<std::string, CppFuncItem>& items);
    static std::vector<CppFuncServiceItem> moveToServiceItems(const std::vector<CppFuncItem>& items);

    static CppFuncPackage moveToPackage(const CppFuncServicePackage& service_package);
    static CppFuncServicePackage moveToServicePackage(const CppFuncPackage& package);    
};
