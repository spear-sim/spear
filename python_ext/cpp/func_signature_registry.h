//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // int32_t, int64_t, uint64_t

#include <iostream>
#include <map>
#include <memory>  // std::make_shared, std::shared_ptr
#include <string>
#include <utility> // std::move
#include <vector>

#include <nanobind/nanobind.h>

#include "types.h"

#ifdef _MSC_VER
    #define SP_CURRENT_FUNCTION __FUNCSIG__
#else
    #define SP_CURRENT_FUNCTION __PRETTY_FUNCTION__
#endif

class Client;

//
// The type IDs used in getTypeNames(), getArg(...), and getReturnValue(...) need to match the ones in:
//     cpp/unreal_plugins/SpServices/Source/SpServices/FuncSignatureRegistry.h
//

class FuncSignatureRegistry
{
    inline static std::vector<std::string> s_type_names_ = {
        /*  0 */ "void",
        /*  1 */ "bool",
        /*  2 */ "float",
        /*  3 */ "int32",
        /*  4 */ "int64",
        /*  5 */ "uint64",
        /*  6 */ "string",
        /*  7 */ "vector_of_uint64",
        /*  8 */ "vector_of_string",
        /*  9 */ "vector_of_static_struct_desc",
        /* 10 */ "vector_of_static_class_desc",
        /* 11 */ "map_of_string_to_uint64",
        /* 12 */ "map_of_string_to_string",
        /* 13 */ "map_of_string_to_packed_array",
        /* 14 */ "map_of_string_to_shared_memory_view",
        /* 15 */ "map_of_string_to_property_value",
        /* 16 */ "map_of_string_to_func_signature_desc",
        /* 17 */ "map_of_string_to_world_desc",
        /* 18 */ "console_messages",
        /* 19 */ "packed_array",
        /* 20 */ "shared_memory_view",
        /* 21 */ "data_bundle",
        /* 22 */ "property_desc",
        /* 23 */ "property_value",
        /* 24 */ "future",
        /* 25 */ "static_struct_desc",
        /* 26 */ "static_class_desc",
        /* 27 */ "world_desc" };

public:
    static const std::vector<std::string>& getTypeNames()
    {
        return s_type_names_;
    }

    static clmdep_msgpack::object getArg(clmdep_msgpack::zone& zone, int type_id, nanobind::handle py_obj)
    {
        switch (type_id) {
            case  1: return clmdep_msgpack::object(nanobind::cast<bool>                               (py_obj), zone);
            case  2: return clmdep_msgpack::object(nanobind::cast<float>                              (py_obj), zone);
            case  3: return clmdep_msgpack::object(nanobind::cast<int32_t>                            (py_obj), zone);
            case  4: return clmdep_msgpack::object(nanobind::cast<int64_t>                            (py_obj), zone);
            case  5: return clmdep_msgpack::object(nanobind::cast<uint64_t>                           (py_obj), zone);
            case  6: return clmdep_msgpack::object(nanobind::cast<std::string>                        (py_obj), zone);
            case  7: return clmdep_msgpack::object(nanobind::cast<std::vector<uint64_t>>              (py_obj), zone);
            case  8: return clmdep_msgpack::object(nanobind::cast<std::vector<std::string>>           (py_obj), zone);
            case 11: return clmdep_msgpack::object(nanobind::cast<std::map<std::string, uint64_t>>    (py_obj), zone);
            case 12: return clmdep_msgpack::object(nanobind::cast<std::map<std::string, std::string>> (py_obj), zone);
            case 13: return clmdep_msgpack::object(nanobind::cast<std::map<std::string, PackedArray>> (py_obj), zone);
            case 19: return clmdep_msgpack::object(nanobind::cast<PackedArray>                        (py_obj), zone);
            case 21: return clmdep_msgpack::object(nanobind::cast<DataBundle>                         (py_obj), zone);
            case 22: return clmdep_msgpack::object(nanobind::cast<PropertyDesc>                       (py_obj), zone);
            case 24: return clmdep_msgpack::object(nanobind::cast<Future>                             (py_obj), zone);
            default: SP_ASSERT(false); return clmdep_msgpack::object();
        }
    }

    static nanobind::object getReturnValue(const Client* client, int type_id, clmdep_msgpack::object_handle&& result)
    {
        switch (type_id) {
            case  0: return nanobind::none();
            case  1: return nanobind::cast(result.get().as<bool>());
            case  2: return nanobind::cast(result.get().as<float>());
            case  3: return nanobind::cast(result.get().as<int32_t>());
            case  4: return nanobind::cast(result.get().as<int64_t>());
            case  5: return nanobind::cast(result.get().as<uint64_t>());
            case  6: return nanobind::cast(result.get().as<std::string>());
            case  7: return nanobind::cast(result.get().as<std::vector<uint64_t>>());
            case  8: return nanobind::cast(result.get().as<std::vector<std::string>>());
            case  9: return nanobind::cast(result.get().as<std::vector<StaticStructDesc>>());
            case 10: return nanobind::cast(result.get().as<std::vector<StaticClassDesc>>());
            case 11: return nanobind::cast(result.get().as<std::map<std::string, uint64_t>>());
            case 12: return nanobind::cast(result.get().as<std::map<std::string, std::string>>());
            case 13: return nanobind::cast(getConvertedReturnValue<std::map<std::string, PackedArray>, std::map<std::string, PackedArrayView>>(client, std::move(result)));
            case 14: return nanobind::cast(result.get().as<std::map<std::string, SharedMemoryView>>());
            case 15: return nanobind::cast(result.get().as<std::map<std::string, PropertyValue>>());
            case 16: return nanobind::cast(result.get().as<std::map<std::string, FuncSignatureDesc>>());
            case 17: return nanobind::cast(result.get().as<std::map<std::string, WorldDesc>>());
            case 18: return nanobind::cast(result.get().as<ConsoleMessages>());
            case 19: return nanobind::cast(getConvertedReturnValue<PackedArray, PackedArrayView>(client, std::move(result)));
            case 20: return nanobind::cast(result.get().as<SharedMemoryView>());
            case 21: return nanobind::cast(getConvertedReturnValue<DataBundle, DataBundleView>(client, std::move(result)));
            case 22: return nanobind::cast(result.get().as<PropertyDesc>());
            case 23: return nanobind::cast(result.get().as<PropertyValue>());
            case 24: return nanobind::cast(result.get().as<Future>());
            case 25: return nanobind::cast(result.get().as<StaticStructDesc>());
            case 26: return nanobind::cast(result.get().as<StaticClassDesc>());
            case 27: return nanobind::cast(result.get().as<WorldDesc>());
            default: SP_ASSERT(false); return nanobind::none();
        }
    }

    // defined for each type in msgpack_adaptors.h
    template <typename TDest, typename TSrc>
    static TDest convert(const Client* client, TSrc&& src, std::shared_ptr<clmdep_msgpack::object_handle> object_handle)
    {
        std::cout << "[SPEAR | spear_ext.cpp] ERROR: Current function: " << SP_CURRENT_FUNCTION << std::endl; SP_ASSERT(false); return TDest();
    };

private:
    template <typename TReturnDest, typename TReturnSrc>
    static TReturnDest getConvertedReturnValue(const Client* client, clmdep_msgpack::object_handle&& object_handle)
    {
        std::shared_ptr<clmdep_msgpack::object_handle> object_handle_ptr = std::make_shared<clmdep_msgpack::object_handle>(std::move(object_handle));
        TReturnSrc return_value_src = object_handle_ptr->template as<TReturnSrc>(); // .template needed on macOS
        return convert<TReturnDest>(client, std::move(return_value_src), object_handle_ptr);
    };
};
