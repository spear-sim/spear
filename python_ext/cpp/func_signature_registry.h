//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // int32_t, int64_t, uint64_t

#include <iostream>
#include <map>
#include <memory>      // std::make_shared, std::shared_ptr
#include <ranges>      // std::views::transform
#include <string>
#include <type_traits> // std::remove_cvref_t
#include <utility>     // std::move
#include <vector>

#include "std.h"
#include "types.h"

#ifdef _MSC_VER
    #define SP_CURRENT_FUNCTION __FUNCSIG__
#else
    #define SP_CURRENT_FUNCTION __PRETTY_FUNCTION__
#endif

class Client;

class FuncSignatureRegistry
{
    static FuncSignatureTypeDesc getTypeDesc(std::map<std::string, std::string> type_names, std::map<std::string, std::string> const_strings, std::map<std::string, std::string> ref_strings)
    {
        FuncSignatureTypeDesc type_desc;
        type_desc.type_names_ = type_names;
        type_desc.const_strings_ = const_strings;
        type_desc.ref_strings_ = ref_strings;
        return type_desc;
    }

    //
    // If a type appears as an argument or a return value anywhere in our RPC interface, it needs to be
    // listed here, and this list needs to match the one in cpp/unreal_plugins/SpServices/Source/SpServices/FuncSignatureRegistry.h
    //

    template <typename T> static int getTypeId() { std::cout << "[SPEAR | spear_ext.cpp] ERROR: Current function: " << SP_CURRENT_FUNCTION << std::endl; SP_ASSERT(false); return -1; }
    /*  0 */ template <> int getTypeId<void>                                                  () { return  0; }
    /*  1 */ template <> int getTypeId<bool>                                                  () { return  1; }
    /*  2 */ template <> int getTypeId<float>                                                 () { return  2; }
    /*  3 */ template <> int getTypeId<int32_t>                                               () { return  3; }
    /*  4 */ template <> int getTypeId<int64_t>                                               () { return  4; }
    /*  5 */ template <> int getTypeId<uint64_t>                                              () { return  5; }
    /*  6 */ template <> int getTypeId<std::string>                                           () { return  6; }
    /*  7 */ template <> int getTypeId<std::vector<uint64_t>>                                 () { return  7; }
    /*  8 */ template <> int getTypeId<std::vector<std::string>>                              () { return  8; }
    /*  9 */ template <> int getTypeId<std::vector<FuncSignatureTypeDesc>>                    () { return  9; }
    /* 10 */ template <> int getTypeId<std::map<std::string, uint64_t>>                       () { return 10; }
    /* 11 */ template <> int getTypeId<std::map<std::string, std::string>>                    () { return 11; }
    /* 12 */ template <> int getTypeId<std::map<std::string, SharedMemoryView>>               () { return 12; }
    /* 13 */ template <> int getTypeId<std::map<std::string, PackedArray>>                    () { return 13; }
    /* 14 */ template <> int getTypeId<std::map<std::string, std::vector<FuncSignatureDesc>>> () { return 14; }
    /* 15 */ template <> int getTypeId<PropertyDesc>                                          () { return 15; }
    /* 16 */ template <> int getTypeId<SharedMemoryView>                                      () { return 16; }
    /* 17 */ template <> int getTypeId<PackedArray>                                           () { return 17; }
    /* 18 */ template <> int getTypeId<DataBundle>                                            () { return 18; }
    /* 19 */ template <> int getTypeId<Future>                                                () { return 19; }

public:
    // needed for Python extension but not for EngineService
    template <typename TReturn> static TReturn getReturnValue(const Client* client, clmdep_msgpack::object_handle&& object_handle) { std::cout << "[SPEAR | spear_ext.cpp] ERROR: Current function: " << SP_CURRENT_FUNCTION << std::endl; SP_ASSERT(false); return TReturn(); }
    /*  0 */ template <> void                                                  getReturnValue<void>                                                  (const Client* client, clmdep_msgpack::object_handle&& object_handle) { return; }
    /*  1 */ template <> bool                                                  getReturnValue<bool>                                                  (const Client* client, clmdep_msgpack::object_handle&& object_handle) { return getReturnValueImpl<bool>                                                                       (client, std::move(object_handle)); }
    /*  2 */ template <> float                                                 getReturnValue<float>                                                 (const Client* client, clmdep_msgpack::object_handle&& object_handle) { return getReturnValueImpl<float>                                                                      (client, std::move(object_handle)); }
    /*  3 */ template <> int32_t                                               getReturnValue<int32_t>                                               (const Client* client, clmdep_msgpack::object_handle&& object_handle) { return getReturnValueImpl<int32_t>                                                                    (client, std::move(object_handle)); }
    /*  4 */ template <> int64_t                                               getReturnValue<int64_t>                                               (const Client* client, clmdep_msgpack::object_handle&& object_handle) { return getReturnValueImpl<int64_t>                                                                    (client, std::move(object_handle)); }
    /*  5 */ template <> uint64_t                                              getReturnValue<uint64_t>                                              (const Client* client, clmdep_msgpack::object_handle&& object_handle) { return getReturnValueImpl<uint64_t>                                                                   (client, std::move(object_handle)); }
    /*  6 */ template <> std::string                                           getReturnValue<std::string>                                           (const Client* client, clmdep_msgpack::object_handle&& object_handle) { return getReturnValueImpl<std::string>                                                                (client, std::move(object_handle)); }
    /*  7 */ template <> std::vector<uint64_t>                                 getReturnValue<std::vector<uint64_t>>                                 (const Client* client, clmdep_msgpack::object_handle&& object_handle) { return getReturnValueImpl<std::vector<uint64_t>>                                                      (client, std::move(object_handle)); }
    /*  8 */ template <> std::vector<std::string>                              getReturnValue<std::vector<std::string>>                              (const Client* client, clmdep_msgpack::object_handle&& object_handle) { return getReturnValueImpl<std::vector<std::string>>                                                   (client, std::move(object_handle)); }
    /*  9 */ template <> std::vector<FuncSignatureTypeDesc>                    getReturnValue<std::vector<FuncSignatureTypeDesc>>                    (const Client* client, clmdep_msgpack::object_handle&& object_handle) { return getReturnValueImpl<std::vector<FuncSignatureTypeDesc>>                                         (client, std::move(object_handle)); }
    /* 10 */ template <> std::map<std::string, uint64_t>                       getReturnValue<std::map<std::string, uint64_t>>                       (const Client* client, clmdep_msgpack::object_handle&& object_handle) { return getReturnValueImpl<std::map<std::string, uint64_t>>                                            (client, std::move(object_handle)); }
    /* 11 */ template <> std::map<std::string, std::string>                    getReturnValue<std::map<std::string, std::string>>                    (const Client* client, clmdep_msgpack::object_handle&& object_handle) { return getReturnValueImpl<std::map<std::string, std::string>>                                         (client, std::move(object_handle)); }
    /* 12 */ template <> std::map<std::string, SharedMemoryView>               getReturnValue<std::map<std::string, SharedMemoryView>>               (const Client* client, clmdep_msgpack::object_handle&& object_handle) { return getReturnValueImpl<std::map<std::string, SharedMemoryView>>                                    (client, std::move(object_handle)); }
    /* 13 */ template <> std::map<std::string, PackedArray>                    getReturnValue<std::map<std::string, PackedArray>>                    (const Client* client, clmdep_msgpack::object_handle&& object_handle) { return getReturnValueImpl<std::map<std::string, PackedArray>, std::map<std::string, PackedArrayView>> (client, std::move(object_handle)); }
    /* 14 */ template <> std::map<std::string, std::vector<FuncSignatureDesc>> getReturnValue<std::map<std::string, std::vector<FuncSignatureDesc>>> (const Client* client, clmdep_msgpack::object_handle&& object_handle) { return getReturnValueImpl<std::map<std::string, std::vector<FuncSignatureDesc>>>                      (client, std::move(object_handle)); }
    /* 15 */ template <> PropertyDesc                                          getReturnValue<PropertyDesc>                                          (const Client* client, clmdep_msgpack::object_handle&& object_handle) { return getReturnValueImpl<PropertyDesc>                                                               (client, std::move(object_handle)); }
    /* 16 */ template <> SharedMemoryView                                      getReturnValue<SharedMemoryView>                                      (const Client* client, clmdep_msgpack::object_handle&& object_handle) { return getReturnValueImpl<SharedMemoryView>                                                           (client, std::move(object_handle)); }
    /* 17 */ template <> PackedArray                                           getReturnValue<PackedArray>                                           (const Client* client, clmdep_msgpack::object_handle&& object_handle) { return getReturnValueImpl<PackedArray, PackedArrayView>                                               (client, std::move(object_handle)); }
    /* 18 */ template <> Future                                                getReturnValue<Future>                                                (const Client* client, clmdep_msgpack::object_handle&& object_handle) { return getReturnValueImpl<Future>                                                                     (client, std::move(object_handle)); }
    /* 19 */ template <> DataBundle                                            getReturnValue<DataBundle>                                            (const Client* client, clmdep_msgpack::object_handle&& object_handle) { return getReturnValueImpl<DataBundle, DataBundleView>                                                 (client, std::move(object_handle)); }

    template <typename TDest, typename TSrc> static TDest convert(const Client* client, TSrc&& src, std::shared_ptr<clmdep_msgpack::object_handle> object_handle) { std::cout << "[SPEAR | spear_ext.cpp] ERROR: Current function: " << SP_CURRENT_FUNCTION << std::endl; SP_ASSERT(false); return TDest(); };

private:
    inline static std::vector<FuncSignatureTypeDesc> s_func_signature_type_descs_ = {
        //                   type names,                                                                                                                                       const strings,              ref strings
        /*  0 */ getTypeDesc({{"python_ext", "void"},                                                      {"entry_point", "void"}},                                           {{"python_ext", ""}},       {{"python_ext", ""}}),
        /*  1 */ getTypeDesc({{"python_ext", "bool"},                                                      {"entry_point", "bool"}},                                           {{"python_ext", ""}},       {{"python_ext", ""}}),
        /*  2 */ getTypeDesc({{"python_ext", "float"},                                                     {"entry_point", "float"}},                                          {{"python_ext", ""}},       {{"python_ext", ""}}),
        /*  3 */ getTypeDesc({{"python_ext", "int32_t"},                                                   {"entry_point", "int32"}},                                          {{"python_ext", ""}},       {{"python_ext", ""}}),
        /*  4 */ getTypeDesc({{"python_ext", "int64_t"},                                                   {"entry_point", "int64"}},                                          {{"python_ext", ""}},       {{"python_ext", ""}}),
        /*  5 */ getTypeDesc({{"python_ext", "uint64_t"},                                                  {"entry_point", "uint64"}},                                         {{"python_ext", ""}},       {{"python_ext", ""}}),
        /*  6 */ getTypeDesc({{"python_ext", "std::string"},                                               {"entry_point", "string"}},                                         {{"python_ext", "const "}}, {{"python_ext", "&"}}),
        /*  7 */ getTypeDesc({{"python_ext", "std::vector<uint64_t>"},                                     {"entry_point", "vector_of_uint64"}},                               {{"python_ext", "const "}}, {{"python_ext", "&"}}),
        /*  8 */ getTypeDesc({{"python_ext", "std::vector<std::string>"},                                  {"entry_point", "vector_of_string"}},                               {{"python_ext", "const "}}, {{"python_ext", "&"}}),
        /*  9 */ getTypeDesc({{"python_ext", "std::vector<FuncSignatureTypeDesc>"},                        {"entry_point", "vector_of_func_signature_type_desc"}},             {{"python_ext", "const "}}, {{"python_ext", "&"}}),
        /* 10 */ getTypeDesc({{"python_ext", "std::map<std::string, uint64_t>"},                           {"entry_point", "map_of_string_to_uint64"}},                        {{"python_ext", "const "}}, {{"python_ext", "&"}}),
        /* 11 */ getTypeDesc({{"python_ext", "std::map<std::string, std::string>"},                        {"entry_point", "map_of_string_to_string"}},                        {{"python_ext", "const "}}, {{"python_ext", "&"}}),
        /* 12 */ getTypeDesc({{"python_ext", "std::map<std::string, SharedMemoryView>"},                   {"entry_point", "map_of_string_to_shared_memory_view"}},            {{"python_ext", "const "}}, {{"python_ext", "&"}}),
        /* 13 */ getTypeDesc({{"python_ext", "std::map<std::string, PackedArray>"},                        {"entry_point", "map_of_string_to_packed_array"}},                  {{"python_ext", "const "}}, {{"python_ext", "&"}}),
        /* 14 */ getTypeDesc({{"python_ext", "std::map<std::string, std::vector<FuncSignatureTypeDesc>>"}, {"entry_point", "map_of_string_to_vector_of_func_signature_desc"}}, {{"python_ext", "const "}}, {{"python_ext", "&"}}),
        /* 15 */ getTypeDesc({{"python_ext", "PropertyDesc"},                                              {"entry_point", "property_desc"}},                                  {{"python_ext", "const "}}, {{"python_ext", "&"}}),
        /* 16 */ getTypeDesc({{"python_ext", "SharedMemoryView"},                                          {"entry_point", "shared_memory_view"}},                             {{"python_ext", "const "}}, {{"python_ext", "&"}}),
        /* 17 */ getTypeDesc({{"python_ext", "PackedArray"},                                               {"entry_point", "packed_array"}},                                   {{"python_ext", "const "}}, {{"python_ext", "&"}}),
        /* 18 */ getTypeDesc({{"python_ext", "DataBundle"},                                                {"entry_point", "data_bundle"}},                                    {{"python_ext", "const "}}, {{"python_ext", "&"}}),
        /* 19 */ getTypeDesc({{"python_ext", "Future"},                                                    {"entry_point", "future"}},                                         {{"python_ext", "const "}}, {{"python_ext", "&"}})};

public:
    template <typename TReturn, typename... TArgs>
    void registerFuncSignature(const std::string& func_name)
    {
        std::vector<int> func_signature_id = getFuncSignatureIdImpl<TReturn, TArgs...>();
        std::vector<FuncSignatureTypeDesc> func_signature = getFuncSignature(func_signature_id);
        registerFuncSignature(func_name, func_signature_id, func_signature);
    }

    template <typename TReturn, typename... TArgs>
    bool isFuncSignatureRegistered(const std::string& func_name) const
    {
        std::vector<int> func_signature_id = getFuncSignatureIdImpl<TReturn, TArgs...>();
        return isFuncSignatureRegistered(func_name, func_signature_id);
    }

    std::vector<FuncSignatureDesc> getFuncSignatureDescs() const
    {
        return func_signature_descs_;
    }

    static std::vector<FuncSignatureTypeDesc> getFuncSignatureTypeDescs()
    {
        return s_func_signature_type_descs_;
    }

    template <typename T>
    static FuncSignatureTypeDesc getFuncSignatureTypeDesc()
    {
        return Std::at(s_func_signature_type_descs_, getTypeId<T>());
    }

private:
    bool isFuncSignatureRegistered(const std::string& func_name, const std::vector<int>& func_signature_id) const
    {
        for (auto& func_signature_desc : func_signature_descs_) {
            if (func_name == func_signature_desc.name_ && func_signature_id == func_signature_desc.func_signature_id_) {
                return true;
            }
        }
        return false;
    }

    void registerFuncSignature(const std::string& func_name, const std::vector<int>& func_signature_id, const std::vector<FuncSignatureTypeDesc>& func_signature)
    {
        FuncSignatureDesc func_signature_desc;
        func_signature_desc.name_ = func_name;
        func_signature_desc.func_signature_ = func_signature;
        func_signature_desc.func_signature_id_ = func_signature_id;
        func_signature_descs_.push_back(std::move(func_signature_desc));
    }

    std::vector<FuncSignatureDesc> func_signature_descs_;

    template <typename TReturn, typename... TArgs>
    static std::vector<int> getFuncSignatureIdImpl()
    {
        std::vector<int> func_signature_id;
        func_signature_id.push_back(getTypeId<std::remove_cvref_t<TReturn>>());
        (func_signature_id.push_back(getTypeId<std::remove_cvref_t<TArgs>>()), ...);
        return func_signature_id;
    }

    static std::vector<FuncSignatureTypeDesc> getFuncSignature(const std::vector<int>& func_signature_id)
    {
        return Std::toVector<FuncSignatureTypeDesc>(
            func_signature_id |
            std::views::transform([](auto type_id) {
                return Std::at(s_func_signature_type_descs_, type_id); }));
    }

    // needed for Python extension but not for EngineService

    template <typename TReturn>
    static TReturn getReturnValueImpl(const Client* client, clmdep_msgpack::object_handle&& object_handle)
    {
        return object_handle.template as<TReturn>(); // .template needed on macOS
    };

    template <typename TReturnDest, typename TReturnSrc>
    static TReturnDest getReturnValueImpl(const Client* client, clmdep_msgpack::object_handle&& object_handle)
    {
        std::shared_ptr<clmdep_msgpack::object_handle> object_handle_ptr = std::make_shared<clmdep_msgpack::object_handle>(std::move(object_handle));
        TReturnSrc return_value_src = object_handle_ptr->template as<TReturnSrc>(); // .template needed on macOS
        return convert<TReturnDest>(client, std::move(return_value_src), object_handle_ptr);
    };
};
