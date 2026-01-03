//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <map>
#include <string>
#include <type_traits> // std::remove_cvref_t
#include <vector>

#include "SpCore/Assert.h"
#include "SpCore/SpFuncComponent.h"
#include "SpCore/SpArray.h"
#include "SpCore/Std.h"
#include "SpCore/UnrealUtils.h"

#include "SpServices/FuncInfo.h"
#include "SpServices/SpTypes.h"

class FuncSignatureRegistry
{
    static SpFuncSignatureTypeDesc getTypeDesc(std::map<std::string, std::string> type_names, std::map<std::string, std::string> const_strings, std::map<std::string, std::string> ref_strings)
    {
        SpFuncSignatureTypeDesc type_desc;
        type_desc.type_names_ = type_names;
        type_desc.const_strings_ = const_strings;
        type_desc.ref_strings_ = ref_strings;
        return type_desc;
    }

    //
    // If a type appears as an argument or a return value anywhere in our RPC interface, it needs to be
    // listed here, and this list needs to match the one in python_ext/cpp/func_signature_registry.h.
    //

    template <typename T> static int getTypeId() { SP_ASSERT(false); return -1; }
    /*  0 */ template <> int getTypeId<void>                                                    () { return  0; }
    /*  1 */ template <> int getTypeId<bool>                                                    () { return  1; }
    /*  2 */ template <> int getTypeId<float>                                                   () { return  2; }
    /*  3 */ template <> int getTypeId<int32_t>                                                 () { return  3; }
    /*  4 */ template <> int getTypeId<int64_t>                                                 () { return  4; }
    /*  5 */ template <> int getTypeId<uint64_t>                                                () { return  5; }
    /*  6 */ template <> int getTypeId<std::string>                                             () { return  6; }
    /*  7 */ template <> int getTypeId<std::vector<uint64_t>>                                   () { return  7; }
    /*  8 */ template <> int getTypeId<std::vector<std::string>>                                () { return  8; }
    /*  9 */ template <> int getTypeId<std::vector<SpFuncSignatureTypeDesc>>                    () { return  9; }
    /* 10 */ template <> int getTypeId<std::vector<SpStaticStructDesc>>                         () { return 10; }
    /* 11 */ template <> int getTypeId<std::map<std::string, uint64_t>>                         () { return 11; }
    /* 12 */ template <> int getTypeId<std::map<std::string, std::string>>                      () { return 12; }
    /* 13 */ template <> int getTypeId<std::map<std::string, SpPropertyValue>>                  () { return 13; }
    /* 14 */ template <> int getTypeId<std::map<std::string, SpArraySharedMemoryView>>          () { return 14; }
    /* 15 */ template <> int getTypeId<std::map<std::string, SpPackedArray>>                    () { return 15; }
    /* 16 */ template <> int getTypeId<std::map<std::string, std::vector<SpFuncSignatureDesc>>> () { return 16; }
    /* 17 */ template <> int getTypeId<SpPropertyDesc>                                          () { return 17; }
    /* 18 */ template <> int getTypeId<SpPropertyValue>                                         () { return 18; }
    /* 19 */ template <> int getTypeId<SpArraySharedMemoryView>                                 () { return 19; }
    /* 20 */ template <> int getTypeId<SpPackedArray>                                           () { return 20; }
    /* 21 */ template <> int getTypeId<SpFuncDataBundle>                                        () { return 21; }
    /* 22 */ template <> int getTypeId<SpFuture>                                                () { return 22; }
    /* 23 */ template <> int getTypeId<SpStaticStructDesc>                                      () { return 23; }

    inline static std::vector<SpFuncSignatureTypeDesc> s_func_signature_type_descs_ = {
        //       type names,                                                                                                                                                   const strings,              ref strings
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
        /* 10 */ getTypeDesc({{"python_ext", "std::vector<StaticStructDesc>"},                             {"entry_point", "vector_of_static_struct_desc"}},                   {{"python_ext", "const "}}, {{"python_ext", "&"}}),
        /* 11 */ getTypeDesc({{"python_ext", "std::map<std::string, uint64_t>"},                           {"entry_point", "map_of_string_to_uint64"}},                        {{"python_ext", "const "}}, {{"python_ext", "&"}}),
        /* 12 */ getTypeDesc({{"python_ext", "std::map<std::string, std::string>"},                        {"entry_point", "map_of_string_to_string"}},                        {{"python_ext", "const "}}, {{"python_ext", "&"}}),
        /* 13 */ getTypeDesc({{"python_ext", "std::map<std::string, PropertyValue>"},                      {"entry_point", "map_of_string_to_property_value"}},                {{"python_ext", "const "}}, {{"python_ext", "&"}}),
        /* 14 */ getTypeDesc({{"python_ext", "std::map<std::string, SharedMemoryView>"},                   {"entry_point", "map_of_string_to_shared_memory_view"}},            {{"python_ext", "const "}}, {{"python_ext", "&"}}),
        /* 15 */ getTypeDesc({{"python_ext", "std::map<std::string, PackedArray>"},                        {"entry_point", "map_of_string_to_packed_array"}},                  {{"python_ext", "const "}}, {{"python_ext", "&"}}),
        /* 16 */ getTypeDesc({{"python_ext", "std::map<std::string, std::vector<FuncSignatureTypeDesc>>"}, {"entry_point", "map_of_string_to_vector_of_func_signature_desc"}}, {{"python_ext", "const "}}, {{"python_ext", "&"}}),
        /* 17 */ getTypeDesc({{"python_ext", "PropertyDesc"},                                              {"entry_point", "property_desc"}},                                  {{"python_ext", "const "}}, {{"python_ext", "&"}}),
        /* 18 */ getTypeDesc({{"python_ext", "PropertyValue"},                                             {"entry_point", "property_value"}},                                 {{"python_ext", "const "}}, {{"python_ext", "&"}}),
        /* 19 */ getTypeDesc({{"python_ext", "SharedMemoryView"},                                          {"entry_point", "shared_memory_view"}},                             {{"python_ext", "const "}}, {{"python_ext", "&"}}),
        /* 20 */ getTypeDesc({{"python_ext", "PackedArray"},                                               {"entry_point", "packed_array"}},                                   {{"python_ext", "const "}}, {{"python_ext", "&"}}),
        /* 21 */ getTypeDesc({{"python_ext", "DataBundle"},                                                {"entry_point", "data_bundle"}},                                    {{"python_ext", "const "}}, {{"python_ext", "&"}}),
        /* 22 */ getTypeDesc({{"python_ext", "Future"},                                                    {"entry_point", "future"}},                                         {{"python_ext", "const "}}, {{"python_ext", "&"}}),
        /* 23 */ getTypeDesc({{"python_ext", "StaticStructDesc"},                                          {"entry_point", "static_struct_desc"}},                             {{"python_ext", "const "}}, {{"python_ext", "&"}})};

public:
    template <typename TFunc>
    void registerFuncSignature(const std::string& func_name, const TFunc& func)
    {
        std::vector<int> func_signature_id = getFuncSignatureId(func);
        std::vector<SpFuncSignatureTypeDesc> func_signature = getFuncSignature(func_signature_id);
        registerFuncSignature(func_name, func_signature_id, func_signature);
    }

    template <typename TReturn, typename... TArgs>
    void registerFuncSignature(const std::string& func_name)
    {
        std::vector<int> func_signature_id = getFuncSignatureIdImpl<TReturn, TArgs...>();
        std::vector<SpFuncSignatureTypeDesc> func_signature = getFuncSignature(func_signature_id);
        registerFuncSignature(func_name, func_signature_id, func_signature);
    }

    template <typename TFunc>
    bool isFuncSignatureRegistered(const std::string& func_name, const TFunc& func) const
    {
        std::vector<int> func_signature_id = getFuncSignatureId(func);
        return isFuncSignatureRegistered(func_name, func_signature_id);
    }

    template <typename TReturn, typename... TArgs>
    bool isFuncSignatureRegistered(const std::string& func_name) const
    {
        std::vector<int> func_signature_id = getFuncSignatureIdImpl<TReturn, TArgs...>();
        return isFuncSignatureRegistered(func_name, func_signature_id);
    }

    std::vector<SpFuncSignatureDesc> getFuncSignatureDescs() const
    {
        return func_signature_descs_;
    }

    static std::vector<SpFuncSignatureTypeDesc> getFuncSignatureTypeDescs()
    {
        return s_func_signature_type_descs_;
    }

    template <typename T>
    static SpFuncSignatureTypeDesc getFuncSignatureTypeDesc()
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

    void registerFuncSignature(const std::string& func_name, const std::vector<int>& func_signature_id, const std::vector<SpFuncSignatureTypeDesc>& func_signature)
    {
        SpFuncSignatureDesc func_signature_desc;
        func_signature_desc.name_ = func_name;
        func_signature_desc.func_signature_id_ = func_signature_id;
        func_signature_desc.func_signature_ = func_signature;
        func_signature_descs_.push_back(std::move(func_signature_desc));
    }

    std::vector<SpFuncSignatureDesc> func_signature_descs_;

    template <typename TFunc>
    static std::vector<int> getFuncSignatureId(const TFunc& func)
    {
        return getFuncSignatureIdImpl(FuncInfoUtils::getFuncInfo<TFunc>());
    }

    template <typename TReturn, typename... TArgs>
    static std::vector<int> getFuncSignatureIdImpl(const FuncInfo<TReturn, TArgs...>& fi)
    {
        return getFuncSignatureIdImpl<TReturn, TArgs...>();
    }

    template <typename TReturn, typename... TArgs>
    static std::vector<int> getFuncSignatureIdImpl()
    {
        std::vector<int> func_signature_id;
        func_signature_id.push_back(getTypeId<std::remove_cvref_t<TReturn>>());
        (func_signature_id.push_back(getTypeId<std::remove_cvref_t<TArgs>>()), ...);
        return func_signature_id;
    }

    static std::vector<SpFuncSignatureTypeDesc> getFuncSignature(const std::vector<int>& func_signature_id)
    {
        return Std::toVector<SpFuncSignatureTypeDesc>(
            func_signature_id |
            std::views::transform([](auto type_id) {
                return Std::at(s_func_signature_type_descs_, type_id); }));
    }
};
