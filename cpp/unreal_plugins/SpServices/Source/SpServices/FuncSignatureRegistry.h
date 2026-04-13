//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <map>
#include <string>
#include <type_traits> // std::remove_cvref_t
#include <vector>

#include "SpCore/Assert.h"
#include "SpCore/SpArray.h"
#include "SpCore/SpFuncComponent.h"
#include "SpCore/Std.h"
#include "SpCore/UnrealUtils.h"

#include "SpServices/FuncInfo.h"
#include "SpServices/SpTypes.h"

class FuncSignatureRegistry
{
    //
    // If a type appears as an argument or a return value anywhere in our RPC interface, it needs to be
    // listed here, and these lists need to match the ones in (keep custom types sorted by the order the
    // types appear in MsgpackAdaptors.h:
    //     python_ext/cpp/func_signature_registry.h
    //     python/spear/utils/editor_utils.py
    //

    template <typename T> static int getTypeId() { SP_ASSERT(false); return -1; }
    /*  0 */ template <> int getTypeId<void>                                           () { return  0; }
    /*  1 */ template <> int getTypeId<bool>                                           () { return  1; }
    /*  2 */ template <> int getTypeId<float>                                          () { return  2; }
    /*  3 */ template <> int getTypeId<int32_t>                                        () { return  3; }
    /*  4 */ template <> int getTypeId<int64_t>                                        () { return  4; }
    /*  5 */ template <> int getTypeId<uint64_t>                                       () { return  5; }
    /*  6 */ template <> int getTypeId<std::string>                                    () { return  6; }
    /*  7 */ template <> int getTypeId<std::vector<uint64_t>>                          () { return  7; }
    /*  8 */ template <> int getTypeId<std::vector<std::string>>                       () { return  8; }
    /*  9 */ template <> int getTypeId<std::vector<SpStaticStructDesc>>                () { return  9; }
    /* 10 */ template <> int getTypeId<std::vector<SpStaticClassDesc>>                 () { return 10; }
    /* 11 */ template <> int getTypeId<std::map<std::string, uint64_t>>                () { return 11; }
    /* 12 */ template <> int getTypeId<std::map<std::string, std::string>>             () { return 12; }
    /* 13 */ template <> int getTypeId<std::map<std::string, SpPackedArray>>           () { return 13; }
    /* 14 */ template <> int getTypeId<std::map<std::string, SpArraySharedMemoryView>> () { return 14; }
    /* 15 */ template <> int getTypeId<std::map<std::string, SpPropertyValue>>         () { return 15; }
    /* 16 */ template <> int getTypeId<std::map<std::string, SpFuncSignatureDesc>>     () { return 16; }
    /* 17 */ template <> int getTypeId<std::map<std::string, SpWorldDesc>>             () { return 17; }
    /* 18 */ template <> int getTypeId<SpPackedArray>                                  () { return 18; }
    /* 19 */ template <> int getTypeId<SpArraySharedMemoryView>                        () { return 19; }
    /* 20 */ template <> int getTypeId<SpFuncDataBundle>                               () { return 20; }
    /* 21 */ template <> int getTypeId<SpPropertyDesc>                                 () { return 21; }
    /* 22 */ template <> int getTypeId<SpPropertyValue>                                () { return 22; }
    /* 23 */ template <> int getTypeId<SpFuture>                                       () { return 23; }
    /* 24 */ template <> int getTypeId<SpStaticStructDesc>                             () { return 24; }
    /* 25 */ template <> int getTypeId<SpStaticClassDesc>                              () { return 25; }
    /* 26 */ template <> int getTypeId<SpWorldDesc>                                    () { return 26; }

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
        /* 18 */ "packed_array",
        /* 19 */ "shared_memory_view",
        /* 20 */ "data_bundle",
        /* 21 */ "property_desc",
        /* 22 */ "property_value",
        /* 23 */ "future",
        /* 24 */ "static_struct_desc",
        /* 25 */ "static_class_desc",
        /* 26 */ "world_desc" };

public:
    template <typename TFunc>
    void registerFuncSignature(const std::string& func_name, const TFunc& func)
    {
        std::vector<int> type_ids = getTypeIds(func);
        std::vector<std::string> type_names = getTypeNames(type_ids);
        registerFuncSignature(func_name, type_ids, type_names);
    }

    template <typename TReturn, typename... TArgs>
    void registerFuncSignature(const std::string& func_name)
    {
        std::vector<int> type_ids = getTypeIds<TReturn, TArgs...>();
        std::vector<std::string> type_names = getTypeNames(type_ids);
        registerFuncSignature(func_name, type_ids, type_names);
    }

    bool isFuncSignatureRegistered(const std::string& func_name) const
    {
        return Std::containsKey(func_signature_descs_, func_name);
    }

    template <typename T>
    static std::string getTypeName()
    {
        return s_type_names_.at(getTypeId<T>());
    }

    static std::vector<std::string> getTypeNames()
    {
        return s_type_names_;
    }

    std::map<std::string, SpFuncSignatureDesc> getFuncSignatureDescs() const
    {
        return func_signature_descs_;
    }

private:
    void registerFuncSignature(const std::string& func_name, const std::vector<int>& type_ids, const std::vector<std::string>& type_names)
    {
        SpFuncSignatureDesc func_signature_desc;
        func_signature_desc.name_ = func_name;
        func_signature_desc.type_ids_ = type_ids;
        func_signature_desc.type_names_ = type_names;
        Std::insert(func_signature_descs_, func_name, std::move(func_signature_desc));
    }

    template <typename TFunc>
    static std::vector<int> getTypeIds(const TFunc& func)
    {
        return getTypeIdsImpl(FuncInfoUtils::getFuncInfo<TFunc>());
    }

    template <typename TReturn, typename... TArgs>
    static std::vector<int> getTypeIds()
    {
        return getTypeIdsImpl(FuncInfo<TReturn, TArgs...>());
    }

    template <typename TReturn, typename... TArgs>
    static std::vector<int> getTypeIdsImpl(const FuncInfo<TReturn, TArgs...>& fi)
    {
        std::vector<int> type_ids;
        type_ids.push_back(getTypeId<std::remove_cvref_t<TReturn>>());
        (type_ids.push_back(getTypeId<std::remove_cvref_t<TArgs>>()), ...);
        return type_ids;
    }

    static std::vector<std::string> getTypeNames(const std::vector<int>& type_ids)
    {
        return Std::toVector<std::string>(type_ids | std::views::transform([](auto type_id) { return getTypeName(type_id); }));
    }

    static std::string getTypeName(int i)
    {
        return s_type_names_.at(i);
    }

    std::map<std::string, SpFuncSignatureDesc> func_signature_descs_;
};
