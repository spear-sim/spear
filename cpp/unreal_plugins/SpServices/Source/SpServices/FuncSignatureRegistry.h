//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

//
// This header participates in SP_PLATFORM_PURE builds.
//

#pragma once

#include <stdint.h> // int32_t, int64_t, uint64_t

#include <map>
#include <string>
#include <vector>

#include "SpCore/Assert.h"
#include "SpCore/SpTypes.h"

#include "SpServices/Platform.h"
#include "SpServices/SpTypes.h"

#if SP_PLATFORM_SERVER
    #include <ranges>      // std::views::transform
    #include <type_traits> // std::remove_cvref_t

    #include "SpCore/Std.h"

    #include "SpServices/FuncInfo.h"
#elif SP_PLATFORM_CLIENT
    #include <memory>  // std::make_shared, std::shared_ptr
    #include <utility> // std::move

    #include <nanobind/nanobind.h>
    #include <nanobind/stl/map.h>
    #include <nanobind/stl/string.h>
    #include <nanobind/stl/vector.h>

    #include "SpServices/MsgpackAdaptors.h"
    #include "SpServices/MsgpackUtils.h"
    #include "SpServices/Rpclib.h"
#else
    #error
#endif

class FuncSignatureRegistry
{
    //
    // If a type appears as an argument or a return value anywhere in our RPC interface, it needs to be listed in
    // the type tables below, and the type IDs must be consistent across all of the tables. The type names also need
    // to match the ones in python/spear/utils/editor_utils.py. Keep custom types sorted by the order the types
    // appear in MsgpackAdaptors.h.
    //

    #if SP_PLATFORM_SERVER || SP_PLATFORM_CLIENT
        inline static std::vector<std::string> s_type_names_ = {
            /*  0 */ "void",
            /*  1 */ "bool",
            /*  2 */ "float",
            /*  3 */ "int32",
            /*  4 */ "int64",
            /*  5 */ "uint64",
            /*  6 */ "string",
            /*  7 */ "vector_of_string",
            /*  8 */ "vector_of_sp_handle",
            /*  9 */ "vector_of_sp_static_struct_desc",
            /* 10 */ "vector_of_sp_static_class_desc",
            /* 11 */ "map_of_string_to_string",
            /* 12 */ "map_of_string_to_sp_array_shared_memory_view",
            /* 13 */ "map_of_string_to_sp_packed_array",
            /* 14 */ "map_of_string_to_sp_handle",
            /* 15 */ "map_of_string_to_sp_property_value",
            /* 16 */ "map_of_string_to_sp_func_signature_desc",
            /* 17 */ "map_of_string_to_sp_world_desc",
            /* 18 */ "sp_output_log_messages",
            /* 19 */ "sp_shared_memory_view",
            /* 20 */ "sp_array_shared_memory_view",
            /* 21 */ "sp_packed_array",
            /* 22 */ "sp_func_data_bundle",
            /* 23 */ "sp_handle",
            /* 24 */ "sp_property_desc",
            /* 25 */ "sp_property_value",
            /* 26 */ "sp_func_signature_desc",
            /* 27 */ "sp_future",
            /* 28 */ "sp_function_desc",
            /* 29 */ "sp_static_struct_desc",
            /* 30 */ "sp_static_class_desc",
            /* 31 */ "sp_world_desc" };
    #endif

    #if SP_PLATFORM_SERVER
        template <typename T> static int getTypeId() = delete;
        /*  0 */ template <> int getTypeId<void>                                           () { return  0; }
        /*  1 */ template <> int getTypeId<bool>                                           () { return  1; }
        /*  2 */ template <> int getTypeId<float>                                          () { return  2; }
        /*  3 */ template <> int getTypeId<int32_t>                                        () { return  3; }
        /*  4 */ template <> int getTypeId<int64_t>                                        () { return  4; }
        /*  5 */ template <> int getTypeId<uint64_t>                                       () { return  5; }
        /*  6 */ template <> int getTypeId<std::string>                                    () { return  6; }
        /*  7 */ template <> int getTypeId<std::vector<std::string>>                       () { return  7; }
        /*  8 */ template <> int getTypeId<std::vector<SpHandle>>                          () { return  8; }
        /*  9 */ template <> int getTypeId<std::vector<SpStaticStructDesc>>                () { return  9; }
        /* 10 */ template <> int getTypeId<std::vector<SpStaticClassDesc>>                 () { return 10; }
        /* 11 */ template <> int getTypeId<std::map<std::string, std::string>>             () { return 11; }
        /* 12 */ template <> int getTypeId<std::map<std::string, SpArraySharedMemoryView>> () { return 12; }
        /* 13 */ template <> int getTypeId<std::map<std::string, SpPackedArray>>           () { return 13; }
        /* 14 */ template <> int getTypeId<std::map<std::string, SpHandle>>                () { return 14; }
        /* 15 */ template <> int getTypeId<std::map<std::string, SpPropertyValue>>         () { return 15; }
        /* 16 */ template <> int getTypeId<std::map<std::string, SpFuncSignatureDesc>>     () { return 16; }
        /* 17 */ template <> int getTypeId<std::map<std::string, SpWorldDesc>>             () { return 17; }
        /* 18 */ template <> int getTypeId<SpOutputLogMessages>                            () { return 18; }
        /* 19 */ template <> int getTypeId<SpSharedMemoryView>                             () { return 19; }
        /* 20 */ template <> int getTypeId<SpArraySharedMemoryView>                        () { return 20; }
        /* 21 */ template <> int getTypeId<SpPackedArray>                                  () { return 21; }
        /* 22 */ template <> int getTypeId<SpFuncDataBundle>                               () { return 22; }
        /* 23 */ template <> int getTypeId<SpHandle>                                       () { return 23; }
        /* 24 */ template <> int getTypeId<SpPropertyDesc>                                 () { return 24; }
        /* 25 */ template <> int getTypeId<SpPropertyValue>                                () { return 25; }
        /* 26 */ template <> int getTypeId<SpFuncSignatureDesc>                            () { return 26; }
        /* 27 */ template <> int getTypeId<SpFuture>                                       () { return 27; }
        /* 28 */ template <> int getTypeId<SpFunctionDesc>                                 () { return 28; }
        /* 29 */ template <> int getTypeId<SpStaticStructDesc>                             () { return 29; }
        /* 30 */ template <> int getTypeId<SpStaticClassDesc>                              () { return 30; }
        /* 31 */ template <> int getTypeId<SpWorldDesc>                                    () { return 31; }
    #endif

public:
    #if SP_PLATFORM_CLIENT
        static clmdep_msgpack::object getArg(clmdep_msgpack::zone& zone, int type_id, nanobind::handle py_obj)
        {
            switch (type_id) {
                case  1: return clmdep_msgpack::object(nanobind::cast<bool>                                 (py_obj), zone);
                case  2: return clmdep_msgpack::object(nanobind::cast<float>                                (py_obj), zone);
                case  3: return clmdep_msgpack::object(nanobind::cast<int32_t>                              (py_obj), zone);
                case  4: return clmdep_msgpack::object(nanobind::cast<int64_t>                              (py_obj), zone);
                case  5: return clmdep_msgpack::object(nanobind::cast<uint64_t>                             (py_obj), zone);
                case  6: return clmdep_msgpack::object(nanobind::cast<std::string>                          (py_obj), zone);
                case  7: return clmdep_msgpack::object(nanobind::cast<std::vector<std::string>>             (py_obj), zone);
                case  8: return clmdep_msgpack::object(nanobind::cast<std::vector<SpHandle>>                (py_obj), zone);
                case 11: return clmdep_msgpack::object(nanobind::cast<std::map<std::string, std::string>>   (py_obj), zone);
                case 13: return clmdep_msgpack::object(nanobind::cast<std::map<std::string, SpPackedArray>> (py_obj), zone);
                case 14: return clmdep_msgpack::object(nanobind::cast<std::map<std::string, SpHandle>>      (py_obj), zone);
                case 21: return clmdep_msgpack::object(nanobind::cast<SpPackedArray>                        (py_obj), zone);
                case 22: return clmdep_msgpack::object(nanobind::cast<SpFuncDataBundle>                     (py_obj), zone);
                case 23: return clmdep_msgpack::object(nanobind::cast<SpHandle>                             (py_obj), zone);
                case 24: return clmdep_msgpack::object(nanobind::cast<SpPropertyDesc>                       (py_obj), zone);
                case 27: return clmdep_msgpack::object(nanobind::cast<SpFuture>                             (py_obj), zone);
                default: SP_ASSERT(false); return clmdep_msgpack::object();
            }
        }
    #endif

    #if SP_PLATFORM_CLIENT
        static nanobind::object getReturnValue(const SpClientBase* client, int type_id, clmdep_msgpack::object_handle&& result)
        {
            switch (type_id) {
                case  0: return nanobind::none();
                case  1: return nanobind::cast(result.get().as<bool>());
                case  2: return nanobind::cast(result.get().as<float>());
                case  3: return nanobind::cast(result.get().as<int32_t>());
                case  4: return nanobind::cast(result.get().as<int64_t>());
                case  5: return nanobind::cast(result.get().as<uint64_t>());
                case  6: return nanobind::cast(result.get().as<std::string>());
                case  7: return nanobind::cast(result.get().as<std::vector<std::string>>());
                case  8: return nanobind::cast(result.get().as<std::vector<SpHandle>>());
                case  9: return nanobind::cast(result.get().as<std::vector<SpStaticStructDesc>>());
                case 10: return nanobind::cast(result.get().as<std::vector<SpStaticClassDesc>>());
                case 11: return nanobind::cast(result.get().as<std::map<std::string, std::string>>());
                case 12: return nanobind::cast(result.get().as<std::map<std::string, SpArraySharedMemoryView>>());
                case 13: return nanobind::cast(getConvertedReturnValue<std::map<std::string, SpPackedArray>, std::map<std::string, SpPackedArrayView>>(client, std::move(result)));
                case 14: return nanobind::cast(result.get().as<std::map<std::string, SpHandle>>());
                case 15: return nanobind::cast(result.get().as<std::map<std::string, SpPropertyValue>>());
                case 16: return nanobind::cast(result.get().as<std::map<std::string, SpFuncSignatureDesc>>());
                case 17: return nanobind::cast(result.get().as<std::map<std::string, SpWorldDesc>>());
                case 18: return nanobind::cast(result.get().as<SpOutputLogMessages>());
                case 19: return nanobind::cast(result.get().as<SpSharedMemoryView>());
                case 20: return nanobind::cast(result.get().as<SpArraySharedMemoryView>());
                case 21: return nanobind::cast(getConvertedReturnValue<SpPackedArray, SpPackedArrayView>(client, std::move(result)));
                case 22: return nanobind::cast(getConvertedReturnValue<SpFuncDataBundle, SpFuncDataBundleView>(client, std::move(result)));
                case 23: return nanobind::cast(result.get().as<SpHandle>());
                case 24: return nanobind::cast(result.get().as<SpPropertyDesc>());
                case 25: return nanobind::cast(result.get().as<SpPropertyValue>());
                case 26: return nanobind::cast(result.get().as<SpFuncSignatureDesc>());
                case 27: return nanobind::cast(result.get().as<SpFuture>());
                case 28: return nanobind::cast(result.get().as<SpFunctionDesc>());
                case 29: return nanobind::cast(result.get().as<SpStaticStructDesc>());
                case 30: return nanobind::cast(result.get().as<SpStaticClassDesc>());
                case 31: return nanobind::cast(result.get().as<SpWorldDesc>());
                default: SP_ASSERT(false); return nanobind::none();
            }
        }
    #endif

    #if SP_PLATFORM_SERVER || SP_PLATFORM_CLIENT
        static const std::vector<std::string>& getTypeNames()
        {
            return s_type_names_;
        }
    #endif

    #if SP_PLATFORM_SERVER
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

        std::map<std::string, SpFuncSignatureDesc> getFuncSignatureDescs() const
        {
            return func_signature_descs_;
        }
    #endif

private:
    #if SP_PLATFORM_SERVER
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
    #endif

    #if SP_PLATFORM_CLIENT
        template <typename TReturnDest, typename TReturnSrc>
        static TReturnDest getConvertedReturnValue(const SpClientBase* client, clmdep_msgpack::object_handle&& object_handle)
        {
            std::shared_ptr<clmdep_msgpack::object_handle> object_handle_ptr = std::make_shared<clmdep_msgpack::object_handle>(std::move(object_handle));
            TReturnSrc return_value_src = object_handle_ptr->template as<TReturnSrc>(); // .template needed on macOS
            return MsgpackUtils::convert<TReturnDest>(client, std::move(return_value_src), object_handle_ptr);
        };
    #endif
};
