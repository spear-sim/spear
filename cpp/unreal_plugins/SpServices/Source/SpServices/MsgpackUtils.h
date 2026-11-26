//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

//
// This header participates in SP_PLATFORM_PURE builds.
//

#pragma once

#include <stdint.h> // uint8_t, uint64_t

#include <map>
#include <memory>      // std::shared_ptr
#include <span>
#include <string>
#include <type_traits> // std::is_pointer_v, std::remove_pointer_t
#include <utility>     // std::move
#include <vector>

#include "SpCore/Assert.h"
#include "SpCore/Std.h"

#include "SpServices/Platform.h"
#include "SpServices/Rpclib.h"

#if SP_PLATFORM_SERVER
    #include "SpCore/Unreal.h"
#elif SP_PLATFORM_CLIENT
    #include "SpServices/SpTypes.h"
#else
    #error
#endif

class MsgpackUtils
{
public:
    MsgpackUtils() = delete;
    ~MsgpackUtils() = delete;

    //
    // functions for receiving custom types, i.e., as args on the server or as return values on the client
    //

    #if SP_PLATFORM_SERVER
        static std::vector<clmdep_msgpack::object> toVectorOfMsgpackObjects(clmdep_msgpack::object const& object)
        {
            SP_ASSERT(object.type == clmdep_msgpack::type::ARRAY);
            clmdep_msgpack::object* objects_ptr = static_cast<clmdep_msgpack::object*>(object.via.array.ptr);
            SP_ASSERT(objects_ptr);
            std::vector<clmdep_msgpack::object> objects;
            for (unsigned int i = 0; i < object.via.array.size; i++) { // unsigned int needed on Windows
                objects.push_back(std::move(objects_ptr[i]));
            }
            return objects;
        }
    #endif

    #if SP_PLATFORM_SERVER || SP_PLATFORM_CLIENT
        static std::map<std::string, clmdep_msgpack::object> toMapOfMsgpackObjects(clmdep_msgpack::object const& object)
        {
            SP_ASSERT(object.type == clmdep_msgpack::type::MAP);
            clmdep_msgpack::object_kv* object_kvs = static_cast<clmdep_msgpack::object_kv*>(object.via.map.ptr);
            std::map<std::string, clmdep_msgpack::object> objects;
            for (unsigned int i = 0; i < object.via.map.size; i++) { // unsigned int needed on Windows
                Std::insert(objects, MsgpackUtils::to<std::string>(object_kvs[i].key), std::move(object_kvs[i].val));
            }
            return objects;
        }
    #endif

    #if SP_PLATFORM_SERVER || SP_PLATFORM_CLIENT
        template <typename TNonPtr> requires (!std::is_pointer_v<TNonPtr>)
        static TNonPtr to(clmdep_msgpack::object const& object)
        {
            TNonPtr t;
            object.convert(t);
            return t;
        }
    #endif

    #if SP_PLATFORM_SERVER
        template <typename TPtr> requires std::is_pointer_v<TPtr> && CNonObject<std::remove_pointer_t<TPtr>> // UObject pointers must be sent as SpHandles
        static TPtr to(clmdep_msgpack::object const& object)
        {
            SP_ASSERT(object.type == clmdep_msgpack::type::POSITIVE_INTEGER);
            uint64_t value;
            object.convert(value);
            return reinterpret_cast<TPtr>(value);
        }
    #endif

    // returns a zero-copy view into object, so the caller must keep the underlying object_handle alive for as long as the view is in use
    #if SP_PLATFORM_CLIENT
        template <>
        std::span<uint8_t> to<std::span<uint8_t>>(clmdep_msgpack::object const& object)
        {
            SP_ASSERT(object.type == clmdep_msgpack::type::BIN);
            return std::span<uint8_t>(reinterpret_cast<uint8_t*>(const_cast<char*>(object.via.bin.ptr)), object.via.bin.size);
        }
    #endif

    #if SP_PLATFORM_SERVER
        template <typename T>
        static std::vector<T> toVector(const std::vector<clmdep_msgpack::object>& objects)
        {
            std::vector<T> values;
            for (auto& object : objects) {
                values.push_back(to<T>(object));
            }
            return values;
        }
    #endif

    #if SP_PLATFORM_SERVER
        template <typename T>
        static std::map<std::string, T> toMap(const std::map<std::string, clmdep_msgpack::object>& objects)
        {
            std::map<std::string, T> values;
            for (auto& [name, object] : objects) {
                Std::insert(values, name, to<T>(object));
            }
            return values;
        }
    #endif

    //
    // functions for sending custom types as return values from the server
    //

    #if SP_PLATFORM_SERVER
        template <typename TNonPtr> requires (!std::is_pointer_v<TNonPtr>)
        static clmdep_msgpack::object toMsgpackObject(const TNonPtr& value, clmdep_msgpack::object::with_zone& object_with_zone)
        {
            return clmdep_msgpack::object(value, object_with_zone.zone);
        }
    #endif

    #if SP_PLATFORM_SERVER
        template <typename TPtr> requires std::is_pointer_v<TPtr> && CNonObject<std::remove_pointer_t<TPtr>> // UObject pointers must be sent as SpHandles
        static clmdep_msgpack::object toMsgpackObject(TPtr value, clmdep_msgpack::object::with_zone& object_with_zone)
        {
            return clmdep_msgpack::object(reinterpret_cast<uint64_t>(value), object_with_zone.zone);
        }
    #endif

    #if SP_PLATFORM_SERVER
        template <typename TPtr> requires std::is_pointer_v<TPtr> && CNonObject<std::remove_pointer_t<TPtr>> // UObject pointers must be sent as SpHandles
        static clmdep_msgpack::object toVectorMsgpackObject(const std::vector<TPtr>& values, clmdep_msgpack::object::with_zone& object_with_zone)
        {
            std::vector<uint64_t> ptrs;
            for (auto value : values) {
                ptrs.push_back(reinterpret_cast<uint64_t>(value));
            }
            return clmdep_msgpack::object(ptrs, object_with_zone.zone);
        }
    #endif

    #if SP_PLATFORM_SERVER
        template <typename TPtr> requires std::is_pointer_v<TPtr> && CNonObject<std::remove_pointer_t<TPtr>> // UObject pointers must be sent as SpHandles
        static clmdep_msgpack::object toMapMsgpackObject(const std::map<std::string, TPtr>& values, clmdep_msgpack::object::with_zone& object_with_zone)
        {
            std::map<std::string, uint64_t> ptrs;
            for (auto& [name, value] : values) {
                Std::insert(ptrs, name, reinterpret_cast<uint64_t>(value));
            }
            return clmdep_msgpack::object(ptrs, object_with_zone.zone);
        }
    #endif

    #if SP_PLATFORM_SERVER
        static void setMsgpackObjectToMap(clmdep_msgpack::object::with_zone& object_with_zone, const std::map<std::string, clmdep_msgpack::object>& objects)
        {
            object_with_zone.type = clmdep_msgpack::type::MAP;
            object_with_zone.via.map.size = objects.size();
            object_with_zone.via.map.ptr = static_cast<clmdep_msgpack::object_kv*>(object_with_zone.zone.allocate_align(sizeof(clmdep_msgpack::object_kv)*object_with_zone.via.map.size));
            int i = 0;
            for (auto& [key, value] : objects) {
                object_with_zone.via.map.ptr[i].key = clmdep_msgpack::object(key, object_with_zone.zone);
                object_with_zone.via.map.ptr[i].val = value;
                i++;
            }
        }
    #endif

    //
    // functions for converting custom view types, received as return values on the client, into custom types that
    // can be returned to Python, specialized for each type in SpServices/MsgpackAdaptors.h
    //

    #if SP_PLATFORM_CLIENT
        template <typename TDest, typename TSrc>
        static TDest convert(const SpClientBase* client, TSrc&& src, std::shared_ptr<clmdep_msgpack::object_handle> object_handle) = delete;
    #endif
};
