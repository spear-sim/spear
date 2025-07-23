//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stddef.h> // uint64_t

#include <map>
#include <string>
#include <utility> // std::move

#include "SpCore/Assert.h"
#include "SpCore/Std.h"

#include "SpServices/Rpclib.h"

class MsgpackUtils
{
public:
    MsgpackUtils() = delete;
    ~MsgpackUtils() = delete;

    //
    // functions for receiving custom types as args from the client
    //

    static std::map<std::string, clmdep_msgpack::object> toMapOfMsgpackObjects(clmdep_msgpack::object const& object)
    {
        SP_ASSERT(object.type == clmdep_msgpack::type::MAP);
        clmdep_msgpack::object_kv* object_kvs = static_cast<clmdep_msgpack::object_kv*>(object.via.map.ptr);
        std::map<std::string, clmdep_msgpack::object> objects;
        for (unsigned int i = 0; i < object.via.map.size; i++) { // unsigned int needed on Windows
            Std::insert(objects, MsgpackUtils::to<std::string>(object_kvs[i].key), std::move(object_kvs[i].val));
        }
        return objects;
    };

    template <typename T>
    static T to(clmdep_msgpack::object const& object)
    {
        T t;
        object.convert(t);
        return t;
    };

    template <typename T> requires std::is_pointer_v<T>
    static T to(clmdep_msgpack::object const& object)
    {
        SP_ASSERT(object.type == clmdep_msgpack::type::POSITIVE_INTEGER);
        uint64_t ptr;
        object.convert(ptr);
        return reinterpret_cast<T>(ptr);
    };

    //
    // functions for sending custom types as return values to the client
    //

    template <typename T> requires (!std::is_pointer_v<T>)
    static clmdep_msgpack::object toMsgpackObject(const T& value, clmdep_msgpack::object::with_zone& object_with_zone)
    {
        return clmdep_msgpack::object(value, object_with_zone.zone);
    }

    template <typename T> requires std::is_pointer_v<T>
    static clmdep_msgpack::object toMsgpackObject(T ptr, clmdep_msgpack::object::with_zone& object_with_zone)
    {
        return clmdep_msgpack::object(reinterpret_cast<uint64_t>(ptr), object_with_zone.zone);
    }

    static void insertMapOfMsgpackObjects(clmdep_msgpack::object::with_zone& object_with_zone, const std::map<std::string, clmdep_msgpack::object>& objects)
    {
        object_with_zone.type = clmdep_msgpack::type::MAP;
        object_with_zone.via.map.size = objects.size();
        object_with_zone.via.map.ptr = static_cast<clmdep_msgpack::object_kv*>(object_with_zone.zone.allocate_align(sizeof(clmdep_msgpack::object_kv)*object_with_zone.via.map.size));
        int i = 0;
        for (auto& [key, value] : objects) {
            object_with_zone.via.map.ptr[i].key = clmdep_msgpack::object(std::move(key), object_with_zone.zone);
            object_with_zone.via.map.ptr[i].val = std::move(value);
            i++;
        }
    };
};
