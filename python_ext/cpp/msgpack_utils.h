//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // uint8_t

#include <map>
#include <span>
#include <string>
#include <utility> // std::move

#include <rpc/msgpack.hpp> // clmdep_msgpack

class MsgpackUtils
{
public:
    MsgpackUtils() = delete;
    ~MsgpackUtils() = delete;

    //
    // functions for receiving custom types as return values from the server
    //

    static std::map<std::string, clmdep_msgpack::object> toMapOfMsgpackObjects(clmdep_msgpack::object const& object)
    {
        SP_ASSERT(object.type == clmdep_msgpack::type::MAP);
        clmdep_msgpack::object_kv* object_kvs = static_cast<clmdep_msgpack::object_kv*>(object.via.map.ptr);
        std::map<std::string, clmdep_msgpack::object> objects;
        for (unsigned int i = 0; i < object.via.map.size; i++) { // unsigned int needed on Windows
            auto [itr, success] = objects.insert({MsgpackUtils::to<std::string>(object_kvs[i].key), std::move(object_kvs[i].val)});
            SP_ASSERT(success); // will only succeed if key wasn't already present
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

    template <>
    std::span<uint8_t> to<std::span<uint8_t>>(clmdep_msgpack::object const& object)
    {
        SP_ASSERT(object.type == clmdep_msgpack::type::BIN);
        return std::span<uint8_t>(reinterpret_cast<uint8_t*>(const_cast<char*>(object.via.bin.ptr)), object.via.bin.size);
    }
};
