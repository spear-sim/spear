//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stddef.h> // uint64_t

#include <map>
#include <string>

#include "SpCore/Assert.h"

#include "SpServices/Rpclib.h"

class Msgpack
{
public:
    Msgpack() = delete;
    ~Msgpack() = delete;

    //
    // functions for receiving custom types as args
    //

    static std::map<std::string, clmdep_msgpack::object> toMap(clmdep_msgpack::object const& object);

    template <typename T>
    static T to(clmdep_msgpack::object const& object)
    {
        T t;
        object.convert(t);
        return t;
    };

    template <typename T>
    static T* toPtr(clmdep_msgpack::object const& object)
    {
        SP_ASSERT(object.type == clmdep_msgpack::type::POSITIVE_INTEGER);
        uint64_t ptr;
        object.convert(ptr);
        return reinterpret_cast<T*>(ptr);
    };

    //
    // functions for sending custom types as return values
    //

    static clmdep_msgpack::object toObject(const void* ptr, clmdep_msgpack::zone& zone); // use instead of clmdep_msgpack::object(ptr, zone) for pointers
    static void toObject(clmdep_msgpack::object::with_zone& object, const std::map<std::string, clmdep_msgpack::object>& objects);
};
