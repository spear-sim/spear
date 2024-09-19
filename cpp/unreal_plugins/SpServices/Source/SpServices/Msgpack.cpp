//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpServices/Msgpack.h"

#include <stddef.h> // uint64_t

#include <map>
#include <string>
#include <utility> // std::move

#include "SpCore/Assert.h"
#include "SpCore/Std.h"

#include "SpServices/Rpclib.h"

//
// functions for receiving custom types as args
//

std::map<std::string, clmdep_msgpack::object> Msgpack::toMap(clmdep_msgpack::object const& object)
{
    SP_ASSERT(object.type == clmdep_msgpack::type::MAP);
    clmdep_msgpack::object_kv* object_kvs = static_cast<clmdep_msgpack::object_kv*>(object.via.map.ptr);
    std::map<std::string, clmdep_msgpack::object> objects;
    for (unsigned int i = 0; i < object.via.map.size; i++) { // unsigned int needed on Windows
        Std::insert(objects, Msgpack::to<std::string>(object_kvs[i].key), std::move(object_kvs[i].val));
    }
    return objects;
}

//
// functions for sending custom types as return values
//

clmdep_msgpack::object Msgpack::toObject(void* ptr, clmdep_msgpack::zone& zone)
{
    return clmdep_msgpack::object(reinterpret_cast<uint64_t>(ptr), zone);
}

void Msgpack::toObject(clmdep_msgpack::object::with_zone& object, const std::map<std::string, clmdep_msgpack::object>& objects)
{
    object.type = clmdep_msgpack::type::MAP;
    object.via.map.size = objects.size();
    object.via.map.ptr = static_cast<clmdep_msgpack::object_kv*>(object.zone.allocate_align(sizeof(clmdep_msgpack::object_kv)*object.via.map.size));
    int i = 0;
    for (auto& [key, value] : objects) {
        object.via.map.ptr[i].key = clmdep_msgpack::object(std::move(key), object.zone);
        object.via.map.ptr[i].val = std::move(value);
        i++;
    }
}
