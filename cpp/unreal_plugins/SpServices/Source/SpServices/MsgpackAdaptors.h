//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stddef.h> // uint16_t, uint64_t

#include <map>
#include <string>
#include <vector>

#include "SpCore/Assert.h"
#include "SpCore/SpArray.h"
#include "SpCore/SpTypes.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"

#include "SpServices/MsgpackUtils.h"
#include "SpServices/Rpclib.h"

//
// SpCore/SpTypes.h
//

// SpFuncDataBundle

template <> // needed to receive a custom type as an arg from the client
struct clmdep_msgpack::adaptor::convert<SpFuncDataBundle> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& object, SpFuncDataBundle& sp_func_data_bundle) const {
        std::map<std::string, clmdep_msgpack::object> objects = MsgpackUtils::toMapOfMsgpackObjects(object);
        SP_ASSERT(objects.size() == 3);
        sp_func_data_bundle.packed_arrays_      = MsgpackUtils::to<std::map<std::string, SpPackedArray>>(objects.at("packed_arrays"));
        sp_func_data_bundle.unreal_obj_strings_ = MsgpackUtils::to<std::map<std::string, std::string>>(objects.at("unreal_obj_strings"));
        sp_func_data_bundle.info_               = MsgpackUtils::to<std::string>(objects.at("info"));
        return object;
    }
};

template <> // needed to send a custom type as a return value to the client
struct clmdep_msgpack::adaptor::object_with_zone<SpFuncDataBundle> {
    void operator()(clmdep_msgpack::object::with_zone& object_with_zone, SpFuncDataBundle const& sp_func_data_bundle) const {
        std::map<std::string, clmdep_msgpack::object> objects;
        Std::insert(objects, "packed_arrays",      MsgpackUtils::toMsgpackObject(sp_func_data_bundle.packed_arrays_,      object_with_zone));
        Std::insert(objects, "unreal_obj_strings", MsgpackUtils::toMsgpackObject(sp_func_data_bundle.unreal_obj_strings_, object_with_zone));
        Std::insert(objects, "info",               MsgpackUtils::toMsgpackObject(sp_func_data_bundle.info_,               object_with_zone));
        MsgpackUtils::insertMapOfMsgpackObjects(object_with_zone, objects);
    }
};

// SpFuture

template <> // needed to receive a custom type as an arg from the client
struct clmdep_msgpack::adaptor::convert<SpFuture> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& object, SpFuture& future) const {
        std::map<std::string, clmdep_msgpack::object> objects = MsgpackUtils::toMapOfMsgpackObjects(object);
        SP_ASSERT(objects.size() == 2);
        future.future_ptr_ = MsgpackUtils::to<void*>(objects.at("future_ptr"));
        future.type_id_    = MsgpackUtils::to<std::string>(objects.at("type_id"));
        return object;
    }
};

template <> // needed to send a custom type as a return value to the client
struct clmdep_msgpack::adaptor::object_with_zone<SpFuture> {
    void operator()(clmdep_msgpack::object::with_zone& object_with_zone, SpFuture const& future) const {
        std::map<std::string, clmdep_msgpack::object> objects;
        Std::insert(objects, "future_ptr", MsgpackUtils::toMsgpackObject(future.future_ptr_, object_with_zone));
        Std::insert(objects, "type_id",    MsgpackUtils::toMsgpackObject(future.type_id_,    object_with_zone));
        MsgpackUtils::insertMapOfMsgpackObjects(object_with_zone, objects);
    }
};

//
// SpCore/SpArray.h
//

// SpPackedArray

template <> // needed to receive a custom type as an arg from the client
struct clmdep_msgpack::adaptor::convert<SpPackedArray> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& object, SpPackedArray& sp_packed_array) const {
        std::map<std::string, clmdep_msgpack::object> objects = MsgpackUtils::toMapOfMsgpackObjects(object);
        SP_ASSERT(objects.size() == 5);
        sp_packed_array.data_               = MsgpackUtils::to<std::vector<uint8_t, SpPackedArrayAllocator>>(objects.at("data"));
        sp_packed_array.data_source_        = Unreal::getEnumValueFromStringAs<SpArrayDataSource, ESpArrayDataSource>(MsgpackUtils::to<std::string>(objects.at("data_source")));
        sp_packed_array.shape_              = MsgpackUtils::to<std::vector<uint64_t>>(objects.at("shape"));
        sp_packed_array.data_type_          = Unreal::getEnumValueFromStringAs<SpArrayDataType, ESpArrayShortDataType>(MsgpackUtils::to<std::string>(objects.at("data_type")));
        sp_packed_array.shared_memory_name_ = MsgpackUtils::to<std::string>(objects.at("shared_memory_name"));
        SP_ASSERT(sp_packed_array.data_source_ == SpArrayDataSource::Internal || sp_packed_array.data_source_ == SpArrayDataSource::Shared);
        return object;
    }
};

template <> // needed to send a custom type as a return value to the client
struct clmdep_msgpack::adaptor::object_with_zone<SpPackedArray> {
    void operator()(clmdep_msgpack::object::with_zone& object_with_zone, SpPackedArray const& sp_packed_array) const {
        SP_ASSERT(sp_packed_array.data_source_ == SpArrayDataSource::Internal || sp_packed_array.data_source_ == SpArrayDataSource::Shared);
        std::map<std::string, clmdep_msgpack::object> objects;
        Std::insert(objects, "data",               MsgpackUtils::toMsgpackObject(sp_packed_array.data_,                                                               object_with_zone));
        Std::insert(objects, "data_source",        MsgpackUtils::toMsgpackObject(Unreal::getStringFromEnumValueAs<ESpArrayDataSource>(sp_packed_array.data_source_),  object_with_zone));
        Std::insert(objects, "shape",              MsgpackUtils::toMsgpackObject(sp_packed_array.shape_,                                                              object_with_zone));
        Std::insert(objects, "data_type",          MsgpackUtils::toMsgpackObject(Unreal::getStringFromEnumValueAs<ESpArrayShortDataType>(sp_packed_array.data_type_), object_with_zone));
        Std::insert(objects, "shared_memory_name", MsgpackUtils::toMsgpackObject(sp_packed_array.shared_memory_name_,                                                 object_with_zone));
        MsgpackUtils::insertMapOfMsgpackObjects(object_with_zone, objects);
    }
};

// SpArraySharedMemoryView

template <> // needed to receive a custom type as an arg from the client
struct clmdep_msgpack::adaptor::convert<SpArraySharedMemoryView> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& object, SpArraySharedMemoryView& sp_shared_memory_view) const {
        std::map<std::string, clmdep_msgpack::object> map = MsgpackUtils::toMapOfMsgpackObjects(object);
        SP_ASSERT(map.size() == 4);
        sp_shared_memory_view.id_           = MsgpackUtils::to<std::string>(map.at("id"));
        sp_shared_memory_view.num_bytes_    = MsgpackUtils::to<uint64_t>(map.at("num_bytes"));
        sp_shared_memory_view.offset_bytes_ = MsgpackUtils::to<uint16_t>(map.at("offset_bytes"));
        sp_shared_memory_view.usage_flags_  = Unreal::getCombinedEnumFlagValueFromStringsAs<SpArraySharedMemoryUsageFlags, ESpArraySharedMemoryUsageFlags>(MsgpackUtils::to<std::vector<std::string>>(map.at("usage_flags")));
        return object;
    }
};

template <> // needed to send a custom type as a return value to the client
struct clmdep_msgpack::adaptor::object_with_zone<SpArraySharedMemoryView> {
    void operator()(clmdep_msgpack::object::with_zone& object_with_zone, SpArraySharedMemoryView const& sp_shared_memory_view) const {
        std::map<std::string, clmdep_msgpack::object> objects;
        Std::insert(objects, "id",           MsgpackUtils::toMsgpackObject(sp_shared_memory_view.id_,           object_with_zone));
        Std::insert(objects, "num_bytes",    MsgpackUtils::toMsgpackObject(sp_shared_memory_view.num_bytes_,    object_with_zone));
        Std::insert(objects, "offset_bytes", MsgpackUtils::toMsgpackObject(sp_shared_memory_view.offset_bytes_, object_with_zone));
        Std::insert(objects, "usage_flags",  MsgpackUtils::toMsgpackObject(Unreal::getStringsFromCombinedEnumFlagValueAs<ESpArraySharedMemoryUsageFlags>(sp_shared_memory_view.usage_flags_), object_with_zone));
        MsgpackUtils::insertMapOfMsgpackObjects(object_with_zone, objects);
    }
};

//
// SpCore/Unreal.h
//

// SpPropertyDesc

template <> // needed to receive a custom type as an arg from the client
struct clmdep_msgpack::adaptor::convert<SpPropertyDesc> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& object, SpPropertyDesc& property_desc) const {
        std::map<std::string, clmdep_msgpack::object> map = MsgpackUtils::toMapOfMsgpackObjects(object);
        SP_ASSERT(map.size() == 2);
        property_desc.property_  = MsgpackUtils::to<FProperty*>(map.at("property"));
        property_desc.value_ptr_ = MsgpackUtils::to<void*>(map.at("value_ptr"));
        return object;
    }
};

template <> // needed to send a custom type as a return value to the client
struct clmdep_msgpack::adaptor::object_with_zone<SpPropertyDesc> {
    void operator()(clmdep_msgpack::object::with_zone& object_with_zone, SpPropertyDesc const& property_desc) const {
        std::map<std::string, clmdep_msgpack::object> objects;
        Std::insert(objects, "property",  MsgpackUtils::toMsgpackObject(property_desc.property_,  object_with_zone));
        Std::insert(objects, "value_ptr", MsgpackUtils::toMsgpackObject(property_desc.value_ptr_, object_with_zone));
        MsgpackUtils::insertMapOfMsgpackObjects(object_with_zone, objects);
    }
};
