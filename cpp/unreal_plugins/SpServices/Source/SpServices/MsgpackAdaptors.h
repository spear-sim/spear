//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stddef.h> // uint16_t, uint64_t

#include <map>
#include <string>
#include <vector>

#include "SpCore/Assert.h"
#include "SpCore/SpArray.h"
#include "SpCore/SpFuncDataBundle.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"

#include "SpServices/MsgpackUtils.h"
#include "SpServices/Rpclib.h"

//
// SpArraySharedMemoryView
//

template <> // needed to receive a custom type as an arg from the client
struct clmdep_msgpack::adaptor::convert<SpArraySharedMemoryView> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& object, SpArraySharedMemoryView& sp_shared_memory_view) const {
        std::map<std::string, clmdep_msgpack::object> map = MsgpackUtils::toMap(object);
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
        Std::insert(objects, "id",           clmdep_msgpack::object(sp_shared_memory_view.id_,           object_with_zone.zone));
        Std::insert(objects, "num_bytes",    clmdep_msgpack::object(sp_shared_memory_view.num_bytes_,    object_with_zone.zone));
        Std::insert(objects, "offset_bytes", clmdep_msgpack::object(sp_shared_memory_view.offset_bytes_, object_with_zone.zone));
        Std::insert(objects, "usage_flags",  clmdep_msgpack::object(Unreal::getStringsFromCombinedEnumFlagValueAs<ESpArraySharedMemoryUsageFlags>(sp_shared_memory_view.usage_flags_), object_with_zone.zone));
        MsgpackUtils::toMsgpackObject(object_with_zone, objects);
    }
};

//
// SpPackedArray
//

template <> // needed to receive a custom type as an arg from the client
struct clmdep_msgpack::adaptor::convert<SpPackedArray> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& object, SpPackedArray& sp_packed_array) const {
        std::map<std::string, clmdep_msgpack::object> map = MsgpackUtils::toMap(object);
        SP_ASSERT(map.size() == 5);
        sp_packed_array.data_               = MsgpackUtils::to<std::vector<uint8_t, SpPackedArrayAllocator>>(map.at("data"));
        sp_packed_array.data_source_        = Unreal::getEnumValueFromStringAs<SpArrayDataSource, ESpArrayDataSource>(MsgpackUtils::to<std::string>(map.at("data_source")));
        sp_packed_array.shape_              = MsgpackUtils::to<std::vector<uint64_t>>(map.at("shape"));
        sp_packed_array.data_type_          = Unreal::getEnumValueFromStringAs<SpArrayDataType, ESpArrayShortDataType>(MsgpackUtils::to<std::string>(map.at("data_type")));
        sp_packed_array.shared_memory_name_ = MsgpackUtils::to<std::string>(map.at("shared_memory_name"));
        SP_ASSERT(sp_packed_array.data_source_ == SpArrayDataSource::Internal || sp_packed_array.data_source_ == SpArrayDataSource::Shared);
        return object;
    }
};

template <> // needed to send a custom type as a return value to the client
struct clmdep_msgpack::adaptor::object_with_zone<SpPackedArray> {
    void operator()(clmdep_msgpack::object::with_zone& object_with_zone, SpPackedArray const& sp_packed_array) const {
        SP_ASSERT(sp_packed_array.data_source_ == SpArrayDataSource::Internal || sp_packed_array.data_source_ == SpArrayDataSource::Shared);
        std::map<std::string, clmdep_msgpack::object> objects;
        Std::insert(objects, "data",               clmdep_msgpack::object(sp_packed_array.data_, object_with_zone.zone));
        Std::insert(objects, "data_source",        clmdep_msgpack::object(Unreal::getStringFromEnumValueAs<ESpArrayDataSource>(sp_packed_array.data_source_), object_with_zone.zone));
        Std::insert(objects, "shape",              clmdep_msgpack::object(sp_packed_array.shape_, object_with_zone.zone));
        Std::insert(objects, "data_type",          clmdep_msgpack::object(Unreal::getStringFromEnumValueAs<ESpArrayShortDataType>(sp_packed_array.data_type_), object_with_zone.zone));
        Std::insert(objects, "shared_memory_name", clmdep_msgpack::object(sp_packed_array.shared_memory_name_, object_with_zone.zone));
        MsgpackUtils::toMsgpackObject(object_with_zone, objects);
    }
};

//
// SpFuncDataBundle
//

template <> // needed to receive a custom type as an arg from the client
struct clmdep_msgpack::adaptor::convert<SpFuncDataBundle> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& object, SpFuncDataBundle& sp_func_data_bundle) const {
        std::map<std::string, clmdep_msgpack::object> map = MsgpackUtils::toMap(object);
        SP_ASSERT(map.size() == 3);
        sp_func_data_bundle.packed_arrays_      = MsgpackUtils::to<std::map<std::string, SpPackedArray>>(map.at("packed_arrays"));
        sp_func_data_bundle.unreal_obj_strings_ = MsgpackUtils::to<std::map<std::string, std::string>>(map.at("unreal_obj_strings"));
        sp_func_data_bundle.info_               = MsgpackUtils::to<std::string>(map.at("info"));
        return object;
    }
};

template <> // needed to send a custom type as a return value to the client
struct clmdep_msgpack::adaptor::object_with_zone<SpFuncDataBundle> {
    void operator()(clmdep_msgpack::object::with_zone& object_with_zone, SpFuncDataBundle const& sp_func_data_bundle) const {
        std::map<std::string, clmdep_msgpack::object> objects;
        Std::insert(objects, "packed_arrays",      clmdep_msgpack::object(sp_func_data_bundle.packed_arrays_, object_with_zone.zone));
        Std::insert(objects, "unreal_obj_strings", clmdep_msgpack::object(sp_func_data_bundle.unreal_obj_strings_, object_with_zone.zone));
        Std::insert(objects, "info",               clmdep_msgpack::object(sp_func_data_bundle.info_, object_with_zone.zone));
        MsgpackUtils::toMsgpackObject(object_with_zone, objects);
    }
};

//
// Unreal::PropertyDesc
//

template <> // needed to receive a custom type as an arg from the client
struct clmdep_msgpack::adaptor::convert<Unreal::PropertyDesc> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& object, Unreal::PropertyDesc& property_desc) const {
        std::map<std::string, clmdep_msgpack::object> map = MsgpackUtils::toMap(object);
        SP_ASSERT(map.size() == 2);
        property_desc.property_  = MsgpackUtils::toPtr<FProperty>(map.at("property"));
        property_desc.value_ptr_ = MsgpackUtils::toPtr<void>(map.at("value_ptr"));
        return object;
    }
};

template <> // needed to send a custom type as a return value to the client
struct clmdep_msgpack::adaptor::object_with_zone<Unreal::PropertyDesc> {
    void operator()(clmdep_msgpack::object::with_zone& object_with_zone, Unreal::PropertyDesc const& property_desc) const {
        std::map<std::string, clmdep_msgpack::object> objects;
        Std::insert(objects, "property",  MsgpackUtils::toMsgpackObject(property_desc.property_, object_with_zone.zone));
        Std::insert(objects, "value_ptr", MsgpackUtils::toMsgpackObject(property_desc.value_ptr_, object_with_zone.zone));
        MsgpackUtils::toMsgpackObject(object_with_zone, objects);
    }
};
