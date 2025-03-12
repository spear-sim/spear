//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <map>
#include <string>
#include <vector>

#include "SpCore/Assert.h"
#include "SpCore/SpArray.h"
#include "SpCore/SpFuncDataBundle.h"
#include "SpCore/Unreal.h"

#include "SpServices/Msgpack.h"
#include "SpServices/Rpclib.h"

//
// SpArraySharedMemoryView
//

template <> // needed to receive a custom type as an arg
struct clmdep_msgpack::adaptor::convert<SpArraySharedMemoryView> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& object, SpArraySharedMemoryView& shared_memory_view) const {
        std::map<std::string, clmdep_msgpack::object> map = Msgpack::toMap(object);
        SP_ASSERT(map.size() == 3);
        shared_memory_view.id_ = Msgpack::to<std::string>(map.at("id"));
        shared_memory_view.num_bytes_ = Msgpack::to<int>(map.at("num_bytes"));
        shared_memory_view.usage_flags_ = Unreal::getCombinedEnumFlagValueFromStringsAs<SpArraySharedMemoryUsageFlags, ESpArraySharedMemoryUsageFlags>(Msgpack::to<std::vector<std::string>>(map.at("usage_flags")));
        return object;
    }
};

template <> // needed to send a custom type as a return value
struct clmdep_msgpack::adaptor::object_with_zone<SpArraySharedMemoryView> {
    void operator()(clmdep_msgpack::object::with_zone& object, SpArraySharedMemoryView const& shared_memory_view) const {
        Msgpack::toObject(object, {
            {"id", clmdep_msgpack::object(shared_memory_view.id_, object.zone)},
            {"num_bytes", clmdep_msgpack::object(shared_memory_view.num_bytes_, object.zone)},
            {"usage_flags", clmdep_msgpack::object(Unreal::getStringsFromCombinedEnumFlagValueAs<ESpArraySharedMemoryUsageFlags>(shared_memory_view.usage_flags_), object.zone)}});
    }
};

//
// SpPackedArray
//

template <> // needed to receive a custom type as an arg
struct clmdep_msgpack::adaptor::convert<SpPackedArray> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& object, SpPackedArray& packed_array) const {
        std::map<std::string, clmdep_msgpack::object> map = Msgpack::toMap(object);
        SP_ASSERT(map.size() == 5);
        packed_array.data_ = Msgpack::to<std::vector<uint8_t>>(map.at("data"));
        packed_array.data_source_ = Unreal::getEnumValueFromStringAs<SpArrayDataSource, ESpArrayDataSource>(Msgpack::to<std::string>(map.at("data_source")));
        packed_array.shape_ = Msgpack::to<std::vector<uint64_t>>(map.at("shape"));
        packed_array.data_type_ = Unreal::getEnumValueFromStringAs<SpArrayDataType, ESpArrayShortDataType>(Msgpack::to<std::string>(map.at("data_type")));
        packed_array.shared_memory_name_ = Msgpack::to<std::string>(map.at("shared_memory_name"));
        return object;
    }
};

template <> // needed to send a custom type as a return value
struct clmdep_msgpack::adaptor::object_with_zone<SpPackedArray> {
    void operator()(clmdep_msgpack::object::with_zone& object, SpPackedArray const& packed_array) const {
        Msgpack::toObject(object, {
            {"data", clmdep_msgpack::object(packed_array.data_, object.zone)},
            {"data_source", clmdep_msgpack::object(Unreal::getStringFromEnumValueAs<ESpArrayDataSource>(packed_array.data_source_), object.zone)},
            {"shape", clmdep_msgpack::object(packed_array.shape_, object.zone)},
            {"data_type", clmdep_msgpack::object(Unreal::getStringFromEnumValueAs<ESpArrayShortDataType>(packed_array.data_type_), object.zone)},
            {"shared_memory_name", clmdep_msgpack::object(packed_array.shared_memory_name_, object.zone)}});
    }
};

//
// SpFuncDataBundle
//

template <> // needed to receive a custom type as an arg
struct clmdep_msgpack::adaptor::convert<SpFuncDataBundle> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& object, SpFuncDataBundle& data_bundle) const {
        std::map<std::string, clmdep_msgpack::object> map = Msgpack::toMap(object);
        SP_ASSERT(map.size() == 3);
        data_bundle.packed_arrays_ = Msgpack::to<std::map<std::string, SpPackedArray>>(map.at("packed_arrays"));
        data_bundle.unreal_obj_strings_ = Msgpack::to<std::map<std::string, std::string>>(map.at("unreal_obj_strings"));
        data_bundle.info_ = Msgpack::to<std::string>(map.at("info"));
        return object;
    }
};

template <> // needed to send a custom type as a return value
struct clmdep_msgpack::adaptor::object_with_zone<SpFuncDataBundle> {
    void operator()(clmdep_msgpack::object::with_zone& object, SpFuncDataBundle const& data_bundle) const {
        Msgpack::toObject(object, {
            {"packed_arrays", clmdep_msgpack::object(data_bundle.packed_arrays_, object.zone)},
            {"unreal_obj_strings", clmdep_msgpack::object(data_bundle.unreal_obj_strings_, object.zone)},
            {"info", clmdep_msgpack::object(data_bundle.info_, object.zone)}});
    }
};

//
// Unreal::PropertyDesc
//

template <> // needed to receive a custom type as an arg
struct clmdep_msgpack::adaptor::convert<Unreal::PropertyDesc> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& object, Unreal::PropertyDesc& property_desc) const {
        std::map<std::string, clmdep_msgpack::object> map = Msgpack::toMap(object);
        SP_ASSERT(map.size() == 2);
        property_desc.property_ = Msgpack::toPtr<FProperty>(map.at("property"));
        property_desc.value_ptr_ = Msgpack::toPtr<void>(map.at("value_ptr"));
        return object;
    }
};

template <> // needed to send a custom type as a return value
struct clmdep_msgpack::adaptor::object_with_zone<Unreal::PropertyDesc> {
    void operator()(clmdep_msgpack::object::with_zone& object, Unreal::PropertyDesc const& property_desc) const {
        Msgpack::toObject(object, {
            {"property", Msgpack::toObject(property_desc.property_, object.zone)},
            {"value_ptr", Msgpack::toObject(property_desc.value_ptr_, object.zone)}});
    }
};
