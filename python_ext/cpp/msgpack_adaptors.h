//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stddef.h> // size_t
#include <stdint.h> // int8_t, int16_t, int32_t, int64_t, uint8_t, uint16_t, uint32_t, uint64_t

#include <map>
#include <new> // std::align_val_t
#include <span>
#include <string>
#include <vector>

#include <nanobind/nanobind.h>

#include <rpc/msgpack.hpp> // clmdep_msgpack

#include "assert.h"
#include "client.h"
#include "func_signature_registry.h"
#include "msgpack_utils.h"
#include "std.h"
#include "types.h"

//
// Minimal 16-bit floating point data type
//

struct sp_float16_t { uint16_t data; };
template <>
struct nanobind::detail::dtype_traits<sp_float16_t>
{
    //                                       type code,                                       size in bits, SIMD lanes (usually set to 1) 
    static constexpr dlpack::dtype value = { static_cast<uint8_t>(dlpack::dtype_code::Float), 16,           1};
    static constexpr auto name = const_name("sp_float16_t");
};

//
// Minimal aligned allocator
//

template <typename TValue, size_t TAlignmentNumBytes>
struct SpAlignedAllocator
{
    using value_type = TValue;

    static_assert((TAlignmentNumBytes & (TAlignmentNumBytes - 1)) == 0); // TAlignmentNumBytes must be a power of two
    static_assert(TAlignmentNumBytes % alignof(TValue) == 0);            // TAlignmentNumBytes must be a multiple of alignof(TValue)

    template <typename TOtherValue>
    struct rebind { using other = SpAlignedAllocator<TOtherValue, TAlignmentNumBytes>; };

    SpAlignedAllocator() noexcept {};

    template <typename TOtherValue, size_t TOtherAlignmentNumBytes>
    SpAlignedAllocator(const SpAlignedAllocator<TOtherValue, TOtherAlignmentNumBytes>&) noexcept {}

    TValue* allocate(const size_t num_elements) const
    {
        SP_ASSERT(num_elements > 0);
        size_t num_bytes = num_elements*sizeof(TValue);
        TValue* ptr = static_cast<TValue*>(::operator new(num_bytes, std::align_val_t{TAlignmentNumBytes}));
        SP_ASSERT(ptr);
        SP_ASSERT(Std::isPtrSufficientlyAligned(ptr, TAlignmentNumBytes));
        return ptr;
    }

    void deallocate(TValue* const ptr, size_t) const
    {
        SP_ASSERT(ptr);
        ::operator delete(ptr, std::align_val_t{TAlignmentNumBytes});
    }

    template <typename TOtherValue, size_t TOtherAlignmentNumBytes>
    bool operator==(const SpAlignedAllocator<TOtherValue, TOtherAlignmentNumBytes>&) const noexcept { return TAlignmentNumBytes == TOtherAlignmentNumBytes; }

    template <typename TOtherValue, size_t TOtherAlignmentNumBytes>
    bool operator!=(const SpAlignedAllocator<TOtherValue, TOtherAlignmentNumBytes>&) const noexcept { return TAlignmentNumBytes != TOtherAlignmentNumBytes; }
};

//
// Helper functions for implementing adaptors
//

class DataTypeUtils
{
public:
    DataTypeUtils() = delete;
    ~DataTypeUtils() = delete;

    static std::string getDataType(const nanobind::dlpack::dtype& data_type)
    {
        if (data_type == nanobind::dtype<uint8_t>())      return "u1";
        if (data_type == nanobind::dtype<int8_t>())       return "i1";
        if (data_type == nanobind::dtype<uint16_t>())     return "u2";
        if (data_type == nanobind::dtype<int16_t>())      return "i2";
        if (data_type == nanobind::dtype<uint32_t>())     return "u4";
        if (data_type == nanobind::dtype<int32_t>())      return "i4";
        if (data_type == nanobind::dtype<uint64_t>())     return "u8";
        if (data_type == nanobind::dtype<int64_t>())      return "i8";
        if (data_type == nanobind::dtype<sp_float16_t>()) return "f2";
        if (data_type == nanobind::dtype<float>())        return "f4";
        if (data_type == nanobind::dtype<double>())       return "f8";
        SP_ASSERT(false);
        return "Invalid";
    };

    static nanobind::dlpack::dtype getDataType(const std::string& data_type)
    {
        if (data_type == "u1") return nanobind::dtype<uint8_t>();
        if (data_type == "i1") return nanobind::dtype<int8_t>();
        if (data_type == "u2") return nanobind::dtype<uint16_t>();
        if (data_type == "i2") return nanobind::dtype<int16_t>();
        if (data_type == "u4") return nanobind::dtype<uint32_t>();
        if (data_type == "i4") return nanobind::dtype<int32_t>();
        if (data_type == "u8") return nanobind::dtype<uint64_t>();
        if (data_type == "i8") return nanobind::dtype<int64_t>();
        if (data_type == "f2") return nanobind::dtype<sp_float16_t>();
        if (data_type == "f4") return nanobind::dtype<float>();
        if (data_type == "f8") return nanobind::dtype<double>();
        SP_ASSERT(false);
        return nanobind::dtype<nanobind::ndarray<>::Scalar>();
    };
};

//
// PropertyDesc
//

template <> // needed to send a custom type as an arg to the server
struct clmdep_msgpack::adaptor::pack<PropertyDesc> {
    template <typename TStream>
    clmdep_msgpack::packer<TStream>& operator()(clmdep_msgpack::packer<TStream>& packer, PropertyDesc const& property_desc) const {
        packer.pack_map(3);
        packer.pack("property");  packer.pack(property_desc.property_);
        packer.pack("value_ptr"); packer.pack(property_desc.value_ptr_);
        packer.pack("type_id");   packer.pack(property_desc.type_id_);
        return packer;
    }
};

template <> // needed to send a custom type as an arg to the server
struct clmdep_msgpack::adaptor::object_with_zone<PropertyDesc> {
    void operator()(clmdep_msgpack::object::with_zone& o, const PropertyDesc& v) const {
        o.type = clmdep_msgpack::type::MAP;
        o.via.map.size = 3;
        o.via.map.ptr = static_cast<clmdep_msgpack::object_kv*>(o.zone.allocate_align(sizeof(clmdep_msgpack::object_kv) * 3, MSGPACK_ZONE_ALIGNOF(clmdep_msgpack::object_kv)));
        o.via.map.ptr[0].key = clmdep_msgpack::object("property",  o.zone); o.via.map.ptr[0].val = clmdep_msgpack::object(v.property_,  o.zone);
        o.via.map.ptr[1].key = clmdep_msgpack::object("value_ptr", o.zone); o.via.map.ptr[1].val = clmdep_msgpack::object(v.value_ptr_, o.zone);
        o.via.map.ptr[2].key = clmdep_msgpack::object("type_id",   o.zone); o.via.map.ptr[2].val = clmdep_msgpack::object(v.type_id_,   o.zone);
    }
};

template <> // needed to receive a custom type as a return value from the server
struct clmdep_msgpack::adaptor::convert<PropertyDesc> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& object, PropertyDesc& property_desc) const {
        std::map<std::string, clmdep_msgpack::object> objects = MsgpackUtils::toMapOfMsgpackObjects(object);
        SP_ASSERT(objects.size() == 3);
        property_desc.property_  = MsgpackUtils::to<uint64_t>(objects.at("property"));
        property_desc.value_ptr_ = MsgpackUtils::to<uint64_t>(objects.at("value_ptr"));
        property_desc.type_id_   = MsgpackUtils::to<std::string>(objects.at("type_id"));
        return object;
    }
};

//
// PropertyValue (never sent as an arg to the server)
//

template <> // needed to receive a custom type as a return value from the server
struct clmdep_msgpack::adaptor::convert<PropertyValue> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& object, PropertyValue& property_value) const {
        std::map<std::string, clmdep_msgpack::object> objects = MsgpackUtils::toMapOfMsgpackObjects(object);
        SP_ASSERT(objects.size() == 2);
        property_value.value_   = MsgpackUtils::to<std::string>(objects.at("value"));
        property_value.type_id_ = MsgpackUtils::to<std::string>(objects.at("type_id"));
        return object;
    }
};

//
// SharedMemoryView (never sent as an arg to the server)
//

template <> // needed to receive a custom type as a return value from the server
struct clmdep_msgpack::adaptor::convert<SharedMemoryView> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& object, SharedMemoryView& shared_memory_view) const {
        std::map<std::string, clmdep_msgpack::object> objects = MsgpackUtils::toMapOfMsgpackObjects(object);
        SP_ASSERT(objects.size() == 5);
        shared_memory_view.id_           = MsgpackUtils::to<std::string>(objects.at("id"));
        shared_memory_view.num_bytes_    = MsgpackUtils::to<uint64_t>(objects.at("num_bytes"));
        shared_memory_view.offset_bytes_ = MsgpackUtils::to<uint16_t>(objects.at("offset_bytes"));
        shared_memory_view.usage_flags_  = MsgpackUtils::to<std::vector<std::string>>(objects.at("usage_flags"));
        shared_memory_view.name_         = MsgpackUtils::to<std::string>(objects.at("name"));
        return object;
    }
};

//
// PackedArray
//

template<> // needed to send a custom type as an arg to the server
struct clmdep_msgpack::adaptor::pack<PackedArray> {
    template <typename TStream>
    clmdep_msgpack::packer<TStream>& operator()(clmdep_msgpack::packer<TStream>& packer, PackedArray const& packed_array) const {
        packer.pack_map(5);
        packer.pack("data");               packer.pack(clmdep_msgpack::type::raw_ref(static_cast<const char*>(packed_array.data_.data()), packed_array.data_.nbytes()));
        packer.pack("data_source");        packer.pack(packed_array.data_source_);
        packer.pack("shape");              packer.pack(packed_array.shape_);
        packer.pack("shared_memory_name"); packer.pack(packed_array.shared_memory_name_);
        packer.pack("data_type");          packer.pack(DataTypeUtils::getDataType(packed_array.data_.dtype()));
        return packer;
    }
};

template <> // needed to send a custom type as an arg to the server
struct clmdep_msgpack::adaptor::object_with_zone<PackedArray> {
    void operator()(clmdep_msgpack::object::with_zone& o, const PackedArray& v) const {
        o.type = clmdep_msgpack::type::MAP;
        o.via.map.size = 5;
        o.via.map.ptr = static_cast<clmdep_msgpack::object_kv*>(o.zone.allocate_align(sizeof(clmdep_msgpack::object_kv) * 5, MSGPACK_ZONE_ALIGNOF(clmdep_msgpack::object_kv)));
        o.via.map.ptr[0].key = clmdep_msgpack::object("data",               o.zone); o.via.map.ptr[0].val.type = clmdep_msgpack::type::BIN; o.via.map.ptr[0].val.via.bin.ptr = static_cast<const char*>(v.data_.data()); o.via.map.ptr[0].val.via.bin.size = static_cast<uint32_t>(v.data_.nbytes());
        o.via.map.ptr[1].key = clmdep_msgpack::object("data_source",        o.zone); o.via.map.ptr[1].val = clmdep_msgpack::object(v.data_source_,                              o.zone);
        o.via.map.ptr[2].key = clmdep_msgpack::object("shape",              o.zone); o.via.map.ptr[2].val = clmdep_msgpack::object(v.shape_,                                    o.zone);
        o.via.map.ptr[3].key = clmdep_msgpack::object("shared_memory_name", o.zone); o.via.map.ptr[3].val = clmdep_msgpack::object(v.shared_memory_name_,                       o.zone);
        o.via.map.ptr[4].key = clmdep_msgpack::object("data_type",          o.zone); o.via.map.ptr[4].val = clmdep_msgpack::object(DataTypeUtils::getDataType(v.data_.dtype()), o.zone);
    }
};

template <> // needed to receive a custom type as a return value from the server
struct clmdep_msgpack::adaptor::convert<PackedArrayView> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& object, PackedArrayView& packed_array_view) const {

        std::map<std::string, clmdep_msgpack::object> objects = MsgpackUtils::toMapOfMsgpackObjects(object);
        SP_ASSERT(objects.size() == 5);
        packed_array_view.view_               = MsgpackUtils::to<std::span<uint8_t>>(objects.at("data"));
        packed_array_view.data_source_        = MsgpackUtils::to<std::string>(objects.at("data_source"));
        packed_array_view.shape_              = MsgpackUtils::to<std::vector<size_t>>(objects.at("shape"));
        packed_array_view.shared_memory_name_ = MsgpackUtils::to<std::string>(objects.at("shared_memory_name"));
        packed_array_view.data_type_          = MsgpackUtils::to<std::string>(objects.at("data_type"));
        SP_ASSERT(packed_array_view.data_source_ == "Internal" || packed_array_view.data_source_ == "Shared");
        return object;
    }
};

template <> // needed to convert a custom view type, received as a return value from the server, into a custom type that be returned to Python
PackedArray FuncSignatureRegistry::convert<PackedArray>(const Client* client, PackedArrayView&& packed_array_view, std::shared_ptr<clmdep_msgpack::object_handle> object_handle)
{
    SP_ASSERT(client);

    PackedArray packed_array;

    if (client->verbose_allocations_) {
        std::cout << "[SPEAR | spear_ext.cpp] Constructing nanobind::ndarray<nanobind::numpy>..." << std::endl;
        std::cout << "[SPEAR | spear_ext.cpp] packed_array_view.view_.data():        " << packed_array_view.view_.data() << std::endl;
        std::cout << "[SPEAR | spear_ext.cpp] packed_array_view.view_.size():        " << packed_array_view.view_.size() << std::endl;
        std::cout << "[SPEAR | spear_ext.cpp] packed_array_view.data_source_:        " << packed_array_view.data_source_ << std::endl;
        std::cout << "[SPEAR | spear_ext.cpp] packed_array_view.shape_:              "; for (int i = 0; i < packed_array_view.shape_.size(); i++) { std::cout << packed_array_view.shape_.at(i) << " "; } std::cout << std::endl;
        std::cout << "[SPEAR | spear_ext.cpp] packed_array_view.data_type_:          " << packed_array_view.data_type_ << std::endl;
        std::cout << "[SPEAR | spear_ext.cpp] packed_array_view.shared_memory_name_: " << packed_array_view.shared_memory_name_ << std::endl;
    }

    // construct ndarray
    nanobind::ndarray<nanobind::numpy> ndarray;
    if (packed_array_view.data_source_ == "Internal") {

        struct Capsule
        {
            std::vector<uint8_t, SpAlignedAllocator<uint8_t, 4096>> data_;
            std::shared_ptr<clmdep_msgpack::object_handle> object_handle_ = nullptr;
        };

        // Allocate a Capsule on the heap. Deleted in the deletion callback below.
        Capsule* capsule_ptr = new Capsule();
        SP_ASSERT(capsule_ptr);
        if (client->verbose_allocations_) { std::cout << "[SPEAR | spear_ext.cpp] Allocated Capsule object at memory location: " << capsule_ptr << std::endl; }

        void* data_ptr = nullptr;

        // If we're forcing memory alignment, then copy the clmdep_msgpack::object_handle's internal data
        // buffer into the Capsule's internal data buffer, which is guaranteed to be aligned to very large
        // boundaries, and use the Capsule's internal data buffer as the persistent storage for our
        // nanobind array.

        if (client->force_return_aligned_arrays_) {
            Std::resizeUninitialized(capsule_ptr->data_, packed_array_view.view_.size());
            std::memcpy(capsule_ptr->data_.data(), packed_array_view.view_.data(), packed_array_view.view_.size());
            data_ptr = capsule_ptr->data_.data();

        // If we're not forcing memory alignment, then assign to the Capsule's std::shared_ptr to indicate
        // that we want to keep the clmdep_msgpack::object_handle alive, and use the clmdep_msgpack::object_handle's
        // internal data buffer as the persistent storage for our nanobind array.

        } else {
            capsule_ptr->object_handle_ = object_handle;
            data_ptr = packed_array_view.view_.data();
        }

        // Construct a nanobind capsule object that is responsible for deleting our heap-allocated Capsule.
        // In the deletion callback, we assign to our Capsule's std::shared_ptr object to indicate that we
        // don't need to keep the clmdep_msgpack::object_handle alive any more and clean up our
        // heap-allocated Capsule.

        nanobind::capsule capsule = nanobind::capsule(capsule_ptr, [](void* ptr) noexcept -> void {
            if (!ptr) {
                std::cout << "[SPEAR | spear_ext.cpp] ERROR: Unexpected nullptr (ptr) in deletion callback for Capsule." << std::endl;
                std::terminate(); // can't assert because of noexcept
            }
            // if (!client) {
            //     std::cout << "[SPEAR | spear_ext.cpp] ERROR: Unexpected nullptr (client) in deletion callback for Capsule." << std::endl;
            //     std::terminate(); // can't assert because of noexcept
            // }
            Capsule* capsule_ptr = static_cast<Capsule*>(ptr);
            // if (client->verbose_allocations_) { std::cout << "[SPEAR | spear_ext.cpp] Deleting Capsule object at memory location: " << capsule_ptr << std::endl; }
            capsule_ptr->object_handle_ = nullptr;
            delete capsule_ptr;
            capsule_ptr = nullptr;
        });

        ndarray = nanobind::ndarray<nanobind::numpy>(
            data_ptr,                                                  // data_ptr
            packed_array_view.shape_.size(),                           // ndim
            packed_array_view.shape_.data(),                           // shape_ptr
            capsule,                                                   // owner
            nullptr,                                                   // strides
            DataTypeUtils::getDataType(packed_array_view.data_type_)); // dtype

    } else if (packed_array_view.data_source_ == "Shared") {

        ndarray = nanobind::ndarray<nanobind::numpy>(
            nullptr,                                                   // data_ptr
            0,                                                         // ndim
            nullptr,                                                   // shape_ptr
            {},                                                        // owner
            nullptr,                                                   // strides
            DataTypeUtils::getDataType(packed_array_view.data_type_)); // dtype

    } else {
        SP_ASSERT(false);
    }

    packed_array.data_               = std::move(ndarray);
    packed_array.data_source_        = std::move(packed_array_view.data_source_);
    packed_array.shape_              = std::move(packed_array_view.shape_);
    packed_array.shared_memory_name_ = std::move(packed_array_view.shared_memory_name_);

    return packed_array;
};

//
// std::map<std::string, PackedArray>
//

template <> // needed to convert a custom view type, received as a return value from the server, into a custom type that be returned to Python
std::map<std::string, PackedArray> FuncSignatureRegistry::convert<std::map<std::string, PackedArray>>(const Client* client, std::map<std::string, PackedArrayView>&& packed_array_views, std::shared_ptr<clmdep_msgpack::object_handle> object_handle)
{
    std::map<std::string, PackedArray> packed_arrays;
    for (auto& [name, packed_array_view] : packed_array_views) {
        auto [itr, success] = packed_arrays.insert({std::move(name), convert<PackedArray>(client, std::move(packed_array_view), object_handle)});
        SP_ASSERT(success); // will only succeed if key wasn't already present
    }
    return packed_arrays;
};

//
// DataBundle
//

template <> // needed to send a custom type as an arg to the server
struct clmdep_msgpack::adaptor::pack<DataBundle> {
    template <typename TStream>
    clmdep_msgpack::packer<TStream>& operator()(clmdep_msgpack::packer<TStream>& packer, DataBundle const& data_bundle) const {
        packer.pack_map(3);
        packer.pack("packed_arrays");      packer.pack(data_bundle.packed_arrays_);
        packer.pack("unreal_obj_strings"); packer.pack(data_bundle.unreal_obj_strings_);
        packer.pack("info");               packer.pack(data_bundle.info_);
        return packer;
    }
};

template <> // needed to send a custom type as an arg to the server via dynamic dispatch
struct clmdep_msgpack::adaptor::object_with_zone<DataBundle> {
    void operator()(clmdep_msgpack::object::with_zone& o, const DataBundle& v) const {
        o.type = clmdep_msgpack::type::MAP;
        o.via.map.size = 3;
        o.via.map.ptr = static_cast<clmdep_msgpack::object_kv*>(o.zone.allocate_align(sizeof(clmdep_msgpack::object_kv) * 3, MSGPACK_ZONE_ALIGNOF(clmdep_msgpack::object_kv)));
        o.via.map.ptr[0].key = clmdep_msgpack::object("packed_arrays",      o.zone); o.via.map.ptr[0].val = clmdep_msgpack::object(v.packed_arrays_,      o.zone);
        o.via.map.ptr[1].key = clmdep_msgpack::object("unreal_obj_strings", o.zone); o.via.map.ptr[1].val = clmdep_msgpack::object(v.unreal_obj_strings_, o.zone);
        o.via.map.ptr[2].key = clmdep_msgpack::object("info",               o.zone); o.via.map.ptr[2].val = clmdep_msgpack::object(v.info_,               o.zone);
    }
};

template <> // needed to receive a custom type as a return value from the server
struct clmdep_msgpack::adaptor::convert<DataBundleView> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& object, DataBundleView& data_bundle_view) const {
        std::map<std::string, clmdep_msgpack::object> objects = MsgpackUtils::toMapOfMsgpackObjects(object);
        SP_ASSERT(objects.size() == 3);
        data_bundle_view.packed_array_views_ = MsgpackUtils::to<std::map<std::string, PackedArrayView>>(objects.at("packed_arrays"));
        data_bundle_view.unreal_obj_strings_ = MsgpackUtils::to<std::map<std::string, std::string>>(objects.at("unreal_obj_strings"));
        data_bundle_view.info_               = MsgpackUtils::to<std::string>(objects.at("info"));
        return object;
    }
};

template <> // needed to convert a custom view type, received as a return value from the server, into a custom type that be returned to Python
DataBundle FuncSignatureRegistry::convert<DataBundle>(const Client* client, DataBundleView&& data_bundle_view, std::shared_ptr<clmdep_msgpack::object_handle> object_handle)
{
    DataBundle data_bundle;
    data_bundle.packed_arrays_      = convert<std::map<std::string, PackedArray>>(client, std::move(data_bundle_view.packed_array_views_), object_handle);
    data_bundle.unreal_obj_strings_ = std::move(data_bundle_view.unreal_obj_strings_);
    data_bundle.info_               = std::move(data_bundle_view.info_);
    return data_bundle;
};

//
// WorldDesc (never sent as an arg to the server)
//

template <> // needed to receive a custom type as a return value from the server
struct clmdep_msgpack::adaptor::convert<WorldDesc> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& object, WorldDesc& world_desc) const {
        std::map<std::string, clmdep_msgpack::object> objects = MsgpackUtils::toMapOfMsgpackObjects(object);
        SP_ASSERT(objects.size() == 5);
        world_desc.world_           = MsgpackUtils::to<uint64_t>(objects.at("world"));
        world_desc.world_id_        = MsgpackUtils::to<int64_t>(objects.at("world_id"));
        world_desc.is_editor_world_ = MsgpackUtils::to<bool>(objects.at("is_editor_world"));
        world_desc.is_game_world_   = MsgpackUtils::to<bool>(objects.at("is_game_world"));
        world_desc.is_playing_      = MsgpackUtils::to<bool>(objects.at("is_playing"));
        return object;
    }
};

//
// FuncSignatureDesc (never sent as an arg to the server)
//

template <> // needed to receive a custom type as a return value from the server
struct clmdep_msgpack::adaptor::convert<FuncSignatureDesc> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& object, FuncSignatureDesc& func_signature_desc) const {
        std::map<std::string, clmdep_msgpack::object> objects = MsgpackUtils::toMapOfMsgpackObjects(object);
        SP_ASSERT(objects.size() == 3);
        func_signature_desc.name_       = MsgpackUtils::to<std::string>(objects.at("name"));
        func_signature_desc.type_ids_   = MsgpackUtils::to<std::vector<int>>(objects.at("type_ids"));
        func_signature_desc.type_names_ = MsgpackUtils::to<std::vector<std::string>>(objects.at("type_names"));
        return object;
    }
};

//
// Future
//

template <> // needed to send a custom type as an arg to the server
struct clmdep_msgpack::adaptor::pack<Future> {
    template <typename TStream>
    clmdep_msgpack::packer<TStream>& operator()(clmdep_msgpack::packer<TStream>& packer, Future const& future) const {
        packer.pack_map(2);
        packer.pack("future_ptr"); packer.pack(future.future_ptr_);
        packer.pack("type_id");    packer.pack(future.type_id_);
        return packer;
    }
};

template <> // needed to send a custom type as an arg to the server via dynamic dispatch
struct clmdep_msgpack::adaptor::object_with_zone<Future> {
    void operator()(clmdep_msgpack::object::with_zone& o, const Future& v) const {
        o.type = clmdep_msgpack::type::MAP;
        o.via.map.size = 2;
        o.via.map.ptr = static_cast<clmdep_msgpack::object_kv*>(o.zone.allocate_align(sizeof(clmdep_msgpack::object_kv) * 2, MSGPACK_ZONE_ALIGNOF(clmdep_msgpack::object_kv)));
        o.via.map.ptr[0].key = clmdep_msgpack::object("future_ptr", o.zone); o.via.map.ptr[0].val = clmdep_msgpack::object(v.future_ptr_, o.zone);
        o.via.map.ptr[1].key = clmdep_msgpack::object("type_id",    o.zone); o.via.map.ptr[1].val = clmdep_msgpack::object(v.type_id_,    o.zone);
    }
};

template <> // needed to receive a custom type as a return value from the server
struct clmdep_msgpack::adaptor::convert<Future> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& object, Future& future) const {
        std::map<std::string, clmdep_msgpack::object> objects = MsgpackUtils::toMapOfMsgpackObjects(object);
        SP_ASSERT(objects.size() == 2);
        future.future_ptr_ = MsgpackUtils::to<uint64_t>(objects.at("future_ptr"));
        future.type_id_    = MsgpackUtils::to<std::string>(objects.at("type_id"));
        return object;
    }
};

//
// FunctionDesc (never sent as an arg to the server)
//

template <> // needed to receive a custom type as a return value from the server
struct clmdep_msgpack::adaptor::convert<FunctionDesc> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& object, FunctionDesc& function_desc) const {
        std::map<std::string, clmdep_msgpack::object> objects = MsgpackUtils::toMapOfMsgpackObjects(object);
        SP_ASSERT(objects.size() == 4);
        function_desc.function_          = MsgpackUtils::to<uint64_t>(objects.at("function"));
        function_desc.function_name_     = MsgpackUtils::to<std::string>(objects.at("function_name"));
        function_desc.static_class_      = MsgpackUtils::to<uint64_t>(objects.at("static_class"));
        function_desc.static_class_name_ = MsgpackUtils::to<std::string>(objects.at("static_class_name"));
        return object;
    }
};

//
// StaticStructDesc (never sent as an arg to the server)
//

template <> // needed to receive a custom type as a return value from the server
struct clmdep_msgpack::adaptor::convert<StaticStructDesc> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& object, StaticStructDesc& static_struct_desc) const {
        std::map<std::string, clmdep_msgpack::object> objects = MsgpackUtils::toMapOfMsgpackObjects(object);
        SP_ASSERT(objects.size() == 2);
        static_struct_desc.static_struct_ = MsgpackUtils::to<uint64_t>(objects.at("static_struct"));
        static_struct_desc.name_          = MsgpackUtils::to<std::string>(objects.at("name"));
        return object;
    }
};

//
// StaticClassDesc (never sent as an arg to the server)
//

template <> // needed to receive a custom type as a return value from the server
struct clmdep_msgpack::adaptor::convert<StaticClassDesc> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& object, StaticClassDesc& static_class_desc) const {
        std::map<std::string, clmdep_msgpack::object> objects = MsgpackUtils::toMapOfMsgpackObjects(object);
        SP_ASSERT(objects.size() == 5);
        static_class_desc.static_class_        = MsgpackUtils::to<uint64_t>(objects.at("static_class"));
        static_class_desc.name_                = MsgpackUtils::to<std::string>(objects.at("name"));
        static_class_desc.derived_classes_     = MsgpackUtils::to<std::vector<uint64_t>>(objects.at("derived_classes"));
        static_class_desc.derived_class_names_ = MsgpackUtils::to<std::vector<std::string>>(objects.at("derived_class_names"));
        static_class_desc.function_descs_      = MsgpackUtils::to<std::map<std::string, FunctionDesc>>(objects.at("function_descs"));
        return object;
    }
};
