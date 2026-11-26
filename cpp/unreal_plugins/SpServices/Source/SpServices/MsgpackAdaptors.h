//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

//
// This header participates in SP_PLATFORM_PURE builds.
//

#pragma once

#include <stddef.h> // size_t
#include <stdint.h> // int8_t, int16_t, int32_t, int64_t, uint8_t, uint16_t, uint32_t, uint64_t

#include <map>
#include <string>
#include <vector>

#include "SpCore/Assert.h"
#include "SpCore/SpTypes.h"
#include "SpCore/Std.h"

#include "SpServices/MsgpackUtils.h"
#include "SpServices/Platform.h"
#include "SpServices/Rpclib.h"
#include "SpServices/SpTypes.h"

#if SP_PLATFORM_SERVER
    #include "SpCore/SpArray.h" // ESpArrayDataSource, ESpArrayShortDataType, ESpArraySharedMemoryUsageFlags
    #include "SpCore/Unreal.h"
#elif SP_PLATFORM_CLIENT
    #include <cstring>   // std::memcpy
    #include <exception> // std::terminate
    #include <iostream>
    #include <memory>    // std::make_unique, std::shared_ptr, std::unique_ptr
    #include <span>
    #include <utility>   // std::move

    #include <nanobind/nanobind.h>
    #include <nanobind/ndarray.h>
#else
    #error
#endif

#if SP_PLATFORM_SERVER
    class FProperty;
#endif

//
// This header defines the msgpack adaptors for the mirrored types in SpCore/SpTypes.h and SpServices/SpTypes.h. We
// place the adaptors for each type next to each other so they're easier to keep in sync, and we order them according
// to when they're needed during an RPC call: sending an arg from the client to the server (pack, and object_with_zone
// for dynamic dispatch), receiving an arg from the client on the server (convert), sending a return value from the
// server to the client (object_with_zone), and receiving a return value from the server on the client (convert).
//

//
// Minimal 16-bit floating point data type
//

#if SP_PLATFORM_CLIENT
    struct sp_float16_t { uint16_t data; };
    template <>
    struct nanobind::detail::dtype_traits<sp_float16_t>
    {
        //                                       type code,                                       size in bits, SIMD lanes (usually set to 1) 
        static constexpr dlpack::dtype value = { static_cast<uint8_t>(dlpack::dtype_code::Float), 16,           1};
        static constexpr auto name = const_name("sp_float16_t");
    };
#endif

//
// Helper functions for implementing adaptors
//

#if SP_PLATFORM_CLIENT
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
#endif

//
// SpCore/SpTypes.h
//

// SpOutputLogMessages (only sent from the server to the client)

#if SP_PLATFORM_SERVER
    template <> // needed to send a return value from the server to the client
    struct clmdep_msgpack::adaptor::object_with_zone<SpOutputLogMessages> {
        void operator()(clmdep_msgpack::object::with_zone& object_with_zone, SpOutputLogMessages const& sp_output_log_messages) const {
            std::map<std::string, clmdep_msgpack::object> objects;
            Std::insert(objects, "messages",    MsgpackUtils::toMsgpackObject(sp_output_log_messages.messages_,    object_with_zone));
            Std::insert(objects, "num_evicted", MsgpackUtils::toMsgpackObject(sp_output_log_messages.num_evicted_, object_with_zone));
            MsgpackUtils::setMsgpackObjectToMap(object_with_zone, objects);
        }
    };
#endif

#if SP_PLATFORM_CLIENT
    template <> // needed to receive a return value from the server on the client
    struct clmdep_msgpack::adaptor::convert<SpOutputLogMessages> {
        clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& object, SpOutputLogMessages& sp_output_log_messages) const {
            std::map<std::string, clmdep_msgpack::object> objects = MsgpackUtils::toMapOfMsgpackObjects(object);
            SP_ASSERT(objects.size() == 2);
            sp_output_log_messages.messages_    = MsgpackUtils::to<std::vector<std::string>>(objects.at("messages"));
            sp_output_log_messages.num_evicted_ = MsgpackUtils::to<int32_t>(objects.at("num_evicted"));
            return object;
        }
    };
#endif

// SpSharedMemoryView (only sent from the server to the client)

#if SP_PLATFORM_SERVER
    template <> // needed to send a return value from the server to the client
    struct clmdep_msgpack::adaptor::object_with_zone<SpSharedMemoryView> {
        void operator()(clmdep_msgpack::object::with_zone& object_with_zone, SpSharedMemoryView const& sp_shared_memory_view) const {
            std::map<std::string, clmdep_msgpack::object> objects;
            Std::insert(objects, "id",           MsgpackUtils::toMsgpackObject(sp_shared_memory_view.id_,           object_with_zone));
            Std::insert(objects, "num_bytes",    MsgpackUtils::toMsgpackObject(sp_shared_memory_view.num_bytes_,    object_with_zone));
            Std::insert(objects, "offset_bytes", MsgpackUtils::toMsgpackObject(sp_shared_memory_view.offset_bytes_, object_with_zone));
            MsgpackUtils::setMsgpackObjectToMap(object_with_zone, objects);
        }
    };
#endif

#if SP_PLATFORM_CLIENT
    template <> // needed to receive a return value from the server on the client
    struct clmdep_msgpack::adaptor::convert<SpSharedMemoryView> {
        clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& object, SpSharedMemoryView& sp_shared_memory_view) const {
            std::map<std::string, clmdep_msgpack::object> objects = MsgpackUtils::toMapOfMsgpackObjects(object);
            SP_ASSERT(objects.size() == 3);
            sp_shared_memory_view.id_           = MsgpackUtils::to<std::string>(objects.at("id"));
            sp_shared_memory_view.num_bytes_    = MsgpackUtils::to<uint64_t>(objects.at("num_bytes"));
            sp_shared_memory_view.offset_bytes_ = MsgpackUtils::to<uint16_t>(objects.at("offset_bytes"));
            return object;
        }
    };
#endif

// SpArraySharedMemoryView (only sent from the server to the client)

#if SP_PLATFORM_SERVER
    template <> // needed to send a return value from the server to the client
    struct clmdep_msgpack::adaptor::object_with_zone<SpArraySharedMemoryView> {
        void operator()(clmdep_msgpack::object::with_zone& object_with_zone, SpArraySharedMemoryView const& sp_array_shared_memory_view) const {
            std::map<std::string, clmdep_msgpack::object> objects;
            Std::insert(objects, "id",           MsgpackUtils::toMsgpackObject(sp_array_shared_memory_view.id_,           object_with_zone));
            Std::insert(objects, "num_bytes",    MsgpackUtils::toMsgpackObject(sp_array_shared_memory_view.num_bytes_,    object_with_zone));
            Std::insert(objects, "offset_bytes", MsgpackUtils::toMsgpackObject(sp_array_shared_memory_view.offset_bytes_, object_with_zone));
            Std::insert(objects, "usage_flags",  MsgpackUtils::toMsgpackObject(Unreal::getStringsFromCombinedEnumFlagValueAs<ESpArraySharedMemoryUsageFlags>(sp_array_shared_memory_view.usage_flags_), object_with_zone));
            Std::insert(objects, "name",         MsgpackUtils::toMsgpackObject(sp_array_shared_memory_view.name_,         object_with_zone));
            MsgpackUtils::setMsgpackObjectToMap(object_with_zone, objects);
        }
    };
#endif

#if SP_PLATFORM_CLIENT
    template <> // needed to receive a return value from the server on the client
    struct clmdep_msgpack::adaptor::convert<SpArraySharedMemoryView> {
        clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& object, SpArraySharedMemoryView& sp_array_shared_memory_view) const {
            std::map<std::string, clmdep_msgpack::object> objects = MsgpackUtils::toMapOfMsgpackObjects(object);
            SP_ASSERT(objects.size() == 5);
            sp_array_shared_memory_view.id_           = MsgpackUtils::to<std::string>(objects.at("id"));
            sp_array_shared_memory_view.num_bytes_    = MsgpackUtils::to<uint64_t>(objects.at("num_bytes"));
            sp_array_shared_memory_view.offset_bytes_ = MsgpackUtils::to<uint16_t>(objects.at("offset_bytes"));
            sp_array_shared_memory_view.usage_flags_  = MsgpackUtils::to<std::vector<std::string>>(objects.at("usage_flags"));
            sp_array_shared_memory_view.name_         = MsgpackUtils::to<std::string>(objects.at("name"));
            return object;
        }
    };
#endif

// SpPackedArray

#if SP_PLATFORM_CLIENT
    template <> // needed to send an arg from the client to the server
    struct clmdep_msgpack::adaptor::pack<SpPackedArray> {
        template <typename TStream>
        clmdep_msgpack::packer<TStream>& operator()(clmdep_msgpack::packer<TStream>& packer, SpPackedArray const& sp_packed_array) const {
            packer.pack_map(5);
            packer.pack("data");               packer.pack(clmdep_msgpack::type::raw_ref(static_cast<const char*>(sp_packed_array.data_.data()), sp_packed_array.data_.nbytes()));
            packer.pack("data_source");        packer.pack(sp_packed_array.data_source_);
            packer.pack("shape");              packer.pack(sp_packed_array.shape_);
            packer.pack("shared_memory_name"); packer.pack(sp_packed_array.shared_memory_name_);
            packer.pack("data_type");          packer.pack(DataTypeUtils::getDataType(sp_packed_array.data_.dtype()));
            return packer;
        }
    };
#endif

#if SP_PLATFORM_CLIENT
    template <> // needed to send an arg from the client to the server via dynamic dispatch
    struct clmdep_msgpack::adaptor::object_with_zone<SpPackedArray> {
        void operator()(clmdep_msgpack::object::with_zone& o, const SpPackedArray& v) const {
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
#endif

#if SP_PLATFORM_SERVER
    template <> // needed to receive an arg from the client on the server
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
#endif

#if SP_PLATFORM_SERVER
    template <> // needed to send a return value from the server to the client
    struct clmdep_msgpack::adaptor::object_with_zone<SpPackedArray> {
        void operator()(clmdep_msgpack::object::with_zone& object_with_zone, SpPackedArray const& sp_packed_array) const {
            SP_ASSERT(sp_packed_array.data_source_ == SpArrayDataSource::Internal || sp_packed_array.data_source_ == SpArrayDataSource::Shared);
            std::map<std::string, clmdep_msgpack::object> objects;
            Std::insert(objects, "data",               MsgpackUtils::toMsgpackObject(sp_packed_array.data_,                                                               object_with_zone));
            Std::insert(objects, "data_source",        MsgpackUtils::toMsgpackObject(Unreal::getStringFromEnumValueAs<ESpArrayDataSource>(sp_packed_array.data_source_),  object_with_zone));
            Std::insert(objects, "shape",              MsgpackUtils::toMsgpackObject(sp_packed_array.shape_,                                                              object_with_zone));
            Std::insert(objects, "shared_memory_name", MsgpackUtils::toMsgpackObject(sp_packed_array.shared_memory_name_,                                                 object_with_zone));
            Std::insert(objects, "data_type",          MsgpackUtils::toMsgpackObject(Unreal::getStringFromEnumValueAs<ESpArrayShortDataType>(sp_packed_array.data_type_), object_with_zone));
            MsgpackUtils::setMsgpackObjectToMap(object_with_zone, objects);
        }
    };
#endif

#if SP_PLATFORM_CLIENT
    template <> // needed to receive a return value from the server on the client
    struct clmdep_msgpack::adaptor::convert<SpPackedArrayView> {
        clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& object, SpPackedArrayView& sp_packed_array_view) const {

            std::map<std::string, clmdep_msgpack::object> objects = MsgpackUtils::toMapOfMsgpackObjects(object);
            SP_ASSERT(objects.size() == 5);
            sp_packed_array_view.view_               = MsgpackUtils::to<std::span<uint8_t>>(objects.at("data"));
            sp_packed_array_view.data_source_        = MsgpackUtils::to<std::string>(objects.at("data_source"));
            sp_packed_array_view.shape_              = MsgpackUtils::to<std::vector<size_t>>(objects.at("shape"));
            sp_packed_array_view.shared_memory_name_ = MsgpackUtils::to<std::string>(objects.at("shared_memory_name"));
            sp_packed_array_view.data_type_          = MsgpackUtils::to<std::string>(objects.at("data_type"));
            SP_ASSERT(sp_packed_array_view.data_source_ == "Internal" || sp_packed_array_view.data_source_ == "Shared");
            return object;
        }
    };
#endif

#if SP_PLATFORM_CLIENT
    template <> // needed to convert a view type, received as a return value from the server on the client, into a type that can be returned to Python
    inline SpPackedArray MsgpackUtils::convert<SpPackedArray>(const SpClientBase* client, SpPackedArrayView&& sp_packed_array_view, std::shared_ptr<clmdep_msgpack::object_handle> object_handle)
    {
        SP_ASSERT(client);

        SpPackedArray sp_packed_array;

        if (client->verbose_allocations_) {
            std::cout << "[SPEAR | MsgpackAdaptors.h] Constructing nanobind::ndarray<nanobind::numpy>..." << std::endl;
            std::cout << "[SPEAR | MsgpackAdaptors.h] sp_packed_array_view.view_.data():        " << sp_packed_array_view.view_.data() << std::endl;
            std::cout << "[SPEAR | MsgpackAdaptors.h] sp_packed_array_view.view_.size():        " << sp_packed_array_view.view_.size() << std::endl;
            std::cout << "[SPEAR | MsgpackAdaptors.h] sp_packed_array_view.data_source_:        " << sp_packed_array_view.data_source_ << std::endl;
            std::cout << "[SPEAR | MsgpackAdaptors.h] sp_packed_array_view.shape_:              "; for (int i = 0; i < sp_packed_array_view.shape_.size(); i++) { std::cout << sp_packed_array_view.shape_.at(i) << " "; } std::cout << std::endl;
            std::cout << "[SPEAR | MsgpackAdaptors.h] sp_packed_array_view.data_type_:          " << sp_packed_array_view.data_type_ << std::endl;
            std::cout << "[SPEAR | MsgpackAdaptors.h] sp_packed_array_view.shared_memory_name_: " << sp_packed_array_view.shared_memory_name_ << std::endl;
        }

        // construct ndarray
        nanobind::ndarray<nanobind::numpy> ndarray;
        if (sp_packed_array_view.data_source_ == "Internal") {

            struct Capsule
            {
                std::vector<uint8_t, SpAlignedAllocator<uint8_t, 4096>> data_;
                std::shared_ptr<clmdep_msgpack::object_handle> object_handle_ = nullptr;
                bool verbose_allocations_ = false; // needed because the deletion callback below can't capture client
            };

            // Allocate a Capsule on the heap. Owned by a std::unique_ptr until ownership is transferred to the
            // nanobind capsule below, so the Capsule is deleted if an exception is thrown before then.
            std::unique_ptr<Capsule> capsule_ptr = std::make_unique<Capsule>();
            capsule_ptr->verbose_allocations_ = client->verbose_allocations_;
            if (client->verbose_allocations_) { std::cout << "[SPEAR | MsgpackAdaptors.h] Allocated Capsule object at memory location: " << capsule_ptr.get() << std::endl; }

            void* data_ptr = nullptr;

            // If we're forcing memory alignment, then copy the clmdep_msgpack::object_handle's internal data
            // buffer into the Capsule's internal data buffer, which is guaranteed to be aligned to very large
            // boundaries, and use the Capsule's internal data buffer as the persistent storage for our
            // nanobind array.

            if (client->force_return_aligned_arrays_) {
                Std::resizeUninitialized(capsule_ptr->data_, sp_packed_array_view.view_.size());
                std::memcpy(capsule_ptr->data_.data(), sp_packed_array_view.view_.data(), sp_packed_array_view.view_.size());
                data_ptr = capsule_ptr->data_.data();

            // If we're not forcing memory alignment, then assign to the Capsule's std::shared_ptr to indicate
            // that we want to keep the clmdep_msgpack::object_handle alive, and use the clmdep_msgpack::object_handle's
            // internal data buffer as the persistent storage for our nanobind array.

            } else {
                capsule_ptr->object_handle_ = object_handle;
                data_ptr = sp_packed_array_view.view_.data();
            }

            // Construct a nanobind capsule object that is responsible for deleting our heap-allocated Capsule.
            // In the deletion callback, we assign to our Capsule's std::shared_ptr object to indicate that we
            // don't need to keep the clmdep_msgpack::object_handle alive any more and clean up our
            // heap-allocated Capsule.

            nanobind::capsule capsule = nanobind::capsule(capsule_ptr.get(), [](void* ptr) noexcept -> void {
                if (!ptr) {
                    std::cout << "[SPEAR | MsgpackAdaptors.h] ERROR: Unexpected nullptr (ptr) in deletion callback for Capsule." << std::endl;
                    std::terminate(); // can't assert because of noexcept
                }
                Capsule* capsule_ptr = static_cast<Capsule*>(ptr);
                if (capsule_ptr->verbose_allocations_) { std::cout << "[SPEAR | MsgpackAdaptors.h] Deleting Capsule object at memory location: " << capsule_ptr << std::endl; }
                capsule_ptr->object_handle_ = nullptr;
                delete capsule_ptr; // need to delete manually because capsule_ptr.release() releases ownership without deleting
                capsule_ptr = nullptr;
            });

            capsule_ptr.release(); // the nanobind capsule now owns the Capsule and must call delete

            ndarray = nanobind::ndarray<nanobind::numpy>(
                data_ptr,                                                     // data_ptr
                sp_packed_array_view.shape_.size(),                           // ndim
                sp_packed_array_view.shape_.data(),                           // shape_ptr
                capsule,                                                      // owner
                nullptr,                                                      // strides
                DataTypeUtils::getDataType(sp_packed_array_view.data_type_)); // dtype

        } else if (sp_packed_array_view.data_source_ == "Shared") {

            ndarray = nanobind::ndarray<nanobind::numpy>(
                nullptr,                                                      // data_ptr
                0,                                                            // ndim
                nullptr,                                                      // shape_ptr
                {},                                                           // owner
                nullptr,                                                      // strides
                DataTypeUtils::getDataType(sp_packed_array_view.data_type_)); // dtype

        } else {
            SP_ASSERT(false);
        }

        sp_packed_array.data_               = std::move(ndarray);
        sp_packed_array.data_source_        = std::move(sp_packed_array_view.data_source_);
        sp_packed_array.shape_              = std::move(sp_packed_array_view.shape_);
        sp_packed_array.shared_memory_name_ = std::move(sp_packed_array_view.shared_memory_name_);

        return sp_packed_array;
    };
#endif

#if SP_PLATFORM_CLIENT
    template <> // needed to convert a view type, received as a return value from the server on the client, into a type that can be returned to Python
    inline std::map<std::string, SpPackedArray> MsgpackUtils::convert<std::map<std::string, SpPackedArray>>(const SpClientBase* client, std::map<std::string, SpPackedArrayView>&& sp_packed_array_views, std::shared_ptr<clmdep_msgpack::object_handle> object_handle)
    {
        std::map<std::string, SpPackedArray> sp_packed_arrays;
        for (auto& [name, sp_packed_array_view] : sp_packed_array_views) {
            auto [itr, success] = sp_packed_arrays.insert({std::move(name), convert<SpPackedArray>(client, std::move(sp_packed_array_view), object_handle)});
            SP_ASSERT(success); // will only succeed if key wasn't already present
        }
        return sp_packed_arrays;
    };
#endif

// SpFuncDataBundle

#if SP_PLATFORM_CLIENT
    template <> // needed to send an arg from the client to the server
    struct clmdep_msgpack::adaptor::pack<SpFuncDataBundle> {
        template <typename TStream>
        clmdep_msgpack::packer<TStream>& operator()(clmdep_msgpack::packer<TStream>& packer, SpFuncDataBundle const& sp_func_data_bundle) const {
            packer.pack_map(3);
            packer.pack("packed_arrays");      packer.pack(sp_func_data_bundle.packed_arrays_);
            packer.pack("unreal_obj_strings"); packer.pack(sp_func_data_bundle.unreal_obj_strings_);
            packer.pack("info");               packer.pack(sp_func_data_bundle.info_);
            return packer;
        }
    };
#endif

#if SP_PLATFORM_CLIENT
    template <> // needed to send an arg from the client to the server via dynamic dispatch
    struct clmdep_msgpack::adaptor::object_with_zone<SpFuncDataBundle> {
        void operator()(clmdep_msgpack::object::with_zone& o, const SpFuncDataBundle& v) const {
            o.type = clmdep_msgpack::type::MAP;
            o.via.map.size = 3;
            o.via.map.ptr = static_cast<clmdep_msgpack::object_kv*>(o.zone.allocate_align(sizeof(clmdep_msgpack::object_kv) * 3, MSGPACK_ZONE_ALIGNOF(clmdep_msgpack::object_kv)));
            o.via.map.ptr[0].key = clmdep_msgpack::object("packed_arrays",      o.zone); o.via.map.ptr[0].val = clmdep_msgpack::object(v.packed_arrays_,      o.zone);
            o.via.map.ptr[1].key = clmdep_msgpack::object("unreal_obj_strings", o.zone); o.via.map.ptr[1].val = clmdep_msgpack::object(v.unreal_obj_strings_, o.zone);
            o.via.map.ptr[2].key = clmdep_msgpack::object("info",               o.zone); o.via.map.ptr[2].val = clmdep_msgpack::object(v.info_,               o.zone);
        }
    };
#endif

#if SP_PLATFORM_SERVER
    template <> // needed to receive an arg from the client on the server
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
#endif

#if SP_PLATFORM_SERVER
    template <> // needed to send a return value from the server to the client
    struct clmdep_msgpack::adaptor::object_with_zone<SpFuncDataBundle> {
        void operator()(clmdep_msgpack::object::with_zone& object_with_zone, SpFuncDataBundle const& sp_func_data_bundle) const {
            std::map<std::string, clmdep_msgpack::object> objects;
            Std::insert(objects, "packed_arrays",      MsgpackUtils::toMsgpackObject(sp_func_data_bundle.packed_arrays_,      object_with_zone));
            Std::insert(objects, "unreal_obj_strings", MsgpackUtils::toMsgpackObject(sp_func_data_bundle.unreal_obj_strings_, object_with_zone));
            Std::insert(objects, "info",               MsgpackUtils::toMsgpackObject(sp_func_data_bundle.info_,               object_with_zone));
            MsgpackUtils::setMsgpackObjectToMap(object_with_zone, objects);
        }
    };
#endif

#if SP_PLATFORM_CLIENT
    template <> // needed to receive a return value from the server on the client
    struct clmdep_msgpack::adaptor::convert<SpFuncDataBundleView> {
        clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& object, SpFuncDataBundleView& sp_func_data_bundle_view) const {
            std::map<std::string, clmdep_msgpack::object> objects = MsgpackUtils::toMapOfMsgpackObjects(object);
            SP_ASSERT(objects.size() == 3);
            sp_func_data_bundle_view.packed_array_views_ = MsgpackUtils::to<std::map<std::string, SpPackedArrayView>>(objects.at("packed_arrays"));
            sp_func_data_bundle_view.unreal_obj_strings_ = MsgpackUtils::to<std::map<std::string, std::string>>(objects.at("unreal_obj_strings"));
            sp_func_data_bundle_view.info_               = MsgpackUtils::to<std::string>(objects.at("info"));
            return object;
        }
    };
#endif

#if SP_PLATFORM_CLIENT
    template <> // needed to convert a view type, received as a return value from the server on the client, into a type that can be returned to Python
    inline SpFuncDataBundle MsgpackUtils::convert<SpFuncDataBundle>(const SpClientBase* client, SpFuncDataBundleView&& sp_func_data_bundle_view, std::shared_ptr<clmdep_msgpack::object_handle> object_handle)
    {
        SpFuncDataBundle sp_func_data_bundle;
        sp_func_data_bundle.packed_arrays_      = convert<std::map<std::string, SpPackedArray>>(client, std::move(sp_func_data_bundle_view.packed_array_views_), object_handle);
        sp_func_data_bundle.unreal_obj_strings_ = std::move(sp_func_data_bundle_view.unreal_obj_strings_);
        sp_func_data_bundle.info_               = std::move(sp_func_data_bundle_view.info_);
        return sp_func_data_bundle;
    };
#endif

// SpHandle

#if SP_PLATFORM_CLIENT
    template <> // needed to send an arg from the client to the server
    struct clmdep_msgpack::adaptor::pack<SpHandle> {
        template <typename TStream>
        clmdep_msgpack::packer<TStream>& operator()(clmdep_msgpack::packer<TStream>& packer, SpHandle const& sp_handle) const {
            packer.pack_map(3);
            packer.pack("ptr");                  packer.pack(sp_handle.ptr_);
            packer.pack("object_index");         packer.pack(sp_handle.object_index_);
            packer.pack("object_serial_number"); packer.pack(sp_handle.object_serial_number_);
            return packer;
        }
    };
#endif

#if SP_PLATFORM_CLIENT
    template <> // needed to send an arg from the client to the server via dynamic dispatch
    struct clmdep_msgpack::adaptor::object_with_zone<SpHandle> {
        void operator()(clmdep_msgpack::object::with_zone& o, const SpHandle& v) const {
            o.type = clmdep_msgpack::type::MAP;
            o.via.map.size = 3;
            o.via.map.ptr = static_cast<clmdep_msgpack::object_kv*>(o.zone.allocate_align(sizeof(clmdep_msgpack::object_kv) * 3, MSGPACK_ZONE_ALIGNOF(clmdep_msgpack::object_kv)));
            o.via.map.ptr[0].key = clmdep_msgpack::object("ptr",                  o.zone); o.via.map.ptr[0].val = clmdep_msgpack::object(v.ptr_,                  o.zone);
            o.via.map.ptr[1].key = clmdep_msgpack::object("object_index",         o.zone); o.via.map.ptr[1].val = clmdep_msgpack::object(v.object_index_,         o.zone);
            o.via.map.ptr[2].key = clmdep_msgpack::object("object_serial_number", o.zone); o.via.map.ptr[2].val = clmdep_msgpack::object(v.object_serial_number_, o.zone);
        }
    };
#endif

#if SP_PLATFORM_SERVER
    template <> // needed to receive an arg from the client on the server
    struct clmdep_msgpack::adaptor::convert<SpHandle> {
        clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& object, SpHandle& sp_handle) const {
            std::map<std::string, clmdep_msgpack::object> objects = MsgpackUtils::toMapOfMsgpackObjects(object);
            SP_ASSERT(objects.size() == 3);
            sp_handle.ptr_                  = MsgpackUtils::to<uint64_t>(objects.at("ptr"));
            sp_handle.object_index_         = MsgpackUtils::to<int32_t>(objects.at("object_index"));
            sp_handle.object_serial_number_ = MsgpackUtils::to<int32_t>(objects.at("object_serial_number"));
            return object;
        }
    };
#endif

#if SP_PLATFORM_SERVER
    template <> // needed to send a return value from the server to the client
    struct clmdep_msgpack::adaptor::object_with_zone<SpHandle> {
        void operator()(clmdep_msgpack::object::with_zone& object_with_zone, SpHandle const& sp_handle) const {
            std::map<std::string, clmdep_msgpack::object> objects;
            Std::insert(objects, "ptr",                  MsgpackUtils::toMsgpackObject(sp_handle.ptr_,                  object_with_zone));
            Std::insert(objects, "object_index",         MsgpackUtils::toMsgpackObject(sp_handle.object_index_,         object_with_zone));
            Std::insert(objects, "object_serial_number", MsgpackUtils::toMsgpackObject(sp_handle.object_serial_number_, object_with_zone));
            MsgpackUtils::setMsgpackObjectToMap(object_with_zone, objects);
        }
    };
#endif

#if SP_PLATFORM_CLIENT
    template <> // needed to receive a return value from the server on the client
    struct clmdep_msgpack::adaptor::convert<SpHandle> {
        clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& object, SpHandle& sp_handle) const {
            std::map<std::string, clmdep_msgpack::object> objects = MsgpackUtils::toMapOfMsgpackObjects(object);
            SP_ASSERT(objects.size() == 3);
            sp_handle.ptr_                  = MsgpackUtils::to<uint64_t>(objects.at("ptr"));
            sp_handle.object_index_         = MsgpackUtils::to<int32_t>(objects.at("object_index"));
            sp_handle.object_serial_number_ = MsgpackUtils::to<int32_t>(objects.at("object_serial_number"));
            return object;
        }
    };
#endif

// SpPropertyDesc

#if SP_PLATFORM_CLIENT
    template <> // needed to send an arg from the client to the server
    struct clmdep_msgpack::adaptor::pack<SpPropertyDesc> {
        template <typename TStream>
        clmdep_msgpack::packer<TStream>& operator()(clmdep_msgpack::packer<TStream>& packer, SpPropertyDesc const& sp_property_desc) const {
            packer.pack_map(8);
            packer.pack("property");                  packer.pack(sp_property_desc.property_);
            packer.pack("value_ptr");                 packer.pack(sp_property_desc.value_ptr_);
            packer.pack("type_id");                   packer.pack(sp_property_desc.type_id_);
            packer.pack("notify_objects");            packer.pack(sp_property_desc.notify_objects_);
            packer.pack("notify_member_properties");  packer.pack(sp_property_desc.notify_member_properties_);
            packer.pack("notify_element_properties"); packer.pack(sp_property_desc.notify_element_properties_);
            packer.pack("notify_array_indices");      packer.pack(sp_property_desc.notify_array_indices_);
            packer.pack("notify_map_keys");           packer.pack(sp_property_desc.notify_map_keys_);
            return packer;
        }
    };
#endif

#if SP_PLATFORM_CLIENT
    template <> // needed to send an arg from the client to the server via dynamic dispatch
    struct clmdep_msgpack::adaptor::object_with_zone<SpPropertyDesc> {
        void operator()(clmdep_msgpack::object::with_zone& o, const SpPropertyDesc& v) const {
            o.type = clmdep_msgpack::type::MAP;
            o.via.map.size = 8;
            o.via.map.ptr = static_cast<clmdep_msgpack::object_kv*>(o.zone.allocate_align(sizeof(clmdep_msgpack::object_kv) * 8, MSGPACK_ZONE_ALIGNOF(clmdep_msgpack::object_kv)));
            o.via.map.ptr[0].key = clmdep_msgpack::object("property",                  o.zone); o.via.map.ptr[0].val = clmdep_msgpack::object(v.property_,                  o.zone);
            o.via.map.ptr[1].key = clmdep_msgpack::object("value_ptr",                 o.zone); o.via.map.ptr[1].val = clmdep_msgpack::object(v.value_ptr_,                 o.zone);
            o.via.map.ptr[2].key = clmdep_msgpack::object("type_id",                   o.zone); o.via.map.ptr[2].val = clmdep_msgpack::object(v.type_id_,                   o.zone);
            o.via.map.ptr[3].key = clmdep_msgpack::object("notify_objects",            o.zone); o.via.map.ptr[3].val = clmdep_msgpack::object(v.notify_objects_,            o.zone);
            o.via.map.ptr[4].key = clmdep_msgpack::object("notify_member_properties",  o.zone); o.via.map.ptr[4].val = clmdep_msgpack::object(v.notify_member_properties_,  o.zone);
            o.via.map.ptr[5].key = clmdep_msgpack::object("notify_element_properties", o.zone); o.via.map.ptr[5].val = clmdep_msgpack::object(v.notify_element_properties_, o.zone);
            o.via.map.ptr[6].key = clmdep_msgpack::object("notify_array_indices",      o.zone); o.via.map.ptr[6].val = clmdep_msgpack::object(v.notify_array_indices_,      o.zone);
            o.via.map.ptr[7].key = clmdep_msgpack::object("notify_map_keys",           o.zone); o.via.map.ptr[7].val = clmdep_msgpack::object(v.notify_map_keys_,           o.zone);
        }
    };
#endif

#if SP_PLATFORM_SERVER
    template <> // needed to receive an arg from the client on the server
    struct clmdep_msgpack::adaptor::convert<SpPropertyDesc> {
        clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& object, SpPropertyDesc& sp_property_desc) const {
            std::map<std::string, clmdep_msgpack::object> map = MsgpackUtils::toMapOfMsgpackObjects(object);
            SP_ASSERT(map.size() == 8);
            sp_property_desc.property_                  = MsgpackUtils::to<FProperty*>(map.at("property"));
            sp_property_desc.value_ptr_                 = MsgpackUtils::to<void*>(map.at("value_ptr"));
            sp_property_desc.type_id_                   = MsgpackUtils::to<std::string>(map.at("type_id"));
            sp_property_desc.notify_objects_            = MsgpackUtils::to<std::vector<SpHandle>>(map.at("notify_objects"));
            sp_property_desc.notify_member_properties_  = MsgpackUtils::toVector<FProperty*>(MsgpackUtils::toVectorOfMsgpackObjects(map.at("notify_member_properties")));
            sp_property_desc.notify_element_properties_ = MsgpackUtils::toVector<FProperty*>(MsgpackUtils::toVectorOfMsgpackObjects(map.at("notify_element_properties")));
            sp_property_desc.notify_array_indices_      = MsgpackUtils::to<std::vector<int>>(map.at("notify_array_indices"));
            sp_property_desc.notify_map_keys_           = MsgpackUtils::to<std::vector<std::string>>(map.at("notify_map_keys"));
            return object;
        }
    };
#endif

#if SP_PLATFORM_SERVER
    template <> // needed to send a return value from the server to the client
    struct clmdep_msgpack::adaptor::object_with_zone<SpPropertyDesc> {
        void operator()(clmdep_msgpack::object::with_zone& object_with_zone, SpPropertyDesc const& sp_property_desc) const {
            std::map<std::string, clmdep_msgpack::object> objects;
            Std::insert(objects, "property",                  MsgpackUtils::toMsgpackObject(sp_property_desc.property_,  object_with_zone));
            Std::insert(objects, "value_ptr",                 MsgpackUtils::toMsgpackObject(sp_property_desc.value_ptr_, object_with_zone));
            Std::insert(objects, "type_id",                   MsgpackUtils::toMsgpackObject(sp_property_desc.type_id_,   object_with_zone));
            Std::insert(objects, "notify_objects",            MsgpackUtils::toMsgpackObject(sp_property_desc.notify_objects_,       object_with_zone));
            Std::insert(objects, "notify_member_properties",  MsgpackUtils::toVectorMsgpackObject(sp_property_desc.notify_member_properties_,  object_with_zone));
            Std::insert(objects, "notify_element_properties", MsgpackUtils::toVectorMsgpackObject(sp_property_desc.notify_element_properties_, object_with_zone));
            Std::insert(objects, "notify_array_indices",      MsgpackUtils::toMsgpackObject(sp_property_desc.notify_array_indices_, object_with_zone));
            Std::insert(objects, "notify_map_keys",           MsgpackUtils::toMsgpackObject(sp_property_desc.notify_map_keys_,      object_with_zone));
            MsgpackUtils::setMsgpackObjectToMap(object_with_zone, objects);
        }
    };
#endif

#if SP_PLATFORM_CLIENT
    template <> // needed to receive a return value from the server on the client
    struct clmdep_msgpack::adaptor::convert<SpPropertyDesc> {
        clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& object, SpPropertyDesc& sp_property_desc) const {
            std::map<std::string, clmdep_msgpack::object> objects = MsgpackUtils::toMapOfMsgpackObjects(object);
            SP_ASSERT(objects.size() == 8);
            sp_property_desc.property_                  = MsgpackUtils::to<uint64_t>(objects.at("property"));
            sp_property_desc.value_ptr_                 = MsgpackUtils::to<uint64_t>(objects.at("value_ptr"));
            sp_property_desc.type_id_                   = MsgpackUtils::to<std::string>(objects.at("type_id"));
            sp_property_desc.notify_objects_            = MsgpackUtils::to<std::vector<SpHandle>>(objects.at("notify_objects"));
            sp_property_desc.notify_member_properties_  = MsgpackUtils::to<std::vector<uint64_t>>(objects.at("notify_member_properties"));
            sp_property_desc.notify_element_properties_ = MsgpackUtils::to<std::vector<uint64_t>>(objects.at("notify_element_properties"));
            sp_property_desc.notify_array_indices_      = MsgpackUtils::to<std::vector<int>>(objects.at("notify_array_indices"));
            sp_property_desc.notify_map_keys_           = MsgpackUtils::to<std::vector<std::string>>(objects.at("notify_map_keys"));
            return object;
        }
    };
#endif

// SpPropertyValue (only sent from the server to the client)

#if SP_PLATFORM_SERVER
    template <> // needed to send a return value from the server to the client
    struct clmdep_msgpack::adaptor::object_with_zone<SpPropertyValue> {
        void operator()(clmdep_msgpack::object::with_zone& object_with_zone, SpPropertyValue const& sp_property_value) const {
            std::map<std::string, clmdep_msgpack::object> objects;
            Std::insert(objects, "value",   MsgpackUtils::toMsgpackObject(sp_property_value.value_,   object_with_zone));
            Std::insert(objects, "type_id", MsgpackUtils::toMsgpackObject(sp_property_value.type_id_, object_with_zone));
            MsgpackUtils::setMsgpackObjectToMap(object_with_zone, objects);
        }
    };
#endif

#if SP_PLATFORM_CLIENT
    template <> // needed to receive a return value from the server on the client
    struct clmdep_msgpack::adaptor::convert<SpPropertyValue> {
        clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& object, SpPropertyValue& sp_property_value) const {
            std::map<std::string, clmdep_msgpack::object> objects = MsgpackUtils::toMapOfMsgpackObjects(object);
            SP_ASSERT(objects.size() == 2);
            sp_property_value.value_   = MsgpackUtils::to<std::string>(objects.at("value"));
            sp_property_value.type_id_ = MsgpackUtils::to<std::string>(objects.at("type_id"));
            return object;
        }
    };
#endif

//
// SpServices/SpTypes.h
//

// SpFuncSignatureDesc (only sent from the server to the client)

#if SP_PLATFORM_SERVER
    template <> // needed to send a return value from the server to the client
    struct clmdep_msgpack::adaptor::object_with_zone<SpFuncSignatureDesc> {
        void operator()(clmdep_msgpack::object::with_zone& object_with_zone, SpFuncSignatureDesc const& sp_func_signature_desc) const {
            std::map<std::string, clmdep_msgpack::object> objects;
            Std::insert(objects, "name",       MsgpackUtils::toMsgpackObject(sp_func_signature_desc.name_,       object_with_zone));
            Std::insert(objects, "type_ids",   MsgpackUtils::toMsgpackObject(sp_func_signature_desc.type_ids_,   object_with_zone));
            Std::insert(objects, "type_names", MsgpackUtils::toMsgpackObject(sp_func_signature_desc.type_names_, object_with_zone));
            MsgpackUtils::setMsgpackObjectToMap(object_with_zone, objects);
        }
    };
#endif

#if SP_PLATFORM_CLIENT
    template <> // needed to receive a return value from the server on the client
    struct clmdep_msgpack::adaptor::convert<SpFuncSignatureDesc> {
        clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& object, SpFuncSignatureDesc& sp_func_signature_desc) const {
            std::map<std::string, clmdep_msgpack::object> objects = MsgpackUtils::toMapOfMsgpackObjects(object);
            SP_ASSERT(objects.size() == 3);
            sp_func_signature_desc.name_       = MsgpackUtils::to<std::string>(objects.at("name"));
            sp_func_signature_desc.type_ids_   = MsgpackUtils::to<std::vector<int>>(objects.at("type_ids"));
            sp_func_signature_desc.type_names_ = MsgpackUtils::to<std::vector<std::string>>(objects.at("type_names"));
            return object;
        }
    };
#endif

// SpFuture

#if SP_PLATFORM_CLIENT
    template <> // needed to send an arg from the client to the server
    struct clmdep_msgpack::adaptor::pack<SpFuture> {
        template <typename TStream>
        clmdep_msgpack::packer<TStream>& operator()(clmdep_msgpack::packer<TStream>& packer, SpFuture const& sp_future) const {
            packer.pack_map(2);
            packer.pack("future_ptr"); packer.pack(sp_future.future_ptr_);
            packer.pack("type_id");    packer.pack(sp_future.type_id_);
            return packer;
        }
    };
#endif

#if SP_PLATFORM_CLIENT
    template <> // needed to send an arg from the client to the server via dynamic dispatch
    struct clmdep_msgpack::adaptor::object_with_zone<SpFuture> {
        void operator()(clmdep_msgpack::object::with_zone& o, const SpFuture& v) const {
            o.type = clmdep_msgpack::type::MAP;
            o.via.map.size = 2;
            o.via.map.ptr = static_cast<clmdep_msgpack::object_kv*>(o.zone.allocate_align(sizeof(clmdep_msgpack::object_kv) * 2, MSGPACK_ZONE_ALIGNOF(clmdep_msgpack::object_kv)));
            o.via.map.ptr[0].key = clmdep_msgpack::object("future_ptr", o.zone); o.via.map.ptr[0].val = clmdep_msgpack::object(v.future_ptr_, o.zone);
            o.via.map.ptr[1].key = clmdep_msgpack::object("type_id",    o.zone); o.via.map.ptr[1].val = clmdep_msgpack::object(v.type_id_,    o.zone);
        }
    };
#endif

#if SP_PLATFORM_SERVER
    template <> // needed to receive an arg from the client on the server
    struct clmdep_msgpack::adaptor::convert<SpFuture> {
        clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& object, SpFuture& sp_future) const {
            std::map<std::string, clmdep_msgpack::object> objects = MsgpackUtils::toMapOfMsgpackObjects(object);
            SP_ASSERT(objects.size() == 2);
            sp_future.future_ptr_ = MsgpackUtils::to<void*>(objects.at("future_ptr"));
            sp_future.type_id_    = MsgpackUtils::to<std::string>(objects.at("type_id"));
            return object;
        }
    };
#endif

#if SP_PLATFORM_SERVER
    template <> // needed to send a return value from the server to the client
    struct clmdep_msgpack::adaptor::object_with_zone<SpFuture> {
        void operator()(clmdep_msgpack::object::with_zone& object_with_zone, SpFuture const& sp_future) const {
            std::map<std::string, clmdep_msgpack::object> objects;
            Std::insert(objects, "future_ptr", MsgpackUtils::toMsgpackObject(sp_future.future_ptr_, object_with_zone));
            Std::insert(objects, "type_id",    MsgpackUtils::toMsgpackObject(sp_future.type_id_,    object_with_zone));
            MsgpackUtils::setMsgpackObjectToMap(object_with_zone, objects);
        }
    };
#endif

#if SP_PLATFORM_CLIENT
    template <> // needed to receive a return value from the server on the client
    struct clmdep_msgpack::adaptor::convert<SpFuture> {
        clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& object, SpFuture& sp_future) const {
            std::map<std::string, clmdep_msgpack::object> objects = MsgpackUtils::toMapOfMsgpackObjects(object);
            SP_ASSERT(objects.size() == 2);
            sp_future.future_ptr_ = MsgpackUtils::to<uint64_t>(objects.at("future_ptr"));
            sp_future.type_id_    = MsgpackUtils::to<std::string>(objects.at("type_id"));
            return object;
        }
    };
#endif

// SpFunctionDesc (only sent from the server to the client)

#if SP_PLATFORM_SERVER
    template <> // needed to send a return value from the server to the client
    struct clmdep_msgpack::adaptor::object_with_zone<SpFunctionDesc> {
        void operator()(clmdep_msgpack::object::with_zone& object_with_zone, SpFunctionDesc const& sp_function_desc) const {
            std::map<std::string, clmdep_msgpack::object> objects;
            Std::insert(objects, "function",          MsgpackUtils::toMsgpackObject(sp_function_desc.function_,          object_with_zone));
            Std::insert(objects, "function_name",     MsgpackUtils::toMsgpackObject(sp_function_desc.function_name_,     object_with_zone));
            Std::insert(objects, "static_class",      MsgpackUtils::toMsgpackObject(sp_function_desc.static_class_,      object_with_zone));
            Std::insert(objects, "static_class_name", MsgpackUtils::toMsgpackObject(sp_function_desc.static_class_name_, object_with_zone));
            MsgpackUtils::setMsgpackObjectToMap(object_with_zone, objects);
        }
    };
#endif

#if SP_PLATFORM_CLIENT
    template <> // needed to receive a return value from the server on the client
    struct clmdep_msgpack::adaptor::convert<SpFunctionDesc> {
        clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& object, SpFunctionDesc& sp_function_desc) const {
            std::map<std::string, clmdep_msgpack::object> objects = MsgpackUtils::toMapOfMsgpackObjects(object);
            SP_ASSERT(objects.size() == 4);
            sp_function_desc.function_          = MsgpackUtils::to<SpHandle>(objects.at("function"));
            sp_function_desc.function_name_     = MsgpackUtils::to<std::string>(objects.at("function_name"));
            sp_function_desc.static_class_      = MsgpackUtils::to<SpHandle>(objects.at("static_class"));
            sp_function_desc.static_class_name_ = MsgpackUtils::to<std::string>(objects.at("static_class_name"));
            return object;
        }
    };
#endif

// SpStaticStructDesc (only sent from the server to the client)

#if SP_PLATFORM_SERVER
    template <> // needed to send a return value from the server to the client
    struct clmdep_msgpack::adaptor::object_with_zone<SpStaticStructDesc> {
        void operator()(clmdep_msgpack::object::with_zone& object_with_zone, SpStaticStructDesc const& sp_static_struct_desc) const {
            std::map<std::string, clmdep_msgpack::object> objects;
            Std::insert(objects, "static_struct", MsgpackUtils::toMsgpackObject(sp_static_struct_desc.static_struct_, object_with_zone));
            Std::insert(objects, "name",          MsgpackUtils::toMsgpackObject(sp_static_struct_desc.name_,          object_with_zone));
            MsgpackUtils::setMsgpackObjectToMap(object_with_zone, objects);
        }
    };
#endif

#if SP_PLATFORM_CLIENT
    template <> // needed to receive a return value from the server on the client
    struct clmdep_msgpack::adaptor::convert<SpStaticStructDesc> {
        clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& object, SpStaticStructDesc& sp_static_struct_desc) const {
            std::map<std::string, clmdep_msgpack::object> objects = MsgpackUtils::toMapOfMsgpackObjects(object);
            SP_ASSERT(objects.size() == 2);
            sp_static_struct_desc.static_struct_ = MsgpackUtils::to<SpHandle>(objects.at("static_struct"));
            sp_static_struct_desc.name_          = MsgpackUtils::to<std::string>(objects.at("name"));
            return object;
        }
    };
#endif

// SpStaticClassDesc (only sent from the server to the client)

#if SP_PLATFORM_SERVER
    template <> // needed to send a return value from the server to the client
    struct clmdep_msgpack::adaptor::object_with_zone<SpStaticClassDesc> {
        void operator()(clmdep_msgpack::object::with_zone& object_with_zone, SpStaticClassDesc const& sp_static_class_desc) const {
            std::map<std::string, clmdep_msgpack::object> objects;
            Std::insert(objects, "static_class",                MsgpackUtils::toMsgpackObject(sp_static_class_desc.static_class_,                object_with_zone));
            Std::insert(objects, "name",                        MsgpackUtils::toMsgpackObject(sp_static_class_desc.name_,                        object_with_zone));
            Std::insert(objects, "derived_to_base_classes",     MsgpackUtils::toMsgpackObject(sp_static_class_desc.derived_to_base_classes_,     object_with_zone));
            Std::insert(objects, "derived_to_base_class_names", MsgpackUtils::toMsgpackObject(sp_static_class_desc.derived_to_base_class_names_, object_with_zone));
            Std::insert(objects, "function_descs",              MsgpackUtils::toMsgpackObject(sp_static_class_desc.function_descs_,              object_with_zone));
            MsgpackUtils::setMsgpackObjectToMap(object_with_zone, objects);
        }
    };
#endif

#if SP_PLATFORM_CLIENT
    template <> // needed to receive a return value from the server on the client
    struct clmdep_msgpack::adaptor::convert<SpStaticClassDesc> {
        clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& object, SpStaticClassDesc& sp_static_class_desc) const {
            std::map<std::string, clmdep_msgpack::object> objects = MsgpackUtils::toMapOfMsgpackObjects(object);
            SP_ASSERT(objects.size() == 5);
            sp_static_class_desc.static_class_                = MsgpackUtils::to<SpHandle>(objects.at("static_class"));
            sp_static_class_desc.name_                        = MsgpackUtils::to<std::string>(objects.at("name"));
            sp_static_class_desc.derived_to_base_classes_     = MsgpackUtils::to<std::vector<SpHandle>>(objects.at("derived_to_base_classes"));
            sp_static_class_desc.derived_to_base_class_names_ = MsgpackUtils::to<std::vector<std::string>>(objects.at("derived_to_base_class_names"));
            sp_static_class_desc.function_descs_              = MsgpackUtils::to<std::map<std::string, SpFunctionDesc>>(objects.at("function_descs"));
            return object;
        }
    };
#endif

// SpWorldDesc (only sent from the server to the client)

#if SP_PLATFORM_SERVER
    template <> // needed to send a return value from the server to the client
    struct clmdep_msgpack::adaptor::object_with_zone<SpWorldDesc> {
        void operator()(clmdep_msgpack::object::with_zone& object_with_zone, SpWorldDesc const& sp_world_desc) const {
            std::map<std::string, clmdep_msgpack::object> objects;
            Std::insert(objects, "world",           MsgpackUtils::toMsgpackObject(sp_world_desc.world_,           object_with_zone));
            Std::insert(objects, "world_id",        MsgpackUtils::toMsgpackObject(sp_world_desc.world_id_,        object_with_zone));
            Std::insert(objects, "is_editor_world", MsgpackUtils::toMsgpackObject(sp_world_desc.is_editor_world_, object_with_zone));
            Std::insert(objects, "is_game_world",   MsgpackUtils::toMsgpackObject(sp_world_desc.is_game_world_,   object_with_zone));
            Std::insert(objects, "is_playing",      MsgpackUtils::toMsgpackObject(sp_world_desc.is_playing_,      object_with_zone));
            MsgpackUtils::setMsgpackObjectToMap(object_with_zone, objects);
        }
    };
#endif

#if SP_PLATFORM_CLIENT
    template <> // needed to receive a return value from the server on the client
    struct clmdep_msgpack::adaptor::convert<SpWorldDesc> {
        clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& object, SpWorldDesc& sp_world_desc) const {
            std::map<std::string, clmdep_msgpack::object> objects = MsgpackUtils::toMapOfMsgpackObjects(object);
            SP_ASSERT(objects.size() == 5);
            sp_world_desc.world_           = MsgpackUtils::to<SpHandle>(objects.at("world"));
            sp_world_desc.world_id_        = MsgpackUtils::to<int64_t>(objects.at("world_id"));
            sp_world_desc.is_editor_world_ = MsgpackUtils::to<bool>(objects.at("is_editor_world"));
            sp_world_desc.is_game_world_   = MsgpackUtils::to<bool>(objects.at("is_game_world"));
            sp_world_desc.is_playing_      = MsgpackUtils::to<bool>(objects.at("is_playing"));
            return object;
        }
    };
#endif
