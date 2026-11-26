//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

//
// This header participates in SP_PLATFORM_PURE builds.
//

#pragma once

#include <stdint.h> // int32_t, uint16_t, uint64_t

#include <map>
#include <string>
#include <vector>

#include "SpCore/Platform.h"
#include "SpCore/Std.h"

//
// This header defines types that are exchanged between Python and C++ via msgpack. Each type has a definition inside
// Unreal and a mirrored definition outside of Unreal (e.g., in python_ext), and the two definitions must remain
// compatible with each other according to the msgpack adaptors in SpServices/MsgpackAdaptors.h. We place each pair of
// definitions next to each other so they're easier to keep in sync. If a type has the same definition in both types of
// builds, we define it in an SP_PLATFORM_UNREAL || SP_PLATFORM_PURE block to indicate that it is a mirrored type. Any
// type that is only needed by one of the definitions is defined in the corresponding SP_PLATFORM_UNREAL or
// SP_PLATFORM_PURE block.
//

#if SP_PLATFORM_UNREAL
    class FProperty;
#endif

//
// SpOutputLogMessages stores the messages that have been written to Unreal's output log.
//

#if SP_PLATFORM_UNREAL || SP_PLATFORM_PURE
    struct SpOutputLogMessages // exposed to Python via SpServices/MsgpackAdaptors.h
    {
        std::vector<std::string> messages_;
        int32_t num_evicted_ = 0;
    };
#endif

//
// SpSharedMemoryView stores the data needed to access a shared memory region.
//

#if SP_PLATFORM_UNREAL
    struct SpSharedMemoryView // exposed to Python via SpServices/MsgpackAdaptors.h
    {
        std::string id_; // platform-dependent name used to access the shared memory resource from other processes
        uint64_t num_bytes_ = 0;
        uint16_t offset_bytes_ = 0;
        void* data_ = nullptr;
    };
#elif SP_PLATFORM_PURE
    struct SpSharedMemoryView
    {
        std::string id_;
        uint64_t num_bytes_ = 0;
        uint16_t offset_bytes_ = 0;
    };
#else
    #error
#endif

//
// SpSharedMemoryFenceLayout describes the layout of the shared memory region that backs a SharedMemoryFence. The
// region consists of a uint64_t counter, followed by a fixed-size, null-terminated buffer that contains a key that
// uniquely identifies the fence (see SharedMemoryFence::SharedMemoryFence(...) in SpCore/SharedMemory.cpp).
//

#if SP_PLATFORM_UNREAL || SP_PLATFORM_PURE
    struct SpSharedMemoryFenceLayout
    {
        inline static constexpr uint64_t s_key_num_bytes_ = 1024;
    };
#endif

//
// Each SpArray has a data type (uint8, float32, etc), a data source (backed by an internal std::vector, backed
// by an external pointer, backed by shared memory, etc), and shared memory usage flags that indicate how a shared
// memory region can be used.
//

#if SP_PLATFORM_UNREAL
    enum class SpArrayDataType
    {
        Invalid = -1,
        UInt8   = 0,
        Int8    = 1,
        UInt16  = 2,
        Int16   = 3,
        UInt32  = 4,
        Int32   = 5,
        UInt64  = 6,
        Int64   = 7,
        Float16 = 8,
        Float32 = 9,
        Float64 = 10
    };

    enum class SpArrayDataSource
    {
        Invalid  = -1,
        Internal = 0,
        External = 1,
        Shared   = 2
    };

    enum class SpArraySharedMemoryUsageFlags
    {
        DoNotUse    = 0,
        Arg         = 1 << 0,
        ReturnValue = 1 << 1
    };
    SP_DECLARE_ENUM_FLAG_OPERATORS(SpArraySharedMemoryUsageFlags);
#endif

//
// SpArraySharedMemoryView is similar to an SpSharedMemoryView, but with extra members variables that can be
// specified by the system that owns the shared memory to indicate how the shared memory should be used, and
// a public name that other systems should use to access the memory.
//

#if SP_PLATFORM_UNREAL
    struct SpArraySharedMemoryView : SpSharedMemoryView // exposed to Python via SpServices/MsgpackAdaptors.h
    {
        SpArraySharedMemoryView() = default;
        SpArraySharedMemoryView(const SpSharedMemoryView& view, const std::string& name, SpArraySharedMemoryUsageFlags usage_flags) : SpSharedMemoryView(view)
        {
            name_ = name;
            usage_flags_ = usage_flags;
        }

        std::string name_ = "smem:invalid";
        SpArraySharedMemoryUsageFlags usage_flags_ = SpArraySharedMemoryUsageFlags::DoNotUse;
    };
#elif SP_PLATFORM_PURE
    struct SpArraySharedMemoryView
    {
        std::string id_;
        uint64_t num_bytes_ = 0;
        uint16_t offset_bytes_ = 0;
        std::vector<std::string> usage_flags_ = {"DoNotUse"};
        std::string name_ = "smem:invalid";
    };
#else
    #error
#endif

//
// SpPackedArray represents an array that can be passed to or returned from an SpFunc. We represent the data
// payload in each SpPackedArray as an std::vector<uint8_t>, regardless of its actual data type, because
// vectors of uint8_t get converted to a more efficient msgpack representation than other types. This results
// in faster data movement. See the following link for details:
//     https://github.com/msgpack/msgpack-c/wiki/v2_0_cpp_adaptor
//
// The mirrored definition of SpPackedArray outside of Unreal depends on nanobind, so it is defined in
// SpServices/SpTypes.h instead of here.
//

#if SP_PLATFORM_UNREAL
    using SpPackedArrayAllocator = SpAlignedAllocator<uint8_t, 4096>;

    class SPCORE_API SpPackedArray // exposed to Python via SpServices/MsgpackAdaptors.h
    {
    public:
        inline static constexpr int s_alignment_padding_bytes_ = 4096;

        // typically called before calling an SpFunc to resolve pointers to shared memory
        void resolve();
        void resolve(const SpArraySharedMemoryView& shared_memory_view);

        // typically called before calling an SpFunc to validate that an SpPackedArray can be used as an arg, and
        // after calling an SpFunc to validate that an SpPackedArray can be used as a return value
        void validate(SpArraySharedMemoryUsageFlags usage_flags) const;

        std::vector<uint8_t, SpPackedArrayAllocator> data_;
        void* view_ = nullptr;
        SpArrayDataSource data_source_ = SpArrayDataSource::Invalid;

        std::vector<uint64_t> shape_;
        SpArrayDataType data_type_ = SpArrayDataType::Invalid;

        std::string shared_memory_name_;
        SpArraySharedMemoryUsageFlags shared_memory_usage_flags_ = SpArraySharedMemoryUsageFlags::DoNotUse;
    };
#endif

//
// SpFuncDataBundle is intended as a high-level helper struct that can be used as the argument to, and the
// return value from, an SpFunc. We choose to make this a struct so it will be easier to add fields if
// necessary, without needing to explicitly update the signature of every SpFunc.
//
// The mirrored definition of SpFuncDataBundle outside of Unreal depends on SpPackedArray, so it is defined in
// SpServices/SpTypes.h instead of here.
//

#if SP_PLATFORM_UNREAL
    struct SpFuncDataBundle // exposed to Python via SpServices/MsgpackAdaptors.h
    {
        std::map<std::string, SpPackedArray> packed_arrays_;
        std::map<std::string, std::string> unreal_obj_strings_;
        std::string info_;
    };
#endif
//
// SpHandle is an opaque reference to an object that can be passed back and forth between Python and C++.
// For UObjects, object_index_ and object_serial_number_ store the same data as a TWeakObjectPtr, so we can
// detect when a handle refers to an object that has been destroyed, even if its address has been reused by a
// new object. For all other types, object_index_ and object_serial_number_ are 0.
//

#if SP_PLATFORM_UNREAL || SP_PLATFORM_PURE
    struct SpHandle // exposed to Python via SpServices/MsgpackAdaptors.h
    {
        uint64_t ptr_ = 0;
        int32_t object_index_ = 0;
        int32_t object_serial_number_ = 0;

        bool operator==(const SpHandle& other) const = default; // needed by SpPropertyDesc::operator==
    };
#endif

//
// SpPropertyDesc and SpPropertyValue encapsulate the data needed to get and set a UPROPERTY on a specific UObject.
//

#if SP_PLATFORM_UNREAL
    struct SpPropertyDesc // exposed to Python via SpServices/MsgpackAdaptors.h
    {
        FProperty* property_ = nullptr; // leaf property, not always equal to the tail of notify_member_properties_ when property_ refers to a specific entry in an array or map
        void* value_ptr_ = nullptr;     // pointer to property_'s value for a specific object or struct
        std::string type_id_;           // string ID representing the type of property_

        // The following arrays are intended to be parallel, i.e., all the same length, with one entry per step
        // in the resolved traversal. notify_objects_ and notify_member_properties_ are always set, and the
        // remaining arrays describe an array or map entry at each step and are sentinels (nullptr, -1, "") for a
        // plain member step. The object to notify is notify_objects_.back().
        std::vector<SpHandle> notify_objects_;              // base object at each step (repeated across a struct segment; changes each time a new object is traversed), SpHandle needed so SpPropertyDesc can be serialized safely on the worker thread
        std::vector<FProperty*> notify_member_properties_;  // named member property at each step (i.e., the container for an array or map entry)
        std::vector<FProperty*> notify_element_properties_; // array Inner, fixed-size array property, or map ValueProp at each step, or nullptr for a plain member step
        std::vector<int> notify_array_indices_;             // array index at each step, or -1 for a non-array step
        std::vector<std::string> notify_map_keys_;          // map key at each step, or "" for a non-map step

        bool operator==(const SpPropertyDesc& other) const = default; // used to assert that a parent desc is entirely default
    };
#elif SP_PLATFORM_PURE
    struct SpPropertyDesc
    {
        uint64_t property_ = 0;
        uint64_t value_ptr_ = 0;
        std::string type_id_;

        std::vector<SpHandle> notify_objects_;
        std::vector<uint64_t> notify_member_properties_;
        std::vector<uint64_t> notify_element_properties_;
        std::vector<int> notify_array_indices_;
        std::vector<std::string> notify_map_keys_;
    };
#else
    #error
#endif

#if SP_PLATFORM_UNREAL || SP_PLATFORM_PURE
    struct SpPropertyValue // exposed to Python via SpServices/MsgpackAdaptors.h
    {
        std::string value_;
        std::string type_id_;
    };
#endif
