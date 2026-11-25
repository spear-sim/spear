//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stddef.h> // size_t
#include <stdint.h> // int32_t, int64_t, uint8_t, uint16_t, uint64_t

#include <map>
#include <span>
#include <string>
#include <vector>

#include <nanobind/ndarray.h>

//
// Base class for Client that contains Client's configuration flags, so they can be accessed by code
// that can't include client.h, e.g., MsgpackUtils::convert(...)
//

struct ClientBase
{
    bool force_return_aligned_arrays_ = false;
    bool suppress_default_logging_    = false;
    bool verbose_rpc_calls_           = false;
    bool verbose_allocations_         = false;
    bool verbose_exceptions_          = false; // owner should set to true once a connection to the server is established
};

//
// Custom types
//

struct SpOutputLogMessages
{
    std::vector<std::string> messages_;
    int32_t num_evicted_ = 0;
};

struct SpSharedMemoryView
{
    std::string id_;
    uint64_t num_bytes_ = 0;
    uint16_t offset_bytes_ = 0;
};

struct SpArraySharedMemoryView
{
    std::string id_;
    uint64_t num_bytes_ = 0;
    uint16_t offset_bytes_ = 0;
    std::vector<std::string> usage_flags_ = {"DoNotUse"};
    std::string name_ = "smem:invalid";
};

struct SpPackedArray
{
    nanobind::ndarray<nanobind::numpy> data_;
    std::string data_source_ = "Invalid";
    std::vector<size_t> shape_;
    std::string shared_memory_name_;
};

struct SpFuncDataBundle
{
    std::map<std::string, SpPackedArray> packed_arrays_;
    std::map<std::string, std::string> unreal_obj_strings_;
    std::string info_;
};

struct SpHandle
{
    uint64_t ptr_ = 0;
    int32_t object_index_ = 0;
    int32_t object_serial_number_ = 0;
};

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

struct SpPropertyValue
{
    std::string value_;
    std::string type_id_;
};

struct SpFuncSignatureDesc
{
    std::string name_;
    std::vector<int> type_ids_;
    std::vector<std::string> type_names_;
};

struct SpFuture
{
    uint64_t future_ptr_ = 0;
    std::string type_id_;
};

struct SpFunctionDesc
{
    SpHandle function_;
    std::string function_name_;
    SpHandle static_class_;
    std::string static_class_name_;
};

struct SpStaticStructDesc
{
    SpHandle static_struct_;
    std::string name_;
};

struct SpStaticClassDesc
{
    SpHandle static_class_;
    std::string name_;
    std::vector<SpHandle> derived_to_base_classes_;
    std::vector<std::string> derived_to_base_class_names_;
    std::map<std::string, SpFunctionDesc> function_descs_;
};

struct SpWorldDesc
{
    SpHandle world_;
    int64_t world_id_ = -1;
    bool is_editor_world_ = false;
    bool is_game_world_ = false;
    bool is_playing_ = false;
};

//
// View types
//

struct SpPackedArrayView
{
    std::span<uint8_t> view_;
    std::string data_source_ = "Invalid";
    std::vector<size_t> shape_;
    std::string shared_memory_name_;
    std::string data_type_;
};

struct SpFuncDataBundleView
{
    std::map<std::string, SpPackedArrayView> packed_array_views_;
    std::map<std::string, std::string> unreal_obj_strings_;
    std::string info_;
};
