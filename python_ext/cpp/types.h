//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stddef.h> // size_t
#include <stdint.h> // uint8_t, uint64_t

#include <map>
#include <span>
#include <string>
#include <vector>

#include <nanobind/ndarray.h>

//
// Custom types
//

struct PropertyDesc
{
    uint64_t property_ = 0;
    uint64_t value_ptr_ = 0;
};

struct SharedMemoryView
{
    std::string id_;
    uint64_t num_bytes_ = 0;
    uint64_t offset_bytes_ = 0;
    std::vector<std::string> usage_flags_ = {"DoNotUse"};
};

struct PackedArray
{
    nanobind::ndarray<nanobind::numpy> data_;
    std::string data_source_ = "Invalid";
    std::vector<size_t> shape_;
    std::string shared_memory_name_;
};

struct DataBundle
{
    std::map<std::string, PackedArray> packed_arrays_;
    std::map<std::string, std::string> unreal_obj_strings_;
    std::string info_;
};

struct FuncSignatureTypeDesc
{
    std::map<std::string, std::string> type_names_;
    std::map<std::string, std::string> const_strings_;
    std::map<std::string, std::string> ref_strings_;
};

struct FuncSignatureDesc
{
    std::string name_;
    std::vector<FuncSignatureTypeDesc> func_signature_;
    std::vector<int> func_signature_id_;
};

struct Future
{
    uint64_t future_ptr_ = 0;
    std::string type_id_;
};

//
// View types
//

struct PackedArrayView
{
    std::span<uint8_t> view_;
    std::string data_source_ = "Invalid";
    std::vector<size_t> shape_;
    std::string data_type_;
    std::string shared_memory_name_;
};

struct DataBundleView
{
    std::map<std::string, PackedArrayView> packed_array_views_;
    std::map<std::string, std::string> unreal_obj_strings_;
    std::string info_;
};
