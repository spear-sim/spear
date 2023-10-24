//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // int64_t

#include <cmath> // std::nan
#include <string>
#include <vector>

#include "CoreUtils/Rpclib.h" // MSGPACK_ADD_ENUM, MSGPACK_DEFINE_MAP

// enum values must match python/spear/env.py, which is why we write them out explicitly
enum class DataType
{
    Invalid    = -1,
    UInteger8  = 0,
    Integer8   = 1,
    UInteger16 = 2,
    Integer16  = 3,
    UInteger32 = 4,
    Integer32  = 5,
    Float16    = 6,
    Float32    = 7,
    Float64    = 8,
};
MSGPACK_ADD_ENUM(DataType);

struct ArrayDesc
{
    double low_ = std::nan("");
    double high_ = std::nan("");
    std::vector<int64_t> shape_;
    DataType datatype_ = DataType::Invalid;
    bool use_shared_memory_ = false;
    std::string shared_memory_name_;

    MSGPACK_DEFINE_MAP(low_, high_, shape_, datatype_, use_shared_memory_, shared_memory_name_);
};
