#pragma once

#include <vector>

#include "Rpclib.h"

// Supported types for sending observation and action data types via msgpackrpc
// enum values must match the values in code/python_package/spear/env.py
enum class DataType : uint8_t
{
    Boolean    = 0,
    UInteger8  = 1,
    Integer8   = 2,
    UInteger16 = 3,
    Integer16  = 4,
    UInteger32 = 5,
    Integer32  = 6,
    Float32    = 7,
    Double     = 8,
};
MSGPACK_ADD_ENUM(DataType);

struct Box
{
    double low;
    double high;
    std::vector<int64_t> shape;
    DataType dtype;

    MSGPACK_DEFINE_MAP(low, high, shape, dtype);
};
