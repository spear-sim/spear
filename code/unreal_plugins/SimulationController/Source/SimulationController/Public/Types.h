#pragma once

#include <limits>
#include <utility>

#include "Rpclib.h"

/**
 * Supported types for sending observation and action data types via msgpackrpc
 */
enum class DataType : uint8
{
    Boolean = 0,
    UInteger8 = 1,
    Integer8 = 2,
    UInteger16 = 3,
    Integer16 = 4,
    UInteger32 = 5,
    Integer32 = 6,
    Float32 = 7,
    Double = 8,
};
MSGPACK_ADD_ENUM(DataType);


enum class Endianness : uint8
{
    LittleEndian = 0,
    BigEndian = 1,
};
MSGPACK_ADD_ENUM(Endianness);


constexpr std::pair<double, double> GetDefaultBounds(DataType tDtype)
{
    std::pair<double, double> Bounds;

    switch (tDtype) {
    case DataType::UInteger8:
        Bounds = std::make_pair(0, 255);
        break;
    case DataType::Integer8:
        Bounds = std::make_pair(-128, 127);
        break;
    case DataType::UInteger16:
        Bounds = std::make_pair(0, 65535);
        break;
    case DataType::Integer16:
        Bounds = std::make_pair(-32768, 32767);
        break;
    case DataType::UInteger32:
        Bounds = std::make_pair(std::numeric_limits<uint32>::lowest(), std::numeric_limits<uint32>::max());
        break;
    case DataType::Integer32:
        Bounds = std::make_pair(std::numeric_limits<int32>::lowest(), std::numeric_limits<int32>::max());
        break;
    case DataType::Float32:
        Bounds = std::make_pair(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::max());
        break;
    case DataType::Double:
        Bounds = std::make_pair(std::numeric_limits<double>::lowest(), std::numeric_limits<double>::max());
        break;
    case DataType::Boolean:
        Bounds = std::make_pair(0, 1);
        break;
    }

    return Bounds;
};


static Endianness GetEndianness()
{
    uint32_t Num = 0x01020304;
    return (reinterpret_cast<const char*>(&Num)[3] == 1) ? Endianness::LittleEndian : Endianness::BigEndian;
}


struct Box
{
    float low;
    float high;
    std::vector<int> shape;
    DataType dtype;

    MSGPACK_DEFINE_MAP(low, high, shape, dtype);
};
