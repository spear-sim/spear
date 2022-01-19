#pragma once

#include "RpcMsgpackInclude.h"

#include <limits>

namespace unrealrl
{
/**
 * Different types of observations
 */
enum class DataType : uint8
{
    Boolean = 0,
    UInteger8 = 1,
    Integer8 = 2,
    UInteger16 = 3,
    Integer16 = 4,
    Float32 = 5,
    Double = 6,
};

constexpr std::pair<float, float> GetDefaultBounds(DataType tDtype)
{
    std::pair<float, float> Bounds;
    switch (tDtype)
    {
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
    case DataType::Float32:
        Bounds = std::make_pair(std::numeric_limits<float>::lowest(),
                                std::numeric_limits<float>::max());
        break;
    case DataType::Double:
        Bounds = std::make_pair(std::numeric_limits<float>::lowest(),
                                std::numeric_limits<float>::max());
        break;
    case DataType::Boolean:
        Bounds = std::make_pair(0, 1);
        break;
    }

    return Bounds;
};

} // namespace unrealrl

MSGPACK_ADD_ENUM(unrealrl::DataType);
