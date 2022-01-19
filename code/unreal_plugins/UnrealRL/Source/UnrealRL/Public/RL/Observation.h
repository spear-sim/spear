#pragma once

#include <vector>

#include "Utils/RpcMsgpackInclude.h"

// TRICK_MODE_OFF

namespace unrealrl
{
/**
 * Observation data of a particular observation.
 * For image based observation data, flatten it to a 1-D array.
 */
class Observation
{
public:
    Observation()
    {
    }

    Observation(const Observation& src)
    {
        bIsSet = src.bIsSet;
        Data = src.Data;
    }

    Observation(const std::vector<float>& tData)
    {
        Data = tData;
        bIsSet = true;
    }

    MSGPACK_DEFINE_MAP(bIsSet, Data);

    bool bIsSet = false;
    std::vector<float> Data;
};
} // end namespace unrealrl
