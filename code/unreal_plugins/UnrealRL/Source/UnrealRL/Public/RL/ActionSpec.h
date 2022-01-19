#pragma once

#include <utility>
#include <vector>

#include "Utils/RpcMsgpackInclude.h"
#include "Utils/Types.h"

namespace unrealrl
{
/**
 * Specify spec of an action for an agent.
 * Each ActionSpec is intended to describe actions of the object of class @see
 * Action.h
 *
 * Example;
 *
 * Discrete case
 * - For 4 actions possible for an agent, say you are moving north if action=0,
 * east if action=1, etc and we apply only 1 action at at time,
 * - Shape={1}, bIsDiscrete = true, Bounds=[0, 3], inclusive, where 0 indicates
 * least possible value and 3 indicates max possible value
 * - Here, we just need 1 value to decode information about the action direction
 * and/or value.
 *
 * Continuous case
 * - You have 3 axes (x,y,z) and you want to have cont. values in range [-1,1].
 * - you will have Shape={3}, bIsDiscrete = false, Bounds=[-1,1] inclusive
 * - note that, even though we have 3 axes, it can be interpreted as a 1D vector
 * with 3 values.
 * - Currently we support only one bound space for the entire action space.
 * - However, if you want separate bounds, you can have them as separate actions
 * by having multiple @see Action.h objects.
 */
class ActionSpec
{
public:
    ActionSpec()
    {
    }

    ActionSpec(const ActionSpec& src)
    {
        Shape = src.Shape;
        Dtype = src.Dtype;
        bIsDiscrete = src.bIsDiscrete;
        Bounds = src.Bounds;
        Description = src.Description;
    }

    ActionSpec(bool tbIsDiscrete,
               DataType tDtype,
               const std::vector<size_t>& tShape,
               std::pair<float, float> tBounds)
    {
        bIsDiscrete = tbIsDiscrete;
        Dtype = tDtype;
        Shape = tShape;
        Bounds = tBounds;
        Description = std::string("No Description provided");
    }

    ActionSpec(bool tbIsDiscrete,
               DataType tDtype,
               const std::vector<size_t>& tShape,
               std::pair<float, float> tBounds,
               const std::string& tDescription)
    {
        bIsDiscrete = tbIsDiscrete;
        Dtype = tDtype;
        Shape = tShape;
        Bounds = tBounds;
        Description = tDescription;
    }

    MSGPACK_DEFINE_MAP(bIsDiscrete, Dtype, Shape, Bounds, Description);

    bool bIsDiscrete = true;
    DataType Dtype;
    std::vector<size_t> Shape;
    std::pair<float, float> Bounds;
    std::string Description;
};
} // namespace unrealrl
