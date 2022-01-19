#pragma once

#include <vector>

#include "Utils/RpcMsgpackInclude.h"
#include "Utils/Types.h"

/**
 * Describes the observation/s of an agent.
 * Each ObservationSpec is intended to describe one Observation object @see
 * Observation.h.
 *
 * An agent can have multiple observations of type @see Observation.h.
 * For instance, if an agent observes 6 values: its (x,y,z) position and (x,y,z)
 * velocity. Description = "Agent has 6 observations. It observes its position
 * along x,y,z aces and velocity along x,y,z axes." Shape = {6} Dtype =
 * DataType::Float32
 *
 * For another instance, if an agent observes an image: its size is 84x84x3.
 *   Description: "Agent observed an RGB image of size 84x84x3."
 *   Shape = {84,84,3}
 *   Dtype = DataType::UInteger8
 */
namespace unrealrl
{
class ObservationSpec
{
public:
    // We need default constructor for MSGPACK binding
    ObservationSpec()
    {
    }

    ObservationSpec(const ObservationSpec& src)
    {
        Shape = src.Shape;
        Bounds = src.Bounds;
        Description = src.Description;
        Dtype = src.Dtype;
    }

    ObservationSpec(const std::vector<size_t>& tShape,
                    const std::pair<float, float>& tBounds,
                    const DataType& tDtype)
    {
        Shape = tShape;
        Bounds = tBounds;
        Dtype = tDtype;
        Description = std::string("No Observation defined");
    }

    ObservationSpec(const std::vector<size_t>& tShape,
                    const DataType& tDtype,
                    const std::string& tDescription)
    {
        Shape = tShape;
        Dtype = tDtype;
        Description = tDescription;
        Bounds = GetDefaultBounds(tDtype);
    }

    ObservationSpec(const std::vector<size_t>& tShape,
                    const std::pair<float, float>& tBounds,
                    const DataType& tDtype,
                    const std::string& tDescription)
    {
        Shape = tShape;
        Bounds = tBounds;
        Dtype = tDtype;
        Description = tDescription;
    }

    MSGPACK_DEFINE_MAP(Shape, Bounds, Description, Dtype);

    std::vector<size_t> Shape;
    std::pair<float, float> Bounds;
    std::string Description;
    DataType Dtype;
};
} // namespace unrealrl
