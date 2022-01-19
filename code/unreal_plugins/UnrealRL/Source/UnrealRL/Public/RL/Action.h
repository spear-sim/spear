#pragma once

#include <ostream>
#include <vector>

#include "Utils/RpcMsgpackInclude.h"

namespace unrealrl
{
/**
 * 1D flatten array of actions.
 */
class Action
{
public:
    // We need default constructor for MSGPACK binding
    Action()
    {
    }

    Action(const Action& src)
    {
        Data = src.Data;
    }

    FORCEINLINE const std::vector<float>& GetActions() const
    {
        return Data;
    }

    MSGPACK_DEFINE_MAP(Data);

    FORCEINLINE friend std::ostream& operator<<(std::ostream& os, Action* s)
    {
        for (const auto& a_ : s->Data)
        {
            os << a_ << ", ";
        }
        os << "\n";
        return os;
    }

private:
    std::vector<float> Data;
};

} // end namespace unrealrl
