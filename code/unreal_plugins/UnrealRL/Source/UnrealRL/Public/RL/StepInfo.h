#pragma once

#include "Utils/RpcMsgpackInclude.h"

namespace unrealrl
{
class StepInfo
{
public:
    StepInfo()
    {
    }

    StepInfo(float reward, bool done)
    {
        Reward = reward;
        Done = done;
    }

    StepInfo(const StepInfo& src)
    {
        Reward = src.Reward;
        Done = src.Done;
    }

    FORCEINLINE void SetReward(float reward)
    {
        Reward = reward;
    }

    FORCEINLINE float GetReward() const
    {
        return Reward;
    }

    FORCEINLINE void SetDone(bool done)
    {
        Done = done;
    }

    FORCEINLINE bool IsDone() const
    {
        return Done;
    }

    MSGPACK_DEFINE_MAP(Reward, Done);

private:
    float Reward;
    bool Done;
};

} // namespace unrealrl
