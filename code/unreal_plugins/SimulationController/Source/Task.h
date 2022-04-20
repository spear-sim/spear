#pragma once

#include "Rpclib.h"

// HACK: should not contain task specific details like hit_goal, hit_obstacle here!! 
struct StepInfo
{
    bool hit_goal_;
    bool hit_obstacle_;

    MSGPACK_DEFINE_MAP(hit_goal_, hit_obstacle_);
};


class Task
{
public:
    Task() = default;
    virtual ~Task() = default;

    virtual void beginFrame() = 0;
    virtual void endFrame() = 0;
    virtual float getReward() const = 0;
    virtual bool isEpisodeDone() const = 0;
    virtual void reset() = 0;
    virtual StepInfo getStepInfo() = 0;
};
