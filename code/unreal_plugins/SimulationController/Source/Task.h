#pragma once

struct StepInfo;

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
