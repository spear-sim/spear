#pragma once

class Task
{
public:
    Task() = default;
    virtual ~Task() = default;

    virtual void beginFrame() {}
    virtual void endFrame() {}
    virtual float getReward() = 0;
    virtual bool isEpisodeDone() const = 0;
    virtual void reset() = 0;
};
