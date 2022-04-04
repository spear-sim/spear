#pragma once

class Task
{
public:
    Task() = default;
    virtual ~Task() = default;

    virtual float getReward() = 0;
    virtual void reset() = 0;
};
