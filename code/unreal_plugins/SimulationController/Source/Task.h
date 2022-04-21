#pragma once

#include <map>
#include <string>
#include <vector>

struct Box;

class Task
{
public:
    Task() = default;
    virtual ~Task() = default;

    virtual void beginFrame() = 0;
    virtual void endFrame() = 0;
    virtual float getReward() const = 0;
    virtual bool isEpisodeDone() const = 0;
    virtual std::map<std::string, Box> getStepInfoSpace() const = 0;
    virtual std::map<std::string, std::vector<uint8_t>> getStepInfo() const = 0;
    virtual void reset() = 0;
    virtual bool isReady() const = 0;
};
