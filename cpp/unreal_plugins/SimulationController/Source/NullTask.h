#pragma once

#include "Box.h"
#include "Task.h"

class NullTask: public Task
{
public:

    NullTask() = default;
    ~NullTask() = default;

    void findObjectReferences(UWorld* world) override {}
    void cleanUpObjectReferences() override {}

    void beginFrame() override {}
    void endFrame() override {}

    float getReward() const override
    {
        return 0.0;
    }

    bool isEpisodeDone() const override
    {
        return false;
    }

    std::map<std::string, Box> getStepInfoSpace() const override
    {
        return {};
    }

    std::map<std::string, std::vector<uint8_t>> getStepInfo() const override
    {   
        return {};
    }

    void reset() override {}

    bool isReady() const override 
    {
        return true;
    }
};
