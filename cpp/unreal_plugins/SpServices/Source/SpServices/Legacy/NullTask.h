//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // uint8_t

#include <map>
#include <string>
#include <vector>

#include "SpCore/ArrayDesc.h"

#include "SpServices/Legacy/Task.h"

class UWorld;

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
        return 0.0f;
    }

    bool isEpisodeDone() const override
    {
        return false;
    }

    std::map<std::string, ArrayDesc> getStepInfoSpace() const override
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
