//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <map>
#include <string>
#include <vector>

#include "CoreUtils/ArrayDesc.h"

class UWorld;

class Task
{
public:
    // A Task class must spawn new objects in the constructor if they are intended to be findable by other classes. A Task class must not attempt
    // to find object references in the constructor, because these objects might not be spawned yet. Use findObjectReferences(...) instead.
    Task() = default;
    virtual ~Task() = default;

    virtual void findObjectReferences(UWorld* world) = 0;
    virtual void cleanUpObjectReferences() = 0;

    virtual void beginFrame() = 0;
    virtual void endFrame() = 0;

    virtual float getReward() const = 0;
    virtual bool isEpisodeDone() const = 0;
    virtual std::map<std::string, ArrayDesc> getStepInfoSpace() const = 0;
    virtual std::map<std::string, std::vector<uint8_t>> getStepInfo() const = 0;
    virtual void reset() = 0;
    virtual bool isReady() const = 0;
};
