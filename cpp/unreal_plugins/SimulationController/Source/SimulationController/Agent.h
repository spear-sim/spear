//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <map>
#include <string>
#include <vector>

#include "CoreUtils/ArrayDesc.h"

class UWorld;

class Agent
{
public:
    // An Agent class must spawn new objects in the constructor if they are intended to be
    // findable by other classes. An Agent class must not attempt to find object references
    // in the constructor, because these objects might not be spawned yet. Use findObjectReferences(...)
    // instead.
    Agent() = default;
    virtual ~Agent() = default;

    virtual void findObjectReferences(UWorld* world) = 0;
    virtual void cleanUpObjectReferences() = 0;

    virtual std::map<std::string, ArrayDesc> getActionSpace() const = 0;
    virtual std::map<std::string, ArrayDesc> getObservationSpace() const = 0;
    virtual std::map<std::string, ArrayDesc> getStepInfoSpace() const = 0;   
 
    virtual void applyAction(const std::map<std::string, std::vector<uint8_t>>& action) = 0;
    virtual std::map<std::string, std::vector<uint8_t>> getObservation() const = 0;
    virtual std::map<std::string, std::vector<uint8_t>> getStepInfo() const = 0;
    
    virtual void reset() = 0;
    virtual bool isReady() const = 0;
};
