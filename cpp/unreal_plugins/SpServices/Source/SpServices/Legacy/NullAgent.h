//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // uint8_t

#include <map>
#include <string>
#include <vector>

#include "SpCore/Legacy/ArrayDesc.h" // TODO: remove

class UWorld;

class NullAgent: public Agent
{
public:
    NullAgent() = default;
    NullAgent(UWorld*) {}
    ~NullAgent() = default;

    void findObjectReferences(UWorld* world) override {};
    void cleanUpObjectReferences() override {};

    std::map<std::string, ArrayDesc> getActionSpace() const override
    {
        return {};
    };

    std::map<std::string, ArrayDesc> getObservationSpace() const override
    {
        return {};
    };

    std::map<std::string, ArrayDesc> getStepInfoSpace() const override
    {
        return {};
    };
 
    void applyAction(const std::map<std::string, std::vector<uint8_t>>& action) override {};

    std::map<std::string, std::vector<uint8_t>> getObservation() const override
    {
        return {};
    };

    std::map<std::string, std::vector<uint8_t>> getStepInfo() const override
    {
        return {};
    };
    
    void reset() override {};

    bool isReady() const override
    {
        return true;
    };
};
