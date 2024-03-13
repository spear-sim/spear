//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // uint8_t

#include <map>
#include <string>
#include <vector>

#include "SpCore/ArrayDesc.h"
#include "SpEngine/ClassRegistrationUtils.h"

class UWorld;

class NullAgent: public Agent
{
public:
    NullAgent() = delete;
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

    std::map<std::string, std::vector<uint8_t>> getObservation() override
    {
        return {};
    };

    std::map<std::string, std::vector<uint8_t>> getStepInfo() override
    {
        return {};
    };
    
    void reset() override {};

    bool isReady() const override
    {
        return true;
    };

    inline static auto s_class_registrar_ = ClassRegistrationUtils::registerClass<NullAgent>(Agent::s_class_registrar_, "NullAgent");
};
