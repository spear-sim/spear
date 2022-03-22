#pragma once

#include <map>
#include <string>
#include <vector>

#include "AgentController.h"

class SIMULATIONCONTROLLER_API SphereAgentController : public AgentController
{
public:

    SphereAgentController() = default;
    ~SphereAgentController() = default;
    
    virtual void applyAction(const std::map<std::string, std::vector<float>>& action) override;
    virtual std::map<std::string, std::vector<uint8_t>> getObservation() override;
    virtual std::map<std::string, struct Box> getObservationSpace() override;
    virtual std::map<std::string, struct Box> getActionSpace() override;
};
