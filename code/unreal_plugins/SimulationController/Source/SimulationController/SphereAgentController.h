#pragma once

#include <map>
#include <string>
#include <vector>

#include "AgentController.h"

// forward declarations
struct Box;

class AActor;
class UWorld;

class SIMULATIONCONTROLLER_API SphereAgentController : public AgentController
{
public:

    SphereAgentController(UWorld* world); // passing around UWorld pointer can be dangerous!
    ~SphereAgentController() = default;
    
    virtual void applyAction(const std::map<std::string, std::vector<float>>& action) override;
    virtual std::map<std::string, std::vector<uint8_t>> getObservation() override;
    virtual std::map<std::string, Box> getObservationSpace() override;
    virtual std::map<std::string, Box> getActionSpace() override;

private:
    AActor* sphere_agent_ = nullptr;
};
