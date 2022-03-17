#pragma once


class SIMULATIONCONTROLLER_API AgentController
{
public:
    AgentController() = default;
    ~AgentController() = default;

    /**
     * Call this to prep agent as per the values received from @param Action
     * The actions will be performed during consequent tick call.
     * @todo: should we expose this function to blueprint?
     */
    virtual void applyAction(const std::vector<float>& Action);

    /**
     * This is to record observations for this agent.
     * This function has to be called post tick call to get new observations.
     * @todo: should we expose this function to blueprint?
     */
    virtual std::vector<uint8_t>& getObservation();
};
