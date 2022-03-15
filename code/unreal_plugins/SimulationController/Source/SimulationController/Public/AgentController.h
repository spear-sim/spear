#pragma once

class SIMULATIONCONTROLLER_API AgentController
{
public:
    AgentController() = default;
    ~AgentController() = default;

    /**
     * @brief 
     * 
     */
    virtual void Init();

    /**
     * @brief 
     * 
     */
    virtual void Terminate();

    /**
     * Call this to prep agent as per the values received from @param Action
     * The actions has to be performed during consequent tick call.
     * @todo: should we expose this function to blueprint?
     */
    virtual void SetAction();

    /**
     * Call this to record observation for this agent.
     * This function has to be called post tick call to get new observations.
     * @todo: should we expose this function to blueprint?
     */
    virtual void
    GetObservation();

    /**
     * Use this to set agent's initial state i.e state at the beginning of an
     * episode. At the beginning of each episode, this function will be called.
     */
    virtual void BeginEpisode();

    /**
     * Use this to check if agent/actor has met certain conditions that
     * classifies it as experiment ready. If yes, set return true.
     */
    virtual bool IsAgentReady();
};
