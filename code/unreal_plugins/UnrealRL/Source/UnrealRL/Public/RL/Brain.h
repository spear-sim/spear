// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"

#include "ActionDict.h"
#include "ActionSpecs.h"
#include "ObservationDict.h"
#include "ObservationSpecs.h"
#include "StepInfo.h"

#include "Brain.generated.h"

/**
 * Brain class is the base component class needed to convert any actor or its
 * subclasses to an  RL Agent.
 */
UCLASS(Abstract,
       ClassGroup = UnrealRL,
       meta = (DisplayName = "RL Brain Component"),
       HideCategories(Mobility, Rendering, LOD, Collision, Physics, Cooking))
class UNREALRL_API UBrain : public UActorComponent
{
    GENERATED_BODY()

public:
    UBrain(const FObjectInitializer& ObjectInitializer)
        : Super(ObjectInitializer)
    {
    }

    virtual void Init() PURE_VIRTUAL(UBrain::Init, ;);

    /**
     * Call this to prep agent as per the values received from @param Action
     * The actions has to be performed during consequent tick call.
     * @todo: expose this function to blueprint
     */
    virtual void SetAction(const std::vector<unrealrl::Action>& Action)
        PURE_VIRTUAL(UBrain::SetAction, ;);

    /**
     * Call this to record observation for this agent.
     * This function has to be called post tick call to get new observations.
     * @todo: expose this function to blueprint
     */
    virtual void
    GetObservation(std::vector<unrealrl::Observation>& ObservationVec)
        PURE_VIRTUAL(UBrain::GetObservation, ;);

    /**
     * Use this to set agent's initial state i.e state at the beginning of an
     * episode. At the beginning of each episode, this function will be called.
     */
    virtual void OnEpisodeBegin() PURE_VIRTUAL(UBrain::OnEpisodeBegin, ;);

    /**
     * Use this to check if agent/actor has met certain conditions that
     * classifies it as experiment ready. If yes, set return true.
     */
    virtual bool IsAgentReady()
        PURE_VIRTUAL(UBrain::IsAgentReady, return false;);

    /**
     * Call this to retrieve reward information along with if this agent wants
     * the episode to end. This function has to be called post tick call to get
     * new data.
     */
    FORCEINLINE void GetStepInfo(unrealrl::StepInfo& Info)
    {
        Info.SetDone(bEpisodeEnded);
        Info.SetReward(GetCurrentReward());
    }

    FORCEINLINE void BeginPlay() override
    {
        Super::BeginPlay();

        Init();
    }

    FORCEINLINE const unrealrl::ObservationSpecs& GetObservationSpecs() const
    {
        return ObsSpecs;
    }

    FORCEINLINE void
    SetObservationSpecs(const unrealrl::ObservationSpecs& InObsSpec)
    {
        ObsSpecs = InObsSpec;
    }

    FORCEINLINE void SetActionSpecs(const unrealrl::ActionSpecs& InActSpecs)
    {
        ActSpecs = InActSpecs;
    }

    FORCEINLINE const unrealrl::ActionSpecs& GetActionSpecs() const
    {
        return ActSpecs;
    }

    FORCEINLINE void AddReward(float Value)
    {
        Reward = Reward + Value;
    }

    FORCEINLINE void SetCurrentReward(float Value)
    {
        Reward = Value;
    }

    FORCEINLINE float GetCurrentReward() const
    {
        return Reward;
    }

    FORCEINLINE void BeginEpisode()
    {
        bEpisodeEnded = false;
        OnEpisodeBegin();
    }

    FORCEINLINE void EndEpisode()
    {
        bEpisodeEnded = true;
    }

    FORCEINLINE void SetAgentReady(bool InBool)
    {
        bIsReady = InBool;
    }

private:
    /** This defines the observation space for this agent. */
    unrealrl::ObservationSpecs ObsSpecs;

    /** This defines the action space for this agent. */
    unrealrl::ActionSpecs ActSpecs;

    /**
     * This defines the reward for this agent at each step.
     * This is updated every step.
     */
    float Reward;

    /**
     * This defines the end of an episode boolean for this agent.
     */
    bool bEpisodeEnded = false;

    /**
     * Boolean flag to check readiness of an agent controlled by this brain
     */
    bool bIsReady = false;
};
