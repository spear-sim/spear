#pragma once

#include <RL/Brain.h>
#include "UrdfBotBrain.generated.h"

UCLASS(Blueprintable)
class INTERIORSIMBRIDGE_API UUrdfBotBrain : public UBrain
{
    GENERATED_BODY()

    UUrdfBotBrain(const FObjectInitializer& ObjectInitializer);

public:
    UFUNCTION()
    void OnActorHit(AActor* SelfActor,
                    AActor* OtherActor,
                    FVector NormalImpulse,
                    const FHitResult& Hit);

    // UnrealRL overrides.
    void Init() override;
    void OnEpisodeBegin() override;
    void SetAction(const std::vector<unrealrl::Action>& Action) override;
    void GetObservation(std::vector<unrealrl::Observation>&) override;
    bool IsAgentReady() override;

    // actual robot agent for training
    UPROPERTY()
    class AActor* Owner;

    UPROPERTY()
    class AActor* Goal = nullptr;

    /** Camera through which simulation is viewed. */
    UPROPERTY()
    class AActor* ViewCamera = nullptr;

    /** Torque to apply when trying to roll ball */
    UPROPERTY(EditAnywhere)
    float Force;

private:
    class UStaticMeshComponent* Base;

    enum class UHitInfo
    {
        NoHit,
        Goal,
        Edge
    };

    UHitInfo HitInfo = UHitInfo::NoHit;
};