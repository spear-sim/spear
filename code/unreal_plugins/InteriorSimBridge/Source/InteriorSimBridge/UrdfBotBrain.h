#pragma once

#include <vector>

#include <Brain.h>

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
    bool IsAgentReady() override;
    void OnEpisodeBegin() override;
    void SetAction(const std::vector<unrealrl::Action>& Action) override;
    void GetObservation(std::vector<unrealrl::Observation>&) override;

    // actual robot agent for training
    UPROPERTY()
    class AUrdfBotPawn* Owner = nullptr;

    UPROPERTY()
    class AActor* Goal = nullptr;

private:
    enum class UHitInfo
    {
        NoHit,
        Goal,
        Edge
    };

    UHitInfo HitInfo = UHitInfo::NoHit;
};
