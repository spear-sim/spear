#pragma once

#include "RL/Brain.h"
#include "SimpleVehicleBrain.generated.h"

UCLASS(Blueprintable)
class INSIM_API USimpleVehicleBrain : public UBrain
{
    GENERATED_BODY()

    USimpleVehicleBrain(const FObjectInitializer& ObjectInitializer);

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

    // SimMode owner actor support dynamic destroy and create owner
    UPROPERTY()
    class AActor* SimModeOwner;
    // actual robot agent for training
    UPROPERTY()
    class AActor* Owner;

    UPROPERTY()
    class AActor* Goal = nullptr;

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