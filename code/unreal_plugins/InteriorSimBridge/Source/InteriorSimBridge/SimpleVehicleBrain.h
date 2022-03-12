#pragma once

#include <CoreMinimal.h>
#include <vector>
#include <Brain.h>
#include "SimpleVehicleBrain.generated.h"


UCLASS(Blueprintable)
class INTERIORSIMBRIDGE_API USimpleVehicleBrain : public UBrain
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
    bool IsAgentReady() override;
    void OnEpisodeBegin() override;
    void SetAction(const std::vector<unrealrl::Action>& Action) override;
    void GetObservation(std::vector<unrealrl::Observation>&) override;

    // actual robot agent for training
    UPROPERTY()
    class ASimpleVehiclePawn* Owner = nullptr;

    UPROPERTY()
    class AActor* Goal = nullptr;

private:
    enum class UHitInfo
    {
        NoHit,
        Goal,
        Edge
    };

    std::vector<float> ActionVec;
    
#if USE_IMAGE_OBSERVATIONS
    int Height = 512;
    int Width = 512;
    
    USceneCaptureComponent2D* captureComponent2D;
    UTextureRenderTarget2D* renderTarget2D; // Output render target of the scene capture that can be read in materals.
#endif

    UHitInfo HitInfo = UHitInfo::NoHit;
};
