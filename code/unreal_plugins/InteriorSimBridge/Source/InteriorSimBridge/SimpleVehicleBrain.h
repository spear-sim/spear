#pragma once

#include <CoreMinimal.h>
#include <string>
#include <cstring>
#include <utility>
#include <vector>
#include <Brain.h>

#include <Kismet/KismetMathLibrary.h>
#include <UObject/ConstructorHelpers.h>
#include <SimpleVehicle/SimModeSimpleVehicle.h>
#include <SimpleVehicle/SimpleVehiclePawn.h>
#include <Components/SceneCaptureComponent2D.h>

#include <Engine/TextureRenderTarget2D.h>
#include <EngineUtils.h>
#include <Engine/CollisionProfile.h>
#include <Engine/EngineTypes.h>
#include <Math/Rotator.h>

#include "Config.h"
#include "Action.h"
#include "ActionSpec.h"
#include "Observation.h"
#include "ObservationSpec.h"
#include "ServerManager.h"
#include "Types.h"
#include "SimpleVehicleBrain.generated.h"


UCLASS(Blueprintable)
class INTERIORSIMBRIDGE_API USimpleVehicleBrain : public UBrain
{
    GENERATED_BODY()

    USimpleVehicleBrain(const FObjectInitializer &objectInitializer);

public:
    UFUNCTION()
    void OnActorHit(AActor *selfActor,
                    AActor *otherActor,
                    FVector normalImpulse,
                    const FHitResult &hitFlag);

    // UnrealRL overrides.
    void Init() override;
    bool IsAgentReady() override;
    void OnEpisodeBegin() override;
    void SetAction(const std::vector<unrealrl::Action> &actionVector) override;
    void GetObservation(std::vector<unrealrl::Observation> &) override;

    // actual robot agent for training
    UPROPERTY()
    class ASimpleVehiclePawn *ownerPawn = nullptr;

    UPROPERTY()
    class AActor *goalActor = nullptr;

private:

    /**
     * @brief Enumeates the different collision cases
     * 
     */
    enum class UHitInfo
    {
        NoHit,
        Goal,
        Edge
    };

    const unsigned long height_ = 120;
    const unsigned long width_ = 160;
    APIPCamera* mainCamera_ = nullptr;
    USceneCaptureComponent2D *captureComponent2D_ = nullptr; 

    UHitInfo hitInfo_ = UHitInfo::NoHit;
};

