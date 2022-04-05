#pragma once

#include <Brain.h>
#include <CoreMinimal.h>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

#include <Components/SceneCaptureComponent2D.h>
#include <Kismet/KismetMathLibrary.h>
#include <SimpleVehicle/SimModeSimpleVehicle.h>
#include <SimpleVehicle/SimpleVehiclePawn.h>
#include <UObject/ConstructorHelpers.h>

#include <Engine/CollisionProfile.h>
#include <Engine/EngineTypes.h>
#include <Engine/TextureRenderTarget2D.h>
#include <EngineUtils.h>
#include <Math/Rotator.h>

#include "Action.h"
#include "ActionSpec.h"
#include "Config.h"
#include "Observation.h"
#include "ObservationSpec.h"
#include "ServerManager.h"
#include "Types.h"

#include "SimpleVehicleBrain.generated.h"

UCLASS(Blueprintable)
class INTERIORSIMBRIDGE_API USimpleVehicleBrain : public UBrain
{
    GENERATED_BODY()

    USimpleVehicleBrain(const FObjectInitializer& objectInitializer);

public:
    UFUNCTION()
    void OnActorHit(AActor* selfActor,
                    AActor* otherActor,
                    FVector normalImpulse,
                    const FHitResult& hitFlag);

    // UnrealRL overrides.
    void Init() override;
    bool IsAgentReady() override;
    void OnEpisodeBegin() override;
    void SetAction(const std::vector<unrealrl::Action>& actionVector) override;
    void GetObservation(std::vector<unrealrl::Observation>&) override;

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

    APIPCamera* mainCamera_ = nullptr;
    USceneCaptureComponent2D* captureComponent2D_ = nullptr;

    // Actual robot agent used for training
    class ASimpleVehiclePawn* ownerPawn = nullptr;

    // Goal towards which the robot agent should move
    class AActor* goalActor = nullptr;

    UHitInfo hitInfo_ = UHitInfo::NoHit;
};
