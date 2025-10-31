//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <chrono>

#include <GameFramework/SpectatorPawn.h>
#include <UObject/ObjectMacros.h> // GENERATED_BODY, UCLASS, UPROPERTY

#include "SpCore/Boost.h"

#include "SpSpectatorPawn.generated.h"

class USpectatorPawnMovement;
class USpStableNameComponent;

UCLASS()
class ASpSpectatorPawn : public ASpectatorPawn
{
    GENERATED_BODY()
public:
    ASpSpectatorPawn();
    ~ASpSpectatorPawn() override;

    // ASpectatorPawn interface
    void BeginPlay() override;
    void Tick(float delta_time) override;

private:
    UPROPERTY(VisibleAnywhere, Category="SPEAR")
    USpStableNameComponent* SpStableNameComponent = nullptr;

    UPROPERTY(VisibleAnywhere, Category="SPEAR")
    USpectatorPawnMovement* SpectatorPawnMovement = nullptr;

    boost::circular_buffer<double> previous_time_deltas_;
    std::chrono::time_point<std::chrono::high_resolution_clock> previous_time_point_;
    bool previous_is_benchmarking_ = false;
    float previous_max_speed_ = -1.0f;
};
