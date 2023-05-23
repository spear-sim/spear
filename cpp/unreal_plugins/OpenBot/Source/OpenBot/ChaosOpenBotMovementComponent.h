//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Eigen/Dense>

#include <CoreMinimal.h>
#include <ChaosWheeledVehicleMovementComponent.h>

#include "ChaosOpenBotMovementComponent.generated.h"

UCLASS()
class OPENBOT_API UChaosOpenBotMovementComponent : public UChaosWheeledVehicleMovementComponent
{
    GENERATED_BODY()
public:
    UChaosOpenBotMovementComponent();
    ~UChaosOpenBotMovementComponent();

    UFUNCTION(BlueprintCallable)
    void printDebugValues();
};