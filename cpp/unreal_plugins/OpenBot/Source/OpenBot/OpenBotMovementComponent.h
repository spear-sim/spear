//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Eigen/Dense>

#include <CoreMinimal.h>
#include <ChaosWheeledVehicleMovementComponent.h>

#include "OpenBotMovementComponent.generated.h"

UCLASS()
class OPENBOT_API UOpenBotMovementComponent : public UChaosWheeledVehicleMovementComponent
{
    GENERATED_BODY()
public:
    UOpenBotMovementComponent();
    ~UOpenBotMovementComponent();

    // Provides access to the wheels rotation speed in rad/s
    Eigen::Vector4f getWheelRotationSpeeds() const;
};