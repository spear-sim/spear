//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Eigen/Dense>

#include <CoreMinimal.h>
#include <ChaosWheeledVehicleMovementComponent.h>

#include "VehicleMovementComponent.generated.h"

UCLASS()
class WHEELEDVEHICLE_API UVehicleMovementComponent : public UChaosWheeledVehicleMovementComponent
{
    GENERATED_BODY()

public:

    UVehicleMovementComponent();
    
    ~UVehicleMovementComponent();

    // Provides access to the wheels rotation speed in rad/s
    Eigen::Vector4d getWheelRotationSpeeds() const;
};
