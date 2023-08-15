//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <vector>

#include <CoreMinimal.h> // GENERATED_BODY, UCLASS
#include <ChaosWheeledVehicleMovementComponent.h>

#include "VehicleMovementComponent.generated.h"

UCLASS()
class VEHICLE_API UVehicleMovementComponent : public UChaosWheeledVehicleMovementComponent
{
    GENERATED_BODY()
public:
    UVehicleMovementComponent();
    ~UVehicleMovementComponent();

    // Provides access to wheel rotation speeds in rad/s
    std::vector<double> getWheelRotationSpeeds() const;
};
