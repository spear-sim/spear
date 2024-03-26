//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <vector>

#include <ChaosWheeledVehicleMovementComponent.h>
#include <UObject/ObjectMacros.h> // GENERATED_BODY, UCLASS

#include "VehicleMovementComponent.generated.h"

class AVehiclePawn;

UCLASS()
class VEHICLE_API UVehicleMovementComponent : public UChaosWheeledVehicleMovementComponent
{
    GENERATED_BODY()
    friend class AVehiclePawn;
public:
    UVehicleMovementComponent();
    ~UVehicleMovementComponent();
};
