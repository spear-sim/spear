//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <CoreMinimal.h>
#include <ChaosVehicleWheel.h>

#include "VehicleWheel.generated.h"

UCLASS()
class UVehicleWheel : public UChaosVehicleWheel
{
    GENERATED_BODY()

public:

    UVehicleWheel();

    ~UVehicleWheel();
};
