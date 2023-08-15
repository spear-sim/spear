//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <CoreMinimal.h> // GENERATED_BODY, UCLASS
#include <WheeledVehiclePawn.h>

#include "VehiclePawn.generated.h"

class FObjectInitializer;
class UBoxComponent;
class UCameraComponent;

class UVehicleMovementComponent;

UCLASS()
class VEHICLE_API AVehiclePawn : public AWheeledVehiclePawn
{
    GENERATED_BODY()
public:
    AVehiclePawn(const FObjectInitializer& object_initializer);
    ~AVehiclePawn();

    UVehicleMovementComponent* vehicle_movement_component_ = nullptr;
    UCameraComponent* camera_component_ = nullptr;
    UBoxComponent* imu_component_ = nullptr;
};
