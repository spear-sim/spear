//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <vector>

#include <CoreMinimal.h>
#include <GameFramework/Pawn.h>

#include "VehiclePawn.generated.h"

class UBoxComponent;
class UCameraComponent;
class UVehicleMovementComponent;
class USkeletalMeshComponent;

UCLASS()
class VEHICLE_API AVehiclePawn : public APawn
{
    GENERATED_BODY()
public:
    AVehiclePawn(const FObjectInitializer& object_initializer);
    ~AVehiclePawn();
    
    // VehicleAgent interface
    void setDriveTorques(const std::vector<double>& drive_torques); // Torque expressed in [N.m]. The applied torque persists until the next call to SetBrakeTorques.
    void setBrakeTorques(const std::vector<double>& brake_torques); // Torque expressed in [N.m]. The applied torque persists until the next call to SetBrakeTorques.
    std::vector<double> getWheelRotationSpeeds() const;
    void resetVehicle();

    USkeletalMeshComponent* skeletal_mesh_component_ = nullptr;
    UVehicleMovementComponent* vehicle_movement_component_ = nullptr;
    UCameraComponent* camera_component_ = nullptr;
    UBoxComponent* imu_component_ = nullptr;
};
