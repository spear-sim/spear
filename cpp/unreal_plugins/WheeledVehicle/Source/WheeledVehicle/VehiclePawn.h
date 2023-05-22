//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Eigen/Dense>

#include <CoreMinimal.h>
#include <GameFramework/Pawn.h>

#include "VehiclePawn.generated.h"

class UBoxComponent;
class UCameraComponent;
class UVehicleMovementComponent;
class USkeletalMeshComponent;


UCLASS()
class WHEELEDVEHICLE_API AVehiclePawn : public APawn
{
    GENERATED_BODY()
public:
    AVehiclePawn(const FObjectInitializer& object_initializer);
    ~AVehiclePawn();

    // APawn interface
    void SetupPlayerInputComponent(UInputComponent* input_component) override;
    
    // WheelVehicleAgent interface
    void setDriveTorques(const Eigen::Vector4d& drive_torques);
    void setBrakeTorques(const Eigen::Vector4d& brake_torques); // Torque applied to the brakes, expressed in [N.m]. The applied torque persists until the next call to SetBrakeTorques.
    Eigen::Vector4d getWheelRotationSpeeds() const;
    void resetVehicle();

    USkeletalMeshComponent* skeletal_mesh_component_ = nullptr;
    UVehicleMovementComponent* vehicle_movement_component_ = nullptr;
    UCameraComponent* camera_component_ = nullptr;
    UBoxComponent* imu_component_ = nullptr;
    UBoxComponent* sonar_component_ = nullptr;

private:
    // Function that applies wheel torque on a vehicle to generate linear
    // forward/backward motions. This function is intended to handle keyboard
    // input.
    void moveForward(float forward);

    // Function that applies a differential wheel torque on a vehicle to
    // generate angular yaw motions. This function is intended to handle
    // keyboard input.
    void moveRight(float right);
};