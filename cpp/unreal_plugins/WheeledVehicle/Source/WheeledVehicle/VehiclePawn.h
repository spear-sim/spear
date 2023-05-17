//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Eigen/Dense>

#include <CoreMinimal.h>
#include <GameFramework/Pawn.h>

#include "VehiclePawn.generated.h"

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
    void BeginPlay() override;
    void Tick(float delta_time) override;

    USkeletalMeshComponent* skeletal_mesh_component_ = nullptr;
    UVehicleMovementComponent* vehicle_movement_component_ = nullptr;
    UCameraComponent* camera_component_ = nullptr;

    // The duty cycle of the PWM signal applied to the motors in [%]
    Eigen::Vector4f duty_cycle_;

private:
    // Function that applies wheel torque on a vehicle to generate linear
    // forward/backward motions. This function is intended to handle keyboard
    // input.
    void moveForward(float forward);

    // Function that applies a differential wheel torque on a vehicle to
    // generate angular yaw motions. This function is intended to handle
    // keyboard input.
    void moveRight(float right);

    //
    // Compute motor torques corresponding to the duty cycle specified by
    // setDutyCycle(...) according to the following DC motor model.
    //
    // Electrical equation: Ri + L di/dt + e = V_pwm  (1)
    //
    // where:    V_pwm is the External voltage applied to the motor [V]
    //           i is the motor current [A]
    //           L is the stator winding inductance [H]
    //           R is the stator winding resistance [Ohms]
    //           e = w / Kv = Kt * w is the counter electromotive force [V]
    //           w is the rotor's angular velocity [rad/s]
    //           Kv > 0 is the motor velocity constant [rad/s/V]
    //           Kt ~ 1/Kv is the motor torque constant [N.m/A]
    //
    // Mechanical equation: J dw/dt + Bw = Torque_mot - Torque_load (2)
    //
    // where:    J is the rotor's moment of inertia (here neglected)
    //           B is a viscous friction coefficient describing internal motor
    //           friction Torque_mot = Kt * i
    //
    // On the WheeledVehicle, the normalized control input corresponds to the PWM
    // signal duty cycle. In other words, this is the fraction of input battery
    // voltage applied to the motor through V_pwm in equation (1). Since the
    // motor is very small, we neglect the influence of the rotor inertia: J ~ 0
    // in equation (2). The inductance is also neglected (mostly because the
    // DeltaTime in the simulation is not steady). Accounting for the motor +
    // gearbox friction model would introduce unnecessary complexity. On the
    // real system, the influence of motor+gearbox internal friction is only
    // visible when the robot is static. As a matter of fact, very small PWM
    // duty cycles won't get the vehicle to move. This can hence be reasonably
    // approximated as a command dead-zone when the vehicle is static. The
    // important factor is here the counter electromotove force (e in equation
    // (1)), proportionnal to the rotor angular velocity and which will oppose
    // the input voltage, thereby limiting the torque. More than internal
    // friction, this quantity is what makes the rotation speed of a DC motor
    // saturate.
    //

    void setDriveTorquesFromDutyCycle();

    // Rev per minute to rad/s
    static Eigen::VectorXf rpmToRadSec(Eigen::VectorXf rpm);

    // Rad/s to rev per minute
    static Eigen::VectorXf radSecToRpm(Eigen::VectorXf omega);
};