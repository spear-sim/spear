#pragma once

#include <Eigen/Dense>

#include <CoreMinimal.h>
#include <GameFramework/Pawn.h>

#include "OpenBotPawn.generated.h"

class UBoxComponent;
class UCameraComponent;
class USimpleWheeledVehicleMovementComponent;
class USkeletalMeshComponent;

// This class is inspired by the WheeledVehicle class, defined in:
// UnrealEngine-4.26.2-release/Engine/Plugins/Runtime/PhysXVehicles/Source/PhysXVehicles/Public/WheeledVehicle.h
UCLASS()
class OPENBOT_API AOpenBotPawn : public APawn
{
    GENERATED_BODY()
public:
    AOpenBotPawn(const FObjectInitializer& object_initializer);

    // APawn interface
    virtual void SetupPlayerInputComponent(UInputComponent* input_component) override;
    virtual void Tick(float delta_time) override;

    // Function that applies wheel torque on a vehicle to generate linear
    // forward/backward motions. This function is intended to handle keyboard
    // input.
    void moveForward(float forward);

    // Function that applies a differential wheel torque on a vehicle to
    // generate angular yaw motions. This function is intended to handle
    // keyboard input.
    void moveRight(float right);

    // Function that applies a given command to the vehicle (e.g., as a result
    // of a remote call from a Python client). The input vector will be clamped
    // to be between -1.0 and 1.0.
    void setDutyCycleAndClamp(const Eigen::Vector4f& duty_cycle);

    // Provides access to the current command sent to the OpenBot (e.g., to return
    // to a Python client). This function is required because the command to be
    // executed by the OpenBot might come from keyboard user input.
    Eigen::Vector4f getDutyCycle() const;

    // Apply high braking torque to make sure the wheels don't move
    void activateBrakes();

    // Releases brakes
    void deactivateBrakes();

    // Reset the physical state of the wheels
    void resetPhysicsState();

    USkeletalMeshComponent* skeletal_mesh_component_ = nullptr;
    USimpleWheeledVehicleMovementComponent* vehicle_movement_component_ = nullptr;
    UCameraComponent* camera_component_ = nullptr;
    UBoxComponent* imu_component_ = nullptr;
    UBoxComponent* sonar_component_ = nullptr;

private:

    //
    // Computes the motor torque corresponding to a given command.
    // Clarification about the DC motor model used in this simulation.
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

    // Mechanical equation: J dw/dt + Bw = Torque_mot - Torque_load (2)

    // where:    J is the rotor's moment of inertia (here neglected)
    //           B is a viscous friction coefficient describing internal motor
    //           friction Torque_mot = Kt * i

    // On the OpenBot, the normalized control input corresponds to the PWM
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

    void setDriveTorques(float delta_time);

    // Rev per minute to rad/s
    static Eigen::VectorXf rpmToRadSec(Eigen::VectorXf rpm);

    // Rad/s to rev per minute
    static Eigen::VectorXf radSecToRpm(Eigen::VectorXf omega);

    // The duty cycle of the PWM signal applied to the motors in [%]
    Eigen::Vector4f duty_cycle_;
};
