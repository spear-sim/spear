#pragma once

#include <Eigen/Dense>

#include <CoreMinimal.h>
#include <GameFramework/Pawn.h>

#include "OpenBotPawn.generated.h"

class UCameraComponent;
class USimpleWheeledVehicleMovementComponent;
class USkeletalMeshComponent;

// This class is inspired by the WheeledVehicle class, define in:
// UnrealEngine-4.26.2-release/Engine/Plugins/Runtime/PhysXVehicles/Source/PhysXVehicles/Public/WheeledVehicle.h
UCLASS(meta = (ShortTooltip = "OpenBot Pawn"))
class OPENBOT_API AOpenBotPawn : public APawn
{
    GENERATED_BODY()
public:
    // Construct a new AOpenBotPawn object
    AOpenBotPawn(const FObjectInitializer& object_initializer);

    // APawn interface
    virtual void SetupPlayerInputComponent(UInputComponent* input_component) override;
    virtual void Tick(float delta_time) override;

    // Keyboard callback allowing to apply wheel torque on a vehicle to
    // generate linear forward/backward motions. This command is meant to be
    // bound to a keyboard input. It will be executed at every simulation
    // update.
    void moveForward(float forward);

    // Keyboard callback allowing to apply differential wheel torque on a
    // vehicle to generate angular yaw motions. This command is meant to be
    // bound to a keyboard input. It will be executed at every simulation
    // update.
    void moveRight(float right);

    // Callback function allowing to apply a given command to the
    // vehicle as a result of a remote call from a pyhton client.
    // It will be executed at every simulation update before the Tick function.
    // `leftCtrl` denotes the percentage of input voltage to be applied to
    // the left motors. It should therefore be on a [-1.0, 1.0] scale.
    // `rightCtrl` denotes the percentage of input voltage to be applied to
    // the right motors. It should therefore be on a [-1.0, 1.0] scale.
    void setDutyCycleAndClamp(Eigen::Vector4f duty_cycle);

    // Provides access to the action sent to the agent at the current
    // simulation epoch. This function is required as the action to be executed
    // by the agent may either come from the python client (RL scenario) or from
    // the keyboard (imitation learning scenario//
    Eigen::Vector4f getDutyCycle();

    // reset the physical state of the wheels
    void resetPhysicsState();

    // The main skeletal mesh associated with this Vehicle
    USkeletalMeshComponent* skeletal_mesh_component_ = nullptr;

    // Vehicle simulation component
    USimpleWheeledVehicleMovementComponent* vehicle_movement_component_ = nullptr;

    // Camera component that will be our viewpoint
    UCameraComponent* camera_component_ = nullptr;

private:
    // Computes the motor torque corresponding to a given command.
    // Clarification about the DC motor model used in this simulation:
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
    // friction Torque_mot = Kt * i

    // On the OpenBot, the normalized control input corresponds to the PWM
    // signal duty cycle. In other words, this is the fraction of input battery
    // voltage applied to the motor through V_pwm in equation (1). Since the
    // motor is very small, we neglect the influence of the rotor inertia: J ~ 0
    // in equation (2). The inductance is also neglected (mostly because the
    // DeltaTime in the simulation is not steady...). Accounting for the motor +
    // gearbox friction model would introduce unnecessary complexity. On the
    // real system, the influence of motor+gearbox internal friction is only
    // visible when the robot is static. As a matter of fact, very small PWM
    // duty cycles won't get the vehicle to move. This can hence be reasonably
    // approximated as a command dead-zone when the vehicle is static... The
    // important factor is here the counter electromotove force (e in equation
    // (1)), proportionnal to the rotor angular velocity and which will oppose
    // the input voltage, thereby limiting the torque. More than internal
    // friction, this quantity is what makes the rotation speed of a DC motor
    // saturate...
    void setDriveTorques(float delta_time);

    // clamp a vector between two values.
    static Eigen::Vector4f clamp(Eigen::Vector4f v, Eigen::Vector4f v_min, Eigen::Vector4f v_max);

    // Rev per minute to rad/s
    static Eigen::VectorXf rpmToRadSec(Eigen::VectorXf rpm);

    // rad/s to rev per minute
    static Eigen::VectorXf radSecToRpm(Eigen::VectorXf omega);

    Eigen::Vector4f duty_cycle_; // The duty cycle of the PWM signal applied to the motors in [%]
};
