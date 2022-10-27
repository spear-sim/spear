#pragma once

PRAGMA_DISABLE_DEPRECATION_WARNINGS

#include <Eigen/Dense>

#include <CoreMinimal.h>
#include <Components/SceneCaptureComponent2D.h>
#include <Engine/CollisionProfile.h>
#include <Engine/SceneCapture2D.h>
#include <GameFramework/Pawn.h>
#include <PhysicsPublic.h>
#include <PhysXIncludes.h>
#include <PhysXPublic.h>
#include <PhysXVehicleManager.h>
#include <SimpleWheeledVehicleMovementComponent.h>

#include "Config.h"

#include "OpenBotPawn.generated.h"

// This class is inspired by the WheeledVehicle class, define in:
// UnrealEngine-4.26.2-release/Engine/Plugins/Runtime/PhysXVehicles/Source/PhysXVehicles/Public/WheeledVehicle.h
UCLASS(meta = (ShortTooltip = "OpenBot Pawn"))
class OPENBOT_API AOpenBotPawn : public APawn
{
    GENERATED_BODY()
public:
    // The main skeletal mesh associated with this Vehicle
    class USkeletalMeshComponent* mesh_component_;

    // Vehicle simulation component
    class USimpleWheeledVehicleMovementComponent* vehicle_movement_component_;

    // Camera component that will be our viewpoint
    class UCameraComponent* camera_component_;

    // Construct a new AOpenBotPawn object
    AOpenBotPawn(const FObjectInitializer& object_initializer);

    //Destroy the AOpenBotPawn object
    virtual ~AOpenBotPawn();

    //Pawn interface
    virtual void SetupPlayerInputComponent(UInputComponent* input_component) override;

    virtual void Tick(float delta_time) override;

    // Collision callback
    virtual void NotifyHit(class UPrimitiveComponent* hit_component,
                           class AActor* other_actor,
                           class UPrimitiveComponent* other_component,
                           bool bself_moved,
                           FVector hit_location,
                           FVector hit_normal,
                           FVector normal_impulse,
                           const FHitResult& hit) override;

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
    Eigen::Vector2f GetControlState();

    void reset(FVector location, FQuat rotation);

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
    void setDriveTorques(float DeltaTime);

    //@brief Clamp a vector between two values.
    static Eigen::Vector4f Clamp(Eigen::Vector4f v, Eigen::Vector4f vMin, Eigen::Vector4f vMax);

    //Rev per minute to rad/s
    static Eigen::VectorXf RPMToRadSec(Eigen::VectorXf RPM);

    //rad/s to rev per minute
    static Eigen::VectorXf RadSecToRPM(Eigen::VectorXf Omega);

    Eigen::Vector4f duty_cycle_;                 // The duty cycle of the PWM signal applied to the motors in [%]
    Eigen::Vector2f action_vec_;                 // Vector of action observations to be fed back to the RL algorithm.

};

PRAGMA_ENABLE_DEPRECATION_WARNINGS
