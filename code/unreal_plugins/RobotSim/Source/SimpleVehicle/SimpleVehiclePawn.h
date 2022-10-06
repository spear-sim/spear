#pragma once

#include "CoreMinimal.h"
#include "Engine/CollisionProfile.h"
#include "GameFramework/Pawn.h"
#include "NavigationSystem.h"

#include "PIPCamera.h"
#include "PawnEvents.h"
#include "PhysXIncludes.h"
#include "PhysXPublic.h"
#include "PhysXVehicleManager.h"
#include "PhysicsPublic.h"
#include "RobotBase.h"
#include "RobotBlueprintLib.h"
#include "SimpleWheeledVehicleMovementComponent.h"
#include "common_utils/Common.hpp"
#include "common_utils/RobotSimSettings.hpp"
#include "common_utils/UniqueValueMap.hpp"
#include "common_utils/Utils.hpp"

#include "SimpleVehiclePawn.generated.h"

/**
 * @brief This class is inspired by the WheeledVehicle class, define in:
 * UnrealEngine-4.26.2-release/Engine/Plugins/Runtime/PhysXVehicles/Source/PhysXVehicles/Public/WheeledVehicle.h
 *
 */
UCLASS(Blueprintable, meta = (ShortTooltip = "Simple Vehicle Pawn"))
class ROBOTSIM_API ASimpleVehiclePawn : public APawn, public RobotBase
{
    GENERATED_BODY()

    //  The main skeletal mesh associated with this Vehicle
    UPROPERTY(Category = Vehicle, VisibleDefaultsOnly, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
    class USkeletalMeshComponent* Mesh;

    // Vehicle simulation component
    UPROPERTY(Category = Vehicle, VisibleDefaultsOnly, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
    class UWheeledVehicleMovementComponent* VehicleMovement;

public:
    /**
     * @brief Construct a new ASimpleVehiclePawn object
     *
     * @param ObjectInitializer
     */
    ASimpleVehiclePawn(const FObjectInitializer& ObjectInitializer);

    /**
     * @brief Destroy the ASimpleVehiclePawn object
     *
     */
    virtual ~ASimpleVehiclePawn();

    /**
     * @brief Pawn interface
     *
     * @param InputComponent
     */
    virtual void SetupPlayerInputComponent(UInputComponent* InputComponent) override;

    /**
     * @brief Called every frame
     *
     * @param DeltaTime
     */
    virtual void Tick(float DeltaTime) override;

    /**
     * @brief Collision callback
     *
     * @param myComp
     * @param other
     * @param otherComp
     * @param bSelfMoved
     * @param hitLocation
     * @param hitNormal
     * @param normalImpulse
     * @param hit
     */
    virtual void NotifyHit(class UPrimitiveComponent* myComp,
                           class AActor* other,
                           class UPrimitiveComponent* otherComp,
                           bool bSelfMoved,
                           FVector hitLocation,
                           FVector hitNormal,
                           FVector normalImpulse,
                           const FHitResult& hit) override;

    /**
     * @brief User-defined callback function
     *
     */
    UFUNCTION()
    void OnComponentCollision(UPrimitiveComponent* HitComponent,
                              AActor* OtherActor,
                              UPrimitiveComponent* OtherComp,
                              FVector NormalImpulse,
                              const FHitResult& Hit);

    /**
     * @brief Get the Component object
     *
     * @param componentName
     * @return USceneComponent*
     */
    virtual USceneComponent* GetComponent(FString componentName) override;

    /**
     * @brief Get the Component Reference Transform object
     *
     * @param componentName
     * @param translation
     * @param rotation
     */
    virtual void GetComponentReferenceTransform(FString componentName,
                                                FVector& translation,
                                                FRotator& rotation) override;

    /**
     * @brief Get the Pawn object
     *
     * @return APawn*
     */
    virtual APawn* GetPawn() override
    {
        return this;
    }

    /**
     * @brief
     *
     * @return true
     * @return false
     */
    virtual bool PawnUsesNedCoords() override
    {
        return false;
    }

    /**
     * @brief
     *
     * @param position
     * @param orientation
     * @param teleport
     */
    virtual void TeleportToLocation(FVector position,
                                    FQuat orientation,
                                    bool teleport) override;

    /**
     * @brief Add keyboard/mouse control interface
     *
     */
    void SetupInputBindings();

    /**
     * @brief Keyboard callback allowing to apply wheel torque on a vehicle to
     * generate linear forward/backward motions. This command is meant to be
     * bound to a keyboard input. It will be executed at every simulation
     * update.
     *
     * @param Forward denotes the percentage of forward command.
     */
    void MoveForward(float Forward);

    /**
     * @brief Keyboard callback allowing to apply differential wheel torque on a
     * vehicle to generate angular yaw motions. This command is meant to be
     * bound to a keyboard input. It will be executed at every simulation
     * update.
     *
     * @param Right denotes the percentage of yaw rotation command.
     */
    void MoveRight(float Right);

    /**
     * @brief Callback function allowing to apply a given command to the
     * vehicle as a result of a remote call from a pyhton client.
     * It will be executed at every simulation update before the Tick function.
     *
     * @param leftCtrl denotes the percentage of input voltage to be applied to
     * the left motors. It should therefore be on a [-1.0, 1.0] scale.
     * @param rightCtrl denotes the percentage of input voltage to be applied to
     * the right motors. It should therefore be on a [-1.0, 1.0] scale.
     */
    void MoveLeftRight(float leftCtrl, float rightCtrl);

    /**
     * @brief Provides acces to the action sent to the agent at the current
     * simulation epoch. This function is required as the action to be executed
     * by the agent may either come from the python client (RL scenario) or from
     * the keyboard (imitation learning scenario).
     *
     * @return ActionVec
     */
    Eigen::Vector2f GetControlState();

    /**
     * @brief Get the Pawn Events object
     * 
     * @return PawnEvents* 
     */
    PawnEvents* GetPawnEvents();

    /**
     * @brief Name of the MeshComponent. Use this name if you want to prevent
     * creation of the component (with
     * ObjectInitializer.DoNotCreateDefaultSubobject).
     *
     */
    static FName VehicleMeshComponentName;

    /**
     * @brief Name of the VehicleMovement. Use this name if you want to use a
     * different class (with ObjectInitializer.SetDefaultSubobjectClass).
     *
     */
    static FName VehicleMovementComponentName;

    /**
     * @brief Returns Mesh subobject
     *
     * @return USkeletalMeshComponent*
     */
    class USkeletalMeshComponent* GetMesh() const
    {
        return Mesh;
    }

    /**
     * @brief Returns VehicleMovement subobject
     *
     * @return UWheeledVehicleMovementComponent*
     */
    class UWheeledVehicleMovementComponent* GetVehicleMovementComponent() const
    {
        return VehicleMovement;
    }
    
    /**
     * @brief Set the Wheels Friction Scale
     * 
     * @param WheelsFrictionScale 
     */
    void SetWheelsFrictionScale(TArray<float> &WheelsFrictionScale);

protected:
    /**
     * @brief Called when the game starts or when spawned
     *
     */
    virtual void BeginPlay() override;

private:
    /**
     * @brief Computes the motor torque corresponding to a given command.
     *
     * Clarification about the DC motor model used in this simulation:
     *
     * Electrical equation: Ri + L di/dt + e = V_pwm  (1)
     *
     * where:    V_pwm is the External voltage applied to the motor [V]
     *           i is the motor current [A]
     *           L is the stator winding inductance [H]
     *           R is the stator winding resistance [Ohms]
     *           e = w / Kv = Kt * w is the counter electromotive force [V]
     *           w is the rotor's angular velocity [rad/s]
     *           Kv > 0 is the motor velocity constant [rad/s/V]
     *           Kt ~ 1/Kv is the motor torque constant [N.m/A]
     *
     * Mechanical equation: J dw/dt + Bw = Torque_mot - Torque_load (2)
     *
     * where:    J is the rotor's moment of inertia (here neglected)
     *           B is a viscous friction coefficient describing internal motor
     * friction Torque_mot = Kt * i
     *
     * On the OpenBot, the normalized control input corresponds to the PWM
     * signal duty cycle. In other words, this is the fraction of input battery
     * voltage applied to the motor through V_pwm in equation (1). Since the
     * motor is very small, we neglect the influence of the rotor inertia: J ~ 0
     * in equation (2). The inductance is also neglected (mostly because the
     * DeltaTime in the simulation is not steady...). Accounting for the motor +
     * gearbox friction model would introduce unnecessary complexity. On the
     * real system, the influence of motor+gearbox internal friction is only
     * visible when the robot is static. As a matter of fact, very small PWM
     * duty cycles won't get the vehicle to move. This can hence be reasonably
     * approximated as a command dead-zone when the vehicle is static... The
     * important factor is here the counter electromotove force (e in equation
     * (1)), proportionnal to the rotor angular velocity and which will oppose
     * the input voltage, thereby limiting the torque. More than internal
     * friction, this quantity is what makes the rotation speed of a DC motor
     * saturate...
     */
    void ComputeMotorTorques(float DeltaTime);

    /**
     * @brief Clamp a vector between two values.
     *
     * @param v
     * @param vMin
     * @param vMax
     * @return Eigen::Vector4f The clamped value.
     */
    inline Eigen::Vector4f Clamp(Eigen::Vector4f v, Eigen::Vector4f vMin, Eigen::Vector4f vMax)
    {
        Eigen::Vector4f v_clamped;
        v_clamped = v;
        for (unsigned int i = 0; i < v.size(); i++)
        {
            if (v(i) > vMax(i))
            {
                v_clamped(i) = vMax(i);
            }
            if (v(i) < vMin(i))
            {
                v_clamped(i) = vMin(i);
            }
        }
        return v_clamped;
    }

    /**
     * @brief Rev per minute to rad/s
     *
     * @param RPM
     * @return Eigen::VectorXf
     */
    inline Eigen::VectorXf RPMToRadSec(Eigen::VectorXf RPM)
    {
        return RPM * PI / 30.f;
    }

    /**
     * @brief rad/s to rev per minute
     *
     * @param Omega
     * @return Eigen::VectorXf
     */
    inline Eigen::VectorXf RadSecToRPM(Eigen::VectorXf Omega)
    {
        return Omega * 30.f / PI;
    }

    PawnEvents pawnEvents_;
    int count_ = 0;
    USimpleWheeledVehicleMovementComponent* vehiclePawn_;
    Eigen::Vector4f wheelVelocity_;             // The ground truth velocity of the robot wheels in [RPM]
    Eigen::Vector4f motorVelocity_;             // The ground truth velocity of the motors in [rad/s]
    Eigen::Vector4f counterElectromotiveForce_; // Vector of counter electromotive forces of each motor [V]
    Eigen::Vector4f motorTorque_;               // The torque generated by the motors in [N.m]
    Eigen::Vector4f wheelTorque_;               // The torque applied to the robot wheels in [N.m]
    Eigen::Vector4f dutyCycle_;                 // The duty cycle of the PWM signal applied to the motors in [%]
    Eigen::Vector4f motorWindingCurrent_;       // Electrical current circulating into the DC motor windings in [A]
    Eigen::Vector2f actionVec_;                 // Vector of action observations to be fed back to the RL algorithm.

    // TODO: these values should be set in a proper .yaml parameter file.
    float motorTorqueConstant_ = 0.f;   // Motor torque constant in [N.m/A]
    float motorVelocityConstant_ = 0.f; // Motor torque constant in [rad/s/V]
    float gearRatio_ = 50.f;            // Gear ratio of the OpenBot motors.
    float motorTorqueMax_ = 0.1177f;    // Maximum ammount of torque an OpenBot motor can generate [N.m].
    // Openbot motor torque: 1200 gf.cm (gram force centimeter) = 0.1177 N.m
    // https://www.conrad.de/de/p/joy-it-com-motor01-getriebemotor-gelb-schwarz-passend-fuer-einplatinen-computer-arduino-banana-pi-cubieboard-raspbe-1573543.html)
    // Gear ratio: 50 => max. wheel torque = 5.88399 N.m

    float actionScale_ = 255.f; // Speed multiplier defined in the OpenBot to map a [-1,1] action to a suitable command to
                                // be processed by the low-level microcontroller. For more details, feel free
                                // to check the "speedMultiplier" command in the OpenBot code:
                                // https://github.com/isl-org/OpenBot/blob/d4362e688435155f6c20dfdb756e55556fc12cc8/android/app/src/main/java/org/openbot/vehicle/Vehicle.java#L375

    float batteryVoltage_ = 12.f;       // Expressed in [V]
    float electricalResistance_ = 4.2f; // Electrical resistance of the DC motor windings in [Ohms]
    float electricalInductance_ = 0.f;  // Electrical inductance of the DC motor windings in [Henry]
    float controlDeadZone_ = 5.0;       // Below this command threshold, the torque is set to zero if the motor velocity is "small enough"
};
