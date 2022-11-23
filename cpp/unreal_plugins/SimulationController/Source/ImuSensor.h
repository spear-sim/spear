#pragma once

#include <array>
#include <random>

#include <Engine/EngineBaseTypes.h>
#include <Math/Vector.h>

class AActor;
class UPrimitiveComponent;
class UTickEvent;

class ImuSensor 
{
public:

    // We have found that not all UPrimitiveComponent objects can be used to compute angular rates.
    // In particular, a UBoxComponent that isn't at the root of an actor's component hierarchy will
    // always return 0 when calling the angular rate functions that are part of the UPrimitiveComponent
    // interface. To account for this unexpected behavior, we allow users of this class to pass in
    // a separate component for computing angular rates. We have found that passing in a
    // USkeletalMeshComponent at the root of an actor's component hierarchy behaves as expected.
    ImuSensor(UPrimitiveComponent* linear_acceleration_component, UPrimitiveComponent* angular_rate_component);
    ~ImuSensor();

    // Updates the IMU's linear acceleration and angular rate measurements at once
    void postPhysicsPreRenderTickEventHandler(float delta_time, ELevelTick level_tick);

    // Accelerometer: measures linear acceleration in m/s^2
    FVector linear_acceleration_ = FVector::ZeroVector;

    // Gyroscope: measures angular rate in rad/sec
    FVector angular_rate_ = FVector::ZeroVector;

private:
    void updateLinearAcceleration(float delta_time);
    void updateAngularRate();

    AActor* new_object_parent_actor_ = nullptr;

    UPrimitiveComponent* linear_acceleration_component_ = nullptr;
    UPrimitiveComponent* angular_rate_component_ = nullptr;

    UTickEvent* tick_event_ = nullptr;
    FDelegateHandle tick_event_handle_;

    // Used to compute the acceleration
    std::array<FVector, 2> previous_locations_;
    float previous_delta_time_ = 0.0f;

    // Random number generator
    std::minstd_rand random_gen_;
};
