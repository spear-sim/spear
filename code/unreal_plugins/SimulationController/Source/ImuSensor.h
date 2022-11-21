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
    ImuSensor(UPrimitiveComponent* component);
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

    UPrimitiveComponent* component_ = nullptr;

    UTickEvent* tick_event_ = nullptr;
    FDelegateHandle tick_event_handle_;

    // Used to compute the acceleration
    std::array<FVector, 2> previous_locations_;
    float previous_delta_time_ = 0.0f;

    // Random number generator
    std::minstd_rand random_gen_;
};
