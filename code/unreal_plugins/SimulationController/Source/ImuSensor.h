#pragma once

#include <array>
#include <map>
#include <random>
#include <string>
#include <vector>

#include <Engine/EngineBaseTypes.h>

class AActor;
class UBoxComponent;
class UTickEvent;

class ImuSensor {
public:
    ImuSensor(UBoxComponent* primitive_component);
    ImuSensor(AActor* actor, const FVector& accelerometer_noise_std, const FVector& gyroscope_noise_std, const FVector& gyroscope_bias, const FVector& position_offset = FVector::ZeroVector, const FRotator& orientation_offset = FRotator::ZeroRotator, bool debug = false);
    ~ImuSensor();

    // Updates the IMU's linear acceleration and angular rate measurements at once
    void postPhysicsPreRenderTickEventHandler(float delta_time, enum ELevelTick level_tick);

    // Accelerometer: measures linear acceleration in m/s^2
    FVector linear_acceleration_ = FVector::ZeroVector;

    // Gyroscope: measures angular rate in rad/sec
    FVector angular_rate_ = FVector::ZeroVector;

private:
    
    FVector updateLinearAcceleration(float delta_time);

    FVector updateAngularRate();

    AActor* new_object_parent_actor_ = nullptr;

    UBoxComponent* primitive_component_ = nullptr;

    UTickEvent* post_physics_pre_render_tick_event_ = nullptr;
    FDelegateHandle post_physics_pre_render_tick_event_handle_;

    // Used to compute the acceleration
    std::array<FVector, 2> previous_locations_;
    float previous_delta_time_ = 0.0f;

    // Random number generator
    std::minstd_rand random_gen_;
};
