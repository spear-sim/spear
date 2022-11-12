#pragma once

#include <map>
#include <random>
#include <string>
#include <vector>

class AActor;
class USceneComponent;
class UTickEvent;

class ImuSensor {
public:
    ImuSensor(USceneComponent* component);
    ImuSensor(AActor* actor, const FVector& accelerometer_noise_std, const FVector& gyroscope_noise_std, const FVector& gyroscope_bias, const FVector& position_offset = FVector::ZeroVector, const FRotator& orientation_offset = FRotator::ZeroRotator, bool debug = false);
    ~ImuSensor();

    // Updates the IMU's linear acceleration and angular rate measurements at once
    void postPhysicsPreRenderTickEventHandler(float delta_time, enum ELevelTick level_tick);

    FVector linear_acceleration_measuement = FVector::ZeroVector;
    FVector angular_rate_measuement = FVector::ZeroVector;

private:
    // Accelerometer: measures linear acceleration in m/s^2
    FVector getLinearAcceleration(float delta_time);

    // Gyroscope: measures angular rate in rad/sec
    FVector getAngularRate();

    // Compute the random component of the acceleration sensor measurement
    inline FVector computeAccelerometerNoise(const FVector& accelerometer)
    {
        // Normal (or Gaussian or Gauss) distribution will be used as noise function.
        // A mean of 0.0 is used as a first parameter, the standard deviation is determined by the client
        return FVector{
            accelerometer.X + std::normal_distribution<float>(accelerometer_noise_mean_, accelerometer_noise_std_.X)(random_gen_),
            accelerometer.Y + std::normal_distribution<float>(accelerometer_noise_mean_, accelerometer_noise_std_.Y)(random_gen_),
            accelerometer.Z + std::normal_distribution<float>(accelerometer_noise_mean_, accelerometer_noise_std_.Z)(random_gen_)};
    }

    // Compute the random component and bias of the gyroscope sensor measurement
    inline FVector computeGyroscopeNoise(const FVector& gyroscope)
    {
        // Normal (or Gaussian or Gauss) distribution and a bias will be used as noise function.
        // A mean of 0.0 is used as a first parameter.The standard deviation and the bias are determined by the client
        return FVector{
            gyroscope.X + gyroscope_bias_.X + std::normal_distribution<float>(gyroscope_noise_mean_, gyroscope_noise_std_.X)(random_gen_),
            gyroscope.Y + gyroscope_bias_.Y + std::normal_distribution<float>(gyroscope_noise_mean_, gyroscope_noise_std_.Y)(random_gen_),
            gyroscope.Z + gyroscope_bias_.Z + std::normal_distribution<float>(gyroscope_noise_mean_, gyroscope_noise_std_.Z)(random_gen_)};
    }

    AActor* new_object_parent_actor_ = nullptr;

    USceneComponent* imu_component_ = nullptr;

    // 3D acceleration in [m/s²]
    float accelerometer_noise_mean_ = 0.0f;
    FVector accelerometer_noise_std_;

    // 3D angular rate in [rad/s]
    float gyroscope_noise_mean_ = 0.0f;
    FVector gyroscope_noise_std_;
    FVector gyroscope_bias_;

    // Used to compute the acceleration
    std::array<FVector, 2> previous_location_;

    // Used to compute the acceleration
    float previous_delta_time_ = 0.0f;

    // Random number generator
    std::minstd_rand random_gen_;

    // Debug flag to enable debug markers 
    bool debug_ = false;

    UTickEvent* post_physics_pre_render_tick_event_ = nullptr;
    FDelegateHandle post_physics_pre_render_tick_event_handle_;
};
