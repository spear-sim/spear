#pragma once

#include <map>
#include <string>
#include <vector>
#include <random>

class AActor;

class ImuSensor {
public:
    ImuSensor(AActor* actor, const FVector& accelerometer_noise_std, const FVector& gyroscope_noise_std, const FVector& gyroscope_bias, const FVector& position_offset = FVector::ZeroVector, const FRotator& orientation_offset = FRotator::ZeroRotator, bool debug = false);
    ~ImuSensor();

    // Updates the IMU's linear acceleration and angular rate measurements at once
    void update(FVector& accelerometer, FVector& gyroscope, const float delta_time);

    // Accelerometer: measures linear acceleration in m/s^2
    FVector computeAccelerometer(const float delta_time);

    // Gyroscope: measures angular velocity in rad/sec
    FVector computeGyroscope();

    inline void setAccelerationStandardDeviation(const FVector& vec)
    {
        accelerometer_noise_std_ = vec;
    }

    inline void setGyroscopeStandardDeviation(const FVector& vec)
    {
        gyroscope_noise_std_ = vec;
    }

    inline void setGyroscopeBias(const FVector& vec)
    {
        gyroscope_bias_ = vec;
    }

    inline const FVector& getAccelerationStandardDeviation() const
    {
        return accelerometer_noise_std_;
    }

    inline const FVector& getGyroscopeStandardDeviation() const
    {
        return gyroscope_noise_std_;
    }

    inline const FVector& getGyroscopeBias() const
    {
        return gyroscope_bias_;
    }

private:
    // Compute the random component of the acceleration sensor measurement 
    FVector computeAccelerometerNoise(const FVector& accelerometer);

    // Compute the random component and bias of the gyroscope sensor measurement
    FVector computeGyroscopeNoise(const FVector& gyroscope);

    AActor* imu_actor_ = nullptr;
    UMeshComponent* imu_mesh_component_ = nullptr;

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

    // Position and orientation offsets of the sensor relative to the parent actor frame
    FVector position_offset_ = FVector::ZeroVector; 
    FRotator orientation_offset_ = FRotator::ZeroRotator;

    bool debug_ = false;
};
