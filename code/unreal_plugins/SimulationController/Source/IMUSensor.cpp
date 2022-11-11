#include "IMUSensor.h"

#include <map>
#include <string>
#include <utility>
#include <vector>

#include "DrawDebugHelpers.h"
#include <Engine/World.h>
#include <EngineUtils.h>
#include <GameFramework/Actor.h>
#include <UObject/UObjectGlobals.h>

#include "Assert/Assert.h"
#include "Config.h"
#include "Serialize.h"
#include "TickEvent.h"

IMUSensor::IMUSensor(AActor* actor, const FVector& accelerometer_noise_std, const FVector& gyroscope_noise_std, const FVector& gyroscope_bias, const FVector& position_offset, const FRotator& orientation_offset, bool debug)
{
    previous_location_ = { FVector::ZeroVector, FVector::ZeroVector };
    // Initialized to something hight to minimize the artifacts when the initial values are unknown
    previous_delta_time_ = std::numeric_limits<float>::max();
    imu_actor_ = actor;
    ASSERT(imu_actor_);
    imu_mesh_component_ = Cast<UMeshComponent>(imu_actor_->GetRootComponent());
    ASSERT(imu_mesh_component_);
    accelerometer_noise_std_ = accelerometer_noise_std;
    gyroscope_noise_std_ = gyroscope_noise_std;
    gyroscope_bias_ = gyroscope_bias;
    position_offset_ = position_offset;
    orientation_offset_ = orientation_offset;
    debug_ = debug;
}

IMUSensor::~IMUSensor()
{
}

void IMUSensor::update(FVector& accelerometer, FVector& gyroscope, const float delta_time)
{
    accelerometer = computeAccelerometer(delta_time);
    gyroscope = computeGyroscope();
    if (debug_) {
        const FTransform& actor_transform = imu_actor_->GetActorTransform();
        const FRotator& transform_rotator = FRotator(FQuat(orientation_offset_) * FQuat(actor_transform.Rotator()));
        const FVector& imu_location = imu_actor_->GetActorLocation() + transform_rotator.RotateVector(position_offset_) * imu_actor_->GetWorld()->GetWorldSettings()->WorldToMeters;
        const FVector transform_x_axis = transform_rotator.RotateVector(actor_transform.GetUnitAxis(EAxis::X));
        const FVector transform_y_axis = transform_rotator.RotateVector(actor_transform.GetUnitAxis(EAxis::Y));
        const FVector transform_z_axis = transform_rotator.RotateVector(actor_transform.GetUnitAxis(EAxis::Z));
        // Plot sensor frame
        DrawDebugDirectionalArrow(imu_actor_->GetWorld(), imu_location, imu_location + 5 * actor_transform.GetUnitAxis(EAxis::X), 0.5, FColor(255, 0, 0), false, 0.033, 0, 0.5); // X
        DrawDebugDirectionalArrow(imu_actor_->GetWorld(), imu_location, imu_location + 5 * actor_transform.GetUnitAxis(EAxis::Y), 0.5, FColor(0, 255, 0), false, 0.033, 0, 0.5); // Y
        DrawDebugDirectionalArrow(imu_actor_->GetWorld(), imu_location, imu_location + 5 * actor_transform.GetUnitAxis(EAxis::Z), 0.5, FColor(0, 0, 255), false, 0.033, 0, 0.5); // Z

        // Plot acceleration vector
        DrawDebugDirectionalArrow(imu_actor_->GetWorld(), imu_location, imu_location + transform_rotator.RotateVector(accelerometer), 0.5, FColor(200, 0, 200), false, 0.033, 0, 0.5);

        // Plot angular rate vector
        DrawDebugDirectionalArrow(imu_actor_->GetWorld(), imu_location, imu_location + transform_rotator.RotateVector(gyroscope), 0.5, FColor(0, 200, 200), false, 0.033, 0, 0.5);
    }
}

FVector IMUSensor::computeAccelerometerNoise(const FVector& accelerometer)
{
    // Normal (or Gaussian or Gauss) distribution will be used as noise function.
    // A mean of 0.0 is used as a first parameter, the standard deviation is determined by the client
    return FVector{
        accelerometer.X + std::normal_distribution<float>(accelerometer_noise_mean_, accelerometer_noise_std_.X)(random_gen_),
        accelerometer.Y + std::normal_distribution<float>(accelerometer_noise_mean_, accelerometer_noise_std_.Y)(random_gen_),
        accelerometer.Z + std::normal_distribution<float>(accelerometer_noise_mean_, accelerometer_noise_std_.Z)(random_gen_)};
}

FVector IMUSensor::computeGyroscopeNoise(const FVector& gyroscope)
{
    // Normal (or Gaussian or Gauss) distribution and a bias will be used as noise function.
    // A mean of 0.0 is used as a first parameter.The standard deviation and the bias are determined by the client
    return FVector{
        gyroscope.X + gyroscope_bias_.X + std::normal_distribution<float>(gyroscope_noise_mean_, gyroscope_noise_std_.X)(random_gen_),
        gyroscope.Y + gyroscope_bias_.Y + std::normal_distribution<float>(gyroscope_noise_mean_, gyroscope_noise_std_.Y)(random_gen_),
        gyroscope.Z + gyroscope_bias_.Z + std::normal_distribution<float>(gyroscope_noise_mean_, gyroscope_noise_std_.Z)(random_gen_)};
}

// Accelerometer: measures linear acceleration in m/s^2
FVector IMUSensor::computeAccelerometer(const float delta_time)
{
    // Earth's gravitational acceleration is approximately 9.81 m/s^2
    constexpr float GRAVITY = 9.81f;

    // 2nd derivative of the polynomic (quadratic) interpolation using the point in current time and two previous steps:
    // d2[i] = -2.0*(y1/(h1*h2)-y2/((h2+h1)*h2)-y0/(h1*(h2+h1)))
    const FTransform& actor_transform = imu_actor_->GetActorTransform();
    const FRotator& transform_rotator = FRotator(FQuat(orientation_offset_) * FQuat(actor_transform.Rotator()));
    const FVector& current_location = imu_actor_->GetActorLocation() + transform_rotator.RotateVector(position_offset_) * imu_actor_->GetWorld()->GetWorldSettings()->WorldToMeters;
    const FVector Y2 = previous_location_[0];
    const FVector Y1 = previous_location_[1];
    const FVector Y0 = current_location;
    const float H1 = delta_time;
    const float H2 = previous_delta_time_;
    const float H1AndH2 = H2 + H1;
    const FVector A = Y1 / (H1 * H2);
    const FVector B = Y2 / (H2 * (H1AndH2));
    const FVector C = Y0 / (H1 * (H1AndH2));
    FVector accelerometer_raw = -2.0f * (A - B - C) / imu_actor_->GetWorld()->GetWorldSettings()->WorldToMeters;

    // Update the previous locations
    previous_location_[0] = previous_location_[1];
    previous_location_[1] = current_location;
    previous_delta_time_ = delta_time;

    // Add gravitational acceleration
    accelerometer_raw.Z += GRAVITY;
    // FQuat imu_rotation = imu_actor_->GetRootComponent()->GetComponentTransform().GetRotation();
    // accelerometer_raw = imu_rotation.UnrotateVector(accelerometer_raw);
    accelerometer_raw = transform_rotator.UnrotateVector(accelerometer_raw);

    return computeAccelerometerNoise(accelerometer_raw);
}

// Gyroscope: measures angular velocity in [rad/sec]
FVector IMUSensor::computeGyroscope()
{
    const FQuat actor_global_rotation = imu_actor_->GetRootComponent()->GetComponentTransform().GetRotation();
    const FQuat sensor_local_rotation = imu_actor_->GetRootComponent()->GetRelativeTransform().GetRotation();
    FVector angular_velocity = actor_global_rotation.UnrotateVector(imu_mesh_component_->GetPhysicsAngularVelocityInRadians());

    return computeGyroscopeNoise(sensor_local_rotation.RotateVector(angular_velocity));
}