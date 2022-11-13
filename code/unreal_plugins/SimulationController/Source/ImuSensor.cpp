#include "ImuSensor.h"

#include <map>
#include <string>
#include <utility>
#include <vector>

#include <Components/PrimitiveComponent.h>
#include <DrawDebugHelpers.h>
#include <Engine/World.h>
#include <EngineUtils.h>
#include <GameFramework/Actor.h>
#include <UObject/UObjectGlobals.h>

#include "Assert/Assert.h"
#include "Config.h"
#include "Serialize.h"
#include "TickEvent.h"

ImuSensor::ImuSensor(UPrimitiveComponent* primitive_component)
{
    ASSERT(primitive_component);
    primitive_component_ = primitive_component;
    new_object_parent_actor_ = primitive_component->GetWorld()->SpawnActor<AActor>();
    ASSERT(new_object_parent_actor_);

    post_physics_pre_render_tick_event_ = NewObject<UTickEvent>(new_object_parent_actor_, TEXT("PostPhysicsPreRenderTickEvent"));
    ASSERT(post_physics_pre_render_tick_event_);
    post_physics_pre_render_tick_event_->RegisterComponent();
    post_physics_pre_render_tick_event_->initialize(ETickingGroup::TG_PostPhysics);
    post_physics_pre_render_tick_event_handle_ = post_physics_pre_render_tick_event_->delegate_.AddRaw(this, &ImuSensor::postPhysicsPreRenderTickEventHandler);

    previous_location_ = {FVector::ZeroVector, FVector::ZeroVector};
    previous_delta_time_ = std::numeric_limits<float>::max(); // Initialized to something hight to minimize the artifacts when the initial values are unknown

    accelerometer_noise_std_ = FVector(Config::getValue<float>({"SIMULATION_CONTROLLER", "IMU_SENSOR", "ACCELEROMETER_NOISE_STD", "X"}),
                                       Config::getValue<float>({"SIMULATION_CONTROLLER", "IMU_SENSOR", "ACCELEROMETER_NOISE_STD", "Y"}),
                                       Config::getValue<float>({"SIMULATION_CONTROLLER", "IMU_SENSOR", "ACCELEROMETER_NOISE_STD", "Z"}));
    gyroscope_noise_std_ = FVector(Config::getValue<float>({"SIMULATION_CONTROLLER", "IMU_SENSOR", "GYROSCOPE_NOISE_STD", "X"}),
                                   Config::getValue<float>({"SIMULATION_CONTROLLER", "IMU_SENSOR", "GYROSCOPE_NOISE_STD", "Y"}),
                                   Config::getValue<float>({"SIMULATION_CONTROLLER", "IMU_SENSOR", "GYROSCOPE_NOISE_STD", "Z"}));
    gyroscope_bias_ = FVector(Config::getValue<float>({"SIMULATION_CONTROLLER", "IMU_SENSOR", "GYROSCOPE_BIAS", "X"}),
                              Config::getValue<float>({"SIMULATION_CONTROLLER", "IMU_SENSOR", "GYROSCOPE_BIAS", "Y"}),
                              Config::getValue<float>({"SIMULATION_CONTROLLER", "IMU_SENSOR", "GYROSCOPE_BIAS", "Z"}));
    debug_ = Config::getValue<bool>({"SIMULATION_CONTROLLER", "IMU_SENSOR", "DEBUG"});
}

ImuSensor::~ImuSensor()
{
    ASSERT(new_object_parent_actor_);
    new_object_parent_actor_->Destroy();
    new_object_parent_actor_ = nullptr;

    ASSERT(primitive_component_);
    primitive_component_ = nullptr;
}

FVector ImuSensor::getLinearAcceleration(float delta_time) 
{
    // Earth's gravitational acceleration is approximately 9.81 m/s^2
    float GRAVITY = 9.81f;

    // 2nd derivative of the polynomic (quadratic) interpolation using the point in current time and two previous steps:
    // d2[i] = -2.0*(y1/(h1*h2)-y2/((h2+h1)*h2)-y0/(h1*(h2+h1)))
    const FTransform& actor_transform = new_object_parent_actor_->GetActorTransform();
    const FRotator& transform_rotator = actor_transform.Rotator();
    const FVector& current_location = new_object_parent_actor_->GetActorLocation();
    const FVector Y2 = previous_location_[0];
    const FVector Y1 = previous_location_[1];
    const FVector Y0 = current_location;
    const float H1 = delta_time;
    const float H2 = previous_delta_time_;
    const float H1AndH2 = H2 + H1;
    const FVector A = Y1 / (H1 * H2);
    const FVector B = Y2 / (H2 * (H1AndH2));
    const FVector C = Y0 / (H1 * (H1AndH2));
    FVector linear_acceleration = -2.0f * (A - B - C) / primitive_component_->GetWorld()->GetWorldSettings()->WorldToMeters;

    // Update the previous locations
    previous_location_[0] = previous_location_[1];
    previous_location_[1] = current_location;
    previous_delta_time_ = delta_time;

    // Add gravitational acceleration
    linear_acceleration.Z += GRAVITY;
    // FQuat imu_rotation = new_object_parent_actor_->GetRootComponent()->GetComponentTransform().GetRotation();
    // linear_acceleration = imu_rotation.UnrotateVector(linear_acceleration);
    linear_acceleration = transform_rotator.UnrotateVector(linear_acceleration);

    return computeAccelerometerNoise(linear_acceleration);
}

FVector ImuSensor::getAngularRate() 
{
    const FQuat actor_global_rotation = new_object_parent_actor_->GetRootComponent()->GetComponentTransform().GetRotation();
    const FQuat sensor_local_rotation = new_object_parent_actor_->GetRootComponent()->GetRelativeTransform().GetRotation();
    FVector angular_rate = actor_global_rotation.UnrotateVector(primitive_component_->GetPhysicsAngularVelocityInRadians());

    return computeGyroscopeNoise(sensor_local_rotation.RotateVector(angular_rate));
}

void ImuSensor::postPhysicsPreRenderTickEventHandler(float delta_time, enum ELevelTick level_tick)
{   
    // Accelerometer: measures linear acceleration in m/s^2
    linear_acceleration_ = getLinearAcceleration(delta_time);

    // Gyroscope: measures angular rate in [rad/sec]
    angular_rate_ = getAngularRate(); 

    if (debug_) {
        const FTransform& actor_transform = new_object_parent_actor_->GetActorTransform();
        const FRotator& transform_rotator = actor_transform.Rotator();
        const FVector& imu_location = new_object_parent_actor_->GetActorLocation();
        const FVector transform_x_axis = transform_rotator.RotateVector(actor_transform.GetUnitAxis(EAxis::X));
        const FVector transform_y_axis = transform_rotator.RotateVector(actor_transform.GetUnitAxis(EAxis::Y));
        const FVector transform_z_axis = transform_rotator.RotateVector(actor_transform.GetUnitAxis(EAxis::Z));

        // Plot sensor frame
        DrawDebugDirectionalArrow(primitive_component_->GetWorld(), imu_location, imu_location + 5 * actor_transform.GetUnitAxis(EAxis::X), 0.5, FColor(255, 0, 0), false, 0.033, 0, 0.5); // X
        DrawDebugDirectionalArrow(primitive_component_->GetWorld(), imu_location, imu_location + 5 * actor_transform.GetUnitAxis(EAxis::Y), 0.5, FColor(0, 255, 0), false, 0.033, 0, 0.5); // Y
        DrawDebugDirectionalArrow(primitive_component_->GetWorld(), imu_location, imu_location + 5 * actor_transform.GetUnitAxis(EAxis::Z), 0.5, FColor(0, 0, 255), false, 0.033, 0, 0.5); // Z

        // Plot acceleration vector
        DrawDebugDirectionalArrow(primitive_component_->GetWorld(), imu_location, imu_location + transform_rotator.RotateVector(linear_acceleration_), 0.5, FColor(200, 0, 200), false, 0.033, 0, 0.5);

        // Plot angular rate vector
        DrawDebugDirectionalArrow(primitive_component_->GetWorld(), imu_location, imu_location + transform_rotator.RotateVector(angular_rate_), 0.5, FColor(0, 200, 200), false, 0.033, 0, 0.5);
    }
}

