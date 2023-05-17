//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <array>
#include <limits>

#include <Delegates/IDelegateInstance.h>
#include <Engine/EngineBaseTypes.h>
#include <Math/Vector.h>

class AActor;
class UPrimitiveComponent;

class UTickEventComponent;

class ImuSensor 
{
public:
    ImuSensor(UPrimitiveComponent* component);
    ~ImuSensor();

    // Linear acceleration minus gravity (i.e., will report +980 cm/s^2 for a stationary body aligned with the world-frame origin) in the body frame in cm/s^2
    FVector linear_acceleration_body_ = FVector::ZeroVector;

    // Angular velocity in the body frame in rad/s
    FVector angular_velocity_body_ = FVector::ZeroVector;

private:
    void postPhysicsPreRenderTickEventHandler(float delta_time, ELevelTick level_tick);

    void updateLinearAcceleration(float delta_time);
    void updateAngularRate();

    AActor* parent_actor_ = nullptr;

    UPrimitiveComponent* primitive_component_ = nullptr;

    UTickEventComponent* tick_event_component_ = nullptr;
    FDelegateHandle tick_event_handle_;

    FVector previous_linear_velocity_world_ = FVector::ZeroVector;
};
