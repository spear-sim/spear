//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SimulationController/ImuSensor.h"

#include <array>

#include <Components/PrimitiveComponent.h>
#include <Delegates/IDelegateInstance.h>
#include <DrawDebugHelpers.h>
#include <Engine/EngineBaseTypes.h>
#include <Engine/World.h>
#include <GameFramework/Actor.h>
#include <Math/Rotator.h>
#include <Math/Vector.h>
#include <PhysicsEngine/PhysicsSettings.h>

#include "CoreUtils/Assert.h"
#include "CoreUtils/Config.h"
#include "SimulationController/TickEventComponent.h"

ImuSensor::ImuSensor(UPrimitiveComponent* primitive_component)
{
    SP_ASSERT(primitive_component);
    primitive_component_ = primitive_component;

    parent_actor_ = primitive_component->GetWorld()->SpawnActor<AActor>();
    SP_ASSERT(parent_actor_);

    tick_event_component_ = NewObject<UTickEventComponent>(parent_actor_);
    SP_ASSERT(tick_event_component_);
    tick_event_component_->RegisterComponent();
    tick_event_component_->PrimaryComponentTick.TickGroup = ETickingGroup::TG_PostPhysics;
    tick_event_handle_ = tick_event_component_->delegate_.AddRaw(this, &ImuSensor::postPhysicsPreRenderTickEventHandler);
}

ImuSensor::~ImuSensor()
{
    SP_ASSERT(tick_event_component_);
    tick_event_component_->delegate_.Remove(tick_event_handle_);
    tick_event_handle_.Reset();
    tick_event_component_->DestroyComponent();
    tick_event_component_ = nullptr;

    SP_ASSERT(parent_actor_);
    parent_actor_->Destroy();
    parent_actor_ = nullptr;

    SP_ASSERT(primitive_component_);
    primitive_component_ = nullptr;
}

void ImuSensor::updateLinearAcceleration(float delta_time)
{
    FVector current_linear_velocity_world = primitive_component_->GetPhysicsLinearVelocity();
    FVector linear_acceleration_world = (current_linear_velocity_world - previous_linear_velocity_world_) / delta_time;

    // Roughly speaking, an accelerometer measures deviation from freefall. Therefore, a stationary accelerometer will measure a positive
    // acceleration of +9.81 m/s^2, even though it isn't moving. To account for this detail, we get gravitational acceleration from Unreal,
    // which is negative, and subtract it from our linear acceleration vector to get our final linear acceleration vector.
    float gravity_world = UPhysicsSettings::Get()->DefaultGravityZ;
    FVector linear_acceleration_minus_gravity_world = linear_acceleration_world - gravity_world;

    linear_acceleration_body_ = primitive_component_->GetComponentTransform().Rotator().UnrotateVector(linear_acceleration_minus_gravity_world);

    previous_linear_velocity_world_ = current_linear_velocity_world;
}

void ImuSensor::updateAngularRate()
{
    FVector component_angular_velocity_world = primitive_component_->GetPhysicsAngularVelocityInRadians();
    angular_velocity_body_ = primitive_component_->GetComponentTransform().GetRotation().UnrotateVector(component_angular_velocity_world);
}

void ImuSensor::postPhysicsPreRenderTickEventHandler(float delta_time, ELevelTick level_tick)
{
    updateLinearAcceleration(delta_time);
    updateAngularRate();

    if (Config::get<bool>("SIMULATION_CONTROLLER.IMU_SENSOR.DEBUG_RENDER")) {
        UWorld* world = primitive_component_->GetWorld();
        FTransform transform = primitive_component_->GetComponentTransform();
        FRotator rotation = transform.Rotator();
        FVector location = transform.GetLocation();

        // Plot sensor frame
        DrawDebugDirectionalArrow(world, location, location + 5.0f * transform.GetUnitAxis(EAxis::X), 0.5f, FColor(255, 0, 0), false, 0.033f, 0, 0.5f);
        DrawDebugDirectionalArrow(world, location, location + 5.0f * transform.GetUnitAxis(EAxis::Y), 0.5f, FColor(0, 255, 0), false, 0.033f, 0, 0.5f);
        DrawDebugDirectionalArrow(world, location, location + 5.0f * transform.GetUnitAxis(EAxis::Z), 0.5f, FColor(0, 0, 255), false, 0.033f, 0, 0.5f);

        // Plot linear acceleration vector
        DrawDebugDirectionalArrow(world, location, location + rotation.RotateVector(linear_acceleration_body_), 0.5f, FColor(200, 0, 200), false, 0.033f, 0, 0.5f);

        // Plot angular rate vector
        DrawDebugDirectionalArrow(world, location, location + rotation.RotateVector(angular_velocity_body_), 0.5f, FColor(0, 200, 200), false, 0.033f, 0, 0.5f);
    }
}
