//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SimulationController/ImuSensor.h"

#include <stdint.h> // uint8_t

#include <map>
#include <memory>  // std::make_unique
#include <string>
#include <utility> // std::move
#include <vector>

#include <Components/PrimitiveComponent.h>
#include <DrawDebugHelpers.h>       // DrawDebugDirectionalArrow
#include <Engine/EngineBaseTypes.h> // ELevelTick, ETickingGroup
#include <Engine/World.h>
#include <GameFramework/Actor.h>
#include <Math/Rotator.h>
#include <Math/Vector.h>
#include <PhysicsEngine/PhysicsSettings.h>

#include "SimulationController/StandaloneComponent.h"
#include "SimulationController/TickEventComponent.h"
#include "SpCore/ArrayDesc.h" // DataType
#include "SpCore/Assert.h"
#include "SpCore/Config.h"

struct FActorComponentTickFunction;

ImuSensor::ImuSensor(UPrimitiveComponent* primitive_component)
{
    SP_ASSERT(primitive_component);
    primitive_component_ = primitive_component;

    tick_event_component_ = std::make_unique<StandaloneComponent<UTickEventComponent>>(primitive_component->GetWorld(), "tick_event_component");
    SP_ASSERT(tick_event_component_);
    SP_ASSERT(tick_event_component_->component_);
    tick_event_component_->component_->PrimaryComponentTick.bCanEverTick = true;
    tick_event_component_->component_->PrimaryComponentTick.bTickEvenWhenPaused = false;
    tick_event_component_->component_->PrimaryComponentTick.TickGroup = ETickingGroup::TG_PostPhysics;
    tick_event_component_->component_->tick_func_ = [this](float delta_time, ELevelTick level_tick, FActorComponentTickFunction* this_tick_function) -> void {

        // Update linear acceleration
        FVector current_linear_velocity_world = primitive_component_->GetPhysicsLinearVelocity();
        FVector linear_acceleration_world = (current_linear_velocity_world - previous_linear_velocity_world_) / delta_time;

        // Roughly speaking, an accelerometer measures deviation from freefall. Therefore, a stationary accelerometer will measure a positive
        // acceleration of +9.81 m/s^2, even though it isn't moving. To account for this detail, we get gravitational acceleration from Unreal,
        // which is negative, and subtract it from our linear acceleration vector to get our final linear acceleration vector.
        float gravity_world = UPhysicsSettings::Get()->DefaultGravityZ;
        FVector linear_acceleration_minus_gravity_world = linear_acceleration_world - gravity_world;

        linear_acceleration_body_ = primitive_component_->GetComponentTransform().Rotator().UnrotateVector(linear_acceleration_minus_gravity_world);
        previous_linear_velocity_world_ = current_linear_velocity_world;

        // Update angular velocity
        FVector component_angular_velocity_world = primitive_component_->GetPhysicsAngularVelocityInRadians();
        angular_velocity_body_ = primitive_component_->GetComponentTransform().GetRotation().UnrotateVector(component_angular_velocity_world);

        // Debug render
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
    };
}

ImuSensor::~ImuSensor()
{
    SP_ASSERT(tick_event_component_);
    tick_event_component_->component_->tick_func_ = nullptr;
    tick_event_component_ = nullptr;

    SP_ASSERT(primitive_component_);
    primitive_component_ = nullptr;
}

std::map<std::string, ArrayDesc> ImuSensor::getObservationSpace() const
{
    std::map<std::string, ArrayDesc> observation_space;
    ArrayDesc array_desc;

    // a_x, a_y, a_z in [cm/s^2]
    array_desc.low_ = std::numeric_limits<double>::lowest();
    array_desc.high_ = std::numeric_limits<double>::max();
    array_desc.datatype_ = DataType::Float64;
    array_desc.shape_ = {3};
    Std::insert(observation_space, "imu.linear_acceleration_body", std::move(array_desc));

    // g_x, g_y, g_z in [rad/s]
    array_desc.low_ = std::numeric_limits<double>::lowest();
    array_desc.high_ = std::numeric_limits<double>::max();
    array_desc.datatype_ = DataType::Float64;
    array_desc.shape_ = {3};
    Std::insert(observation_space, "imu.angular_velocity_body", std::move(array_desc));

    return observation_space;
}

std::map<std::string, std::vector<uint8_t>> ImuSensor::getObservation() const
{
    std::map<std::string, std::vector<uint8_t>> observation;

    Std::insert(observation, "imu.linear_acceleration_body", Std::reinterpretAsVector<uint8_t, double>({
        linear_acceleration_body_.X,
        linear_acceleration_body_.Y,
        linear_acceleration_body_.Z}));

    Std::insert(observation, "imu.angular_velocity_body", Std::reinterpretAsVector<uint8_t, double>({
        angular_velocity_body_.X,
        angular_velocity_body_.Y,
        angular_velocity_body_.Z}));

    return observation;
}
