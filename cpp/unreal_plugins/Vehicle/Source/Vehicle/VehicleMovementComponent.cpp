//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "Vehicle/VehicleMovementComponent.h"

#include <vector>

#include <ChaosWheeledVehicleMovementComponent.h>
#include <UObject/NameTypes.h> // FName

#include "SpCore/Config.h"
#include "SpCore/Log.h"
#include "Vehicle/VehicleWheel.h"

UVehicleMovementComponent::UVehicleMovementComponent()
{
    SP_LOG_CURRENT_FUNCTION();

    // In Engine/Plugins/Experimental/ChaosVehiclesPlugin/Source/ChaosVehicles/Private/ChaosVehicleMovementComponent.cpp:1200,
    // Chaos doesn't take into consideration the torque inputs to wheels for determining the sleep state of the body. Since we
    // are providing direct torque inputs to wheels, and these torque inputs are not used to determine the sleep state of the
    // body, the body will be set to sleep every Tick. This prohibits us from being able to control the vehicle. Setting SleepThreshold=0.0
    // ensures that Chaos doesn't put this body to sleep.
    SleepThreshold = 0.0f;

    // We only support vehicles with 4 wheels.
    WheelSetups.SetNum(4);

    UClass* wheel_class = UVehicleWheel::StaticClass();

    WheelSetups[0].WheelClass = wheel_class;
    WheelSetups[0].BoneName = FName("FL");
    WheelSetups[0].AdditionalOffset = FVector::ZeroVector; // Offset the wheel from the bone's location

    WheelSetups[1].WheelClass = wheel_class;
    WheelSetups[1].BoneName = FName("FR");
    WheelSetups[1].AdditionalOffset = FVector::ZeroVector; // Offset the wheel from the bone's location

    WheelSetups[2].WheelClass = wheel_class;
    WheelSetups[2].BoneName = FName("RL");
    WheelSetups[2].AdditionalOffset = FVector::ZeroVector; // Offset the wheel from the bone's location

    WheelSetups[3].WheelClass = wheel_class;
    WheelSetups[3].BoneName = FName("RR");
    WheelSetups[3].AdditionalOffset = FVector::ZeroVector; // Offset the wheel from the bone's location

    if (Config::s_initialized_) {
        Mass                         = Config::get<float>("VEHICLE.VEHICLE_MOVEMENT_COMPONENT.MASS");
        ChassisWidth                 = Config::get<float>("VEHICLE.VEHICLE_MOVEMENT_COMPONENT.CHASSIS_WIDTH");
        ChassisHeight                = Config::get<float>("VEHICLE.VEHICLE_MOVEMENT_COMPONENT.CHASSIS_HEIGHT");
        DragCoefficient              = Config::get<float>("VEHICLE.VEHICLE_MOVEMENT_COMPONENT.DRAG_COEFFICIENT");
        bSuspensionEnabled           = Config::get<bool>("VEHICLE.VEHICLE_MOVEMENT_COMPONENT.SUSPENSION_ENABLED");
        bWheelFrictionEnabled        = Config::get<bool>("VEHICLE.VEHICLE_MOVEMENT_COMPONENT.WHEEL_FRICTION_ENABLED");
        bLegacyWheelFrictionPosition = Config::get<bool>("VEHICLE.VEHICLE_MOVEMENT_COMPONENT.LEGACY_WHEEL_FRICTION_POSITION_ENABLED");
    } else {
        // OpenBot defaults, see python/spear/config/default_config.vehicle.yaml
        Mass                         = 11.2f;
        ChassisWidth                 = 15.0f;
        ChassisHeight                = 15.0f;
        DragCoefficient              = 1.0f;
        bSuspensionEnabled           = true;
        bWheelFrictionEnabled        = true;
        bLegacyWheelFrictionPosition = false;
    }
}

UVehicleMovementComponent::~UVehicleMovementComponent()
{
    SP_LOG_CURRENT_FUNCTION();
}

std::vector<double> UVehicleMovementComponent::getWheelRotationSpeeds() const
{
    // We typically avoid trivial get() methods like this, but VehicleSimulationPT is protected, so we need
    // to explicitly provide access to it through a public function. GetAngularVelocity() returns values in
    // rad/s.
    return std::vector<double>{
        VehicleSimulationPT->PVehicle->GetWheel(0).GetAngularVelocity(),
        VehicleSimulationPT->PVehicle->GetWheel(1).GetAngularVelocity(),
        VehicleSimulationPT->PVehicle->GetWheel(2).GetAngularVelocity(),
        VehicleSimulationPT->PVehicle->GetWheel(3).GetAngularVelocity(),
    };
}
