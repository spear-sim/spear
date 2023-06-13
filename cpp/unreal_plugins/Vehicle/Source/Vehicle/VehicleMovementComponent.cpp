//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "Vehicle/VehicleMovementComponent.h"

#include <vector>

#include "CoreUtils/Config.h"
#include "CoreUtils/Log.h"
#include "Vehicle/VehicleWheel.h"

UVehicleMovementComponent::UVehicleMovementComponent()
{
    SP_LOG_CURRENT_FUNCTION();

    if (!Config::s_initialized_) {
        return;
    }

    // we only support vehicles with 4 wheels. So 
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

    Mass                         = Config::get<float>("VEHICLE.VEHICLE_COMPONENT.MASS");
    ChassisWidth                 = Config::get<float>("VEHICLE.VEHICLE_COMPONENT.CHASSIS_WIDTH");
    ChassisHeight                = Config::get<float>("VEHICLE.VEHICLE_COMPONENT.CHASSIS_HEIGHT");
    DragCoefficient              = Config::get<float>("VEHICLE.VEHICLE_COMPONENT.DRAG_COEFFICIENT");
    bSuspensionEnabled           = Config::get<bool>("VEHICLE.VEHICLE_COMPONENT.ENABLE_SUSPENSION");
    bWheelFrictionEnabled        = Config::get<bool>("VEHICLE.VEHICLE_COMPONENT.ENABLE_WHEEL_FRICTION");
    bLegacyWheelFrictionPosition = Config::get<bool>("VEHICLE.VEHICLE_COMPONENT.ENABLE_LEGACY_WHEEL_FRICTION");
}

UVehicleMovementComponent::~UVehicleMovementComponent()
{
    SP_LOG_CURRENT_FUNCTION();
}

std::vector<double> UVehicleMovementComponent::getWheelRotationSpeeds() const
{
    std::vector<double> wheel_rotation_speeds(4, 0.0);
    wheel_rotation_speeds.at(0) = VehicleSimulationPT->PVehicle->GetWheel(0).GetAngularVelocity(); // Expressed in [rad/s]
    wheel_rotation_speeds.at(1) = VehicleSimulationPT->PVehicle->GetWheel(1).GetAngularVelocity(); // Expressed in [rad/s]
    wheel_rotation_speeds.at(2) = VehicleSimulationPT->PVehicle->GetWheel(2).GetAngularVelocity(); // Expressed in [rad/s]
    wheel_rotation_speeds.at(3) = VehicleSimulationPT->PVehicle->GetWheel(3).GetAngularVelocity(); // Expressed in [rad/s]
    return wheel_rotation_speeds;
}
