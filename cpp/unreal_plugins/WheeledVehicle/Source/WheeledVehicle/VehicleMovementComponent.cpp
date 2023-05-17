//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "WheeledVehicle/VehicleMovementComponent.h"

#include <iostream>

#include "CoreUtils/Config.h"
#include "WheeledVehicle/VehicleWheel.h"

UVehicleMovementComponent::UVehicleMovementComponent()
{
    std::cout << "[SPEAR | WheeledVehicleComponent.cpp] UVehicleMovementComponent::UVehicleMovementComponent" << std::endl;

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

    InertiaTensorScale = FVector::OneVector;
    Mass = Config::get<float>("WHEELED_VEHICLE.VEHICLE_COMPONENT.MASS");
    DragCoefficient = Config::get<float>("WHEELED_VEHICLE.VEHICLE_COMPONENT.DRAG_COEFFICIENT");
    ChassisWidth = Config::get<float>("WHEELED_VEHICLE.VEHICLE_COMPONENT.CHASSIS_WIDTH");
    ChassisHeight = Config::get<float>("WHEELED_VEHICLE.VEHICLE_COMPONENT.CHASSIS_HEIGHT");

    EngineSetup.MaxRPM = Config::get<float>("WHEELED_VEHICLE.VEHICLE_COMPONENT.MOTOR_MAX_RPM");
    EngineSetup.MaxTorque = Config::get<float>("WHEELED_VEHICLE.VEHICLE_COMPONENT.MOTOR_TORQUE_MAX");
}

UVehicleMovementComponent::~UVehicleMovementComponent()
{
    std::cout << "[SPEAR | WheeledVehicleComponent.cpp] UVehicleMovementComponent::~UVehicleMovementComponent" << std::endl;
}

Eigen::Vector4f UVehicleMovementComponent::getWheelRotationSpeeds() const
{
    Eigen::Vector4f wheel_rotation_speeds;
    wheel_rotation_speeds(0) = VehicleSimulationPT->PVehicle->GetWheel(0).GetAngularVelocity(); // Expressed in [RPM]
    wheel_rotation_speeds(1) = VehicleSimulationPT->PVehicle->GetWheel(1).GetAngularVelocity(); // Expressed in [RPM]
    wheel_rotation_speeds(2) = VehicleSimulationPT->PVehicle->GetWheel(2).GetAngularVelocity(); // Expressed in [RPM]
    wheel_rotation_speeds(3) = VehicleSimulationPT->PVehicle->GetWheel(3).GetAngularVelocity(); // Expressed in [RPM]
    return wheel_rotation_speeds;
}

bool UVehicleMovementComponent::isSleeping() const
{
    return VehicleSimulationPT->VehicleState.bSleeping;
}

void UVehicleMovementComponent::printDebugValues()
{
    UE_LOG(LogTemp, Warning, TEXT("UVehicleMovementComponent.cpp::printDebugValues(), VehicleSimulationPT->PVehicle->GetWheel(0).GetAngularVelocity() = %f"), VehicleSimulationPT->PVehicle->GetWheel(0).GetAngularVelocity());
    UE_LOG(LogTemp, Warning, TEXT("UVehicleMovementComponent.cpp::printDebugValues(), VehicleSimulationPT->PVehicle->GetWheel(1).GetAngularVelocity() = %f"), VehicleSimulationPT->PVehicle->GetWheel(1).GetAngularVelocity());
    UE_LOG(LogTemp, Warning, TEXT("UVehicleMovementComponent.cpp::printDebugValues(), VehicleSimulationPT->PVehicle->GetWheel(2).GetAngularVelocity() = %f"), VehicleSimulationPT->PVehicle->GetWheel(2).GetAngularVelocity());
    UE_LOG(LogTemp, Warning, TEXT("UVehicleMovementComponent.cpp::printDebugValues(), VehicleSimulationPT->PVehicle->GetWheel(3).GetAngularVelocity() = %f"), VehicleSimulationPT->PVehicle->GetWheel(3).GetAngularVelocity());

    UE_LOG(LogTemp, Warning, TEXT("UVehicleMovementComponent.cpp::printDebugValues(), VehicleSimulationPT->PVehicle->GetWheel(0).GetDriveTorque() = %f"), VehicleSimulationPT->PVehicle->GetWheel(0).GetDriveTorque());
    UE_LOG(LogTemp, Warning, TEXT("UVehicleMovementComponent.cpp::printDebugValues(), VehicleSimulationPT->PVehicle->GetWheel(1).GetDriveTorque() = %f"), VehicleSimulationPT->PVehicle->GetWheel(1).GetDriveTorque());
    UE_LOG(LogTemp, Warning, TEXT("UVehicleMovementComponent.cpp::printDebugValues(), VehicleSimulationPT->PVehicle->GetWheel(2).GetDriveTorque() = %f"), VehicleSimulationPT->PVehicle->GetWheel(2).GetDriveTorque());
    UE_LOG(LogTemp, Warning, TEXT("UVehicleMovementComponent.cpp::printDebugValues(), VehicleSimulationPT->PVehicle->GetWheel(3).GetDriveTorque() = %f"), VehicleSimulationPT->PVehicle->GetWheel(3).GetDriveTorque());
}
