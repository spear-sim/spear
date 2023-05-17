//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "WheeledVehicle/VehicleWheel.h"

#include <iostream>

#include "CoreUtils/Config.h"

UVehicleWheel::UVehicleWheel()
{
    std::cout << "[SPEAR | VehicleWheelWheel.cpp] UVehicleWheel::UVehicleWheel" << std::endl;

    if (!Config::s_initialized_) {
        return;
    }

    // is this needed?
    //bAffectedByHandbrake = true;

    // we need to set this for external torque values to be used to drive the vehicle
    ExternalTorqueCombineMethod = ETorqueCombineMethod::Override;

    // this needs to be non-zero to be able to steer
    MaxSteerAngle = Config::get<float>("WHEELED_VEHICLE.VEHICLE_WHEEL.STEER_ANGLE");

    MaxBrakeTorque = Config::get<float>("WHEELED_VEHICLE.VEHICLE_WHEEL.MAX_BRAKE_TORQUE");
    MaxHandBrakeTorque = Config::get<float>("WHEELED_VEHICLE.VEHICLE_WHEEL.MAX_HAND_BRAKE_TORQUE");

    WheelMass = Config::get<float>("WHEELED_VEHICLE.VEHICLE_WHEEL.MASS");
    WheelRadius = Config::get<float>("WHEELED_VEHICLE.VEHICLE_WHEEL.SHAPE_RADIUS");
    WheelWidth = Config::get<float>("WHEELED_VEHICLE.VEHICLE_WHEEL.SHAPE_WIDTH");

    SuspensionMaxRaise = Config::get<float>("WHEELED_VEHICLE.VEHICLE_WHEEL.SUSPENSION_MAX_RAISE");
    SuspensionMaxDrop = Config::get<float>("WHEELED_VEHICLE.VEHICLE_WHEEL.SUSPENSION_MAX_DROP");
    SuspensionDampingRatio = Config::get<float>("WHEELED_VEHICLE.VEHICLE_WHEEL.SUSPENSION_DAMPING_RATIO");
}

UVehicleWheel::~UVehicleWheel()
{
    std::cout << "[SPEAR | VehicleWheelWheel.cpp] UVehicleWheel::~UVehicleWheel" << std::endl;
}
