//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "Vehicle/VehicleWheel.h"

#include "CoreUtils/Config.h"
#include "CoreUtils/Log.h"

UVehicleWheel::UVehicleWheel()
{
    SP_LOG_CURRENT_FUNCTION();

    if (!Config::s_initialized_) {
        return;
    }
    
    // We need to set the torque combine method to override mode for external torque 
    // values to be effective in applying drive torques and brake torques to the vehicle.
    ExternalTorqueCombineMethod = ETorqueCombineMethod::Override;

    WheelMass               = Config::get<float>("VEHICLE.VEHICLE_WHEEL.WHEEL_MASS");
    WheelRadius             = Config::get<float>("VEHICLE.VEHICLE_WHEEL.WHEEL_RADIUS");
    WheelWidth              = Config::get<float>("VEHICLE.VEHICLE_WHEEL.WHEEL_WIDTH");
    CorneringStiffness      = Config::get<float>("VEHICLE.VEHICLE_WHEEL.CORNERING_STIFFNESS");
    FrictionForceMultiplier = Config::get<float>("VEHICLE.VEHICLE_WHEEL.FRICTION_FORCE_MULTIPLIER");
    SideSlipModifier        = Config::get<float>("VEHICLE.VEHICLE_WHEEL.SIDE_SLIP_MODIFIER");
    SlipThreshold           = Config::get<float>("VEHICLE.VEHICLE_WHEEL.SLIP_THRESHOLD");
    SkidThreshold           = Config::get<float>("VEHICLE.VEHICLE_WHEEL.SKID_THRESHOLD");
    SuspensionMaxRaise      = Config::get<float>("VEHICLE.VEHICLE_WHEEL.SUSPENSION_MAX_RAISE");
    SuspensionMaxDrop       = Config::get<float>("VEHICLE.VEHICLE_WHEEL.SUSPENSION_MAX_DROP");
    SuspensionDampingRatio  = Config::get<float>("VEHICLE.VEHICLE_WHEEL.SUSPENSION_DAMPING_RATIO");
    SpringRate              = Config::get<float>("VEHICLE.VEHICLE_WHEEL.SPRING_RATE");
    SpringPreload           = Config::get<float>("VEHICLE.VEHICLE_WHEEL.SPRING_PRELOAD");
    SuspensionForceOffset   = FVector(
        Config::get<float>("VEHICLE.VEHICLE_WHEEL.SUSPENSION_FORCE_OFFSET_X"),
        Config::get<float>("VEHICLE.VEHICLE_WHEEL.SUSPENSION_FORCE_OFFSET_Y"),
        Config::get<float>("VEHICLE.VEHICLE_WHEEL.SUSPENSION_FORCE_OFFSET_Z"));
    MaxBrakeTorque          = Config::get<float>("VEHICLE.VEHICLE_WHEEL.MAX_BRAKE_TORQUE");
}

UVehicleWheel::~UVehicleWheel()
{
    SP_LOG_CURRENT_FUNCTION();
}
