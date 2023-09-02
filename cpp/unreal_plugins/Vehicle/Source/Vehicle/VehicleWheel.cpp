//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "Vehicle/VehicleWheel.h"

#include "CoreUtils/Config.h"
#include "CoreUtils/Log.h"

UVehicleWheel::UVehicleWheel()
{
    SP_LOG_CURRENT_FUNCTION();
    
    // We need to set the torque combine method to override mode for external torque 
    // values to be effective in applying drive torques and brake torques to the vehicle.
    ExternalTorqueCombineMethod = ETorqueCombineMethod::Override;

    if (Config::s_initialized_) {
        WheelMass               = Config::get<float>("VEHICLE.VEHICLE_WHEEL.WHEEL_MASS");
        WheelRadius             = Config::get<float>("VEHICLE.VEHICLE_WHEEL.WHEEL_RADIUS");
        WheelWidth              = Config::get<float>("VEHICLE.VEHICLE_WHEEL.WHEEL_WIDTH");
        FrictionForceMultiplier = Config::get<float>("VEHICLE.VEHICLE_WHEEL.FRICTION_FORCE_MULTIPLIER");
        SuspensionMaxRaise      = Config::get<float>("VEHICLE.VEHICLE_WHEEL.SUSPENSION_MAX_RAISE");
        SuspensionMaxDrop       = Config::get<float>("VEHICLE.VEHICLE_WHEEL.SUSPENSION_MAX_DROP");
        SuspensionDampingRatio  = Config::get<float>("VEHICLE.VEHICLE_WHEEL.SUSPENSION_DAMPING_RATIO");
        SpringRate              = Config::get<float>("VEHICLE.VEHICLE_WHEEL.SPRING_RATE");
        SpringPreload           = Config::get<float>("VEHICLE.VEHICLE_WHEEL.SPRING_PRELOAD");
    } else {
        // OpenBot defaults, see python/spear/config/default_config.vehicle.yaml
        WheelMass               = 0.033f;
        WheelRadius             = 3.3f;
        WheelWidth              = 2.5f;
        FrictionForceMultiplier = 3.0f;
        SuspensionMaxRaise      = 0.001f;
        SuspensionMaxDrop       = 0.001f;
        SuspensionDampingRatio  = 0.5f;
        SpringRate              = 0.167f;
        SpringPreload           = 0.033f;
    }
}

UVehicleWheel::~UVehicleWheel()
{
    SP_LOG_CURRENT_FUNCTION();
}
