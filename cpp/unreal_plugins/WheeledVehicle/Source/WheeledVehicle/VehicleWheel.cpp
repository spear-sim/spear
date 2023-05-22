//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "WheeledVehicle/VehicleWheel.h"

#include "CoreUtils/Config.h"
#include "CoreUtils/Log.h"

UVehicleWheel::UVehicleWheel()
{
    SP_LOG_CURRENT_FUNCTION();

    if (!Config::s_initialized_) {
        return;
    }
    
    // we need to set the torque combine method to override for external torque 
    // values to be used to drive and brake the vehicle
    ExternalTorqueCombineMethod = ETorqueCombineMethod::Override;

    bAffectedByHandbrake = Config::get<bool>("WHEELED_VEHICLE.VEHICLE_WHEEL.AFFECTED_BY_HANDBRAKE");

    MaxBrakeTorque     = Config::get<float>("WHEELED_VEHICLE.VEHICLE_WHEEL.MAX_BRAKE_TORQUE");
    MaxHandBrakeTorque = Config::get<float>("WHEELED_VEHICLE.VEHICLE_WHEEL.MAX_HAND_BRAKE_TORQUE");

    WheelMass   = Config::get<float>("WHEELED_VEHICLE.VEHICLE_WHEEL.WHEEL_MASS");
    WheelRadius = Config::get<float>("WHEELED_VEHICLE.VEHICLE_WHEEL.WHEEL_RADIUS");
    WheelWidth  = Config::get<float>("WHEELED_VEHICLE.VEHICLE_WHEEL.WHEEL_WIDTH");

    SuspensionMaxRaise     = Config::get<float>("WHEELED_VEHICLE.VEHICLE_WHEEL.SUSPENSION_MAX_RAISE");
    SuspensionMaxDrop      = Config::get<float>("WHEELED_VEHICLE.VEHICLE_WHEEL.SUSPENSION_MAX_DROP");
    SuspensionForceOffset  = FVector(0.0, 0.0, Config::get<float>("WHEELED_VEHICLE.VEHICLE_WHEEL.SUSPENSION_FORCE_OFFSET"));
    SuspensionDampingRatio = Config::get<float>("WHEELED_VEHICLE.VEHICLE_WHEEL.SUSPENSION_DAMPING_RATIO");
 
    // check if smoothing parameter translates to natural frequency parameter from PhysX
    SuspensionSmoothing    = Config::get<int>("WHEELED_VEHICLE.VEHICLE_WHEEL.SUSPENSION_NATURAL_FREQUENCY");

    /** Wheel Lateral Skid Grip Loss, lower number less grip on skid */
    /*UPROPERTY(EditAnywhere, Category = Wheel, meta = (ClampMin = "0.0", UIMin = "0.0", ClampMax = "1.0", UIMax = "1.0"))
    float SideSlipModifier;*/

    SlipThreshold = Config::get<float>("WHEELED_VEHICLE.VEHICLE_WHEEL.LONG_STIFF_VALUE");
    SkidThreshold = Config::get<float>("WHEELED_VEHICLE.VEHICLE_WHEEL.LAT_STIFF_VALUE");
}

UVehicleWheel::~UVehicleWheel()
{
    SP_LOG_CURRENT_FUNCTION();
}
