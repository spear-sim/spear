#include "SimpleWheel.h"

PRAGMA_DISABLE_DEPRECATION_WARNINGS

USimpleWheel::USimpleWheel()
{
    // ShapeRadius = Config::GetValue<float>({"ROBOT_SIM", "WHEEL_SHAPE_RADIUS"}); // Radius of the wheel in [cm]
    // ShapeWidth = Config::GetValue<float>({"ROBOT_SIM", "WHEEL_SHAPE_WIDTH"});   // Width of the wheel in [cm]
    // bAffectedByHandbrake = true;                                                          // Whether handbrake should affect this wheel
    // SteerAngle = 0.f;                                                                     // Steer angle in degrees for this wheel
    // bAutoAdjustCollisionSize = true;

    // // Small suspension for better stability
    // SuspensionMaxRaise = Config::GetValue<float>({"ROBOT_SIM", "WHEEL_SUSPENSION_MAX_RAISE"}); // How far the wheel can go above the resting position
    // SuspensionMaxDrop = Config::GetValue<float>({"ROBOT_SIM", "WHEEL_SUSPENSION_MAX_DROP"});   // How far the wheel can drop below the resting position
    // Mass = Config::GetValue<float>({"ROBOT_SIM", "WHEEL_MASS"});                               // Mass of this wheel

    ShapeRadius = 3.3f;
    ShapeWidth = 1.5f;
    bAffectedByHandbrake = true;
    SteerAngle = 0.f;

    // Disable suspension
    SuspensionMaxRaise = 0.f;
    SuspensionMaxDrop = 0.f;
}

PRAGMA_ENABLE_DEPRECATION_WARNINGS
