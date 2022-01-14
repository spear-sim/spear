#include "SimpleWheel.h"

PRAGMA_DISABLE_DEPRECATION_WARNINGS

USimpleWheel::USimpleWheel()
{
    ShapeRadius = 10.0f;
    ShapeWidth = 5.0f;
    bAffectedByHandbrake = true;
    SteerAngle = 0.f;

    // disable suspension
    SuspensionMaxRaise = 0.f;
    SuspensionMaxDrop = 0.f;
}

PRAGMA_ENABLE_DEPRECATION_WARNINGS
