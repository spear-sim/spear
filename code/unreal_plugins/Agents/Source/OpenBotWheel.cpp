#include "OpenBotWheel.h"

PRAGMA_DISABLE_DEPRECATION_WARNINGS

UOpenBotWheel::UOpenBotWheel()
{
    ShapeRadius = 3.3f;
    ShapeWidth = 1.5f;
    bAffectedByHandbrake = true;
    SteerAngle = 0.f;

    // Disable suspension
    SuspensionMaxRaise = 0.f;
    SuspensionMaxDrop = 0.f;
}

PRAGMA_ENABLE_DEPRECATION_WARNINGS
