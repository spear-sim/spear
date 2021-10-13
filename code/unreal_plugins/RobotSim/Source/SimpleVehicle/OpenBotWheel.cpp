#include "OpenBotWheel.h"

PRAGMA_DISABLE_DEPRECATION_WARNINGS

UOpenBotWheel::UOpenBotWheel()
{
    ShapeRadius = 4.0f;
    ShapeWidth = 2.0f;
    bAffectedByHandbrake = true;
    SteerAngle = 0.f;

	//small suspension for better stability
    SuspensionMaxRaise = 0.5f;
    SuspensionMaxDrop = 0.5f;
}

PRAGMA_ENABLE_DEPRECATION_WARNINGS
