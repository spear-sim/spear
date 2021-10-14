#include "OpenBotWheel.h"

PRAGMA_DISABLE_DEPRECATION_WARNINGS

UOpenBotWheel::UOpenBotWheel()
{
    ShapeRadius = 3.25f;
    ShapeWidth = 1.4f;
    bAffectedByHandbrake = true;
    SteerAngle = 0.f;

	//small suspension for better stability
    SuspensionMaxRaise = 0.2f;
    SuspensionMaxDrop = 0.2f;
}

PRAGMA_ENABLE_DEPRECATION_WARNINGS
