#include "OpenBotWheel.h"

#include "Config.h"

UOpenBotWheel::UOpenBotWheel()
{
    bAffectedByHandbrake = Config::getValue<bool>({"OPENBOT", "WHEEL", "AFFECTED_BY_HANDBRAKE"});
    ShapeRadius = Config::getValue<float>({"OPENBOT", "WHEEL", "SHAPE_RADIUS"});
    ShapeWidth = Config::getValue<float>({"OPENBOT", "WHEEL", "SHAPE_WIDTH"});
    SteerAngle = Config::getValue<float>({"OPENBOT", "WHEEL", "STEER_ANGLE"});
    SuspensionMaxRaise = Config::getValue<float>({"OPENBOT", "WHEEL", "SUSPENSION_MAX_RAISE"});
    SuspensionMaxDrop = Config::getValue<float>({"OPENBOT", "WHEEL", "SUSPENSION_MAX_DROP"});
}
