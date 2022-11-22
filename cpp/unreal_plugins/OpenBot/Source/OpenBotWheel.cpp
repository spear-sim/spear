#include "OpenBotWheel.h"

#include "CompilerWarningUtils.h"
#include "Config.h"

BEGIN_IGNORE_COMPILER_WARNINGS
UOpenBotWheel::UOpenBotWheel()
{
    bAffectedByHandbrake = Config::getValue<bool>({"OPENBOT", "OPENBOT_WHEEL", "AFFECTED_BY_HANDBRAKE"});
    ShapeRadius          = Config::getValue<float>({"OPENBOT", "OPENBOT_WHEEL", "SHAPE_RADIUS"});
    ShapeWidth           = Config::getValue<float>({"OPENBOT", "OPENBOT_WHEEL", "SHAPE_WIDTH"});
    SteerAngle           = Config::getValue<float>({"OPENBOT", "OPENBOT_WHEEL", "STEER_ANGLE"});
    SuspensionMaxRaise   = Config::getValue<float>({"OPENBOT", "OPENBOT_WHEEL", "SUSPENSION_MAX_RAISE"});
    SuspensionMaxDrop    = Config::getValue<float>({"OPENBOT", "OPENBOT_WHEEL", "SUSPENSION_MAX_DROP"});
    Mass                 = Config::getValue<float>({"OPENBOT", "OPENBOT_WHEEL", "MASS"});
}
END_IGNORE_COMPILER_WARNINGS
