//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "UrdfRobot/UrdfRobot.h"

#include <Modules/ModuleManager.h> // IMPLEMENT_MODULE

#include "CoreUtils/Assert.h"
#include "CoreUtils/Log.h"

#include "UrdfRobot/UrdfParser.h"

void UrdfRobot::StartupModule()
{
    SP_LOG_CURRENT_FUNCTION();
    SP_ASSERT(FModuleManager::Get().IsModuleLoaded(TEXT("CoreUtils")));

    UrdfRobotDesc robot_desc = UrdfParser::parse("/Users/mroberts/code/github/spear/python/spear/urdf/arm.xml");

    SP_LOG(robot_desc.name_);
}

void UrdfRobot::ShutdownModule()
{
    SP_LOG_CURRENT_FUNCTION();
}

IMPLEMENT_MODULE(UrdfRobot, UrdfRobot)
