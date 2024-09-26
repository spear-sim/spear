//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "UrdfRobot/UrdfRobot.h"

#include <Modules/ModuleManager.h> // IMPLEMENT_GAME_MODULE, IMPLEMENT_MODULE

#include "SpCore/AssertModuleLoaded.h"
#include "SpCore/Log.h"

void UrdfRobot::StartupModule()
{
    SP_ASSERT_MODULE_LOADED("SpCore");
    SP_LOG_CURRENT_FUNCTION();
}

void UrdfRobot::ShutdownModule()
{
    SP_LOG_CURRENT_FUNCTION();
}

// use if module does not implement any Unreal classes
// IMPLEMENT_MODULE(UrdfRobot, UrdfRobot);

// use if module implements any Unreal classes
IMPLEMENT_GAME_MODULE(UrdfRobot, UrdfRobot);
