//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "UrdfRobot/UrdfRobot.h"

#include <Modules/ModuleManager.h> // IMPLEMENT_MODULE

#include "SpCore/Assert.h"
#include "SpCore/Log.h"
#include "SpCore/Unreal.h"

void UrdfRobot::StartupModule()
{
    SP_LOG_CURRENT_FUNCTION();
    SP_ASSERT(FModuleManager::Get().IsModuleLoaded(Unreal::toFName("SpCore")));
}

void UrdfRobot::ShutdownModule()
{
    SP_LOG_CURRENT_FUNCTION();
}

IMPLEMENT_MODULE(UrdfRobot, UrdfRobot)
