//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "UrdfBot.h"

#include "Assert/Assert.h"

void UrdfBot::StartupModule()
{
    ASSERT(FModuleManager::Get().IsModuleLoaded(TEXT("CoreUtils")));
}

void UrdfBot::ShutdownModule() {}

IMPLEMENT_MODULE(UrdfBot, UrdfBot)
