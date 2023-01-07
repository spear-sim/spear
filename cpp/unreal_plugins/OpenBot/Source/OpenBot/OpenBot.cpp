//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "OpenBot/OpenBot.h"

#include "CoreUtils/Assert.h"

void OpenBot::StartupModule()
{
    ASSERT(FModuleManager::Get().IsModuleLoaded(TEXT("CoreUtils")));
}

void OpenBot::ShutdownModule() {}

IMPLEMENT_MODULE(OpenBot, OpenBot)
