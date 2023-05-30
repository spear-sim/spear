//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "UrdfBot/UrdfBot.h"

#include "CoreUtils/Assert.h"
#include "CoreUtils/Log.h"

void UrdfBot::StartupModule()
{
    SP_LOG_CURRENT_FUNCTION();
    
    SP_ASSERT(FModuleManager::Get().IsModuleLoaded(TEXT("CoreUtils")));
}

void UrdfBot::ShutdownModule()
{
    SP_LOG_CURRENT_FUNCTION();
}

IMPLEMENT_MODULE(UrdfBot, UrdfBot)
