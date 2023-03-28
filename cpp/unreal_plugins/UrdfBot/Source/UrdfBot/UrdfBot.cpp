//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "UrdfBot/UrdfBot.h"

#include <iostream>

#include "CoreUtils/Assert.h"

void UrdfBot::StartupModule()
{
    std::cout << "[SPEAR | UrdfBot.cpp] UrdfBot::StartupModule" << std::endl;

    ASSERT(FModuleManager::Get().IsModuleLoaded(TEXT("CoreUtils")));
}

void UrdfBot::ShutdownModule()
{
    std::cout << "[SPEAR | UrdfBot.cpp] UrdfBot::ShutdownModule" << std::endl;    
}

IMPLEMENT_MODULE(UrdfBot, UrdfBot)
