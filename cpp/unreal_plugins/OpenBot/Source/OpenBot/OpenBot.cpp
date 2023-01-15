//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "OpenBot/OpenBot.h"

#include <iostream>

#include "CoreUtils/Assert.h"

void OpenBot::StartupModule()
{
    std::cout << "[SPEAR | OpenBot.cpp] OpenBot::StartupModule" << std::endl;

    ASSERT(FModuleManager::Get().IsModuleLoaded(TEXT("CoreUtils")));
}

void OpenBot::ShutdownModule()
{
    std::cout << "[SPEAR | OpenBot.cpp] OpenBot::ShutdownModule" << std::endl;    
}

IMPLEMENT_MODULE(OpenBot, OpenBot)
