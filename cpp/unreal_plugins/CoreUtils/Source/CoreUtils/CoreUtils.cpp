//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "CoreUtils/CoreUtils.h"

#include <iostream>

#include "CoreUtils/Config.h"

void CoreUtils::StartupModule()
{
    std::cout << "[SPEAR | CoreUtils.cpp] CoreUtils::StartupModule" << std::endl;

    Config::initialize();
}

void CoreUtils::ShutdownModule()
{
    std::cout << "[SPEAR | CoreUtils.cpp] CoreUtils::ShutdownModule" << std::endl;

    Config::terminate();
}

IMPLEMENT_MODULE(CoreUtils, CoreUtils)
