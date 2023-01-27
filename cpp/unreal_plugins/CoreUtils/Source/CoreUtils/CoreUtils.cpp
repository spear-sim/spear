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

    // Wait for keyboard input, which is useful when attempting to attach a debugger to the running executable.
    if (Config::get<bool>("CORE_UTILS.WAIT_FOR_KEYBOARD_INPUT_DURING_INITIALIZATION")) {
        std::cout << "[SPEAR | CoreUtils.cpp] Press ENTER to continue..." << std::endl;
        std::cin.get();
    }
}

void CoreUtils::ShutdownModule()
{
    std::cout << "[SPEAR | CoreUtils.cpp] CoreUtils::ShutdownModule" << std::endl;

    Config::terminate();
}

IMPLEMENT_MODULE(CoreUtils, CoreUtils)
