//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "CoreUtils/CoreUtils.h"

#include <iostream>

#include <Modules/ModuleManager.h>

#include "CoreUtils/Config.h"
#include "CoreUtils/Log.h"

void CoreUtils::StartupModule()
{
    SP_LOG_CURRENT_FUNCTION();

    Config::initialize();

    // If the config system is not initialized, i.e., if the -config_file command-line argument is not passed in,
    // then there are no more initialization steps that we can do, so we return. 
    if (!Config::s_initialized_) {
        return;
    }

    // Wait for keyboard input, which is useful when attempting to attach a debugger to the running executable.
    if (Config::get<bool>("CORE_UTILS.WAIT_FOR_KEYBOARD_INPUT_DURING_INITIALIZATION")) {
        SP_LOG("Press ENTER to continue...");
        std::cin.get();
    }
}

void CoreUtils::ShutdownModule()
{
    SP_LOG_CURRENT_FUNCTION();

    Config::terminate();
}

IMPLEMENT_MODULE(CoreUtils, CoreUtils)
