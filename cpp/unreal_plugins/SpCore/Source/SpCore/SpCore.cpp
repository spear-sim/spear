//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpCore/SpCore.h"

#include <iostream> // std::cin

#include <Modules/ModuleManager.h> // IMPLEMENT_MODULE

#include "SpCore/Config.h"
#include "SpCore/Log.h"

void SpCore::StartupModule()
{
    SP_LOG_CURRENT_FUNCTION();

    Config::requestInitialize();

    // If the config system is not initialized, i.e., if the -config_file command-line argument is not passed in,
    // then there are no more initialization steps that we can do, so we return. 
    if (!Config::isInitialized()) {
        return;
    }

    // Wait for keyboard input, which is useful when attempting to attach a debugger to the running executable.
    if (Config::get<bool>("SP_CORE.WAIT_FOR_KEYBOARD_INPUT_DURING_INITIALIZATION")) {
        SP_LOG("Press ENTER to continue...");
        std::cin.get();
    }
}

void SpCore::ShutdownModule()
{
    SP_LOG_CURRENT_FUNCTION();

    Config::terminate();
}

IMPLEMENT_MODULE(SpCore, SpCore)
