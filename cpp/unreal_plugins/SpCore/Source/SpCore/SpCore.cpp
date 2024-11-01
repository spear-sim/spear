//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpCore/SpCore.h"

#include <iostream> // std::cin

#include <Modules/ModuleManager.h> // IMPLEMENT_GAME_MODULE, IMPLEMENT_MODULE

#include "SpCore/Config.h"
#include "SpCore/Log.h"
#include "SpCore/UnrealClassRegistrar.h"

void SpCore::StartupModule()
{
    SP_LOG_CURRENT_FUNCTION();

    Config::requestInitialize();
    UnrealClassRegistrar::initialize();

    // Wait for keyboard input, which is useful when attempting to attach a debugger to the running executable.
    if (Config::isInitialized() && Config::get<bool>("SP_CORE.WAIT_FOR_KEYBOARD_INPUT_DURING_INITIALIZATION")) {
        SP_LOG("Press any key to continue...");
        std::cin.get();
        SP_LOG("Received keyboard input, continuing...");
    }
}

void SpCore::ShutdownModule()
{
    SP_LOG_CURRENT_FUNCTION();

    UnrealClassRegistrar::terminate();
    Config::terminate();
}

// use if module does not implement any Unreal classes
// IMPLEMENT_MODULE(SpCore, SpCore);

// use if module implements any Unreal classes
IMPLEMENT_GAME_MODULE(SpCore, SpCore);
