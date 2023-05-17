//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "WheeledVehicle/WheeledVehicle.h"

#include <iostream>

#include "CoreUtils/Config.h"

void WheeledVehicle::StartupModule()
{
    std::cout << "[SPEAR | WheeledVehicle.cpp] WheeledVehicle::StartupModule" << std::endl;

    Config::initialize();

    // If the config system is not initialized, i.e., if the -config_file command-line argument is not passed in,
    // then there are no more initialization steps that we can do, so we return. 
    if (!Config::s_initialized_) {
        return;
    }

    // Wait for keyboard input, which is useful when attempting to attach a debugger to the running executable.
    if (Config::get<bool>("CORE_UTILS.WAIT_FOR_KEYBOARD_INPUT_DURING_INITIALIZATION")) {
        std::cout << "[SPEAR | WheeledVehicle.cpp] Press ENTER to continue..." << std::endl;
        std::cin.get();
    }
}

void WheeledVehicle::ShutdownModule()
{
    std::cout << "[SPEAR | WheeledVehicle.cpp] WheeledVehicle::ShutdownModule" << std::endl;

    Config::terminate();
}

IMPLEMENT_MODULE(WheeledVehicle, WheeledVehicle)
