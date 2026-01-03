//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpCore/SpCore.h"

#include <stdint.h> // uint64_t

#include <iostream> // std::cin
#include <memory>   // std::make_unique, std::unique_ptr

#include <Modules/ModuleManager.h> // FDefaultGameModuleImpl, FDefaultModuleImpl, IMPLEMENT_GAME_MODULE, IMPLEMENT_MODULE

// Unreal classes to register
#include <AssetRegistry/IAssetRegistry.h>
#include <Engine/LocalPlayer.h>

#include "SpCore/Config.h"
#include "SpCore/Log.h"
#include "SpCore/SharedMemory.h"
#include "SpCore/UnrealClassRegistry.h"

void SpCore::StartupModule()
{
    SP_LOG_CURRENT_FUNCTION();

    Config::requestInitialize();

    // Wait for keyboard input, which is useful when attempting to attach a debugger to the running executable.
    if (Config::isInitialized() && Config::get<bool>("SP_CORE.WAIT_FOR_KEYBOARD_INPUT_DURING_INITIALIZATION")) {
        SP_LOG("    Press any key to continue...");
        std::cin.get();
        SP_LOG("    Received keyboard input, continuing...");
    }

    // Initialize shared memory system
    uint64_t shared_memory_initial_unique_id = 0;
    if (Config::isInitialized()) {
        shared_memory_initial_unique_id = Config::get<unsigned int>("SP_CORE.SHARED_MEMORY_INITIAL_UNIQUE_ID");
    }
    SharedMemory::initialize(shared_memory_initial_unique_id);

    // Register Unreal classes to be accessible from Python
    registerClasses();

    // Try to create a shared memory region so we can provide a meaningful error message if it fails.
    try {
        uint64_t num_bytes = 1;
        shared_memory_region_ = std::make_unique<SharedMemoryRegion>(num_bytes);
    } catch (...) {
        SP_LOG("    ERROR: Couldn't create a shared memory region. The Unreal Editor might be open already, or there might be another SpearSim executable running in the background. Close the Unreal Editor and other SpearSim executables, or change SP_CORE.SHARED_MEMORY_INITIAL_UNIQUE_ID to an unused ID, and try launching again.");
        std::rethrow_exception(std::current_exception());
    }

    SP_LOG_CURRENT_FUNCTION(); // useful to see that StartupModule() finishes executing
}

void SpCore::ShutdownModule()
{
    SP_LOG_CURRENT_FUNCTION();

    shared_memory_region_ = nullptr;

    unregisterClasses();
    SharedMemory::terminate();
    Config::terminate();

    SP_LOG_CURRENT_FUNCTION();  // useful to see that ShutdownModule() finishes executing
}

// Normally we would do the operations in registerClasses() and unregisterClasses(...) in the opposite order.
// But we make an exception here (i.e., we do the operations in the same order) to make it easier and less
// error-prone to register classes.

void SpCore::registerClasses() const
{
    SP_REGISTER_SUBSYSTEM_PROVIDER_CLASS(ULocalPlayer);
    SP_REGISTER_INTERFACE_CLASS(IAssetRegistry);
}

void SpCore::unregisterClasses() const
{
    SP_UNREGISTER_SUBSYSTEM_PROVIDER_CLASS(ULocalPlayer);
    SP_UNREGISTER_INTERFACE_CLASS(IAssetRegistry);
}

// use IMPLEMENT_GAME_MODULE if module implements Unreal classes, use IMPLEMENT_MODULE otherwise
IMPLEMENT_GAME_MODULE(SpCore, SpCore);
