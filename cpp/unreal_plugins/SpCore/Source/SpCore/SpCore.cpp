//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpCore/SpCore.h"

#include <stdint.h> // uint64_t

#include <iostream> // std::cin
#include <map>
#include <memory>   // std::make_unique, std::unique_ptr

#include <CoreGlobals.h>           // GConfig, GEditorIni, GEngineIni, GGameIni, GGameUserSettingsIni, GInputIni
#include <HAL/PlatformMisc.h>      // FPlatformMisc
#include <Misc/ConfigCacheIni.h>   // GConfig
#include <Modules/ModuleManager.h> // FDefaultGameModuleImpl, FDefaultModuleImpl, IMPLEMENT_GAME_MODULE, IMPLEMENT_MODULE

// Unreal classes to register
#include <AssetRegistry/IAssetRegistry.h>
#include <Engine/LocalPlayer.h>
#include <Engine/World.h>

#include "SpCore/Config.h"
#include "SpCore/Console.h"
#include "SpCore/Log.h"
#include "SpCore/SharedMemory.h"
#include "SpCore/Unreal.h"
#include "SpCore/UnrealClassRegistry.h"

void SpCore::StartupModule()
{
    SP_LOG_CURRENT_FUNCTION();

    Config::requestInitialize(); // do this first so we know whether or not to wait for keyboard input

    requestWaitForKeyboardInput(); // no need to undo in ShutdownModule()
    requestInitializeIniConfigs(); // no need to undo in ShutdownModule()

    registerClasses();
    initializeSharedMemory();
    Console::requestInitialize();

    SP_LOG_CURRENT_FUNCTION();
}

void SpCore::ShutdownModule()
{
    SP_LOG_CURRENT_FUNCTION();

    Console::terminate();
    terminateSharedMemory();
    unregisterClasses();

    Config::terminate();

    SP_LOG_CURRENT_FUNCTION();
}

void SpCore::requestWaitForKeyboardInput() const
{
    // Wait for keyboard input, which is useful when attempting to attach a debugger to the running executable.
    if (Config::isInitialized() && Config::get<bool>("SP_CORE.WAIT_FOR_KEYBOARD_INPUT_DURING_INITIALIZATION")) {
        SP_LOG("    Press any key to continue...");
        std::cin.get();
        SP_LOG("    Received keyboard input, continuing...");
    }
}

void SpCore::requestInitializeIniConfigs() const
{
    if (Config::isInitialized()) {
        requestInitializeIniConfig(GEditorIni,           "GEditorIni",           "SP_CORE.OVERRIDE_CONFIG_EDITOR_INI",             "SP_CORE.CONFIG_EDITOR_INI_STRING");
        requestInitializeIniConfig(GEngineIni,           "GEngineIni",           "SP_CORE.OVERRIDE_CONFIG_ENGINE_INI",             "SP_CORE.CONFIG_ENGINE_INI_STRING");
        requestInitializeIniConfig(GGameIni,             "GGameIni",             "SP_CORE.OVERRIDE_CONFIG_GAME_INI",               "SP_CORE.CONFIG_GAME_INI_STRING");
        requestInitializeIniConfig(GGameUserSettingsIni, "GGameUserSettingsIni", "SP_CORE.OVERRIDE_CONFIG_GAME_USER_SETTINGS_INI", "SP_CORE.CONFIG_GAME_USER_SETTINGS_INI_STRING");
        requestInitializeIniConfig(GInputIni,            "GInputIni",            "SP_CORE.OVERRIDE_CONFIG_INPUT_INI",              "SP_CORE.CONFIG_INPUT_INI_STRING");
    }
}

void SpCore::requestInitializeIniConfig(const FString& ini_config_filename, const std::string& ini_config_name, const std::string& sp_config_override_key, const std::string& sp_config_string_key) const
{
    SP_ASSERT(Config::isInitialized());

    if (Config::get<bool>(sp_config_override_key)) {
        std::string ini_config_string = Config::get<std::string>(sp_config_string_key);

        FConfigFile* config_file = GConfig->Find(ini_config_filename);
        SP_ASSERT(config_file);

        SP_LOG("Overriding ", ini_config_name, " (", Unreal::toStdString(ini_config_filename), ") with the following config string:");
        SP_LOG_NO_PREFIX(ini_config_string);

        // CombineFromBuffer parses the string as Unreal INI text (section headers, key=value lines, and the +/-/./!
        // array operators) and combines it onto the in-memory config, exactly as if the text had been appended to
        // the corresponding default INI file.
        config_file->CombineFromBuffer(Unreal::toFString(ini_config_string), ini_config_filename);
    }
}

// Normally we would do the operations in registerClasses() and unregisterClasses(...) in the opposite order.
// But we make an exception here (i.e., we do the operations in the same order) to make it easier and less
// error-prone to register classes.

void SpCore::registerClasses() const
{
    SP_REGISTER_SUBSYSTEM_PROVIDER_CLASS(ULocalPlayer);
    SP_REGISTER_SUBSYSTEM_PROVIDER_CLASS(UWorld);
    SP_REGISTER_INTERFACE_CLASS(IAssetRegistry);
}

void SpCore::unregisterClasses() const
{
    SP_UNREGISTER_SUBSYSTEM_PROVIDER_CLASS(ULocalPlayer);
    SP_UNREGISTER_SUBSYSTEM_PROVIDER_CLASS(UWorld);
    SP_UNREGISTER_INTERFACE_CLASS(IAssetRegistry);
}

void SpCore::initializeSharedMemory()
{
    uint64_t shared_memory_initial_unique_id = 0;
    if (Config::isInitialized()) {
        shared_memory_initial_unique_id = Config::get<unsigned int>("SP_CORE.SHARED_MEMORY_INITIAL_UNIQUE_ID");
    }
    SharedMemory::initialize(shared_memory_initial_unique_id);

    try {
        shared_memory_region_ = std::make_unique<SharedMemoryRegion>(1);
    } catch (const IdMutexError& e) {
        SP_LOG("    ERROR: Couldn't acquire shared memory region id=", e.getId(), " (id_string=", e.getIdString(), "). The Unreal Editor might be open already, or there might be another SPEAR executable running in the background. Close the Unreal Editor and other SPEAR executables, or change SP_CORE.SHARED_MEMORY_INITIAL_UNIQUE_ID to an unused ID, and try launching again.");
        SP_ASSERT(false);
    } catch (...) {
        SP_LOG("    ERROR: Unexpected error when attempting to acquire shared memory.");
        SP_ASSERT(false);
    }
}

void SpCore::terminateSharedMemory()
{
    shared_memory_region_ = nullptr;
    SharedMemory::terminate();
}

// use IMPLEMENT_GAME_MODULE if module implements Unreal classes, use IMPLEMENT_MODULE otherwise
IMPLEMENT_GAME_MODULE(SpCore, SpCore);
