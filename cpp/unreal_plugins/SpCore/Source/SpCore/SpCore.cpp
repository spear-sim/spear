//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpCore/SpCore.h"

#include <stdint.h> // uint64_t

#include <iostream> // std::cin
#include <memory>   // std::make_unique, std::unique_ptr

#include <CoreGlobals.h>                 // GConfig, GEditorIni, GEngineIni, GGameIni, GGameUserSettingsIni, GInputIni
#include <Delegates/IDelegateInstance.h> // FDelegateHandle
#include <Misc/ConfigCacheIni.h>         // GConfig
#include <Misc/CoreDelegates.h>
#include <Modules/ModuleManager.h>       // FDefaultGameModuleImpl, FDefaultModuleImpl, IMPLEMENT_GAME_MODULE, IMPLEMENT_MODULE

// Unreal classes to register
#include <AssetRegistry/IAssetRegistry.h>
#include <Engine/LocalPlayer.h>
#include <Engine/World.h>

#include "SpCore/Config.h"
#include "SpCore/Log.h"
#include "SpCore/SharedMemory.h"
#include "SpCore/Unreal.h"
#include "SpCore/UnrealClassRegistry.h"

void SpCore::StartupModule()
{
    SP_LOG_CURRENT_FUNCTION();

    Config::requestInitialize();

    requestWaitForKeyboardInput(); // no need to undo
    initializeIniConfigs();        // no need to undo
    registerClasses();

    post_engine_init_handle_ = FCoreDelegates::OnPostEngineInit.AddRaw(this, &SpCore::postEngineInitHandler);
    engine_pre_exit_handle_  = FCoreDelegates::OnEnginePreExit.AddRaw(this, &SpCore::enginePreExitHandler);

    SP_LOG_CURRENT_FUNCTION();
}

void SpCore::ShutdownModule()
{
    SP_LOG_CURRENT_FUNCTION();

    FCoreDelegates::OnEnginePreExit.Remove(engine_pre_exit_handle_);
    FCoreDelegates::OnPostEngineInit.Remove(post_engine_init_handle_);
    engine_pre_exit_handle_.Reset();
    post_engine_init_handle_.Reset();

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

void SpCore::initializeIniConfigs() const
{
    if (Config::isInitialized()) {
        initializeIniConfig(GEditorIni,           "GEditorIni",           "SP_CORE.EDITOR_INI_CONFIG_VALUES");
        initializeIniConfig(GEngineIni,           "GEngineIni",           "SP_CORE.ENGINE_INI_CONFIG_VALUES");
        initializeIniConfig(GGameIni,             "GGameIni",             "SP_CORE.GAME_INI_CONFIG_VALUES");
        initializeIniConfig(GGameUserSettingsIni, "GGameUserSettingsIni", "SP_CORE.GAME_USER_SETTINGS_INI_CONFIG_VALUES");
        initializeIniConfig(GInputIni,            "GInputIni",            "SP_CORE.INPUT_INI_CONFIG_VALUES");
    }
}

void SpCore::initializeIniConfig(const FString& ini_config_filename, const std::string& ini_config_name, const std::string& sp_config_key) const
{
    SP_ASSERT(Config::isInitialized());
    std::map<std::string, std::map<std::string, std::string>> ini_config_values = Config::get<std::map<std::string, std::map<std::string, std::string>>>(sp_config_key);
    for (auto& [ini_config_section_name, ini_config_section_values] : ini_config_values) {
        for (auto& [ini_config_key, ini_config_value] : ini_config_section_values) {
            SP_LOG("Setting ", ini_config_name, " (", Unreal::toStdString(ini_config_filename), ") value: [", ini_config_section_name, "] ", ini_config_key, " = ", ini_config_value);
            GConfig->SetString(Unreal::toTCharPtr(ini_config_section_name), Unreal::toTCharPtr(ini_config_key), Unreal::toTCharPtr(ini_config_value), ini_config_filename);
        }
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

void SpCore::postEngineInitHandler()
{
    SP_LOG_CURRENT_FUNCTION();

    uint64_t shared_memory_initial_unique_id = 0;
    if (Config::isInitialized()) {
        shared_memory_initial_unique_id = Config::get<unsigned int>("SP_CORE.SHARED_MEMORY_INITIAL_UNIQUE_ID");
    }
    SharedMemory::initialize(shared_memory_initial_unique_id);

    // Try to create a shared memory region so we can provide a meaningful error message if it fails.
    try {
        uint64_t num_bytes = 1;
        shared_memory_region_ = std::make_unique<SharedMemoryRegion>(num_bytes);
    } catch (...) {
        SP_LOG("    ERROR: Couldn't create a shared memory region. The Unreal Editor might be open already, or there might be another SpearSim executable running in the background. Close the Unreal Editor and other SpearSim executables, or change SP_CORE.SHARED_MEMORY_INITIAL_UNIQUE_ID to an unused ID, and try launching again.");
        std::rethrow_exception(std::current_exception());
    }
}

void SpCore::enginePreExitHandler()
{
    SP_LOG_CURRENT_FUNCTION();

    shared_memory_region_ = nullptr;
    SharedMemory::terminate();
}

// use IMPLEMENT_GAME_MODULE if module implements Unreal classes, use IMPLEMENT_MODULE otherwise
IMPLEMENT_GAME_MODULE(SpCore, SpCore);
