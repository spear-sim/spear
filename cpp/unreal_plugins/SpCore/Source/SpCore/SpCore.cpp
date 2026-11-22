//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpCore/SpCore.h"

#include <stdint.h> // uint64_t

#include <iostream> // std::cin
#include <map>
#include <string>

#include <CoreGlobals.h>                 // GConfig, GEngineIni, GGameIni, GInputIni, GScalabilityIni, etc
#include <Delegates/IDelegateInstance.h> // FDelegateHandle
#include <HAL/Platform.h>                // PLATFORM_DESKTOP
#include <HAL/PlatformMisc.h>            // FPlatformMisc
#include <Misc/ConfigCacheIni.h>         // GConfig
#include <Misc/CoreDelegates.h>          // FCoreDelegates
#include <Modules/ModuleManager.h>       // IMPLEMENT_MODULE

// Unreal classes to register
#include <AssetRegistry/IAssetRegistry.h>
#include <Engine/LocalPlayer.h>
#include <Engine/World.h>

#include "SpCore/Config.h"
#include "SpCore/Log.h"
#include "SpCore/OutputLog.h"
#include "SpCore/SharedMemory.h"
#include "SpCore/Unreal.h"
#include "SpCore/UnrealClassRegistry.h"
#include "SpCore/UnrealUtils.h"

void SpCore::StartupModule()
{
    SP_LOG_CURRENT_FUNCTION();

    Config::requestInitialize(); // do this first so we know whether or not to wait for keyboard input

    requestWaitForKeyboardInput(); // no need to undo in ShutdownModule()
    requestInitializeIniConfigs(); // no need to undo in ShutdownModule()

    registerClasses();
    OutputLog::requestInitialize();

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

    OutputLog::terminate();
    unregisterClasses();

    Config::terminate();

    SP_LOG_CURRENT_FUNCTION();
}

void SpCore::postEngineInitHandler()
{
    SP_LOG_CURRENT_FUNCTION();

    // This can't happen in StartupModule() because if we re-open a SPEAR project in the editor, then
    // StartupModule() will get called again before the first ShutdownModule() gets called.
    initializeSharedMemory(); 

    // This must happen after engine initialization so all UStructs from modules in the Default loading phase
    // (or earlier) are loaded.
    UnrealUtils::initialize();
}

void SpCore::enginePreExitHandler()
{
    SP_LOG_CURRENT_FUNCTION();

    UnrealUtils::terminate();

    // See above.
    terminateSharedMemory();
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
    // See Engine/Source/Runtime/Core/Public/CoreGlobals.h for an exhaustive list of global INI filenames.
    if (Config::isInitialized()) {
        requestInitializeIniConfig(GDeviceProfilesIni,   "GDeviceProfilesIni",    "SP_CORE.OVERRIDE_CONFIG_DEVICE_PROFILES_INI",     "SP_CORE.CONFIG_DEVICE_PROFILES_INI_STRING");
        requestInitializeIniConfig(GEngineIni,           "GEngineIni",            "SP_CORE.OVERRIDE_CONFIG_ENGINE_INI",              "SP_CORE.CONFIG_ENGINE_INI_STRING");
        requestInitializeIniConfig(GGameIni,             "GGameIni",              "SP_CORE.OVERRIDE_CONFIG_GAME_INI",                "SP_CORE.CONFIG_GAME_INI_STRING");
        requestInitializeIniConfig(GGameplayTagsIni,     "GGameplayTagsIni",      "SP_CORE.OVERRIDE_CONFIG_GAMEPLAY_TAGS_INI",       "SP_CORE.CONFIG_GAMEPLAY_TAGS_INI_STRING");
        requestInitializeIniConfig(GGameUserSettingsIni, "GGameUserSettingsIni",  "SP_CORE.OVERRIDE_CONFIG_GAME_USER_SETTINGS_INI",  "SP_CORE.CONFIG_GAME_USER_SETTINGS_INI_STRING");
        requestInitializeIniConfig(GHardwareIni,         "GHardwareIni",          "SP_CORE.OVERRIDE_CONFIG_HARDWARE_INI",            "SP_CORE.CONFIG_HARDWARE_INI_STRING");
        requestInitializeIniConfig(GInputIni,            "GInputIni",             "SP_CORE.OVERRIDE_CONFIG_INPUT_INI",               "SP_CORE.CONFIG_INPUT_INI_STRING");
        requestInitializeIniConfig(GInstallBundleIni,    "GInstallBundleIni",     "SP_CORE.OVERRIDE_CONFIG_INSTALL_BUNDLE_INI",      "SP_CORE.CONFIG_INSTALL_BUNDLE_INI_STRING");
        requestInitializeIniConfig(GRuntimeOptionsIni,   "GRuntimeOptionsIni",    "SP_CORE.OVERRIDE_CONFIG_RUNTIME_OPTIONS_INI",     "SP_CORE.CONFIG_RUNTIME_OPTIONS_INI_STRING");
        requestInitializeIniConfig(GScalabilityIni,      "GScalabilityIni",       "SP_CORE.OVERRIDE_CONFIG_SCALABILITY_INI",         "SP_CORE.CONFIG_SCALABILITY_INI_STRING");

        // These INI files are only loaded in desktop builds, see LoadRemainingConfigFiles(...) in
        // Engine/Source/Runtime/Core/Private/Misc/ConfigCacheIni.cpp.
        #if PLATFORM_DESKTOP
            requestInitializeIniConfig(GCompatIni,    "GCompatIni",    "SP_CORE.OVERRIDE_CONFIG_COMPAT_INI",    "SP_CORE.CONFIG_COMPAT_INI_STRING");
            requestInitializeIniConfig(GLightmassIni, "GLightmassIni", "SP_CORE.OVERRIDE_CONFIG_LIGHTMASS_INI", "SP_CORE.CONFIG_LIGHTMASS_INI_STRING");
        #endif

        // These INI files are only loaded in editor builds, see LoadRemainingConfigFiles(...) in
        // Engine/Source/Runtime/Core/Private/Misc/ConfigCacheIni.cpp.
        #if WITH_EDITOR // defined in an auto-generated header
            requestInitializeIniConfig(GEditorIni,            "GEditorIni",            "SP_CORE.OVERRIDE_CONFIG_EDITOR_INI",              "SP_CORE.CONFIG_EDITOR_INI_STRING");
            requestInitializeIniConfig(GEditorKeyBindingsIni, "GEditorKeyBindingsIni", "SP_CORE.OVERRIDE_CONFIG_EDITOR_KEY_BINDINGS_INI", "SP_CORE.CONFIG_EDITOR_KEY_BINDINGS_INI_STRING");
            requestInitializeIniConfig(GEditorLayoutIni,      "GEditorLayoutIni",      "SP_CORE.OVERRIDE_CONFIG_EDITOR_LAYOUT_INI",       "SP_CORE.CONFIG_EDITOR_LAYOUT_INI_STRING");
            requestInitializeIniConfig(GEditorPerProjectIni,  "GEditorPerProjectIni",  "SP_CORE.OVERRIDE_CONFIG_EDITOR_PER_PROJECT_INI",  "SP_CORE.CONFIG_EDITOR_PER_PROJECT_INI_STRING");
            requestInitializeIniConfig(GEditorSettingsIni,    "GEditorSettingsIni",    "SP_CORE.OVERRIDE_CONFIG_EDITOR_SETTINGS_INI",     "SP_CORE.CONFIG_EDITOR_SETTINGS_INI_STRING");
        #endif
    }
}

void SpCore::requestInitializeIniConfig(const FString& ini_config_filename, const std::string& ini_config_name, const std::string& sp_config_override_key, const std::string& sp_config_string_key) const
{
    SP_ASSERT(Config::isInitialized());

    if (Config::get<bool>(sp_config_override_key)) {
        std::string ini_config_string = Config::get<std::string>(sp_config_string_key);

        FConfigFile* config_file = GConfig->Find(ini_config_filename);
        SP_ASSERT(config_file);

        SP_LOG("    Overriding ", ini_config_name, " (", Unreal::toStdString(ini_config_filename), ") with the following config string:");
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
}

void SpCore::terminateSharedMemory()
{
    SharedMemory::terminate();
}

IMPLEMENT_MODULE(SpCore, SpCore);
