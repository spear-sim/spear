//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <string>

#include <Containers/UnrealString.h>     // FString
#include <Delegates/IDelegateInstance.h> // FDelegateHandle
#include <Modules/ModuleInterface.h>

class SpCore : public IModuleInterface
{
public:
    void StartupModule() override;
    void ShutdownModule() override;

private:
    void postEngineInitHandler();
    void enginePreExitHandler();

    void requestWaitForKeyboardInput() const;

    void requestInitializeIniConfigs() const;
    void requestInitializeIniConfig(const FString& ini_config_filename, const std::string& ini_config_name, const std::string& sp_config_override_key, const std::string& sp_config_string_key) const;

    #if WITH_EDITOR // defined in an auto-generated header
        void requestInitializePathTracingShaderExtension() const;
    #endif

    void registerClasses() const;
    void unregisterClasses() const;

    void initializeSharedMemory();
    void terminateSharedMemory();

    FDelegateHandle post_engine_init_handle_;
    FDelegateHandle engine_pre_exit_handle_;
};
