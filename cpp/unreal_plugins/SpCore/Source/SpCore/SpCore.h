//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <memory> // std::unique_ptr
#include <string>

#include <Modules/ModuleInterface.h>
#include <Containers/UnrealString.h>     // FString>
#include <Delegates/IDelegateInstance.h> // FDelegateHandle

#include "SpCore/SharedMemory.h"

class SpCore : public IModuleInterface
{
public:
    void StartupModule() override;
    void ShutdownModule() override;

private:
    void requestWaitForKeyboardInput() const;

    void initializeIniConfigs() const;
    void initializeIniConfig(const FString& ini_config_filename, const std::string& ini_config_name, const std::string& sp_config_key) const;

    void registerClasses() const;
    void unregisterClasses() const;

    void postEngineInitHandler();
    void enginePreExitHandler();

    FDelegateHandle post_engine_init_handle_;
    FDelegateHandle engine_pre_exit_handle_;

    std::unique_ptr<SharedMemoryRegion> shared_memory_region_;
};
