//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <memory> // std::unique_ptr
#include <string>

#include <Containers/UnrealString.h> // FString
#include <Modules/ModuleInterface.h>

#include "SpCore/SharedMemory.h"

class SpCore : public IModuleInterface
{
public:
    void StartupModule() override;
    void ShutdownModule() override;

private:
    void requestWaitForKeyboardInput() const;

    void requestInitializeIniConfigs() const;
    void requestInitializeIniConfig(const FString& ini_config_filename, const std::string& ini_config_name, const std::string& sp_config_override_key, const std::string& sp_config_string_key) const;

    void registerClasses() const;
    void unregisterClasses() const;

    void initializeSharedMemory();
    void terminateSharedMemory();

    std::unique_ptr<SharedMemoryRegion> shared_memory_region_;
};
