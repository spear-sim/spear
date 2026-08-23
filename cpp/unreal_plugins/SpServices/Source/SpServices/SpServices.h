//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <memory> // std::unique_ptr

#include <Modules/ModuleInterface.h>

#include "SpServices/EntryPointBinder.h"
#include "SpServices/RpcServer.h"

// RpcService
class RpcService;

// EngineService
template <CEntryPointBinder TEntryPointBinder>
class EngineService;

// Services that don't require a reference to EngineService
class InitializeEngineService;

// Services that require a reference to EngineService
class DebugService;
class EngineGlobalsService;
class EnhancedInputService;
class InputService;
class SharedMemoryService;
class UnrealService;
class WorldRegistryService;

// Services that require a reference to EngineService and SharedMemoryService
class NavigationService;
class SpFuncService;

class SpServices : public IModuleInterface
{
public:
    void StartupModule() override;
    void ShutdownModule() override;

    // RpcService
    std::unique_ptr<RpcService> rpc_service_;

    // EngineService
    std::unique_ptr<EngineService<RpcServer>> engine_service_;

    // Services that don't require a reference to EngineService
    std::unique_ptr<InitializeEngineService> initialize_engine_service_;

    // Services that require a reference to EngineService
    std::unique_ptr<DebugService> debug_service_;
    std::unique_ptr<EngineGlobalsService> engine_globals_service_;
    std::unique_ptr<EnhancedInputService> enhanced_input_service_;
    std::unique_ptr<InputService> input_service_;
    std::unique_ptr<SharedMemoryService> shared_memory_service_;
    std::unique_ptr<UnrealService> unreal_service_;
    std::unique_ptr<WorldRegistryService> world_registry_service_;

    // Services that require a reference to EngineService and SharedMemoryService
    std::unique_ptr<NavigationService> navigation_service_;
    std::unique_ptr<SpFuncService> sp_func_service_;

private:
    void registerClasses() const;
    void unregisterClasses() const;
};
