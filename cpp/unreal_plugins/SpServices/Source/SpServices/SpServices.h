//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <memory> // std::unique_ptr

#include <Delegates/IDelegateInstance.h> // FDelegateHandle
#include <HAL/Platform.h>                // SPSERVICES_API
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
class UnrealService_0;
class UnrealService_1;
class WorldRegistryService;

// Services that require a reference to EngineService and SharedMemoryService
class NavigationService;
class SpFuncService;

class SPSERVICES_API SpServices : public IModuleInterface // SPSERVICES_API is needed here because the SpServicesEditor module needs to link against this class
{
public:
    void StartupModule() override;
    void ShutdownModule() override;

    // SpServicesEditor also needs engine_service_ (and our other services) to be valid, but we can't guarantee
    // whether SpServices's or SpServicesEditor's OnPostEngineInit handler will run first. So SpServicesEditor
    // also calls requestInitialize() before accessing engine_service_, and whichever handler runs first is the
    // one that actually performs the initialization.

    void requestInitialize();
    void terminate();
    bool isInitialized() const;

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
    std::unique_ptr<UnrealService_0> unreal_service_0_;
    std::unique_ptr<UnrealService_1> unreal_service_1_;
    std::unique_ptr<WorldRegistryService> world_registry_service_;

    // Services that require a reference to EngineService and SharedMemoryService
    std::unique_ptr<NavigationService> navigation_service_;
    std::unique_ptr<SpFuncService> sp_func_service_;

private:
    void postEngineInitHandler();
    void enginePreExitHandler();

    void registerClasses() const;
    void unregisterClasses() const;

    FDelegateHandle post_engine_init_handle_;
    FDelegateHandle engine_pre_exit_handle_;

    bool initialized_ = false;
};
