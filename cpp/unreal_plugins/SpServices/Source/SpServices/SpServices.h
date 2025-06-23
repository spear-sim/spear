//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <memory> // std::unique_ptr

#include <Modules/ModuleInterface.h>

#include "SpServices/Rpclib.h"
#include "SpServices/Service.h"

// EngineService
#include "SpServices/EngineService.h"

// Services that don't require a reference to EngineService
#include "SpServices/InitializeEngineService.h"

// Services that require a reference to EngineService
#include "SpServices/EnhancedInputService.h"
#include "SpServices/InitializeEditorWorldService.h"
#include "SpServices/InitializeGameWorldService.h"
#include "SpServices/InputService.h"
#include "SpServices/SharedMemoryService.h"
#include "SpServices/UnrealService.h"

// Services that require a reference to EngineService and SharedMemoryService
#include "SpServices/NavigationService.h"
#include "SpServices/SpFuncService.h"

class SpServices : public IModuleInterface
{
public:
    void StartupModule() override;
    void ShutdownModule() override;

    // World filters
    std::unique_ptr<Service::WorldFilter> editor_world_filter = nullptr;
    std::unique_ptr<Service::WorldFilter> game_world_filter = nullptr;

    // EngineService
    std::unique_ptr<EngineService<rpc::server>> engine_service = nullptr;

    // Services that don't require a reference to EngineService
    std::unique_ptr<InitializeEngineService> initialize_engine_service = nullptr;

    // Services that require a reference to EngineService
    std::unique_ptr<EnhancedInputService> enhanced_input_service = nullptr;
    std::unique_ptr<InputService> input_service = nullptr;
    std::unique_ptr<SharedMemoryService> shared_memory_service = nullptr;

    // Services that require a reference to EngineService and SharedMemoryService
    std::unique_ptr<SpFuncService> sp_func_service = nullptr;

    // Editor world services
    std::unique_ptr<InitializeEditorWorldService> initialize_editor_world_service = nullptr;
    std::unique_ptr<UnrealService> editor_unreal_service = nullptr;
    std::unique_ptr<NavigationService> editor_navigation_service = nullptr;

    // Game world services
    std::unique_ptr<InitializeGameWorldService> initialize_game_world_service = nullptr;
    std::unique_ptr<UnrealService> game_unreal_service = nullptr;
    std::unique_ptr<NavigationService> game_navigation_service = nullptr;

private:
    void registerClasses();
    void unregisterClasses();

    std::unique_ptr<rpc::server> rpc_server_ = nullptr;
};
