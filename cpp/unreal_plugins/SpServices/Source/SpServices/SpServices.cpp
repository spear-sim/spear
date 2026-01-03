//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpServices/SpServices.h"

#include <exception>
#include <memory> // std::make_unique

#include <CoreGlobals.h>           // IsRunningCommandlet
#include <Modules/ModuleManager.h> // FDefaultGameModuleImpl, FDefaultModuleImpl, IMPLEMENT_GAME_MODULE, IMPLEMENT_MODULE

#include "SpCore/Assert.h"
#include "SpCore/AssertModuleLoaded.h"
#include "SpCore/Log.h"

#include "SpServices/Service.h"

// RpcService
#include "SpServices/RpcService.h"

// EngineService
#include "SpServices/EngineService.h"

// Services that don't require a reference to EngineService
#include "SpServices/InitializeEngineService.h"

// Services that require a reference to EngineService
#include "SpServices/EngineGlobalsService.h"
#include "SpServices/EnhancedInputService.h"
#include "SpServices/InitializeEditorWorldService.h"
#include "SpServices/InitializeGameWorldService.h"
#include "SpServices/InputService.h"
#include "SpServices/SharedMemoryService.h"
#include "SpServices/UnrealService.h"

// Services that require a reference to EngineService and SharedMemoryService
#include "SpServices/NavigationService.h"
#include "SpServices/SpFuncService.h"

void SpServices::StartupModule()
{
    SP_ASSERT_MODULE_LOADED("SpCore");
    SP_LOG_CURRENT_FUNCTION();

    // If we're cooking, then return early. In this case, there is no need to launch our services, and if
    // if we attempt to launch the RPC server while cooking, and the editor or game is already open, then we
    // will get an error because the port is in use.
    #if WITH_EDITOR // defined in an auto-generated header
        if (IsRunningCommandlet()) {
            return;
        }
    #endif

    // Create world filters.
    editor_world_filter_ = std::make_unique<Service::EditorWorldFilter>();
    game_world_filter_ = std::make_unique<Service::GameWorldFilter>();

    // Create the RPC server. The RPC server won't be launched until beginFrame() to give other services a
    // chance to bind their entry points.
    rpc_service_ = std::make_unique<RpcService>();

    // EngineService needs its own custom logic for binding its entry points, because they are intended to
    // run directly on the RPC server worker thread, whereas most other entry points are intended to run on
    // work queues maintained by EngineService. So we pass in the RPC server when constructing EngineService,
    // and we pass in EngineService when constructing all other services that need to bind entry points. We
    // need to call engine_service_->startup() explicitly.
    engine_service_ = std::make_unique<EngineService<rpc::server>>(rpc_service_->rpc_server_.get());
    engine_service_->startup();

    // Construct services that don't require a reference to EngineService.
    initialize_engine_service_ = std::make_unique<InitializeEngineService>();

    // Construct services that require a reference to EngineService.
    engine_globals_service_ = std::make_unique<EngineGlobalsService>(engine_service_.get());
    enhanced_input_service_ = std::make_unique<EnhancedInputService>(engine_service_.get());
    input_service_ = std::make_unique<InputService>(engine_service_.get());
    shared_memory_service_ = std::make_unique<SharedMemoryService>(engine_service_.get());

    // Construct services that require a reference to EngineService and SharedMemoryService.
    sp_func_service_ = std::make_unique<SpFuncService>(engine_service_.get(), shared_memory_service_.get());

    // Create editor world services.
    initialize_editor_world_service_ = std::make_unique<InitializeEditorWorldService>(engine_service_.get(), editor_world_filter_.get());
    editor_unreal_service_ = std::make_unique<UnrealService>(engine_service_.get(), editor_world_filter_.get());
    editor_navigation_service_ = std::make_unique<NavigationService>(engine_service_.get(), shared_memory_service_.get(), editor_world_filter_.get());

    // Create game world services.
    initialize_game_world_service_ = std::make_unique<InitializeGameWorldService>(engine_service_.get(), game_world_filter_.get());
    game_unreal_service_ = std::make_unique<UnrealService>(engine_service_.get(), game_world_filter_.get());
    game_navigation_service_ = std::make_unique<NavigationService>(engine_service_.get(), shared_memory_service_.get(), game_world_filter_.get());
}

void SpServices::ShutdownModule()
{
    SP_LOG_CURRENT_FUNCTION();

    #if WITH_EDITOR // defined in an auto-generated header
        if (IsRunningCommandlet()) {
            return;
        }
    #endif

    // The logic in EngineService guarantees that no other RPC server functions can ever get called after the
    // engine_service.terminate entry point gets called, which should have already happened at this point. So
    // it is ok to terminate our other services before terminating RpcService. This guarantee is necessary to
    // enable other plugins to implement their own services.

    SP_ASSERT(initialize_game_world_service_);
    SP_ASSERT(game_unreal_service_);
    SP_ASSERT(game_navigation_service_);
    initialize_game_world_service_ = nullptr;
    game_unreal_service_ = nullptr;
    game_navigation_service_ = nullptr;

    SP_ASSERT(initialize_editor_world_service_);
    initialize_editor_world_service_ = nullptr;
    editor_unreal_service_ = nullptr;
    editor_navigation_service_ = nullptr;

    SP_ASSERT(sp_func_service_);
    sp_func_service_ = nullptr;

    SP_ASSERT(engine_globals_service_);
    SP_ASSERT(enhanced_input_service_);
    SP_ASSERT(input_service_);
    SP_ASSERT(shared_memory_service_);
    engine_globals_service_ = nullptr;
    enhanced_input_service_ = nullptr;
    input_service_ = nullptr;
    shared_memory_service_ = nullptr;

    SP_ASSERT(initialize_engine_service_);
    initialize_engine_service_ = nullptr;

    // We need to call engine_service_->shutdown() before shutting down the RPC server, so engine_service_
    // has a chance to stop waiting on any futures that it might be waiting on. It will not be possible to
    // shut down the RPC server gracefully if a live entry point is waiting on a future.
    SP_ASSERT(engine_service_);
    engine_service_->shutdown();
    engine_service_ = nullptr;

    SP_ASSERT(rpc_service_);
    rpc_service_ = nullptr;

    SP_ASSERT(editor_world_filter_);
    SP_ASSERT(game_world_filter_);
    editor_world_filter_ = nullptr;
    game_world_filter_ = nullptr;
}

// use IMPLEMENT_GAME_MODULE if module implements Unreal classes, use IMPLEMENT_MODULE otherwise
IMPLEMENT_GAME_MODULE(SpServices, SpServices);
