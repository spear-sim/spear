//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpServices/SpServices.h"

#include <exception>
#include <memory> // std::make_unique

#include <CoreGlobals.h>           // IsRunningCommandlet
#include <Misc/CoreDelegates.h>    // FCoreDelegates
#include <Modules/ModuleManager.h> // FDefaultGameModuleImpl, FDefaultModuleImpl, IMPLEMENT_GAME_MODULE, IMPLEMENT_MODULE

#include "SpCore/Assert.h"
#include "SpCore/AssertModuleLoaded.h"
#include "SpCore/Log.h"

#include "SpServices/MsgpackRpc.h"

// RpcServer
#include "SpServices/RpcServer.h"

// RpcService
#include "SpServices/RpcService.h"

// EngineService
#include "SpServices/EngineService.h"

// Services that don't require a reference to EngineService
#include "SpServices/InitializeEngineService.h"

// Services that require a reference to EngineService
#include "SpServices/DebugService.h"
#include "SpServices/EngineGlobalsService.h"
#include "SpServices/EnhancedInputService.h"
#include "SpServices/InputService.h"
#include "SpServices/SharedMemoryService.h"
// #include "SpServices/UnrealService.h" must only be included in cpp files, and must be included after all other headers.
#include "SpServices/WorldRegistryService.h"

// Services that require a reference to EngineService and SharedMemoryService
#include "SpServices/NavigationService.h"
#include "SpServices/SpFuncService.h"

// UnrealService.h must only be included in cpp files, and must be included after all other headers.
#include "SpServices/UnrealService.h"

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

    post_engine_init_handle_ = FCoreDelegates::OnPostEngineInit.AddRaw(this, &SpServices::postEngineInitHandler);
    engine_pre_exit_handle_  = FCoreDelegates::OnEnginePreExit.AddRaw(this, &SpServices::enginePreExitHandler);
}

void SpServices::ShutdownModule()
{
    SP_LOG_CURRENT_FUNCTION();

    #if WITH_EDITOR // defined in an auto-generated header
        if (IsRunningCommandlet()) {
            return;
        }
    #endif

    FCoreDelegates::OnEnginePreExit.Remove(engine_pre_exit_handle_);
    FCoreDelegates::OnPostEngineInit.Remove(post_engine_init_handle_);
    engine_pre_exit_handle_.Reset();
    post_engine_init_handle_.Reset();
}

void SpServices::postEngineInitHandler()
{
    SP_LOG_CURRENT_FUNCTION();

    // This can't happen in StartupModule() because if we re-open a SPEAR project in the editor, then
    // StartupModule() will get called again before the first ShutdownModule() gets called.
    requestInitialize();
}

void SpServices::enginePreExitHandler()
{
    SP_LOG_CURRENT_FUNCTION();

    // See above.
    terminate();
}

void SpServices::requestInitialize()
{
    SP_LOG_CURRENT_FUNCTION();

    if (initialized_) {
        return;
    }

    // Create the RPC server. The RPC server won't be launched until beginFrame() to give other services a
    // chance to bind their entry points.
    rpc_service_ = RpcService::create();

    // EngineService needs its own custom logic for binding its entry points, because they are intended to
    // run directly on the RPC server worker thread, whereas most other entry points are intended to run on
    // work queues maintained by EngineService. So we pass in the RPC server when constructing EngineService,
    // and we pass in EngineService when constructing all other services that need to bind entry points. We
    // need to call engine_service_->startup() explicitly.
    engine_service_ = EngineService<RpcServer>::create(rpc_service_->rpc_server_.get());
    engine_service_->startup();

    // Construct services that don't require a reference to EngineService.
    initialize_engine_service_ = InitializeEngineService::create();

    // Construct services that require a reference to EngineService.
    debug_service_ = DebugService::create(engine_service_.get());
    engine_globals_service_ = EngineGlobalsService::create(engine_service_.get());
    enhanced_input_service_ = EnhancedInputService::create(engine_service_.get());
    input_service_ = InputService::create(engine_service_.get());
    shared_memory_service_ = SharedMemoryService::create(engine_service_.get());
    unreal_service_ = UnrealService::create(engine_service_.get());
    world_registry_service_ = WorldRegistryService::create(engine_service_.get());

    // Construct services that require a reference to EngineService and SharedMemoryService.
    navigation_service_ = NavigationService::create(engine_service_.get(), shared_memory_service_.get());
    sp_func_service_ = SpFuncService::create(engine_service_.get(), shared_memory_service_.get());

    initialized_ = true;
}

void SpServices::terminate()
{
    SP_LOG_CURRENT_FUNCTION();

    if (!initialized_) {
        return;
    }

    // The logic in EngineService guarantees that no other RPC server functions can ever get called after the
    // engine_service.terminate entry point gets called, which should have already happened at this point. So
    // it is ok to terminate our other services before terminating RpcService. This guarantee is necessary to
    // enable other plugins to implement their own services.

    SP_ASSERT(sp_func_service_);
    SP_ASSERT(navigation_service_);
    sp_func_service_ = nullptr;
    navigation_service_ = nullptr;

    SP_ASSERT(debug_service_);
    SP_ASSERT(engine_globals_service_);
    SP_ASSERT(enhanced_input_service_);
    SP_ASSERT(input_service_);
    SP_ASSERT(shared_memory_service_);
    SP_ASSERT(unreal_service_);
    SP_ASSERT(world_registry_service_);
    debug_service_ = nullptr;
    engine_globals_service_ = nullptr;
    enhanced_input_service_ = nullptr;
    input_service_ = nullptr;
    shared_memory_service_ = nullptr;
    unreal_service_ = nullptr;
    world_registry_service_ = nullptr;

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

    initialized_ = false;
}

bool SpServices::isInitialized() const
{
    return initialized_;
}

// use IMPLEMENT_GAME_MODULE if module implements Unreal classes, use IMPLEMENT_MODULE otherwise
IMPLEMENT_GAME_MODULE(SpServices, SpServices);
