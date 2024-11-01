//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpServices/SpServices.h"

#include <exception>
#include <memory> // std::make_unique

#include <CoreGlobals.h>           // IsRunningCommandlet
#include <Modules/ModuleManager.h> // IMPLEMENT_MODULE

#include "SpCore/Assert.h"
#include "SpCore/AssertModuleLoaded.h"
#include "SpCore/Config.h"
#include "SpCore/Log.h"

#include "SpServices/Rpclib.h"

#include "SpServices/EngineService.h"
#include "SpServices/EnhancedInputService.h"
#include "SpServices/GameMapSettingsService.h"
#include "SpServices/LegacyService.h"
#include "SpServices/SpFuncService.h"
#include "SpServices/UnrealService.h"

void SpServices::StartupModule()
{
    SP_ASSERT_MODULE_LOADED("SpCore");
    SP_ASSERT_MODULE_LOADED("SpComponents");
    SP_ASSERT_MODULE_LOADED("UrdfRobot");
    SP_ASSERT_MODULE_LOADED("Vehicle");
    SP_LOG_CURRENT_FUNCTION();

    // If we're cooking, then return early. In this case, there is no need to launch our services, and if
    // if we attempt to launch the RPC server while cooking, and the editor or game is already open, then we
    // will get an error because the port is in use.
    #if WITH_EDITOR // defined in an auto-generated header
        if (IsRunningCommandlet()) {
            return;
        }
    #endif

    // Create RPC server.
    try {
        int rpc_server_port = 30000;
        if (Config::isInitialized()) {
            rpc_server_port = Config::get<int>("SP_SERVICES.RPC_SERVER_PORT");
        }        
        rpc_server_ = std::make_unique<rpc::server>(rpc_server_port);
        SP_ASSERT(rpc_server_);
        int num_worker_threads = 1;
        rpc_server_->async_run(num_worker_threads);

    } catch (...) {
        SP_LOG("ERROR: Couldn't launch RPC server. There might be another SPEAR executable running in the background. Check for other SPEAR executables, close them, and relaunch.");
        std::rethrow_exception(std::current_exception());
    }

    // EngineService needs its own custom logic for binding its entry points, because they are intended to
    // run directly on the RPC server worker thread, whereas most other entry points are intended to run on
    // work queues maintained by EngineService. So we pass in the RPC server when constructing EngineService,
    // and we pass in EngineService when constructing all other services.
    engine_service_ = std::make_unique<EngineService<rpc::server>>(rpc_server_.get());

    // Construct all other services by passing in EngineService.
    enhanced_input_service_ = std::make_unique<EnhancedInputService>(engine_service_.get());
    game_map_settings_service_ = std::make_unique<GameMapSettingsService>(engine_service_.get());
    legacy_service_ = std::make_unique<LegacyService>(engine_service_.get());
    sp_func_service_ = std::make_unique<SpFuncService>(engine_service_.get());
    unreal_service_ = std::make_unique<UnrealService>(engine_service_.get());
}

void SpServices::ShutdownModule()
{
    SP_LOG_CURRENT_FUNCTION();

    #if WITH_EDITOR // defined in an auto-generated header
        if (IsRunningCommandlet()) {
            return;
        }
    #endif

    // We need to call engine_service_->close() before shutting down the RPC server, so engine_service_ has a
    // chance to stop waiting on any futures that it might be waiting on. It will not be possible to shut
    // down the RPC server gracefully if a live entry point is waiting on a future.
    SP_ASSERT(engine_service_);
    engine_service_->close();

    // We need to shut down the RPC server before destroying our services. Otherwise, the RPC server might
    // attempt to call a service's entry points after the service has been destroyed. Many of our services
    // bind entry points that capture a pointer back to the service, so we need to make sure that the RPC
    // server is destroyed before our services.
    SP_ASSERT(rpc_server_);
    rpc_server_->close_sessions();
    rpc_server_->stop();
    rpc_server_ = nullptr;

    SP_ASSERT(unreal_service_);
    SP_ASSERT(sp_func_service_);
    SP_ASSERT(legacy_service_);
    SP_ASSERT(enhanced_input_service_);
    unreal_service_ = nullptr;
    sp_func_service_ = nullptr;
    legacy_service_ = nullptr;
    game_map_settings_service_ = nullptr;
    enhanced_input_service_ = nullptr;

    SP_ASSERT(engine_service_);
    engine_service_ = nullptr;
}

// use if module does not implement any Unreal classes
// IMPLEMENT_MODULE(SpComponents, SpComponents);

// use if module implements any Unreal classes
IMPLEMENT_GAME_MODULE(SpServices, SpServices);
