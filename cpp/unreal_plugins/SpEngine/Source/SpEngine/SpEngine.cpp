//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpEngine/SpEngine.h"

#include <memory>                  // std::make_unique, std::unique_ptr

#include <Modules/ModuleManager.h> // IMPLEMENT_MODULE

#include "SpCore/Assert.h"
#include "SpCore/Config.h"
#include "SpCore/Log.h"
#include "SpCore/Rpclib.h" // rpc_server
#include "SpCore/Unreal.h"
#include "SpEngine/EngineService.h"
#include "SpEngine/GameWorldService.h"
#include "SpEngine/LegacyService.h"

void SpEngine::StartupModule()
{
    SP_LOG_CURRENT_FUNCTION();
    SP_ASSERT(FModuleManager::Get().IsModuleLoaded(Unreal::toFName("SpCore")));
    SP_ASSERT(FModuleManager::Get().IsModuleLoaded(Unreal::toFName("UrdfRobot")));
    SP_ASSERT(FModuleManager::Get().IsModuleLoaded(Unreal::toFName("Vehicle")));

    if (Config::s_initialized_) {
        rpc_server_ = std::make_unique<rpc::server>(Config::get<int>("SP_ENGINE.PORT"));
    } else {
        rpc_server_ = std::make_unique<rpc::server>(30000);
    }
    SP_ASSERT(rpc_server_);

    // bind functions here to be consistent with the usage of these functions on the python side
    rpc_server_->bind("sp_engine.ping", []() -> std::string {
        return "SpEngine received a call to ping()...";
    });

    rpc_server_->bind("sp_engine.request_close", []() -> void {
        bool immediate_shutdown = false;
        FGenericPlatformMisc::RequestExit(immediate_shutdown);
    });

    // EngineService needs its own custom logic for binding its entry points, because they are
    // intended to run directly on the RPC server worker thread, whereas all other entry points
    // are intended to run on work queues maintained by EngineService. So we pass in the server
    // when constructing EngineService, and we pass in EngineService when constructing all other
    // services.
    engine_service_ = std::make_unique<EngineService<rpc::server>>(rpc_server_.get());
    game_world_service_ = std::make_unique<GameWorldService>(engine_service_.get());
    legacy_service_ = std::make_unique<LegacyService>(engine_service_.get());

    int num_worker_threads = 1;
    rpc_server_->async_run(num_worker_threads);
}

void SpEngine::ShutdownModule()
{
    SP_LOG_CURRENT_FUNCTION();

    rpc_server_->close_sessions();
    rpc_server_->stop();
}

IMPLEMENT_MODULE(SpEngine, SpEngine)
