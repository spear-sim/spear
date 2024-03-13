//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpEngine/SpEngine.h"

#include <memory>                  // std::make_unique, std::unique_ptr

#include <Modules/ModuleManager.h> // IMPLEMENT_MODULE

#include "SpCore/Assert.h"
#include "SpCore/Config.h"
#include "SpCore/Log.h"
#include "SpCore/Rpclib.h"
#include "SpCore/Unreal.h"
#include "SpEngine/EngineService.h"
#include "SpEngine/GameWorldService.h"
#include "SpEngine/LegacyService.h"

// We would like to decouple the following entities as much as possible: the RPC server, the
// EngineService and its various work queues, and all other services whose entry points are
// intended to run on those work queues. We achieve this design goal through compile-time
// polymorphism. Our EngineService takes as input any class that defines a public templated
// bind(func_name, func) method, as represented by the CBasicEntryPointBinder concept.
// All other services take as input any class that defines a public templated
// bind(service_name, func_name, func) method, as represented by the CEntryPointBinder
// concept.

// This design makes it so the RPC server doesn't need any direct knowledge of our services (it
// just provides a public bind method), our EngineService doesn't need any direct knowledge of
// the RPC server (EngineService just binds to whatever CBasicEntryPointBinder is passed in), and
// our other services don't need any direct knowledge of the EngineService (our other services
// just bind to whatever CEntryPointBinder is passed in). We need compile-time polymorphism
// because templated member functions cannot be virtual, so it is not practical to, e.g., define
// a base class with a virtual bind(...) method.

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
