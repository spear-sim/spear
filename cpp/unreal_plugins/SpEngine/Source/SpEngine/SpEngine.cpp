//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpEngine/SpEngine.h"

#include <memory> // std::make_unique, std::unique_ptr

#include <Modules/ModuleManager.h> // IMPLEMENT_MODULE

#include "CoreUtils/Assert.h"
#include "CoreUtils/Config.h"
#include "CoreUtils/Log.h"
#include "CoreUtils/Unreal.h"
#include "SpEngine/EngineService.h"
#include "SpEngine/GameWorldService.h"

// We would like to decouple the following entities as much as possible: the RPC server, the
// EngineService and its various work queues, and all other services whose entry points are
// intended to run on those work queues. We achieve this design goal through compile-time
// polymorphism. Our EngineService takes as input any class that defines a public templated
// bind(func_name, func) method, as represented by the CBasicServiceEntryPointBinder concept.
// All other services take as input any class that defines a public templated
// bind(service_name, func_name, func) method, as represented by the CServiceEntryPointBinder
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
    SP_ASSERT(FModuleManager::Get().IsModuleLoaded(Unreal::toFName("CoreUtils")));
    SP_ASSERT(FModuleManager::Get().IsModuleLoaded(Unreal::toFName("SimulationController")));

    if (!Config::s_initialized_) {
        return;
    }

    // We use a shared_ptr and not unique_ptr because we do not want the ownership of the rpc::server object to move to the EngineService class.
    // By not moving ownership to EngineService class, lifecycle of the rpc::server object depends on both EngineService class and SpEngine class.
    // (as both these classes would refer to the same object), and not on the EngineService class alone.
    // This is required as we want to use the rpc::server object (we call it's various member functions) after creating EngineService class.
    rpc_server_ = std::make_shared<rpc::server>(Config::get<std::string>("SIMULATION_CONTROLLER.IP"), Config::get<int>("SIMULATION_CONTROLLER.PORT"));
    SP_ASSERT(rpc_server_);

    // EngineService needs its own custom logic for binding its entry points, because they are
    // intended to run directly on the RPC server worker thread, whereas all other entry points
    // are intended to run on work queues maintained by EngineService. So we pass in the server
    // when constructing EngineService, and we pass in EngineService when constructing all other
    // services.
    engine_service_ = EngineService<rpc::server>(rpc_server_);
    game_world_service_ = GameWorldService<EngineService<rpc::server>>(&engine_service_);

    //int num_worker_threads = 1;
    //rpc_server_->async_run(num_worker_threads);
}

void SpEngine::ShutdownModule()
{
    SP_LOG_CURRENT_FUNCTION();

    if (!Config::s_initialized_) {
        return;
    }

    //rpc_server_->close_sessions();
    //rpc_server_->stop();
}

IMPLEMENT_MODULE(SpEngine, SpEngine)
