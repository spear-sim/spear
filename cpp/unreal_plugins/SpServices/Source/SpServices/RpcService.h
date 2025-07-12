//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <exception> // std::current_exception, std::rethrow_exception
#include <memory>    // std::unique_ptr

#include "SpServices/Rpclib.h"
#include "SpServices/Service.h"

class RpcService : public Service {
public:
    RpcService() : Service("RpcService")
    {
        SP_LOG_CURRENT_FUNCTION();
        SP_LOG("    Creating RPC server...");

        // Create RPC server, allocate port, but don't launch yet. We defer launching the server until the
        // first beginFrame() to give all other services a chance to bind their entry points. As long as all
        // services bind their entry points before the first beginFrame(), then all entry points on all
        // services will be available as soon as an RPC client connects.

        int rpc_server_port = 30000;
        if (Config::isInitialized()) {
            rpc_server_port = Config::get<int>("SP_SERVICES.RPC_SERVICE.RPC_SERVER_PORT");
        }

        try {
            rpc_server = std::make_unique<rpc::server>(rpc_server_port);
            SP_ASSERT(rpc_server);
        } catch (...) {
            SP_LOG("    ERROR: Couldn't create an RPC server. The Unreal Editor might be open already, or there might be another SpearSim executable running in the background. Close the Unreal Editor and other SpearSim executables, or change SP_SERVICES.RPC_SERVICE.RPC_SERVER_PORT to a different unused port, and try launching again.");
            std::rethrow_exception(std::current_exception());
        }
    }

    ~RpcService() override
    {
        SP_LOG_CURRENT_FUNCTION();
        SP_LOG("    Destroying RPC server...");

        // Destroy the RPC server. This should be done before destroying our other services. Otherwise, the
        // RPC server might attempt to call a service's entry points after the service has been destroyed.
        // Many of our services bind entry points that capture a pointer back to the service itself, so we
        // need to make sure that these entry points are not called after the services that bound them have
        // been destroyed.
        SP_ASSERT(rpc_server);
        rpc_server->close_sessions();
        rpc_server->stop();
        rpc_server = nullptr;
    }

    std::unique_ptr<rpc::server> rpc_server = nullptr;

protected:
    void beginFrame() override
    {
        Service::beginFrame();

        // Launch RPC server.
        if (!initialized_) {
            SP_LOG_CURRENT_FUNCTION();
            SP_LOG("    Launching RPC server...");
            int num_worker_threads = 1;
            rpc_server->async_run(num_worker_threads);
            initialized_ = true;
        }
    }

private:
    bool initialized_ = false;
};
