//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // uint16_t

#include <limits> // std::numeric_limits
#include <memory> // std::unique_ptr

#include <boost/predef.h> // BOOST_OS_LINUX, BOOST_OS_MACOS

#if BOOST_OS_MACOS || BOOST_OS_LINUX
    #include <fcntl.h>      // FD_CLOEXEC, F_GETFD, F_SETFD, fcntl
    #include <netinet/in.h> // AF_INET, ntohs, sockaddr_in
    #include <sys/socket.h> // getsockname, sockaddr, socklen_t
    #include <sys/stat.h>   // S_ISSOCK, fstat, stat
    #include <unistd.h>     // _SC_OPEN_MAX, sysconf
#endif

#include "SpCore/Config.h"

#include "SpServices/RpcServer.h"
#include "SpServices/Service.h"

class RpcService : public Service
{
public:
    RpcService()
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

        SP_ASSERT(rpc_server_port >= 0 && rpc_server_port <= std::numeric_limits<uint16_t>::max());

        try {
            rpc_server_ = std::make_unique<RpcServer>(static_cast<uint16_t>(rpc_server_port));
            SP_ASSERT(rpc_server_);
            preventChildProcessFromInheritingPortFileDescriptor(rpc_server_port);
        } catch (...) {
            SP_LOG("    ERROR: Couldn't create an RPC server. The Unreal Editor might be open already, or there might be another SPEAR executable running in the background. Close the Unreal Editor and other SPEAR executables, or change SP_SERVICES.RPC_SERVICE.RPC_SERVER_PORT to a different unused port, and try launching again.");
            SP_ASSERT(false);
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
        SP_ASSERT(rpc_server_);
        rpc_server_->closeSessions();
        rpc_server_->stop();
        rpc_server_ = nullptr;
    }

    static std::unique_ptr<RpcService> create();

    std::unique_ptr<RpcServer> rpc_server_;

protected:
    void beginFrame() override
    {
        Service::beginFrame();

        // Launch RPC server.
        if (!initialized_) {
            SP_LOG_CURRENT_FUNCTION();
            SP_LOG("    Launching RPC server...");
            rpc_server_->asyncRun();
            initialized_ = true;
        }
    }

private:
    void preventChildProcessFromInheritingPortFileDescriptor(int port)
    {
        // rpclib opens its listening socket without FD_CLOEXEC, so if this process later spawns a child
        // via posix_spawn/fork+exec (e.g., Unreal Editor's "Switch Project" flow spawns a new editor
        // process while this one is still alive), the child inherits a live duplicate of our socket, and
        // the OS keeps the port bound even after we shut down cleanly. Find the fd bound to our port and
        // mark only that one close-on-exec.
        #if BOOST_OS_MACOS || BOOST_OS_LINUX
            long max_fd = sysconf(_SC_OPEN_MAX);
            SP_LOG("    sysconf reports the maximum file descriptor ID is ", max_fd, ".");
            SP_ASSERT(max_fd > 0 && max_fd <= 2*1000*1000); // larges value observed on Linux was roughly 1,000,000
            SP_LOG("    Attempting to find file descriptor that matches port ", port, ", searching IDs [3, ", max_fd, "]...");

            bool found = false;
            for (int fd = 3; fd < max_fd; fd++) {
                struct stat statbuf;
                if (fstat(fd, &statbuf) != 0 || !S_ISSOCK(statbuf.st_mode)) {
                    continue;
                }

                struct sockaddr_in addr;
                socklen_t addr_len = sizeof(addr);
                if (getsockname(fd, reinterpret_cast<struct sockaddr*>(&addr), &addr_len) != 0) {
                    continue;
                }

                if (addr.sin_family == AF_INET && ntohs(addr.sin_port) == static_cast<uint16_t>(port)) {
                    SP_LOG("    Found file descriptor ", fd, ".");
                    found = true;

                    int flags = fcntl(fd, F_GETFD);
                    if (flags != -1 && !(flags & FD_CLOEXEC)) {
                        SP_LOG("    Setting FD_CLOEXEC flag on file descriptor ", fd, ".");
                        fcntl(fd, F_SETFD, flags | FD_CLOEXEC);
                    } else {
                        SP_LOG("    File descriptor ", fd, " already has FD_CLOEXEC flag set.");
                    }

                    break; // found our listening socket; nothing else to check
                }
            }

            if (!found) {
                SP_LOG("    Couldn't find file descriptor.");
                SP_ASSERT(false);
            }
        #endif
    }

    bool initialized_ = false;
};
