//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // uint16_t

#include <memory>      // std::make_unique
#include <string>
#include <tuple>       // std::apply, std::tuple
#include <type_traits> // std::is_void_v, std::remove_cvref_t
#include <utility>     // std::forward

#include "SpCore/Assert.h"
#include "SpCore/FuncRegistry.h"
#include "SpCore/SharedMemory.h"

#include "SpServices/FuncInfo.h"
#include "SpServices/MsgpackRpc.h"

//
// RpcServer wraps an rpc::server and provides the same interface (bind, closeSessions, stop, asyncRun),
// but routes all bound functions through a single generic RPC entry point called "call". This enables
// clients to call any bound function dynamically by name, without needing to register per-signature
// entry points on the client side.
//
// The bind method has the same template signature as rpc::server::bind, so it satisfies CEntryPointBinder.
//

class RpcServer
{
public:
    RpcServer(uint16_t port) : server_(port)
    {
        server_.bind("rpc_server.call", [this](const std::string& func_name, const clmdep_msgpack::object& args) -> clmdep_msgpack::object {
            object_handle_ = funcs_.call(func_name, args);
            return object_handle_.get(); // need to keep the zone alive until rpclib serializes each response
        });
    }

    template <typename TFunc>
    void bind(const std::string& name, const TFunc& func)
    {
        bindImpl(name, func, FuncInfoUtils::getFuncInfo<TFunc>());
    }

private:
    template <typename TFunc, typename TReturn, typename... TArgs>
    void bindImpl(const std::string& name, const TFunc& func, const FuncInfo<TReturn, TArgs...>& fi)
    {
        // We bind each input function in two ways. First, we bind the function with rpclib, so it is directly
        // callable by name. We use this method for our Python msgpackrpc client (see
        // spear.utils.editor_utils.Client), and for a small number of initialization functions that are called
        // directly by our C++ rpclib client (see python_ext/cpp/client.h) and can't rely on our second binding
        // method. In our second binding method, we bind the function with our own function registry, so it is
        // callable through our generic "rpc_server.call" dispatcher. We use the generic "rpc_server.call"
        // dispatcher for nearly all functions called by our C++ rpclib client. We need this generic dispatcher
        // because rpclib's client-side call(...) function is a variadic template, so it requires the number and
        // types of arguments to be known at compile time. But our C++ rpclib client only knows the arguments at
        // runtime (i.e., when they are passed in from Python). So our client packs the arguments into a single
        // msgpack array, and calls "rpc_server.call" with a fixed signature (the function name and the array),
        // and the dispatcher unpacks the array into the function's actual argument types. If we didn't have
        // this second binding method, we would need to use templates to declare all possible entry point
        // signatures in the C++ client. It is a significant maintenance effort to keep track of all of the
        // signatures, so we rely as much as possible on our generic dispatcher.

        // For both binding methods, we call SharedMemory::fence() before and after calling the input function.
        // Together with corresponding fences on the client, the fences here establish that the client side of a
        // function call synchronizes-with the server side of the call. As consequences of this guarantee, all
        // shared memory (see SharedMemoryFence::fence() for details):
        //   - reads  that happen-before a function call in user code are guaranteed to be unaffected by any writes that happen inside the function call on the server
        //   - writes that happen-before a function call in user code are guaranteed to be visible to    any reads  that happen inside the function call on the server
        //   - reads  that happen-after  a function call in user code are guaranteed to see              any writes that happen inside the function call on the server
        //   - writes that happen-after  a function call in user code are guaranteed not to affect       any reads  that happen inside the function call on the server

        // We fence unconditionally for both binding methods, because any client in any process is allowed to
        // call either code path, and any function call is allowed to participate in a contract that transfers
        // exclusive access to a shared memory region between user code and server code. For example, an
        // "enqueue_read_pixels" function call might transfer exclusive access to the server, and then a
        // corresponding "read_pixels" function might transfer exclusive access back to user code. Our decision
        // to fence unconditionally is overly conservative in most cases, because most functions don't
        // participate in any such shared memory contract. But it's also not very expensive compared to the
        // overall cost of an RPC call. Additionally, declaring that individual entry points need a fence before
        // or after they execute is easy to get wrong, it makes user Python code more complicated, and there
        // would need to be an extra registration mechanism for UFUNCTIONS. So we adopt the simple conservative
        // unconditional strategy.

        server_.bind(name, [func](TArgs... args) -> TReturn {
            if constexpr (std::is_void_v<TReturn>) {
                SharedMemory::fence();
                func(std::forward<TArgs>(args)...);
                SharedMemory::fence();
            } else {
                SharedMemory::fence();
                TReturn return_value = func(std::forward<TArgs>(args)...);
                SharedMemory::fence();
                return return_value;
            }
        });

        funcs_.registerFunc(name, [func](const clmdep_msgpack::object& args) -> clmdep_msgpack::object_handle {
            std::tuple<std::remove_cvref_t<TArgs>...> args_unpacked_tuple;
            args.convert(args_unpacked_tuple);
            if constexpr (std::is_void_v<TReturn>) {
                SharedMemory::fence();
                std::apply(func, args_unpacked_tuple);
                SharedMemory::fence();
                return clmdep_msgpack::object_handle();
            } else {
                SharedMemory::fence();
                TReturn return_value = std::apply(func, args_unpacked_tuple);
                SharedMemory::fence();
                std::unique_ptr<clmdep_msgpack::zone> return_value_zone = std::make_unique<clmdep_msgpack::zone>();
                clmdep_msgpack::object return_value_object(return_value, *return_value_zone);
                return clmdep_msgpack::object_handle(return_value_object, std::move(return_value_zone));
            }
        });
    }

public:
    void asyncRun() { server_.async_run(1); } // we intentionally only support a single RPC worker thread
    void stop() { server_.stop(); }
    void closeSessions() { server_.close_sessions(); }

private:
    rpc::server server_;
    FuncRegistry<clmdep_msgpack::object_handle, const clmdep_msgpack::object&> funcs_;
    clmdep_msgpack::object_handle object_handle_; // need to keep the zone alive until rpclib serializes each response
};
