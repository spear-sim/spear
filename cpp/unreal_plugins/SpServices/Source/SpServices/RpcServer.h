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

#include "SpCore/Assert.h"
#include "SpCore/FuncRegistry.h"

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
        server_.bind(name, func);
        bindImpl(name, func, FuncInfoUtils::getFuncInfo<TFunc>());
    }

private:
    template <typename TFunc, typename TReturn, typename... TArgs>
    void bindImpl(const std::string& name, const TFunc& func, const FuncInfo<TReturn, TArgs...>& fi)
    {
        funcs_.registerFunc(name, [func](const clmdep_msgpack::object& args) -> clmdep_msgpack::object_handle {
            std::tuple<std::remove_cvref_t<TArgs>...> args_unpacked_tuple;
            args.convert(args_unpacked_tuple);
            if constexpr (std::is_void_v<TReturn>) {
                std::apply(func, args_unpacked_tuple);
                return clmdep_msgpack::object_handle();
            } else {
                TReturn return_value = std::apply(func, args_unpacked_tuple);
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
