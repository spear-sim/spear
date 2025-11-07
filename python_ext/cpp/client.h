//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // int64_t, uint16_t, uint64_t

#include <iostream>
#include <map>
#include <memory>      // std::make_unique, std::unique_ptr
#include <ranges>      // std::views::transform
#include <string>
#include <type_traits> // std::is_void_v
#include <utility>     // std::make_pair, std::move
#include <vector>

#include <nanobind/nanobind.h>

#include <rpc/client.h>

#include "assert.h"
#include "func_signature_registry.h"
#include "types.h"

class Client
{
public:
    Client() = delete;
    Client(const std::string& address, const uint16_t& port)
    {
        std::cout << "[SPEAR | spear_ext.cpp] Client::Client(...)" << std::endl;
        client_ = std::make_unique<rpc::client>(address, port);
    };

    ~Client()
    {
        std::cout << "[SPEAR | spear_ext.cpp] Client::~Client()" << std::endl;
        client_ = nullptr;
    };

    void initialize(const std::string& address, const uint16_t& port)
    {
        std::cout << "[SPEAR | spear_ext.cpp] Client::initialize(...)" << std::endl;
        SP_ASSERT(!client_);
        client_ = std::make_unique<rpc::client>(address, port);
    };

    void terminate()
    {
        std::cout << "[SPEAR | spear_ext.cpp] Client::terminate()" << std::endl;
        SP_ASSERT(client_);
        client_ = nullptr;
    };

    int64_t getTimeout() const
    {
        SP_ASSERT(client_);
        nonstd::optional<int64_t> timeout = client_->get_timeout();
        if (timeout.has_value()) {
            return *timeout;
        } else {
            return -1;
        }
    };

    void setTimeout(int64_t value)
    {
        SP_ASSERT(client_);
        client_->set_timeout(value);
    };

    void clearTimeout()
    {
        SP_ASSERT(client_);
        client_->clear_timeout();
    };

    template <typename TReturn, typename... TArgs>
    TReturn callSync(const std::string& func_name, TArgs... args)
    {
        if (verbose_rpc_calls_) { std::cout << "[SPEAR | spear_ext.cpp] callSync<...>(...): " << func_name << std::endl; }

        return executeFuncInTryCatch(func_name, [this, &func_name, &args...]() -> TReturn {
            clmdep_msgpack::object_handle object_handle = client_->call(func_name, args...);
            return FuncSignatureRegistry::getReturnValue<TReturn>(this, std::move(object_handle));
        });
    };

    template <typename... TArgs>
    Future callAsync(const std::string& func_name, TArgs... args)
    {
        if (verbose_rpc_calls_) { std::cout << "[SPEAR | spear_ext.cpp] callAsync<...>(...): " << func_name << std::endl; }

        return executeFuncInTryCatch(func_name, [this, &func_name, &args...]() -> Future {
            clmdep_msgpack::object_handle object_handle = client_->call(func_name, args...);
            return FuncSignatureRegistry::getReturnValue<Future>(this, std::move(object_handle));
        });
    };

    template <typename... TArgs>
    void sendAsync(const std::string& func_name, TArgs... args)
    {
        if (verbose_rpc_calls_) { std::cout << "[SPEAR | spear_ext.cpp] sendAsync<...>(...): " << func_name << std::endl; }

        executeFuncInTryCatch(func_name, [this, &func_name, &args...]() -> void {
            client_->call(func_name, args...);
        });
    };

    template <typename TReturn>
    TReturn getFutureResult(const Future& future)
    {
        if (verbose_rpc_calls_) { std::cout << "[SPEAR | spear_ext.cpp] getFutureResult<...>(...): " << future.future_ptr_ << std::endl; }

        return executeFuncInTryCatch("getFutureResult<...>(...)", [this, &future]() -> TReturn {
            std::string return_type_name = FuncSignatureRegistry::getFuncSignatureTypeDesc<TReturn>().type_names_.at("entry_point");
            clmdep_msgpack::object_handle object_handle = client_->call("engine_service.get_future_result_on_game_thread_as_" + return_type_name, future);
            return FuncSignatureRegistry::getReturnValue<TReturn>(this, std::move(object_handle));
        });
    };

    template <typename... TArgs>
    uint64_t callAsyncFast(const std::string& func_name, TArgs... args)
    {
        if (verbose_rpc_calls_) { std::cout << "[SPEAR | spear_ext.cpp] callAsyncFast<...>(...): " << func_name << std::endl; }

        return executeFuncInTryCatch(func_name, [this, &func_name, &args...]() -> uint64_t {
            std::future<clmdep_msgpack::object_handle> std_future = client_->async_call(func_name, args...);
            return createFutureFast(std::move(std_future));
        });
    };

    template <typename... TArgs>
    void sendAsyncFast(const std::string& func_name, TArgs... args)
    {
        if (verbose_rpc_calls_) { std::cout << "[SPEAR | spear_ext.cpp] sendAsyncFast<...>(...): " << func_name << std::endl; }

        executeFuncInTryCatch(func_name, [this, &func_name, &args...]() -> void {
            client_->send(func_name, args...);
        });
    };

    template <typename TReturn>
    TReturn getFutureResultFast(uint64_t future_handle)
    {
        if (verbose_rpc_calls_) { std::cout << "[SPEAR | spear_ext.cpp] getFutureResultFast<...>(...): " << future_handle << std::endl; }

        return destroyFutureFast<TReturn>(future_handle);
    };

    static std::vector<FuncSignatureTypeDesc> getEntryPointSignatureTypeDescs()
    {
        return FuncSignatureRegistry::getFuncSignatureTypeDescs();
    };

    static std::map<std::string, std::vector<FuncSignatureDesc>> getEntryPointSignatureDescs()
    {
        return Std::toMap<std::string, std::vector<FuncSignatureDesc>>(
            s_entry_point_signature_registries_ |
            std::views::transform([](auto& pair) {
                auto& [entry_point_registry_name, entry_point_registry] = pair;
                return std::make_pair(entry_point_registry_name, entry_point_registry.getFuncSignatureDescs());
            }));
    };

    static void initializeEntryPointSignatures()
    {
        s_entry_point_signature_registries_.clear();

        s_entry_point_signature_registries_["call_sync_on_worker_thread"]              = FuncSignatureRegistry();
        s_entry_point_signature_registries_["call_async_fast_on_worker_thread"]        = FuncSignatureRegistry();
        s_entry_point_signature_registries_["send_async_fast_on_worker_thread"]        = FuncSignatureRegistry();
        s_entry_point_signature_registries_["get_future_result_fast_on_worker_thread"] = FuncSignatureRegistry();

        s_entry_point_signature_registries_["call_sync_on_game_thread"]              = FuncSignatureRegistry();
        s_entry_point_signature_registries_["call_async_on_game_thread"]             = FuncSignatureRegistry();
        s_entry_point_signature_registries_["send_async_on_game_thread"]             = FuncSignatureRegistry();
        s_entry_point_signature_registries_["get_future_result_on_game_thread"]      = FuncSignatureRegistry();
        s_entry_point_signature_registries_["call_async_fast_on_game_thread"]        = FuncSignatureRegistry();
        s_entry_point_signature_registries_["send_async_fast_on_game_thread"]        = FuncSignatureRegistry();
        s_entry_point_signature_registries_["get_future_result_fast_on_game_thread"] = FuncSignatureRegistry();
    };

    template <typename TReturn, typename... TArgs>
    static void registerWorkerThreadEntryPointSignature(nanobind::class_<Client>& client_class)
    {
        FuncSignatureTypeDesc return_type_desc = FuncSignatureRegistry::getFuncSignatureTypeDesc<TReturn>();
        std::string return_type_name = return_type_desc.type_names_.at("entry_point");

        requestRegisterEntryPointSignature<TReturn, TArgs...>(client_class, "call_sync_on_worker_thread",              "_call_sync_on_worker_thread_as_" + return_type_name,              &Client::callSync<TReturn, TArgs...>);
        requestRegisterEntryPointSignature<Future,  TArgs...>(client_class, "call_async_fast_on_worker_thread",        "_call_async_fast_on_worker_thread",                               &Client::callAsyncFast<TArgs...>);
        requestRegisterEntryPointSignature<void,    TArgs...>(client_class, "send_async_fast_on_worker_thread",        "_send_async_fast_on_worker_thread",                               &Client::sendAsyncFast<TArgs...>);
        requestRegisterEntryPointSignature<TReturn, Future>  (client_class, "get_future_result_fast_on_worker_thread", "_get_future_result_fast_on_worker_thread_as_" + return_type_name, &Client::getFutureResultFast<TReturn>);
    };

    template <typename TReturn, typename... TArgs>
    static void registerGameThreadEntryPointSignature(nanobind::class_<Client>& client_class)
    {
        FuncSignatureTypeDesc return_type_desc = FuncSignatureRegistry::getFuncSignatureTypeDesc<TReturn>();
        std::string return_type_name = return_type_desc.type_names_.at("entry_point");

        requestRegisterEntryPointSignature<TReturn, TArgs...>(client_class, "call_sync_on_game_thread",              "_call_sync_on_game_thread_as_" + return_type_name,              &Client::callSync<TReturn, TArgs...>);
        requestRegisterEntryPointSignature<Future,  TArgs...>(client_class, "call_async_on_game_thread",             "_call_async_on_game_thread",                                    &Client::callAsync<TArgs...>);
        requestRegisterEntryPointSignature<void,    TArgs...>(client_class, "send_async_on_game_thread",             "_send_async_on_game_thread",                                    &Client::sendAsync<TArgs...>);
        requestRegisterEntryPointSignature<TReturn, Future>  (client_class, "get_future_result_on_game_thread",      "_get_future_result_on_game_thread_as_" + return_type_name,      &Client::getFutureResult<TReturn>);
        requestRegisterEntryPointSignature<Future,  TArgs...>(client_class, "call_async_fast_on_game_thread",        "_call_async_fast_on_game_thread",                               &Client::callAsyncFast<TArgs...>);
        requestRegisterEntryPointSignature<void,    TArgs...>(client_class, "send_async_fast_on_game_thread",        "_send_async_fast_on_game_thread",                               &Client::sendAsyncFast<TArgs...>);
        requestRegisterEntryPointSignature<TReturn, Future>  (client_class, "get_future_result_fast_on_game_thread", "_get_future_result_fast_on_game_thread_as_" + return_type_name, &Client::getFutureResultFast<TReturn>);
    };

    bool force_return_aligned_arrays_ = false;
    bool verbose_rpc_calls_           = false;
    bool verbose_allocations_         = false;
    bool verbose_exceptions_          = false; // owner should set to true once a connection to the server is established

private:
    template <typename TFunc>
    auto executeFuncInTryCatch(const std::string& func_name, const TFunc& func)
    {
        using TReturn = std::invoke_result_t<TFunc>;

        SP_ASSERT(client_);

        try {
            if constexpr (std::is_void_v<TReturn>) {
                if (verbose_rpc_calls_) { std::cout << "[SPEAR | spear_ext.cpp] Executing function: " << func_name << std::endl; }

                func();

                if (verbose_rpc_calls_) { std::cout << "[SPEAR | spear_ext.cpp] Finished executing function: " << func_name << std::endl; }

            } else {
                if (verbose_rpc_calls_) { std::cout << "[SPEAR | spear_ext.cpp] Executing function: " << func_name << std::endl; }

                TReturn return_value = func();

                if (verbose_rpc_calls_) { std::cout << "[SPEAR | spear_ext.cpp] Finished executing function: " << func_name << std::endl; }
                return return_value;
            }
        } catch (const std::exception& e) {
            if (verbose_exceptions_) { std::cout << "[SPEAR | spear_ext.cpp] ERROR: Caught exception when calling \"" << func_name << "\": " << e.what() << std::endl; }
            std::rethrow_exception(std::current_exception());
        } catch (...) {
            if (verbose_exceptions_) { std::cout << "[SPEAR | spear_ext.cpp] ERROR: Caught unknown exception when calling \"" << func_name << "\"." << std::endl; }
            std::rethrow_exception(std::current_exception());
        }

        return TReturn();
    };

    uint64_t createFutureFast(std::future<clmdep_msgpack::object_handle>&& std_future)
    {
        std::future<clmdep_msgpack::object_handle>* std_future_ptr = new std::future<clmdep_msgpack::object_handle>(std::move(std_future));
        SP_ASSERT(std_future_ptr);
        if (verbose_rpc_calls_) { std::cout << "[SPEAR | spear_ext.cpp] Allocated std::future<clmdep_msgpack::object_handle> object at memory location: " << std_future_ptr << std::endl; }
        uint64_t future_handle = reinterpret_cast<uint64_t>(std_future_ptr);
        return future_handle;
    }

    template <typename TReturn>
    TReturn destroyFutureFast(uint64_t future_handle)
    {
        SP_ASSERT(future_handle);
        std::future<clmdep_msgpack::object_handle>* std_future_ptr = reinterpret_cast<std::future<clmdep_msgpack::object_handle>*>(future_handle);
        if (verbose_rpc_calls_) { std::cout << "[SPEAR | spear_ext.cpp] Deleting std::future<clmdep_msgpack::object_handle> object at memory location: " << std_future_ptr << std::endl; }
        clmdep_msgpack::object_handle object_handle = std_future_ptr->get();
        if (verbose_rpc_calls_) { std::cout << "[SPEAR | spear_ext.cpp] Obtained clmdep_msgpack::object_handle from std::future<clmdep_msgpack::object_handle>..." << std::endl; }
        delete std_future_ptr;
        std_future_ptr = nullptr;

        Future future = object_handle.template as<Future>();
        if (verbose_rpc_calls_) { std::cout << "[SPEAR | spear_ext.cpp] Obtained Future from clmdep_msgpack::object_handle." << std::endl; }
        std::string return_type_name = FuncSignatureRegistry::getFuncSignatureTypeDesc<TReturn>().type_names_.at("entry_point");
        std::string func_name = "engine_service.get_future_result_as_" + return_type_name;
        if (verbose_rpc_calls_) { std::cout << "[SPEAR | spear_ext.cpp] Preparing to call function to obtain future result: " << func_name << std::endl; }
        return callSync<TReturn>(func_name, future);
    }

    template <typename TReturn, typename... TArgs, typename TFunc>
    static void requestRegisterEntryPointSignature(nanobind::class_<Client>& client_class, const std::string& entry_point_registry_name, const std::string& func_name, TFunc&& func)
    {
        FuncSignatureRegistry& entry_point_registry = s_entry_point_signature_registries_.at(entry_point_registry_name);

        if (!entry_point_registry.isFuncSignatureRegistered<TReturn, TArgs...>(func_name)) {
            entry_point_registry.registerFuncSignature<TReturn, TArgs...>(func_name);
            client_class.def(func_name.c_str(), std::forward<TFunc>(func));
        }
    };

    std::unique_ptr<rpc::client> client_ = nullptr;

    inline static std::map<std::string, FuncSignatureRegistry> s_entry_point_signature_registries_;
};
