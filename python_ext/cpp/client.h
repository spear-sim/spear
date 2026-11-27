//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stddef.h> // size_t
#include <stdint.h> // int64_t, uint16_t, uint32_t

#include <algorithm> // std::max
#include <chrono>    // std::chrono::duration
#include <condition_variable>
#include <cstdlib>   // std::_Exit
#include <exception> // std::exception
#include <map>
#include <memory>    // std::make_unique, std::unique_ptr
#include <mutex>     // std::lock_guard, std::unique_lock
#include <string>
#include <thread>
#include <utility>   // std::move
#include <vector>

#include <nanobind/nanobind.h>

#include <rpc/nonstd/optional.hpp> // nonstd::optional

#include "SpCore/Assert.h"
#include "SpCore/Boost.h"
#include "SpCore/Log.h"

#include "SpServices/FuncSignatureRegistry.h"
#include "SpServices/MsgpackAdaptors.h"
#include "SpServices/Rpclib.h"
#include "SpServices/SpTypes.h"

#include "shared_memory.h"

class Client : public SpClientBase
{
public:
    Client() = delete;
    Client(const std::string& address, const uint16_t& port, bool suppress_default_logging)
    {
        suppress_default_logging_ = suppress_default_logging;
        client_ = std::make_unique<rpc::client>(address, port);
        SP_ASSERT(client_);
    };

    ~Client()
    {
        stopWatchdog();
        client_ = nullptr;
    };

    //
    // Lifetime management
    //

    std::string ping()
    {
        if (!suppress_default_logging_) { SP_LOG_CURRENT_FUNCTION(); }
        SP_ASSERT(client_);
        return client_->call("engine_globals_service.call_sync_on_worker_thread.ping").get().as<std::string>();
    }

    void initialize()
    {
        if (!suppress_default_logging_) { SP_LOG_CURRENT_FUNCTION(); }
        SP_ASSERT(client_);
        SP_ASSERT(!is_initialized_);

        clmdep_msgpack::object_handle object_handle;

        object_handle = client_->call("engine_service.call_sync_on_worker_thread.get_entry_point_signature_type_names");
        entry_point_signature_type_names_ = object_handle.get().as<std::vector<std::string>>();
        validateEntryPointSignatureTypeNames();

        object_handle = client_->call("engine_service.call_sync_on_worker_thread.get_entry_point_signature_descs");
        entry_point_signature_descs_ = object_handle.get().as<std::map<std::string, SpFuncSignatureDesc>>();

        object_handle = client_->call("shared_memory_service.call_sync_on_worker_thread.get_fence_view");
        SpSharedMemoryView fence_view = object_handle.get().as<SpSharedMemoryView>();

        object_handle = client_->call("shared_memory_service.call_sync_on_worker_thread.get_fence_key");
        std::string fence_key_from_rpc = object_handle.get().as<std::string>();

        // We consider the server to be local if we can open the fence region, and the fence region contains the
        // key reported by the server (see SharedMemoryFence::SharedMemoryFence(...) in SpCore/SharedMemory.cpp).
        // Opening the fence region by name alone is not sufficient, because an unrelated server running on this
        // machine might own a region with the same name. If the server is not local, we don't create a fence,
        // and Client::call(...) doesn't call fence().

        std::unique_ptr<SharedMemoryFence> shared_memory_fence;

        try {
            shared_memory_fence = std::make_unique<SharedMemoryFence>(fence_view);
            SP_ASSERT(shared_memory_fence);
        } catch (const boost::interprocess::interprocess_exception&) {
            if (!suppress_default_logging_) { SP_LOG("Couldn't open shared memory fence, assuming the server is remote..."); }
        }

        if (shared_memory_fence) {
            std::string fence_key_from_shared_memory_region = shared_memory_fence->getKey();
            if (fence_key_from_shared_memory_region == fence_key_from_rpc) {
                if (!suppress_default_logging_) { SP_LOG("Opened shared memory fence: ", fence_key_from_rpc); }
                shared_memory_fence_ = std::move(shared_memory_fence);
            } else {
                if (!suppress_default_logging_) {
                    SP_LOG("Opened shared memory fence but it belongs to a different server than the one we're connected to, assuming our server is remote...");
                    SP_LOG("Expected key from RPC call with server: ", fence_key_from_rpc);
                    SP_LOG("Actual key from shared memory region:   ", fence_key_from_shared_memory_region);
                }
            }
        }

        startWatchdog();
        is_initialized_ = true;
    };

    void requestExit(bool force)
    {
        // The server is expected to disconnect after we call request_exit, so we stop the watchdog first. We send
        // request_exit as a notification rather than a call, because the server can be destroyed before it sends
        // its response, which would cause a call to time out. Since all previous entry points were called
        // synchronously, request_exit is guaranteed to be the last entry point the server receives from us.

        if (!suppress_default_logging_) { SP_LOG_CURRENT_FUNCTION(); }
        SP_ASSERT(client_);
        SP_ASSERT(is_initialized_);
        stopWatchdog();
        is_initialized_ = false;
        client_->send("engine_globals_service.call_sync_on_worker_thread.request_exit", force);
    }

    void terminate()
    {
        if (!suppress_default_logging_) { SP_LOG_CURRENT_FUNCTION(); }
        SP_ASSERT(client_);
        stopWatchdog();
        is_initialized_ = false;
        client_ = nullptr;
        shared_memory_fence_ = nullptr;
        entry_point_signature_descs_.clear();
        entry_point_signature_type_names_.clear();
    };

    //
    // Interface for calling server entry points 
    //

    nanobind::object call(const std::string& func_name, const nanobind::args& args)
    {
        if (!suppress_default_logging_ && verbose_rpc_calls_) { SP_LOG("Calling entry point: ", func_name); }

        SP_ASSERT(client_);
        SP_ASSERT(is_initialized_);

        const SpFuncSignatureDesc& desc = entry_point_signature_descs_.at(func_name);
        int return_type_id = desc.type_ids_[0];
        SP_ASSERT(args.size() == desc.type_ids_.size() - 1);

        // pack args into a msgpack array
        clmdep_msgpack::zone zone;
        clmdep_msgpack::object args_msgpack_array;
        args_msgpack_array.type = clmdep_msgpack::type::ARRAY;
        args_msgpack_array.via.array.size = static_cast<uint32_t>(args.size());
        args_msgpack_array.via.array.ptr = static_cast<clmdep_msgpack::object*>(
            zone.allocate_align(sizeof(clmdep_msgpack::object) * args.size(), MSGPACK_ZONE_ALIGNOF(clmdep_msgpack::object)));

        for (size_t i = 0; i < args.size(); i++) {
            args_msgpack_array.via.array.ptr[i] = FuncSignatureRegistry::getArg(zone, desc.type_ids_[i + 1], args[i]);
        }

        // Call via rpclib. We call SharedMemoryFence::fence() before sending the request and after receiving
        // the response. Together with corresponding fences on the server (see RpcServer::bindImpl(...)), these
        // fences establish that all memory operations in this process before the call happen-before all memory
        // operations on the server during the call, and all memory operations on the server during the call
        // happen-before all memory operations in this process after the call, including memory operations on
        // shared memory (see SharedMemoryFence::fence() in SpCore/SharedMemory.cpp for details). If the server
        // is not local, there is no shared memory, so there is nothing to fence.

        // Every entry point is called via "rpc_server.call", so exceptions thrown by rpclib don't include
        // func_name. So we catch exceptions here, optionally log func_name, and rethrow.

        try {
            if (shared_memory_fence_) { shared_memory_fence_->fence(); }
            clmdep_msgpack::object_handle result = client_->call("rpc_server.call", func_name, args_msgpack_array);
            if (shared_memory_fence_) { shared_memory_fence_->fence(); }

            // unpack result
            return FuncSignatureRegistry::getReturnValue(this, return_type_id, std::move(result));

        } catch (const std::exception& e) {
            if (!suppress_default_logging_ && verbose_exceptions_) { SP_LOG("ERROR: Caught exception when calling \"", func_name, "\": ", e.what()); }
            throw;
        } catch (...) {
            if (!suppress_default_logging_ && verbose_exceptions_) { SP_LOG("ERROR: Caught unknown exception when calling \"", func_name, "\"."); }
            throw;
        }
    }

    //
    // Functions for initializing state in the client before calling initialize()
    //

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

    double getWatchdogSleepTimeSeconds()
    {
        std::lock_guard<std::mutex> lock(watchdog_mutex_);
        return watchdog_sleep_time_seconds_;
    };

    void setWatchdogSleepTimeSeconds(double value)
    {
        std::lock_guard<std::mutex> lock(watchdog_mutex_);
        watchdog_sleep_time_seconds_ = value;
    };

    double getWatchdogMaxTimeSeconds()
    {
        std::lock_guard<std::mutex> lock(watchdog_mutex_);
        return watchdog_max_time_seconds_;
    };

    void setWatchdogMaxTimeSeconds(double value)
    {
        std::lock_guard<std::mutex> lock(watchdog_mutex_);
        watchdog_max_time_seconds_ = value;
    };

    //
    // Functions for accessing cached state in the client after calling initialize()
    //

    bool isServerLocal() const
    {
        SP_ASSERT(is_initialized_);
        return shared_memory_fence_ != nullptr;
    }

    std::map<std::string, SpFuncSignatureDesc> getEntryPointSignatureDescs() const
    {
        SP_ASSERT(is_initialized_);
        return entry_point_signature_descs_;
    }

    std::vector<std::string> getEntryPointSignatureTypeNames() const
    {
        SP_ASSERT(is_initialized_);
        return entry_point_signature_type_names_;
    }

private:
    void validateEntryPointSignatureTypeNames() const
    {
        const std::vector<std::string>& client_type_names = FuncSignatureRegistry::getTypeNames();
        const std::vector<std::string>& server_type_names = entry_point_signature_type_names_;

        auto get_display_string_func = [](const std::vector<std::string>& names, size_t i) -> std::string {
            if (i < names.size()) {
                return "\"" + names.at(i) + "\"";
            } else {
                return "<missing>";
            }
        };

        bool success = true;
        for (size_t i = 0; i < std::max(client_type_names.size(), server_type_names.size()); i++) {
            bool has_client_name = i < client_type_names.size();
            bool has_server_name = i < server_type_names.size();
            if (!has_client_name || !has_server_name || client_type_names.at(i) != server_type_names.at(i)) {
                if (!suppress_default_logging_) { SP_LOG("ERROR: type ID/name mismatch at id=", i, " (server=", get_display_string_func(server_type_names, i), ", client=", get_display_string_func(client_type_names, i), ")"); }
                success = false;
            }
        }

        if (!success) {
            if (!suppress_default_logging_) { SP_LOG("ERROR: The spear_ext Python extension module is out of sync with the server. Rebuild both the server and the spear_ext Python extension module."); }
        }

        SP_ASSERT(success);
    }

    void startWatchdog()
    {
        SP_ASSERT(!watchdog_thread_.joinable());
        watchdog_stop_requested_ = false;
        watchdog_thread_ = std::thread([this]() -> void { watchdog(); });
    }

    // Request the watchdog thread to stop, and wake it up so it stops immediately rather than after its current
    // sleep time. The watchdog thread might not be running, e.g., if initialize() was never called, or if
    // requestExit() already stopped it.

    void stopWatchdog()
    {
        if (!watchdog_thread_.joinable()) {
            return;
        }

        {
            std::lock_guard<std::mutex> lock(watchdog_mutex_);
            watchdog_stop_requested_ = true;
        }
        watchdog_condition_variable_.notify_one();
        watchdog_thread_.join();
    }

    // Executes on the watchdog thread. Periodically checks if the server has been disconnected for longer than
    // max_time_seconds. rpclib doesn't fail in-flight calls when the server disconnects, so if the server
    // disconnects in the middle of a call, e.g., because the server process crashed or was terminated
    // externally, then the call won't return until it times out. Additionally, the Python interpreter can't
    // respond to signals (e.g., Ctrl+C) until the call returns. There is no way to un-stall an in-flight rpclib
    // call, so we exit immediately. We call std::_Exit(...) rather than std::exit(...), because std::exit(...)
    // would run static destructors on this thread while the main thread might still be blocked inside rpclib,
    // which could crash or deadlock. Neither function runs Python atexit handlers, because those only run when
    // the main thread finalizes the Python interpreter, which can't happen while it is blocked inside rpclib.

    void watchdog()
    {
        bool disconnected = false;
        std::chrono::steady_clock::time_point disconnected_time;

        std::unique_lock<std::mutex> lock(watchdog_mutex_);
        while (!watchdog_condition_variable_.wait_for(lock, std::chrono::duration<double>(watchdog_sleep_time_seconds_), [this]() -> bool { return watchdog_stop_requested_; })) {
            std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
            if (client_->get_connection_state() == rpc::client::connection_state::disconnected) {
                if (!disconnected) {
                    disconnected = true;
                    disconnected_time = now;
                }
                double seconds = std::chrono::duration<double>(now - disconnected_time).count();
                if (seconds >= watchdog_max_time_seconds_) {
                    if (!suppress_default_logging_) { SP_LOG("watchdog_thread: ERROR: The server disconnected unexpectedly. Exiting immediately..."); }
                    std::_Exit(1);
                }
            }
        }
    }

    std::unique_ptr<rpc::client> client_ = nullptr;
    bool is_initialized_ = false; // true after initialize() and before requestExit() or terminate()
    std::unique_ptr<SharedMemoryFence> shared_memory_fence_ = nullptr;
    std::map<std::string, SpFuncSignatureDesc> entry_point_signature_descs_;
    std::vector<std::string> entry_point_signature_type_names_;

    // Watchdog state.

    // The watchdog thread is started in initialize() and stopped in requestExit() or terminate(). The mutex and
    // condition variable are used to wake up the watchdog thread immediately when stopWatchdog() requests it to
    // stop. The mutex also protects the watchdog's sleep and max times, which can be set from Python while the
    // watchdog thread is running. The watchdog thread reads client_, so it must be stopped before client_ is
    // destroyed.
    std::thread watchdog_thread_;
    std::mutex watchdog_mutex_;
    std::condition_variable watchdog_condition_variable_;
    bool watchdog_stop_requested_ = false;
    double watchdog_sleep_time_seconds_ = 0.5;
    double watchdog_max_time_seconds_ = 2.0;
};
