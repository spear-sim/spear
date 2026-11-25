//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stddef.h> // size_t
#include <stdint.h> // int64_t, uint16_t, uint32_t

#include <algorithm> // std::max
#include <exception> // std::exception
#include <iostream>
#include <map>
#include <memory>    // std::make_unique, std::unique_ptr
#include <string>
#include <utility>   // std::move
#include <vector>

#include <nanobind/nanobind.h>

#include <rpc/client.h>
#include <rpc/msgpack.hpp>         // clmdep_msgpack, MSGPACK_ZONE_ALIGNOF
#include <rpc/nonstd/optional.hpp> // nonstd::optional

#include "assert.h"
#include "boost.h"
#include "func_signature_registry.h"
#include "msgpack_adaptors.h"
#include "shared_memory.h"
#include "types.h"

class Client : public ClientBase
{
public:
    Client() = delete;
    Client(const std::string& address, const uint16_t& port, bool suppress_default_logging = false)
    {
        suppress_default_logging_ = suppress_default_logging;
        if (!suppress_default_logging_) { std::cout << "[SPEAR | client.h] Client::Client(...)" << std::endl; }
        client_ = std::make_unique<rpc::client>(address, port);
    };

    ~Client()
    {
        if (!suppress_default_logging_) { std::cout << "[SPEAR | client.h] Client::~Client()" << std::endl; }
        client_ = nullptr;
    };

    void initialize()
    {
        if (!suppress_default_logging_) { std::cout << "[SPEAR | client.h] Client::initialize()" << std::endl; }
        SP_ASSERT(client_);

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
        } catch (const boost::interprocess::interprocess_exception&) {
            if (!suppress_default_logging_) { std::cout << "[SPEAR | client.h]     Couldn't open shared memory fence, assuming the server is remote..." << std::endl; }
        }

        if (shared_memory_fence) {
            std::string fence_key_from_shared_memory_region = shared_memory_fence->getKey();
            if (fence_key_from_shared_memory_region == fence_key_from_rpc) {
                if (!suppress_default_logging_) { std::cout << "[SPEAR | client.h]     Opened shared memory fence: " << fence_key_from_rpc << std::endl; }
                shared_memory_fence_ = std::move(shared_memory_fence);
            } else if (!suppress_default_logging_) {
                std::cout << "[SPEAR | client.h]     Opened shared memory fence but it belongs to a different server than the one we're connected to, assuming our server is remote..." << std::endl;
                std::cout << "[SPEAR | client.h]         Expected key from RPC call with server: " << fence_key_from_rpc << std::endl;
                std::cout << "[SPEAR | client.h]         Actual key from shared memory region:   " << fence_key_from_shared_memory_region << std::endl;
            }
        }
    };

    void terminate()
    {
        if (!suppress_default_logging_) { std::cout << "[SPEAR | client.h] Client::terminate()" << std::endl; }
        SP_ASSERT(client_);
        client_ = nullptr;
        shared_memory_fence_ = nullptr;
        entry_point_signature_descs_.clear();
        entry_point_signature_type_names_.clear();
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

    std::string ping()
    {
        SP_ASSERT(client_);
        return client_->call("engine_globals_service.call_sync_on_worker_thread.ping").get().as<std::string>();
    }

    bool isServerLocal() const
    {
        return shared_memory_fence_ != nullptr;
    }

    nanobind::object call(const std::string& func_name, const nanobind::args& args)
    {
        if (verbose_rpc_calls_) { std::cout << "[SPEAR | client.h] call(...): " << func_name << std::endl; }

        SP_ASSERT(client_);

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
            if (verbose_exceptions_) { std::cout << "[SPEAR | client.h] ERROR: Caught exception when calling \"" << func_name << "\": " << e.what() << std::endl; }
            throw;
        } catch (...) {
            if (verbose_exceptions_) { std::cout << "[SPEAR | client.h] ERROR: Caught unknown exception when calling \"" << func_name << "\"." << std::endl; }
            throw;
        }
    }

    std::map<std::string, SpFuncSignatureDesc> getEntryPointSignatureDescs() const
    {
        return entry_point_signature_descs_;
    }

    std::vector<std::string> getEntryPointSignatureTypeNames() const
    {
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
                std::cout << "[SPEAR | client.h] ERROR: type ID/name mismatch at id=" << i << " (server=" << get_display_string_func(server_type_names, i) << ", client=" << get_display_string_func(client_type_names, i) << ")" << std::endl;
                success = false;
            }
        }

        if (!success) {
            std::cout << "[SPEAR | client.h] ERROR: The spear_ext Python extension module is out of sync with the server. Rebuild both the server and the spear_ext Python extension module." << std::endl;
        }

        SP_ASSERT(success);
    }

    std::unique_ptr<rpc::client> client_ = nullptr;
    std::unique_ptr<SharedMemoryFence> shared_memory_fence_ = nullptr;
    std::map<std::string, SpFuncSignatureDesc> entry_point_signature_descs_;
    std::vector<std::string> entry_point_signature_type_names_;
};
