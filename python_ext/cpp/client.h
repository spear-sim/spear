//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stddef.h> // size_t
#include <stdint.h> // int64_t, uint16_t

#include <algorithm> // std::max
#include <iostream>
#include <map>
#include <memory>    // std::make_unique, std::unique_ptr
#include <string>
#include <vector>

#include <nanobind/nanobind.h>

#include <rpc/client.h>

#include "assert.h"
#include "func_signature_registry.h"
#include "shared_memory.h"
#include "types.h"

class Client
{
public:
    Client() = delete;
    Client(const std::string& address, const uint16_t& port, bool suppress_default_logging = false)
    {
        suppress_default_logging_ = suppress_default_logging;
        if (!suppress_default_logging_) { std::cout << "[SPEAR | spear_ext.cpp] Client::Client(...)" << std::endl; }
        client_ = std::make_unique<rpc::client>(address, port);
    };

    ~Client()
    {
        if (!suppress_default_logging_) { std::cout << "[SPEAR | spear_ext.cpp] Client::~Client()" << std::endl; }
        client_ = nullptr;
    };

    void initialize()
    {
        if (!suppress_default_logging_) { std::cout << "[SPEAR | spear_ext.cpp] Client::initialize()" << std::endl; }
        SP_ASSERT(client_);

        clmdep_msgpack::object_handle object_handle;

        object_handle = client_->call("engine_service.call_sync_on_worker_thread.get_entry_point_signature_type_names");
        entry_point_signature_type_names_ = object_handle.get().as<std::vector<std::string>>();
        validateEntryPointSignatureTypeNames();

        object_handle = client_->call("engine_service.call_sync_on_worker_thread.get_entry_point_signature_descs");
        entry_point_signature_descs_ = object_handle.get().as<std::map<std::string, SpFuncSignatureDesc>>();

        object_handle = client_->call("shared_memory_service.call_sync_on_worker_thread.get_fence_view");
        SharedMemoryView fence_view = object_handle.get().as<SharedMemoryView>();
        shared_memory_fence_ = std::make_unique<SharedMemoryFence>(fence_view);
    };

    void terminate()
    {
        if (!suppress_default_logging_) { std::cout << "[SPEAR | spear_ext.cpp] Client::terminate()" << std::endl; }
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

    nanobind::object call(const std::string& func_name, const nanobind::args& args)
    {
        if (verbose_rpc_calls_) { std::cout << "[SPEAR | spear_ext.cpp] call(...): " << func_name << std::endl; }

        SP_ASSERT(client_);
        SP_ASSERT(shared_memory_fence_);

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
        // shared memory (see SharedMemoryFence::fence() in SpCore/SharedMemory.cpp for details).

        shared_memory_fence_->fence();
        clmdep_msgpack::object_handle result = client_->call("rpc_server.call", func_name, args_msgpack_array);
        shared_memory_fence_->fence();

        // unpack result
        return FuncSignatureRegistry::getReturnValue(this, return_type_id, std::move(result));
    }

    bool force_return_aligned_arrays_ = false;
    bool suppress_default_logging_    = false;
    bool verbose_rpc_calls_           = false;
    bool verbose_allocations_         = false;
    bool verbose_exceptions_          = false; // owner should set to true once a connection to the server is established

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
                std::cout << "[SPEAR | spear_ext.cpp] ERROR: type ID/name mismatch at id=" << i << " (server=" << get_display_string_func(server_type_names, i) << ", client=" << get_display_string_func(client_type_names, i) << ")" << std::endl;
                success = false;
            }
        }

        if (!success) {
            std::cout << "[SPEAR | spear_ext.cpp] ERROR: The spear_ext Python extension module is out of sync with the server. Rebuild both the server and the spear_ext Python extension module." << std::endl;
        }

        SP_ASSERT(success);
    }

    std::unique_ptr<rpc::client> client_ = nullptr;
    std::unique_ptr<SharedMemoryFence> shared_memory_fence_ = nullptr;
    std::map<std::string, SpFuncSignatureDesc> entry_point_signature_descs_;
    std::vector<std::string> entry_point_signature_type_names_;
};
