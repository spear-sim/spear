//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // int64_t, uint16_t

#include <iostream>
#include <map>
#include <memory> // std::make_unique, std::unique_ptr
#include <string>
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

    void initialize()
    {
        std::cout << "[SPEAR | spear_ext.cpp] Client::initialize()" << std::endl;
        SP_ASSERT(client_);

        clmdep_msgpack::object_handle result = client_->call("engine_service.call_sync_on_worker_thread.get_entry_point_signature_descs");
        std::map<std::string, std::vector<FuncSignatureDesc>> entry_point_signature_descs = result.get().as<std::map<std::string, std::vector<FuncSignatureDesc>>>();

        entry_point_signature_descs_.clear();
        for (const auto& [registry_name, descs] : entry_point_signature_descs) {
            for (const auto& desc : descs) {
                entry_point_signature_descs_[desc.name_] = desc;
            }
        }
    };

    void terminate()
    {
        std::cout << "[SPEAR | spear_ext.cpp] Client::terminate()" << std::endl;
        SP_ASSERT(client_);
        client_ = nullptr;
        entry_point_signature_descs_.clear();
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

        const FuncSignatureDesc& desc = entry_point_signature_descs_.at(func_name);
        int return_type_id = desc.func_signature_id_[0];
        SP_ASSERT(args.size() == desc.func_signature_id_.size() - 1);

        // pack args into a msgpack array
        clmdep_msgpack::zone zone;
        clmdep_msgpack::object args_msgpack_array;
        args_msgpack_array.type = clmdep_msgpack::type::ARRAY;
        args_msgpack_array.via.array.size = static_cast<uint32_t>(args.size());
        args_msgpack_array.via.array.ptr = static_cast<clmdep_msgpack::object*>(zone.allocate_align(sizeof(clmdep_msgpack::object) * args.size(), MSGPACK_ZONE_ALIGNOF(clmdep_msgpack::object)));

        for (size_t i = 0; i < args.size(); i++) {
            args_msgpack_array.via.array.ptr[i] = FuncSignatureRegistry::getArg(zone, desc.func_signature_id_[i + 1], args[i]);
        }

        // call via rpclib
        clmdep_msgpack::object_handle result = client_->call("rpc_server.call", func_name, args_msgpack_array);

        // unpack result
        return FuncSignatureRegistry::getReturnValue(this, return_type_id, std::move(result));
    }

    bool force_return_aligned_arrays_ = false;
    bool verbose_rpc_calls_           = false;
    bool verbose_allocations_         = false;
    bool verbose_exceptions_          = false; // owner should set to true once a connection to the server is established

    const std::map<std::string, FuncSignatureDesc>& getEntryPointSignatureDescs() const
    {
        return entry_point_signature_descs_;
    }

private:
    std::unique_ptr<rpc::client> client_ = nullptr;
    std::map<std::string, FuncSignatureDesc> entry_point_signature_descs_;
};
