//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include "CoreUtils/Rpclib.h"

template <typename T>
concept CBasicEntryPointBinder = requires(T rpc_server) {
    rpc_server.bind("dummy", []() {});
};

template <CBasicEntryPointBinder TBasicEntryPointBinder>
class EngineService {
public:
    EngineService() = default;
    EngineService(std::shared_ptr<TBasicEntryPointBinder> basic_entry_point_binder)
    {
        basic_entry_point_binder_ = basic_entry_point_binder;
    }

    void bind(const std::string& service_name, const std::string& func_name, auto&& func)
    {
        //basic_entry_point_binder_->bind(
        //    service_name + "." + func_name,
        //    WorkQueue::wrapFuncToExecuteInWorkQueueBlocking(
        //        service_name + func_name,
        //        std::forward<decltype(func)>(func),
        //        &current_work_queue_));

        //basic_entry_point_binder_->bind(
        //    service_name + ".async." + func_name,
        //    WorkQueue::wrapFuncToExecuteInWorkQueueNonBlocking(
        //        service_name + func_name,
        //        std::forward<decltype(func)>(func),
        //        &current_work_queue_));
    }

private:
    std::shared_ptr<TBasicEntryPointBinder> basic_entry_point_binder_ = nullptr;
};
