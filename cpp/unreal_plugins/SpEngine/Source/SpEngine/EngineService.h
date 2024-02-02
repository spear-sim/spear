//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <atomic>

#include "SimulationController/RpcServer.h"

template <typename T> 
concept CBasicEntryPointBinder = requires(T rpc_server) {
	rpc_server.bind();
};

template <CBasicEntryPointBinder TBasicEntryPointBinder>
class EngineService {

public:
	EngineService() == delete;
	EngineService(TBasicEntryPointBinder* basic_entry_point_binder)
	{
		basic_entry_point_binder_ = basic_entry_point_binder;

		current_work_queue_ = nullptr; // needs to be std::atomic<WorkQueue*>*

		basic_entry_point_binder->bind(“begin_tick”, ...);
		basic_entry_point_binder->bind(“tick”, ...);
		basic_entry_point_binder->bind(“end_tick”, ...);
	}

    void EngineService::bind(const std::string& service_name, const std::string& func_name, auto&& func)
    {
        basic_entry_point_binder_->bind(
            service_name + "." + func_name,
            WorkQueue::wrapFuncToExecuteInWorkQueueBlocking(
                service_name + func_name,
                std::forward<decltype(func)>(func),
                &current_work_queue_));

        basic_entry_point_binder_->bind(
            service_name + ".async." + func_name,
            WorkQueue::wrapFuncToExecuteInWorkQueueNonBlocking(
                service_name + func_name,
                std::forward<decltype(func)>(func),
                &current_work_queue_));
    }

private:
    TBasicEntryPointBinder* basic_entry_point_binder_ = nullptr;
	std::atomic<WorkQueue*>* current_work_queue_ = nullptr;
};
