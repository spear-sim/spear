//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <exception>
#include <string>

#include "SpCore/Assert.h"

#include "SpServices/EntryPointBinder.h"
#include "SpServices/Service.h"

class DebugService : public Service
{
public:
    DebugService() = delete;
    DebugService(CUnrealEntryPointBinder auto* unreal_entry_point_binder)
    {
        SP_ASSERT(unreal_entry_point_binder);

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("debug_service", "assert_false_on_game_thread", []() -> void {
            SP_ASSERT(false);
        });

        unreal_entry_point_binder->bindFuncToExecuteOnWorkerThread("debug_service", "assert_false_on_worker_thread", []() -> void {
            SP_ASSERT(false);
        });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("debug_service", "throw_exception_on_game_thread", []() -> void {
            throw std::runtime_error("DebugService: intentional exception on game thread");
        });

        unreal_entry_point_binder->bindFuncToExecuteOnWorkerThread("debug_service", "throw_exception_on_worker_thread", []() -> void {
            throw std::runtime_error("DebugService: intentional exception on worker thread");
        });
    }
};
