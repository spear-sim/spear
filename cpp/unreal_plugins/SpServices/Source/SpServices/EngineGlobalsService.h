//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // int64_t, uint64_t

#include <string>

#include <boost/predef.h> // BOOST_ENDIAN_BIG_BYTE, BOOST_ENDIAN_LITTLE_BYTE

#include <CoreGlobals.h>   // IsAsyncLoading, IsRunningCommandlet
#include <Engine/Engine.h> // GEngine
#include <GenericPlatform/GenericPlatformMisc.h>
#include <Misc/CommandLine.h>

#include "SpCore/Assert.h"
#include "SpCore/Boost.h"
#include "SpCore/Unreal.h"

#include "SpServices/EntryPointBinder.h"
#include "SpServices/Service.h"

class EngineGlobalsService : public Service
{
public:
    EngineGlobalsService() = delete;
    EngineGlobalsService(CUnrealEntryPointBinder auto* unreal_entry_point_binder) : Service("EngineGlobalsService")
    {
        SP_ASSERT(unreal_entry_point_binder);

        //
        // Miscellaneous low-level entry points to support spear.Instance and do not interact with Unreal.
        //

        unreal_entry_point_binder->bindFuncToExecuteOnWorkerThread("engine_globals_service", "ping", []() -> std::string {
            return "ping";
        });

        unreal_entry_point_binder->bindFuncToExecuteOnWorkerThread("engine_globals_service", "get_id", []() -> int64_t {
            return static_cast<int>(boost::this_process::get_id());
        });

        unreal_entry_point_binder->bindFuncToExecuteOnWorkerThread("engine_globals_service", "get_byte_order", []() -> std::string {
            SP_ASSERT(BOOST_ENDIAN_BIG_BYTE + BOOST_ENDIAN_LITTLE_BYTE == 1);
            if (BOOST_ENDIAN_BIG_BYTE) {
                return "big";
            } else if (BOOST_ENDIAN_LITTLE_BYTE) {
                return "little";
            } else {
                return "";
            }
        });

        //
        // Miscellaneous low-level entry points that interact with Unreal globals and can be called from the
        // worker thread.
        //
        // By calling IsRunningCommandlet() and FCommandLine::Get() below, we are accessing global variables
        // from the worker thread that are set on the game thread, and these global variables are not protected
        // by any synchronization primitive. At first glance, this seems like it would lead to undefined
        // behavior, but in this case it does not, because all RPC server worker threads are created on the
        // game thread after these global variables are set, and therefore there is a formal happens-before
        // relationship between the write operation and subsequent read operations.
        //
        // Calling FGenericPlatformMisc::RequestExit(false) from the worker thread is technically undefined
        // behavior because there isn't a happens-before relationship when accessing Unreal's underlying
        // global variable from multiple threads. But this undefined behavior is benign in practice, because
        // as long as the write operation from the worker thread is eventually visible on the game thread,
        // the overall system behavior is correct.
        //
        // Note that We could implement UCLASSES and UFUNCTIONS to access FCommandLine, FGenericPlatformMisc,
        // etc, but then we would be limited to accessing them on the game thread. Providing access through
        // the entry points below enables access from the worker thread, which simplifies the implementation
        // of spear.Instance.
        //

        unreal_entry_point_binder->bindFuncToExecuteOnWorkerThread("engine_globals_service", "get_engine", []() -> uint64_t {
            return toUInt64(GEngine);
        });

        unreal_entry_point_binder->bindFuncToExecuteOnWorkerThread("engine_globals_service", "is_with_editor", []() -> bool {
            return WITH_EDITOR;
        });

        unreal_entry_point_binder->bindFuncToExecuteOnWorkerThread("engine_globals_service", "is_running_commandlet", []() -> bool {
            #if WITH_EDITOR // defined in an auto-generated header
                return IsRunningCommandlet();
            #else
                return false;
            #endif
        });

        unreal_entry_point_binder->bindFuncToExecuteOnWorkerThread("engine_globals_service", "get_command_line", []() -> std::string {
            return Unreal::toStdString(FCommandLine::Get());
        });

        unreal_entry_point_binder->bindFuncToExecuteOnWorkerThread("engine_globals_service", "request_exit", [](bool immediate_shutdown) -> void {
            FGenericPlatformMisc::RequestExit(immediate_shutdown);
        });

        //
        // Miscellaneous low-level entry points that interact with Unreal globals and must be called from the
        // game thread.
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("engine_globals_service", "is_async_loading", []() -> bool {
            return IsAsyncLoading();
        });
    }
};
