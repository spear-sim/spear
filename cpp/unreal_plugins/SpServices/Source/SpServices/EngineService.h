//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <atomic>
#include <future> // std::promise
#include <mutex>  // std::lock_guard
#include <string>

#include <boost/predef.h> // BOOST_ENDIAN_BIG_BYTE, BOOST_ENDIAN_LITTLE_BYTE, BOOST_OS_LINUX, BOOST_OS_MACOS, BOOST_OS_WINDOWS

#include <CoreGlobals.h>          // IsRunningCommandlet
#include <Engine/Engine.h>        // GEngine
#include <Engine/GameViewportClient.h>
#include <Engine/World.h>         // FWorldDelegates
#include <Misc/CommandLine.h>
#include <UObject/ObjectMacros.h> // UENUM

#include "SpCore/Assert.h"
#include "SpCore/Boost.h"
#include "SpCore/Log.h"
#include "SpCore/Unreal.h"

#include "SpServices/EntryPointBinder.h"
#include "SpServices/FuncInfo.h"
#include "SpServices/Service.h"
#include "SpServices/WorkQueue.h"

#include "EngineService.generated.h"

UENUM()
enum class EFrameState
{
    Idle                = 0,
    RequestBeginFrame   = 1,
    ExecutingBeginFrame = 2,
    ExecutingFrame      = 3,
    ExecutingEndFrame   = 4,
    Closing             = 5,
    Error               = 6
};

template <CEntryPointBinder TEntryPointBinder>
class EngineService : public Service {
public:
    EngineService() = delete;
    EngineService(TEntryPointBinder* entry_point_binder) : Service("EngineService")
    {
        SP_ASSERT(entry_point_binder);

        entry_point_binder_ = entry_point_binder;

        //
        // Entry points for managing the frame state.
        //

        bindFuncToExecuteOnWorkerThread("engine_service", "begin_frame", [this]() -> void {

            {
                std::lock_guard<std::mutex> lock(frame_state_mutex_);

                SP_ASSERT(frame_state_ == EFrameState::Idle || frame_state_ == EFrameState::Closing || frame_state_ == EFrameState::Error,
                    "frame_state_: %s", Unreal::getStringFromEnumValue(frame_state_.load()).c_str());

                if (frame_state_ == EFrameState::Closing || frame_state_ == EFrameState::Error) {
                    return;
                }

                // Reset promises and futures.
                frame_state_idle_promise_ = std::promise<void>();
                frame_state_executing_begin_frame_promise_ = std::promise<void>();
                frame_state_executing_end_frame_promise_ = std::promise<void>();

                frame_state_idle_future_ = frame_state_idle_promise_.get_future();
                frame_state_executing_begin_frame_future_ = frame_state_executing_begin_frame_promise_.get_future();
                frame_state_executing_end_frame_future_ = frame_state_executing_end_frame_promise_.get_future();

                // Allow beginFrameHandler() to start executing.
                frame_state_ = EFrameState::RequestBeginFrame;
            }

            // Note that close() could execute here on the game thread, which will change the value of frame_state_.

            if (frame_state_ == EFrameState::RequestBeginFrame) {
                // Wait here until beginFrameHandler() or close() updates frame_state_ and calls frame_state_executing_begin_frame_promise_.set_value().
                frame_state_executing_begin_frame_future_.wait();

                EFrameState frame_state = frame_state_;
                SP_ASSERT(frame_state == EFrameState::ExecutingBeginFrame || frame_state == EFrameState::Closing,
                    "frame_state_: %s", Unreal::getStringFromEnumValue(frame_state).c_str());
            }
        });

        bindFuncToExecuteOnWorkerThread("engine_service", "execute_frame", [this]() -> void {

            EFrameState frame_state;

            frame_state = frame_state_;
            SP_ASSERT(frame_state == EFrameState::ExecutingBeginFrame || frame_state == EFrameState::Closing || frame_state == EFrameState::Error,
                "frame_state_: %s", Unreal::getStringFromEnumValue(frame_state).c_str());

            if (frame_state == EFrameState::Closing || frame_state == EFrameState::Error) {
                return;
            }

            // Allow beginFrameHandler() to finish executing.
            work_queue_.reset();

            // Wait here until endFrameHandler() updates frame_state_ and calls frame_state_executing_end_frame_promise_.set_value().
            frame_state_executing_end_frame_future_.wait();

            frame_state = frame_state_;
            SP_ASSERT(frame_state == EFrameState::ExecutingEndFrame,
                "frame_state_: %s", Unreal::getStringFromEnumValue(frame_state).c_str());
        });

        bindFuncToExecuteOnWorkerThread("engine_service", "end_frame", [this]() -> void {

            EFrameState frame_state;

            frame_state = frame_state_;
            SP_ASSERT(frame_state == EFrameState::ExecutingEndFrame || frame_state == EFrameState::Closing || frame_state == EFrameState::Error,
                "frame_state_: %s", Unreal::getStringFromEnumValue(frame_state).c_str());

            if (frame_state == EFrameState::Closing || frame_state == EFrameState::Error) {
                return;
            }

            // Allow endFrameHandler() to finish executing.
            work_queue_.reset();

            // Wait here until endFrameHandler() updates frame_state_ and calls frame_state_idle_promise_.set_value().
            frame_state_idle_future_.wait();

            frame_state = frame_state_;
            SP_ASSERT(frame_state == EFrameState::Idle,
                "frame_state_: %s", Unreal::getStringFromEnumValue(frame_state).c_str());
        });

        // Miscellaneous low-level entry points to support initializing a spear.Instance

        bindFuncToExecuteOnWorkerThread("engine_service", "ping", []() -> std::string {
            return "ping";
        });

        bindFuncToExecuteOnWorkerThread("engine_service", "get_id", []() -> int64_t {            
            return static_cast<int>(boost::this_process::get_id());
        });

        // Miscellaneous low-level entry points to support initializing a spear.services.EngineService

        bindFuncToExecuteOnWorkerThread("engine_service", "initialize", [this]() -> void {
            std::lock_guard<std::mutex> lock(frame_state_mutex_);

            SP_ASSERT(frame_state_ == EFrameState::Idle || frame_state_ == EFrameState::Error,
                "frame_state_: %s", Unreal::getStringFromEnumValue(frame_state_.load()).c_str());

            work_queue_.initialize();
            frame_state_ = EFrameState::Idle;
        });

        bindFuncToExecuteOnWorkerThread("engine_service", "get_byte_order", []() -> std::string {
            SP_ASSERT(BOOST_ENDIAN_BIG_BYTE + BOOST_ENDIAN_LITTLE_BYTE == 1);
            if (BOOST_ENDIAN_BIG_BYTE) {
                return "big";
            } else if (BOOST_ENDIAN_LITTLE_BYTE) {
                return "little";
            } else {
                return "";
            }
        });

        // Miscellaneous low-level entry points that interact with Unreal globals

        // By calling IsRunningCommandlet() and FCommandLine::Get() below, we are accessing global variables
        // from a worker thread that are set on the game thread, and these global variables are not protected
        // by any synchronization primitive. At first glance, this seems like it would lead to undefined
        // behavior, but in this case it does not, because all RPC server worker threads are created on the
        // game thread after these global variables are set, and therefore there is a formal happens-before
        // relationship between the write and subsequent reads.

        // Calling FGenericPlatformMisc::RequestExit(false) is technically undefined behavior because there
        // isn't a happens-before relationship when accessing the underlying global variable from multiple
        // threads. But this undefined behavior is benign in practice, because as long as the game thread
        // eventually sees the write from the worker thread, the overall system behavior is correct.

        bindFuncToExecuteOnWorkerThread("engine_service", "is_with_editor", []() -> bool {
            return WITH_EDITOR;
        });

        bindFuncToExecuteOnWorkerThread("engine_service", "is_running_commandlet", []() -> bool {
            #if WITH_EDITOR // defined in an auto-generated header
                return IsRunningCommandlet();
            #else
                return false;
            #endif
        });

        bindFuncToExecuteOnWorkerThread("engine_service", "get_command_line", []() -> std::string {
            return Unreal::toStdString(FCommandLine::Get());
        });

        bindFuncToExecuteOnWorkerThread("engine_service", "request_exit", [](bool immediate_shutdown) -> void {
            FGenericPlatformMisc::RequestExit(immediate_shutdown);
        });

        // Miscellaneous low-level entry points that interact with GEngine

        bindFuncToExecuteOnGameThread("engine_service", "get_viewport_size", [this]() -> std::vector<double> {
            SP_ASSERT(GEngine);
            SP_ASSERT(GEngine->GameViewport);
            FVector2D viewport_size;
            GEngine->GameViewport->GetViewportSize(viewport_size); // GameViewport is a UPROPERTY but GetViewportSize(...) isn't a UFUNCTION
            return {viewport_size.X, viewport_size.Y};
        });
    }

    void bindFuncToExecuteOnWorkerThread(const std::string& service_name, const std::string& func_name, const auto& func)
    {
        std::string long_func_name = service_name + "." + func_name;
        entry_point_binder_->bind(long_func_name, wrapFuncToExecuteInTryCatch(long_func_name, func));
    }

    void bindFuncToExecuteOnGameThread(const std::string& service_name, const std::string& func_name, const auto& func)
    {
        std::string long_func_name = service_name + "." + func_name;
        entry_point_binder_->bind(long_func_name, WorkQueue::wrapFuncToExecuteInWorkQueueBlocking(work_queue_, long_func_name, func));
    }

    void close()
    {
        std::lock_guard<std::mutex> lock(frame_state_mutex_);

        // We need platform-specific error handling here because the int64_t returned by Unreal::getEnumValue(...)
        // needs to be formatted with %lld" on Windows and macOS, but "%ld" on Linux.
        
        #if BOOST_OS_WINDOWS || BOOST_OS_MACOS
            SP_ASSERT(frame_state_ == EFrameState::Idle || frame_state_ == EFrameState::RequestBeginFrame || frame_state_ == EFrameState::Error,
                "frame_state_: %lld", Unreal::getEnumValue(frame_state_.load())); // don't try to print string because Unreal's reflection system is already shut down
        #elif BOOST_OS_LINUX
            SP_ASSERT(frame_state_ == EFrameState::Idle || frame_state_ == EFrameState::RequestBeginFrame || frame_state_ == EFrameState::Error,
                "frame_state_: %ld", Unreal::getEnumValue(frame_state_.load()));  // don't try to print string because Unreal's reflection system is already shut down
        #else
            #error
        #endif

        EFrameState frame_state = frame_state_;
        frame_state_ = EFrameState::Closing;
        if (frame_state == EFrameState::RequestBeginFrame) {
            frame_state_executing_begin_frame_promise_.set_value(); // avoids deadlock if close() executes after begin_frame()
        }
    }

protected:
    void beginFrame() override
    {
        Service::beginFrame();

        {
            std::lock_guard<std::mutex> lock(frame_state_mutex_);

            SP_ASSERT(frame_state_ == EFrameState::Idle || frame_state_ == EFrameState::RequestBeginFrame || frame_state_ == EFrameState::Error,
                "frame_state_: %s", Unreal::getStringFromEnumValue(frame_state_.load()).c_str());

            if (frame_state_ == EFrameState::RequestBeginFrame) {

                // Allow begin_frame() to finish executing.
                frame_state_ = EFrameState::ExecutingBeginFrame;
                frame_state_executing_begin_frame_promise_.set_value();

                try {
                    // Execute all pre-frame work and wait here until execute_frame() calls work_queue_.reset().
                    work_queue_.run();

                    // Set frame state.
                    frame_state_ = EFrameState::ExecutingFrame;

                // In case of an exception, set the frame state to be in an error state and return.
                } catch(const std::exception& e) {
                    SP_LOG_CURRENT_FUNCTION();
                    SP_LOG("    ERROR: Caught exception when executing beginFrameHandler(): ", e.what());
                    frame_state_ = EFrameState::Error;
                    return;
                } catch(...) {
                    SP_LOG_CURRENT_FUNCTION();
                    SP_LOG("    ERROR: Caught unknown exception when executing beginFrameHandler().");
                    frame_state_ = EFrameState::Error;
                    return;
                }
            }
        }
    }

    void endFrame() override
    {
        {
            std::lock_guard<std::mutex> lock(frame_state_mutex_);

            SP_ASSERT(frame_state_ == EFrameState::Idle || frame_state_ == EFrameState::RequestBeginFrame || frame_state_ == EFrameState::ExecutingFrame || frame_state_ == EFrameState::Error,
                "frame_state_: %s", Unreal::getStringFromEnumValue(frame_state_.load()).c_str());

            if (frame_state_ == EFrameState::ExecutingFrame) {

                // Allow execute_frame() to finish executing.
                frame_state_ = EFrameState::ExecutingEndFrame;
                frame_state_executing_end_frame_promise_.set_value();

                try {
                    // Execute all post-frame work and wait here until end_frame() calls work_queue_.reset().
                    work_queue_.run();

                    // Allow end_frame() to finish executing.
                    frame_state_ = EFrameState::Idle;
                    frame_state_idle_promise_.set_value();

                // In case of an exception, set the frame state to be in an error state and return.
                } catch(const std::exception& e) {
                    SP_LOG_CURRENT_FUNCTION();
                    SP_LOG("    ERROR: Caught exception when executing endFrameHandler(): ", e.what());
                    frame_state_ = EFrameState::Error;
                    return;
                } catch(...) {
                    SP_LOG_CURRENT_FUNCTION();
                    SP_LOG("    ERROR: Caught unknown exception when executing endFrameHandler().");
                    frame_state_ = EFrameState::Error;
                    return;
                }
            }
        }

        Service::endFrame();
    }

private:
    template <typename TFunc>
    auto wrapFuncToExecuteInTryCatch(const std::string& long_func_name, const TFunc& func)
    {
        return wrapFuncToExecuteInTryCatchImpl(long_func_name, func, FuncInfoUtils::getFuncInfo<TFunc>());
    }

    template <typename TFunc, typename TReturn, typename... TArgs> requires
        CFuncReturnsAndIsCallableWithArgs<TFunc, TReturn, TArgs&...>
    auto wrapFuncToExecuteInTryCatchImpl(const std::string& long_func_name, const TFunc& func, const FuncInfo<TReturn, TArgs...>& fi)
    {
        // The lambda returned here is typically bound to a specific RPC entry point and called from a worker
        // thread by the RPC server.

        // Note that we capture long_func_name and func by value because we want to guarantee that func is still
        // accessible after this wrapFuncToHandleExceptionsImpl(...) function returns.

        // Note also that we assume that the user's function always accepts all arguments by non-const
        // reference. This will avoid unnecessary copying when we eventually call the user's function. We
        // can't assume the user's function accepts arguments by const reference, because the user's function
        // might want to modify the arguments, e.g., when a user function resolves pointers to shared memory
        // for an input SpPackedArray& before forwarding it to an inner function.

        return [this, long_func_name, func](TArgs&... args) -> TReturn {
            try {
                return func(args...);

            } catch (const std::exception& e) {
                SP_LOG_CURRENT_FUNCTION();
                SP_LOG("    ERROR: Caught exception when executing ", long_func_name, ": ", e.what());
                work_queue_.reset();
                {
                    std::lock_guard<std::mutex> lock(frame_state_mutex_);
                    frame_state_ = EFrameState::Error;
                }
                return TReturn();
            } catch(...) {
                SP_LOG_CURRENT_FUNCTION();
                SP_LOG("    ERROR: Caught unknown exception when executing ", long_func_name, ".");
                work_queue_.reset();
                {
                    std::lock_guard<std::mutex> lock(frame_state_mutex_);
                    frame_state_ = EFrameState::Error;
                }
                return TReturn();
            }
        };
    }

    TEntryPointBinder* entry_point_binder_ = nullptr;

    WorkQueue work_queue_;
    
    std::atomic<EFrameState> frame_state_ = EFrameState::Idle; // need std::atomic because not all frame_state_ reads are synchronized with frame_state_mutex_
    std::mutex frame_state_mutex_;                             // used to coordinate write-access to frame_state_

    std::promise<void> frame_state_idle_promise_;
    std::promise<void> frame_state_executing_begin_frame_promise_;
    std::promise<void> frame_state_executing_end_frame_promise_;

    std::future<void> frame_state_idle_future_;
    std::future<void> frame_state_executing_begin_frame_future_;
    std::future<void> frame_state_executing_end_frame_future_;
};
