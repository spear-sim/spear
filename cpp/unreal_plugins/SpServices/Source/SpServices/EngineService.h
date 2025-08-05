//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stddef.h> // int64_t, uint64_t

#include <atomic>
#include <future>      // std::promise
#include <mutex>       // std::lock_guard
#include <string>
#include <utility>     // std::pair
#include <vector>
#include <type_traits> // std::is_void_v, std::remove_cvref_t

#include <boost/predef.h> // BOOST_ENDIAN_BIG_BYTE, BOOST_ENDIAN_LITTLE_BYTE, BOOST_OS_LINUX, BOOST_OS_MACOS, BOOST_OS_WINDOWS

#include <CoreGlobals.h>   // IsRunningCommandlet
#include <Engine/Engine.h> // GEngine
#include <Engine/GameViewportClient.h>
#include <Misc/CommandLine.h>

#include "SpCore/Assert.h"
#include "SpCore/Boost.h"
#include "SpCore/Config.h"
#include "SpCore/Log.h"
#include "SpCore/SpTypes.h"
#include "SpCore/Unreal.h"

#include "SpServices/EntryPointBinder.h"
#include "SpServices/FuncInfo.h"
#include "SpServices/Service.h"
#include "SpServices/WorkQueue.h"

template <CEntryPointBinder TEntryPointBinder>
class EngineService : public Service
{
public:
    EngineService() = delete;
    EngineService(TEntryPointBinder* entry_point_binder) : Service("EngineService")
    {
        SP_ASSERT(entry_point_binder);

        entry_point_binder_ = entry_point_binder;

        //
        // Entry points for managing the frame state.
        //

        bindFuncToExecuteOnWorkerThread("engine_service", "initialize", [this]() -> void {

            // Reset frame state.
            frame_state_ = FrameState::Idle;

            // Reset request flags.
            request_begin_frame_ = false;
            request_close_ = false;
            request_error_ = false;

            // There is no need to lock because it is safe to access WorkQueue from multiple threads.
            begin_frame_work_queue_.initialize();
            end_frame_work_queue_.initialize();

            // Reset promises and futures.

            {
                std::lock_guard<std::mutex> lock(begin_frame_mutex_);
                begin_frame_work_queue_ready_promise_ = std::promise<void>();
                begin_frame_work_queue_ready_future_  = begin_frame_work_queue_ready_promise_.get_future();
                begin_frame_work_queue_ready_promise_.set_value();
                begin_frame_work_queue_ready_promise_set_ = true;
            }

            {
                std::lock_guard<std::mutex> lock(end_frame_mutex_);
                end_frame_work_queue_ready_promise_ = std::promise<void>();
                end_frame_work_queue_ready_future_  = end_frame_work_queue_ready_promise_.get_future();
                end_frame_work_queue_ready_promise_.set_value();
                end_frame_work_queue_ready_promise_set_ = true;
            }
        });

        bindFuncToExecuteOnWorkerThread("engine_service", "close", [this]() -> void {
            // Indicate to the game thread that we want to close the application.
            request_close_ = true;
        });

        bindFuncToExecuteOnWorkerThread("engine_service", "begin_frame", [this]() -> void {

            if (request_close_ || request_error_) {
                return;
            }

            if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_FRAME_DEBUG_INFO")) {
                SP_LOG("engine_service.begin_frame: Waiting for beginFrame() to finish executing...");
            }

            // Wait until beginFrame() has finished executing before we allow the user to start queueing
            // the next frame of work.
            begin_frame_work_queue_ready_future_.get();

            // Reset promise and future.

            {
                std::lock_guard<std::mutex> lock(begin_frame_mutex_);
                begin_frame_work_queue_ready_promise_ = std::promise<void>();
                begin_frame_work_queue_ready_future_  = begin_frame_work_queue_ready_promise_.get_future();
                begin_frame_work_queue_ready_promise_set_ = false;
            }

            if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_FRAME_DEBUG_INFO")) {
                SP_LOG("engine_service.begin_frame: Finished waiting.");
            }

            // Set current work queue.
            SP_ASSERT(current_work_queue_ == nullptr);
            current_work_queue_ = &begin_frame_work_queue_;

            // Indicate to the game thread that we will be queueing additional work.
            request_begin_frame_ = true;
        });

        bindFuncToExecuteOnWorkerThread("engine_service", "execute_frame", [this]() -> void {

            if (request_close_ || request_error_) {
                return;
            }

            if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_FRAME_DEBUG_INFO")) {
                SP_LOG("engine_service.execute_frame: Waiting for endFrame() to finish executing...");
            }

            // Wait until endFrame() has finished executing before we allow the user to start queueing
            // the next frame of work.
            end_frame_work_queue_ready_future_.get();

            // Reset promise and future.

            {
                std::lock_guard<std::mutex> lock(end_frame_mutex_);
                end_frame_work_queue_ready_promise_ = std::promise<void>();
                end_frame_work_queue_ready_future_  = end_frame_work_queue_ready_promise_.get_future();
                end_frame_work_queue_ready_promise_set_ = false;
            }

            if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_FRAME_DEBUG_INFO")) {
                SP_LOG("engine_service.execute_frame: Finished waiting.");
            }

            // Set current work queue.
            SP_ASSERT(current_work_queue_ == &begin_frame_work_queue_);
            current_work_queue_ = &end_frame_work_queue_;

            if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_FRAME_DEBUG_INFO")) {
                SP_LOG("engine_service.execute_frame: Scheduling begin_frame_work_queue_.reset() on work queue...");
            }

            // Allow beginFrame() to finish executing.
            begin_frame_work_queue_.scheduleFunc("begin_frame_work_queue_.reset()", [this]() -> void {
                begin_frame_work_queue_.reset();
            });

            if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_FRAME_DEBUG_INFO")) {
                SP_LOG("engine_service.execute_frame: Finished scheduling begin_frame_work_queue_.reset().");
            }
        });

        bindFuncToExecuteOnWorkerThread("engine_service", "end_frame", [this]() -> void {

            if (request_close_ || request_error_) {
                return;
            }

            SP_ASSERT(current_work_queue_ == &end_frame_work_queue_);
            current_work_queue_ = nullptr;

            if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_FRAME_DEBUG_INFO")) {
                SP_LOG("engine_service.end_frame: Scheduling end_frame_work_queue_.reset() on work queue...");
            }

            // Allow endFrame() to finish executing.
            end_frame_work_queue_.scheduleFunc("end_frame_work_queue_.reset()", [this]() -> void {
                end_frame_work_queue_.reset();
            });

            if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_FRAME_DEBUG_INFO")) {
                SP_LOG("engine_service.end_frame: Finished scheduling end_frame_work_queue_.reset().");
            }
        });

        //
        // Miscellaneous low-level entry points to support initializing a spear.Instance
        //

        bindFuncToExecuteOnWorkerThread("engine_service", "ping", []() -> std::string {
            return "ping";
        });

        bindFuncToExecuteOnWorkerThread("engine_service", "get_id", []() -> int64_t {
            return static_cast<int>(boost::this_process::get_id());
        });

        // Miscellaneous low-level entry points to support initializing a spear.services.EngineService

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
        // from the worker thread that are set on the game thread, and these global variables are not protected
        // by any synchronization primitive. At first glance, this seems like it would lead to undefined
        // behavior, but in this case it does not, because all RPC server worker threads are created on the
        // game thread after these global variables are set, and therefore there is a formal happens-before
        // relationship between the write operation and subsequent read operations.

        // Calling FGenericPlatformMisc::RequestExit(false) is technically undefined behavior because there
        // isn't a happens-before relationship when accessing Unreal's underlying global variable from
        // multiple threads. But this undefined behavior is benign in practice, because as long as the write
        // operation from the worker thread is eventually visible on the game thread, the overall system
        // behavior is correct.

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

        //
        // Entry points to get a result from a future.
        //

        bindFuncToExecuteOnWorkerThread("engine_service", "get_future_result_as_void",                                [this](SpFuture& future) -> void                                           { return destroyFuture<void>(future);                                           });
        bindFuncToExecuteOnWorkerThread("engine_service", "get_future_result_as_bool",                                [this](SpFuture& future) -> bool                                           { return destroyFuture<bool>(future);                                           });
        bindFuncToExecuteOnWorkerThread("engine_service", "get_future_result_as_float",                               [this](SpFuture& future) -> float                                          { return destroyFuture<float>(future);                                          });
        bindFuncToExecuteOnWorkerThread("engine_service", "get_future_result_as_int32",                               [this](SpFuture& future) -> int32_t                                        { return destroyFuture<int32_t>(future);                                        });
        bindFuncToExecuteOnWorkerThread("engine_service", "get_future_result_as_int64",                               [this](SpFuture& future) -> int64_t                                        { return destroyFuture<int64_t>(future);                                        });
        bindFuncToExecuteOnWorkerThread("engine_service", "get_future_result_as_uint64",                              [this](SpFuture& future) -> uint64_t                                       { return destroyFuture<uint64_t>(future);                                       });
        bindFuncToExecuteOnWorkerThread("engine_service", "get_future_result_as_string",                              [this](SpFuture& future) -> std::string                                    { return destroyFuture<std::string>(future);                                    });
        bindFuncToExecuteOnWorkerThread("engine_service", "get_future_result_as_vector_of_double",                    [this](SpFuture& future) -> std::vector<double>                            { return destroyFuture<std::vector<double>>(future);                            });
        bindFuncToExecuteOnWorkerThread("engine_service", "get_future_result_as_vector_of_uint64",                    [this](SpFuture& future) -> std::vector<uint64_t>                          { return destroyFuture<std::vector<uint64_t>>(future);                          });
        bindFuncToExecuteOnWorkerThread("engine_service", "get_future_result_as_vector_of_string",                    [this](SpFuture& future) -> std::vector<std::string>                       { return destroyFuture<std::vector<std::string>>(future);                       });
        bindFuncToExecuteOnWorkerThread("engine_service", "get_future_result_as_map_of_string_to_uint64",             [this](SpFuture& future) -> std::map<std::string, uint64_t>                { return destroyFuture<std::map<std::string, uint64_t>>(future);                });
        bindFuncToExecuteOnWorkerThread("engine_service", "get_future_result_as_map_of_string_to_shared_memory_view", [this](SpFuture& future) -> std::map<std::string, SpArraySharedMemoryView> { return destroyFuture<std::map<std::string, SpArraySharedMemoryView>>(future); });
        bindFuncToExecuteOnWorkerThread("engine_service", "get_future_result_as_map_of_string_to_packed_array",       [this](SpFuture& future) -> std::map<std::string, SpPackedArray>           { return destroyFuture<std::map<std::string, SpPackedArray>>(future);           });
        bindFuncToExecuteOnWorkerThread("engine_service", "get_future_result_as_property_desc",                       [this](SpFuture& future) -> SpPropertyDesc                                 { return destroyFuture<SpPropertyDesc>(future);                                 });
        bindFuncToExecuteOnWorkerThread("engine_service", "get_future_result_as_shared_memory_view",                  [this](SpFuture& future) -> SpArraySharedMemoryView                        { return destroyFuture<SpArraySharedMemoryView>(future);                        });
        bindFuncToExecuteOnWorkerThread("engine_service", "get_future_result_as_packed_array",                        [this](SpFuture& future) -> SpPackedArray                                  { return destroyFuture<SpPackedArray>(future);                                  });
        bindFuncToExecuteOnWorkerThread("engine_service", "get_future_result_as_data_bundle",                         [this](SpFuture& future) -> SpFuncDataBundle                               { return destroyFuture<SpFuncDataBundle>(future);                               });
    }

    void bindFuncToExecuteOnWorkerThread(const std::string& service_name, const std::string& func_name, const auto& func)
    {
        std::string long_func_name = service_name + "." + func_name;

        if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO")) {
            SP_LOG("Binding function to execute on the worker thread:");
            printFuncSignature(long_func_name, func);
        }

        entry_point_binder_->bind(long_func_name, wrapFuncToExecuteInTryCatch(long_func_name, func));
    }

    void bindFuncToExecuteOnGameThread(const std::string& service_name, const std::string& func_name, const auto& func)
    {
        UnrealEntryPointBindFlags bind_flags = UnrealEntryPointBindFlags::CallSync | UnrealEntryPointBindFlags::CallAsync | UnrealEntryPointBindFlags::SendAsync;
        bindFuncToExecuteOnGameThread(service_name, func_name, bind_flags, func);
    }

    void bindFuncToExecuteOnGameThread(const std::string& service_name, const std::string& func_name, UnrealEntryPointBindFlags bind_flags, const auto& func)
    {
        if (Std::toBool(bind_flags & UnrealEntryPointBindFlags::CallSync)) {
            std::string long_func_name = service_name + "." + func_name;

            if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO")) {
                SP_LOG("Binding function to execute on game thread (blocking):");
                printFuncSignature(long_func_name, func);
            }

            entry_point_binder_->bind(long_func_name, wrapFuncToExecuteInWorkQueueBlocking(long_func_name, func));
        }

        if (Std::toBool(bind_flags & UnrealEntryPointBindFlags::CallAsync)) {
            std::string long_func_name = service_name + ".call_async." + func_name;

            if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO")) {
                SP_LOG("Binding function to game thread (non-blocking):");
                printFuncSignature(long_func_name, func);
            }

            entry_point_binder_->bind(long_func_name, wrapFuncToExecuteInWorkQueueNonBlocking(long_func_name, func));
        }

        if (Std::toBool(bind_flags & UnrealEntryPointBindFlags::SendAsync)) {
            std::string long_func_name = service_name + ".send_async." + func_name;

            if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO")) {
                SP_LOG("Binding function to game thread (non-blocking, discard return value):");
                printFuncSignature(long_func_name, func);
            }

            entry_point_binder_->bind(long_func_name, wrapFuncToExecuteInWorkQueueNonBlockingDiscardReturnValue(long_func_name, func));
        }
    }

    void close()
    {
        frame_state_ = FrameState::Closing;

        if (!begin_frame_work_queue_ready_promise_set_) {
            std::lock_guard lock(begin_frame_mutex_);
            begin_frame_work_queue_ready_promise_.set_value();
            begin_frame_work_queue_ready_promise_set_ = true;
        }

        if (!end_frame_work_queue_ready_promise_set_) {
            std::lock_guard lock(end_frame_mutex_);
            end_frame_work_queue_ready_promise_.set_value();
            end_frame_work_queue_ready_promise_set_ = true;
        }
    }

protected:
    void beginFrame() override
    {
        Service::beginFrame();

        bool handle_frame = request_begin_frame_;

        if (request_close_ || request_error_) {
            handle_frame = false;
        }

        if (frame_state_ == FrameState::Closing) {
            handle_frame = false;
        }

        if (handle_frame) {

            // Reset the request flag set by engine_service.begin_frame.
            request_begin_frame_ = false;

            // Update frame state.
            frame_state_ = FrameState::ExecutingBeginFrame;

            if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_FRAME_DEBUG_INFO")) {
                SP_LOG("beginFrame(): Executing begin_frame_work_queue_...");
            }

            // Execute all pre-frame work and wait here until engine_service.execute_frame calls begin_frame_work_queue_.reset().
            executeFuncInTryCatch("begin_frame_work_queue_.run()", [this]() -> void {
                begin_frame_work_queue_.run();
            });

            if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_FRAME_DEBUG_INFO")) {
                SP_LOG("beginFrame(): Finished executing begin_frame_work_queue_.");
            }

            frame_state_ = FrameState::ExecutingFrame;

            // Allow engine_service.begin_frame to finish executing.

            {
                std::lock_guard<std::mutex> lock(begin_frame_mutex_);
                begin_frame_work_queue_ready_promise_.set_value();
                begin_frame_work_queue_ready_promise_set_ = true;
            }
        }
    }

    void endFrame() override
    {
        bool handle_frame = frame_state_ == FrameState::ExecutingFrame;

        if (request_close_ || request_error_) {
            handle_frame = false;
        }

        if (frame_state_ == FrameState::Closing) {
            handle_frame = false;
        }

        if (handle_frame) {

            // Update frame state.
            frame_state_ = FrameState::ExecutingEndFrame;

            if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_FRAME_DEBUG_INFO")) {
                SP_LOG("endFrame(): Executing end_frame_work_queue_...");
            }

            // Execute all post-frame work and wait here until engine_service.end_frame calls end_frame_work_queue_.reset().
            executeFuncInTryCatch("end_frame_work_queue_.run()", [this]() -> void {
                end_frame_work_queue_.run();
            });

            if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_FRAME_DEBUG_INFO")) {
                SP_LOG("endFrame(): Finished executing end_frame_work_queue_.");
            }

            // Update frame state.
            frame_state_ = FrameState::Idle;

            // Allow engine_service.execute_frame to finish executing.

            {
                std::lock_guard<std::mutex> lock(end_frame_mutex_);
                end_frame_work_queue_ready_promise_.set_value();
                end_frame_work_queue_ready_promise_set_ = true;
            }
        }

        Service::endFrame();
    }

private:

    enum class FrameState
    {
        Idle                = 0,
        ExecutingBeginFrame = 1,
        ExecutingFrame      = 2,
        ExecutingEndFrame   = 3,
        Closing             = 4,
        Error               = 5
    };

    // In the wrapFunc(...) functions below, we assume that the input function always accepts all arguments
    // by non-const reference. This will avoid unnecessary copying when we eventually call the input function.
    // We can't assume the input function accepts arguments by const reference, because the input function
    /// might want to modify the arguments, e.g., when an input function resolves pointers to shared memory
    // for an input SpPackedArray& before forwarding it to an inner function.

    // Typically called from the game thread in EngineService::bindFuncToExecuteOnWorkerThread(...) to get an
    // outer func that executes the given inner func in a try-catch block.

    template <typename TFunc>
    auto wrapFuncToExecuteInTryCatch(const std::string& func_name, const TFunc& func)
    {
        return wrapFuncToExecuteInTryCatch(func_name, func, FuncInfoUtils::getFuncInfo<TFunc>());
    }

    template <typename TFunc, typename TReturn, typename... TArgs> requires
        CFuncReturnsAndIsCallableWithArgs<TFunc, TReturn, TArgs&...>
    auto wrapFuncToExecuteInTryCatch(const std::string& func_name, const TFunc& func, const FuncInfo<TReturn, TArgs...>& fi)
    {
        // The lambda declared below is typically bound to a specific RPC entry point and called from the
        // worker thread by the RPC server. Note that we capture func_name and func by value because we want
        // to guarantee that func_name and func are still accessible after wrapFuncToExecuteInTryCatch(...)
        // returns.

        return [this, func_name, func](TArgs&... args) -> TReturn {
            return executeFuncInTryCatch(func_name, [&func_name, &func, &args...]() -> TReturn {
                if constexpr (std::is_void_v<TReturn>) {

                    if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO")) {
                        SP_LOG("Executing function ", func_name, "...");
                    }

                    func(args...);

                    if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO")) {
                        SP_LOG("Finished executing function ", func_name, ".");
                    }

                } else {

                    if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO")) {
                        SP_LOG("Executing function ", func_name, "...");
                    }

                    TReturn return_value = func(args...);

                    if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO")) {
                        SP_LOG("Finished executing function ", func_name, ".");
                    }

                    return return_value;

                }
            });
        };
    }

    // Typically called from the game thread in EngineService::bindFuncToExecuteOnGameThread(...) to get an
    // outer func that executes the given inner func on a work queue and blocks until the result is ready.

    template <typename TFunc>
    auto wrapFuncToExecuteInWorkQueueBlocking(const std::string& func_name, const TFunc& func)
    {
        return wrapFuncToExecuteInWorkQueueBlocking(func_name, func, FuncInfoUtils::getFuncInfo<TFunc>());
    }
    
    template <typename TFunc, typename TReturn, typename... TArgs> requires
        CFuncReturnsAndIsCallableWithArgs<TFunc, TReturn, TArgs&...>
    auto wrapFuncToExecuteInWorkQueueBlocking(const std::string& func_name, const TFunc& func, const FuncInfo<TReturn, TArgs...>& fi)
    {
        // The lambda declared below is typically bound to a specific RPC entry point and called from the
        // worker thread by the RPC server. Note that we capture func_name and func by value because we want
        // to guarantee that func_name and func are still accessible after wrapFuncToExecuteInWorkQueueBlocking(...)
        // returns.

        return [this, func_name, func](TArgs&... args) -> TReturn {
            return executeFuncInTryCatch(func_name, [this, &func_name, &func, &args...]() -> TReturn {

                if (!current_work_queue_) {
                    SP_LOG_CURRENT_FUNCTION();
                    SP_LOG("    ERROR: Current work queue is null when attempting to execute function: ", func_name);
                }
                SP_ASSERT(current_work_queue_);

                if constexpr (std::is_void_v<TReturn>) {
                    if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO")) {
                        SP_LOG("Executing function ", func_name, " on work queue ", current_work_queue_->getName(), " (blocking)...");
                    }

                    current_work_queue_->scheduleFunc(func_name, func, args...).get();

                    if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO")) {
                        SP_LOG("Finished executing function ", func_name, " on work queue ", current_work_queue_->getName(), " (blocking).");
                    }

                } else {
                    if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO")) {
                        SP_LOG("Executing function ", func_name, " on work queue ", current_work_queue_->getName(), " (blocking)...");
                    }

                    TReturn return_value = current_work_queue_->scheduleFunc(func_name, func, args...).get();

                    if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO")) {
                        SP_LOG("Finished executing function ", func_name, " on work queue ", current_work_queue_->getName(), " (blocking).");
                    }

                    return return_value;
                }
            });
        };
    }

    // Typically called from the game thread in EngineService::bindFuncToExecuteOnGameThread(...) to get an
    // outer func that executes the given inner func on a work queue and returns a future.

    template <typename TFunc>
    auto wrapFuncToExecuteInWorkQueueNonBlocking(const std::string& func_name, const TFunc& func)
    {
        return wrapFuncToExecuteInWorkQueueNonBlocking(func_name, func, FuncInfoUtils::getFuncInfo<TFunc>());
    }
    
    template <typename TFunc, typename TReturn, typename... TArgs> requires
        CFuncReturnsAndIsCallableWithArgs<TFunc, TReturn, TArgs&...>
    auto wrapFuncToExecuteInWorkQueueNonBlocking(const std::string& func_name, const TFunc& func, const FuncInfo<TReturn, TArgs...>& fi)
    {
        // The lambda declared below is typically bound to a specific RPC entry point and called from the
        // worker thread by the RPC server. Note that we capture func_name and func by value because we want
        // to guarantee that func_name and func are still accessible after wrapFuncToExecuteInWorkQueueNonBlocking(...)
        // returns.

        return [this, func_name, func](TArgs&... args) -> SpFuture {
            return executeFuncInTryCatch(func_name, [this, &func_name, &func, &args...]() -> SpFuture {

                if (!current_work_queue_) {
                    SP_LOG_CURRENT_FUNCTION();
                    SP_LOG("    ERROR: Current work queue is null when attempting to execute function: ", func_name);
                }
                SP_ASSERT(current_work_queue_);

                if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO")) {
                    SP_LOG("Scheduling function ", func_name, " on work queue ", current_work_queue_->getName(), " (non-blocking)...");
                }

                SpFuture future = createFuture(current_work_queue_->scheduleFunc(func_name, func, args...));

                if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO")) {
                    SP_LOG("Finished scheduling function ", func_name, " on work queue ", current_work_queue_->getName(), " (non-blocking).");
                }

                return future;
            });
        };
    }

    // Typically called from the game thread in EngineService::bindFuncToExecuteOnGameThread(...) to get an
    // outer func that executes the given inner func on a work queue and ignores the return value.

    template <typename TFunc>
    auto wrapFuncToExecuteInWorkQueueNonBlockingDiscardReturnValue(const std::string& func_name, const TFunc& func)
    {
        return wrapFuncToExecuteInWorkQueueNonBlockingDiscardReturnValue(func_name, func, FuncInfoUtils::getFuncInfo<TFunc>());
    }
    
    template <typename TFunc, typename TReturn, typename... TArgs> requires
        CFuncReturnsAndIsCallableWithArgs<TFunc, TReturn, TArgs&...>
    auto wrapFuncToExecuteInWorkQueueNonBlockingDiscardReturnValue(const std::string& func_name, const TFunc& func, const FuncInfo<TReturn, TArgs...>& fi)
    {
        // The lambda declared below is typically bound to a specific RPC entry point and called from the
        // worker thread by the RPC server. Note that we capture func_name and func by value because we want
        // to guarantee that func_name and func are still accessible after wrapFuncToExecuteInWorkQueueNonBlockingDiscardReturnValue(...)
        // returns.

        return [this, func_name, func](TArgs&... args) -> void {
            executeFuncInTryCatch(func_name, [this, &func_name, &func, &args...]() -> void {

                if (!current_work_queue_) {
                    SP_LOG_CURRENT_FUNCTION();
                    SP_LOG("    ERROR: Current work queue is null when attempting to execute function: ", func_name);
                }
                SP_ASSERT(current_work_queue_);

                if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO")) {
                    SP_LOG("Scheduling function ", func_name, " on work queue ", current_work_queue_->getName(), " (non-blocking, disarding return value)...");
                }

                current_work_queue_->scheduleFunc(func_name, func, args...);

                if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO")) {
                    SP_LOG("Finished scheduling function ", func_name, " on work queue ", current_work_queue_->getName(), " (non-blocking, disarding return value)...");
                }
            });
        };
    }

    // Called from the worker thread and the game thread to execute a function in a try-catch block.

    template <typename TFunc>
    auto executeFuncInTryCatch(const std::string& func_name, const TFunc& func)
    {
        using TReturn = std::invoke_result_t<TFunc>;

        try {
            return func();
        } catch (const std::exception& e) {
            SP_LOG_CURRENT_FUNCTION();
            SP_LOG("    ERROR: Caught exception when executing ", func_name, ": ", e.what());
        } catch (...) {
            SP_LOG_CURRENT_FUNCTION();
            SP_LOG("    ERROR: Caught unknown exception when executing ", func_name, ".");
        }

        // If we get this far, an exception has occurred, so set the error state, reset the work
        // queue to allow the game thread to proceed, and return a default-constructed object.

        request_error_ = true;

        // There is no need to lock because it is safe to access WorkQueue from multiple threads.
        begin_frame_work_queue_.reset();
        end_frame_work_queue_.reset();

        return TReturn();
    }

    // Functions for creating and destroying futures

    template <typename TReturn>
    static SpFuture createFuture(std::future<TReturn>&& std_future)
    {
        SpFuture future;
        future.future_ptr_ = new std::future<TReturn>(std::move(std_future));
        future.type_id_ = Std::getTypeIdString<TReturn>();
        SP_ASSERT(future.future_ptr_);
        return future;
    }

    template <typename TReturn>
    static TReturn destroyFuture(SpFuture& future)
    {
        SP_ASSERT(future.future_ptr_);
        SP_ASSERT(future.type_id_ == Std::getTypeIdString<TReturn>());
        std::future<TReturn>* future_ptr = static_cast<std::future<TReturn>*>(future.future_ptr_);
        TReturn result = future_ptr->get();
        delete future_ptr;
        future_ptr = nullptr;
        return result;
    }

    template <>
    static void destroyFuture<void>(SpFuture& future)
    {
        SP_ASSERT(future.future_ptr_);
        SP_ASSERT(future.type_id_ == Std::getTypeIdString<void>());
        std::future<void>* future_ptr = static_cast<std::future<void>*>(future.future_ptr_);
        future_ptr->get();
        delete future_ptr;
        future_ptr = nullptr;
    }

    // Debug printing

    template <typename TFunc>
    static void printFuncSignature(const std::string& func_name, const TFunc& func)
    {
        return printFuncSignature(func_name, func, FuncInfoUtils::getFuncInfo<TFunc>());
    }

    template <typename TFunc, typename TReturn, typename... TArgs>
    static void printFuncSignature(const std::string& func_name, const TFunc& func, const FuncInfo<TReturn, TArgs...>& fi)
    {
        auto [return_value_string, arg_strings] = getFuncSignatureStrings<TReturn, TArgs...>();
        if (arg_strings.empty()) {
            SP_LOG("    ", func_name);
            SP_LOG("        () -> ", return_value_string);
        } else {
            SP_LOG("    ", func_name);
            SP_LOG("        (", Std::join(arg_strings, ", "), ") -> ", return_value_string);
        }
    }

    template <typename TReturn, typename... TArgs>
    static std::pair<std::string, std::vector<std::string>> getFuncSignatureStrings()
    {
        std::string return_value = getTypeString<TReturn>();
        std::vector<std::string> args;
        (args.push_back(getTypeString<TArgs>()), ...);
        return std::make_pair(return_value, std::move(args));
    }

    template <typename T> static std::string getTypeString()
    {
        return getTypeNameString<std::remove_reference_t<T>>() + getTypeRefString<T>();
    }

    template <typename T> static std::string getTypeNameString() { SP_ASSERT(false); return ""; }
    template <> std::string getTypeNameString<void>                                           () { return "void";                                           }
    template <> std::string getTypeNameString<bool>                                           () { return "bool";                                           }
    template <> std::string getTypeNameString<float>                                          () { return "float";                                          }
    template <> std::string getTypeNameString<int32_t>                                        () { return "int32_t";                                        }
    template <> std::string getTypeNameString<int64_t>                                        () { return "int64_t";                                        }
    template <> std::string getTypeNameString<uint64_t>                                       () { return "uint64_t";                                       }
    template <> std::string getTypeNameString<std::string>                                    () { return "std::string";                                    }
    template <> std::string getTypeNameString<std::vector<double>>                            () { return "std::vector<double>";                            }
    template <> std::string getTypeNameString<std::vector<uint64_t>>                          () { return "std::vector<uint64_t>";                          }
    template <> std::string getTypeNameString<std::vector<std::string>>                       () { return "std::vector<std::string>";                       }
    template <> std::string getTypeNameString<std::map<std::string, uint64_t>>                () { return "std::map<std::string, uint64_t>";                }
    template <> std::string getTypeNameString<std::map<std::string, std::string>>             () { return "std::map<std::string, std::string>";             }
    template <> std::string getTypeNameString<std::map<std::string, SpArraySharedMemoryView>> () { return "std::map<std::string, SpArraySharedMemoryView>"; }
    template <> std::string getTypeNameString<std::map<std::string, SpPackedArray>>           () { return "std::map<std::string, SpPackedArray>";           }
    template <> std::string getTypeNameString<SpPropertyDesc>                                 () { return "SpPropertyDesc";                                 }
    template <> std::string getTypeNameString<SpArraySharedMemoryView>                        () { return "SpArraySharedMemoryView";                        }
    template <> std::string getTypeNameString<SpPackedArray>                                  () { return "SpPackedArray";                                  }
    template <> std::string getTypeNameString<SpFuture>                                       () { return "SpFuture";                                       }
    template <> std::string getTypeNameString<SpFuncDataBundle>                               () { return "SpFuncDataBundle";                               }

    template <typename T> requires std::is_reference_v<T> static std::string getTypeRefString() { return "&"; }
    template <typename T> requires (!std::is_reference_v<T>) static std::string getTypeRefString() { return ""; }

    TEntryPointBinder* entry_point_binder_ = nullptr;

    WorkQueue begin_frame_work_queue_ = WorkQueue("begin_frame_work_queue");
    WorkQueue end_frame_work_queue_ = WorkQueue("end_frame_work_queue");

    // Only accessed on the worker thread.
    WorkQueue* current_work_queue_ = nullptr;

    // Written by the game thread in beginFrame(), endFrame(), and close(). Written by the worker thread in
    // engine_service.initialize.
    std::atomic<FrameState> frame_state_ = FrameState::Idle;

    // Written by the worker thread in engine_service.initialize and engine_service.begin_frame to indicate
    // to the game thread that we want to begin queueing a batch of work. Written and read by the game thread
    // in beginFrame().
    std::atomic<bool> request_begin_frame_ = false;

    // Written by the worker thread in engine_service.initialize and engine_service.close to indicate to the
    // game thread that we want to close the application. Read by the worker thread in engine_service.begin_frame,
    // engine_service.execute_frame, and engine_service.end_frame. Read by the game thread in beginFrame()
    // and endFrame().
    std::atomic<bool> request_close_ = false;

    // Written by the worker thread in executeFuncInTryCatch(...) to indicate to the game thread that an
    // error has occurred. Read by the worker thread in engine_service.begin_frame, engine_service.execute_frame,
    // and engine_service.end_frame. Read by the game thread in beginFrame() and endFrame().
    std::atomic<bool> request_error_ = false;

    // These promises and futures are each initialized by the worker thread in engine_service.initialize, engine_service.begin_frame,
    // and engine_service.execute_frame. Subsequently, the promises are only ever set by the game thread in
    // beginFrame(), endFrame(), and close(). The futures are only ever read on the worker thread in engine_service.begin_frame
    // and engine_service.execute_frame. Each bool variable is set by the game thread whenever the corresponding
    // promise is set, and is read by the game thread in close() to avoid deadlocks when closing the
    // application. The mutexes are used to coordinate access to the promises, the futures, and the bools.
    std::mutex begin_frame_mutex_;
    std::mutex end_frame_mutex_;
    std::promise<void> begin_frame_work_queue_ready_promise_;
    std::promise<void> end_frame_work_queue_ready_promise_;
    std::future<void> begin_frame_work_queue_ready_future_;
    std::future<void> end_frame_work_queue_ready_future_;
    std::atomic<bool> begin_frame_work_queue_ready_promise_set_ = false;
    std::atomic<bool> end_frame_work_queue_ready_promise_set_ = false;
};
