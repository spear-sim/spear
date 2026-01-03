//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // int64_t, uint64_t

#include <atomic>
#include <exception>   // std::exception
#include <future>      // std::promise
#include <map>
#include <mutex>       // std::lock_guard
#include <string>
#include <utility>     // std::make_pair, std::move
#include <type_traits> // std::invoke_result_t, std::is_void_v

#include "SpCore/Assert.h"
#include "SpCore/Log.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"

#include "SpServices/EntryPointBinder.h"
#include "SpServices/FuncInfo.h"
#include "SpServices/FuncSignatureRegistry.h"
#include "SpServices/Service.h"
#include "SpServices/SpTypes.h"
#include "SpServices/WorkQueue.h"

template <CEntryPointBinder TEntryPointBinder>
class EngineService : public Service
{
public:
    EngineService() = delete;
    EngineService(TEntryPointBinder* entry_point_binder) : Service("EngineService")
    {
        SP_ASSERT(entry_point_binder);

        entry_point_signature_registries_["call_sync_on_worker_thread"] = FuncSignatureRegistry();

        entry_point_signature_registries_["call_sync_on_game_thread"]           = FuncSignatureRegistry();
        entry_point_signature_registries_["call_async_on_game_thread"]          = FuncSignatureRegistry();
        entry_point_signature_registries_["send_async_on_game_thread"]          = FuncSignatureRegistry();
        entry_point_signature_registries_["get_future_result_from_game_thread"] = FuncSignatureRegistry();

        entry_point_binder_ = entry_point_binder;

        //
        // Entry points for initializing and terminating the frame state.
        //

        bindFuncToExecuteOnWorkerThread("engine_service", "initialize", [this]() -> void {

            // Reset frame state.
            frame_state_ = FrameState::Idle;

            // Reset request flags.
            request_begin_frame_ = false;
            request_terminate_ = false;
            request_error_ = false;

            // There is no need to lock because it is safe to access WorkQueue objects from multiple threads.
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

        bindFuncToExecuteOnWorkerThread("engine_service", "terminate", [this]() -> void {

            // Set the frame state to prevent calling any more entry points.
            frame_state_ = FrameState::Terminating;

            // There is no need to lock because it is safe to access WorkQueue objects from multiple threads.
            begin_frame_work_queue_.terminate();
            end_frame_work_queue_.terminate();

            // Indicate to the game thread that we want to terminate the application.
            request_terminate_ = true;
        });

        //
        // Entry points for validating entry points.
        //

        bindFuncToExecuteOnWorkerThread("engine_service", "get_entry_point_signature_type_descs", [this]() -> std::vector<SpFuncSignatureTypeDesc> {
            return FuncSignatureRegistry::getFuncSignatureTypeDescs();
        });

        bindFuncToExecuteOnWorkerThread("engine_service", "get_entry_point_signature_descs", [this]() -> std::map<std::string, std::vector<SpFuncSignatureDesc>> {
            return Std::toMap<std::string, std::vector<SpFuncSignatureDesc>>(
                entry_point_signature_registries_ |
                std::views::transform([](auto& pair) {
                    auto& [entry_point_registry_name, entry_point_registry] = pair;
                    return std::make_pair(entry_point_registry_name, entry_point_registry.getFuncSignatureDescs());
                }));
        });

        //
        // Entry points for managing the frame state.
        //

        bindFuncToExecuteOnWorkerThread("engine_service", "begin_frame", [this]() -> bool {

            if (request_terminate_ || request_error_) {
                return false;
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

            return true;
        });

        bindFuncToExecuteOnWorkerThread("engine_service", "execute_frame", [this]() -> bool {

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

            return !request_error_;
        });

        bindFuncToExecuteOnWorkerThread("engine_service", "end_frame", [this]() -> bool {

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

            return !request_error_;
        });
    }

    template <typename TFunc>
    void bindFuncToExecuteOnWorkerThread(const std::string& service_name, const std::string& func_name, const TFunc& func)
    {
        std::string long_func_name = service_name + ".call_sync_on_worker_thread." + func_name;
        FuncSignatureRegistry& entry_point_registry = entry_point_signature_registries_.at("call_sync_on_worker_thread");

        SP_ASSERT(!entry_point_registry.isFuncSignatureRegistered(long_func_name, func));
        entry_point_registry.registerFuncSignature(long_func_name, func);
        entry_point_binder_->bind(long_func_name, wrapFuncToExecuteInTryCatch(long_func_name, func));
    }

    template <typename TFunc>
    void bindFuncToExecuteOnGameThread(const std::string& service_name, const std::string& func_name, const TFunc& func)
    {
        UnrealEntryPointBindFlags bind_flags = UnrealEntryPointBindFlags::CallSync | UnrealEntryPointBindFlags::CallAsync | UnrealEntryPointBindFlags::SendAsync;
        bindFuncToExecuteOnGameThread(service_name, func_name, bind_flags, func);
    }

    template <typename TFunc>
    void bindFuncToExecuteOnGameThread(const std::string& service_name, const std::string& func_name, UnrealEntryPointBindFlags bind_flags, const TFunc& func)
    {
        bindFuncToExecuteOnGameThread(service_name, func_name, bind_flags, func, FuncInfoUtils::getFuncInfo<TFunc>());
    }

    template <typename TFunc, typename TReturn, typename... TArgs>
    void bindFuncToExecuteOnGameThread(const std::string& service_name, const std::string& func_name, UnrealEntryPointBindFlags bind_flags, const TFunc& func, const FuncInfo<TReturn, TArgs...>& fi)
    {
        if (Std::toBool(bind_flags & UnrealEntryPointBindFlags::CallSync)) {
            std::string long_func_name = service_name + ".call_sync_on_game_thread." + func_name;
            FuncSignatureRegistry& entry_point_registry = entry_point_signature_registries_.at("call_sync_on_game_thread");

            SP_ASSERT((!entry_point_registry.isFuncSignatureRegistered<TReturn, TArgs...>(long_func_name))); // extra parentheses needed because of comma
            entry_point_registry.registerFuncSignature<TReturn, TArgs...>(long_func_name);
            entry_point_binder_->bind(long_func_name, wrapFuncToExecuteInWorkQueueBlocking(long_func_name, func));
        }

        if (Std::toBool(bind_flags & UnrealEntryPointBindFlags::CallAsync)) {
            std::string call_async_long_func_name = service_name + ".call_async_on_game_thread." + func_name;
            FuncSignatureRegistry& call_async_entry_point_registry = entry_point_signature_registries_.at("call_async_on_game_thread");

            SP_ASSERT((!call_async_entry_point_registry.isFuncSignatureRegistered<SpFuture, TArgs...>(call_async_long_func_name))); // extra parentheses needed because of comma
            call_async_entry_point_registry.registerFuncSignature<SpFuture, TArgs...>(call_async_long_func_name);
            entry_point_binder_->bind(call_async_long_func_name, wrapFuncToExecuteInWorkQueueNonBlocking(call_async_long_func_name, func)); // bound function calls createFuture

            // if we're binding a call_async function, then we also need to bind a corresponding
            // engine_service.get_future_result_as_return_type function if we haven't already

            std::string get_future_result_long_func_name = "engine_service.get_future_result_from_game_thread_as_" + FuncSignatureRegistry::getFuncSignatureTypeDesc<TReturn>().type_names_.at("entry_point");
            FuncSignatureRegistry& get_future_result_entry_point_registry = entry_point_signature_registries_.at("get_future_result_from_game_thread");

            if (!get_future_result_entry_point_registry.isFuncSignatureRegistered<TReturn, SpFuture&>(get_future_result_long_func_name)) {
                get_future_result_entry_point_registry.registerFuncSignature<TReturn, SpFuture&>(get_future_result_long_func_name);
                entry_point_binder_->bind(get_future_result_long_func_name, wrapFuncToExecuteInTryCatch(get_future_result_long_func_name, [this](SpFuture& future) -> TReturn {
                    return destroyFuture<TReturn>(future); // bound function calls destroyFuture
                }));
            }
        }

        if (Std::toBool(bind_flags & UnrealEntryPointBindFlags::SendAsync)) {
            std::string long_func_name = service_name + ".send_async_on_game_thread." + func_name;
            FuncSignatureRegistry& entry_point_registry = entry_point_signature_registries_.at("send_async_on_game_thread");

            SP_ASSERT((!entry_point_registry.isFuncSignatureRegistered<void, TArgs...>(long_func_name))); // extra parentheses needed because of comma
            entry_point_registry.registerFuncSignature<void, TArgs...>(long_func_name);
            entry_point_binder_->bind(long_func_name, wrapFuncToExecuteInWorkQueueNonBlockingDiscardReturnValue(long_func_name, func));
        }
    }

    // Called on the game thread from SpServices::StartupModule() and SpServices::ShutdownModule()

    void startup() const {}

    void shutdown()
    {
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

        if (request_terminate_ || request_error_) {
            handle_frame = false;
        }

        if (frame_state_ == FrameState::Terminating) {
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

        if (request_terminate_ || request_error_) {
            handle_frame = false;
        }

        if (frame_state_ == FrameState::Terminating) {
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
        Terminating         = 4,
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

            if (frame_state_ == FrameState::Terminating) {
                return TReturn();
            }

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

                if (request_error_) {
                    SP_LOG("ERROR: In an error state when attempting to execute ", func_name, " returning a default-constructed return value...");
                    return TReturn();
                }

                if (!current_work_queue_) {
                    SP_LOG_CURRENT_FUNCTION();
                    SP_LOG("ERROR: Current work queue is null when attempting to execute function: ", func_name);
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

                if (request_error_) {
                    SP_LOG("ERROR: In an error state when attempting to execute ", func_name, " returning a default-constructed SpFuture...");
                    return SpFuture();
                }

                if (!current_work_queue_) {
                    SP_LOG_CURRENT_FUNCTION();
                    SP_LOG("ERROR: Current work queue is null when attempting to execute function: ", func_name);
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

                if (request_error_) {
                    SP_LOG("ERROR: In an error state when attempting to execute ", func_name, " returning...");
                    return;
                }

                if (!current_work_queue_) {
                    SP_LOG_CURRENT_FUNCTION();
                    SP_LOG("ERROR: Current work queue is null when attempting to execute function: ", func_name);
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
            SP_LOG("ERROR: Caught exception when executing ", func_name, ": ", e.what());
        } catch (...) {
            SP_LOG_CURRENT_FUNCTION();
            SP_LOG("ERROR: Caught unknown exception when executing ", func_name, ".");
        }

        // If we get this far, an exception has occurred, so set the error state, reset the work
        // queue to allow the game thread to proceed, and return a default-constructed object.

        request_error_ = true;

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
        if (future.future_ptr_) {
            SP_ASSERT(future.type_id_ == Std::getTypeIdString<TReturn>());
            std::future<TReturn>* future_ptr = static_cast<std::future<TReturn>*>(future.future_ptr_);
            TReturn result = future_ptr->get();
            delete future_ptr;
            future_ptr = nullptr;
            return result;
        } else {
            SP_LOG("ERROR: Attempting to destroy a default-constructed SpFuture, returning a default-constructed return value...");
            return TReturn();
        }
    }

    template <>
    void destroyFuture<void>(SpFuture& future)
    {
        if (future.future_ptr_) {
            SP_ASSERT(future.type_id_ == Std::getTypeIdString<void>());
            std::future<void>* future_ptr = static_cast<std::future<void>*>(future.future_ptr_);
            future_ptr->get();
            delete future_ptr;
            future_ptr = nullptr;
        } else {
            SP_LOG("ERROR: Attempting to destroy a default-constructed SpFuture, returning...");
        }
    }

    std::map<std::string, FuncSignatureRegistry> entry_point_signature_registries_;
    TEntryPointBinder* entry_point_binder_ = nullptr;

    WorkQueue begin_frame_work_queue_ = WorkQueue("begin_frame_work_queue");
    WorkQueue end_frame_work_queue_ = WorkQueue("end_frame_work_queue");

    // Only accessed on the worker thread.
    WorkQueue* current_work_queue_ = nullptr;

    // Written and read by the game thread in beginFrame() and endFrame(). Written by the worker thread in engine_service.initialize
    // and engine_service.terminate. Read by the worker thread in the lambda returned by wrapFuncToExecuteInTryCatch(...),
    // which is used when executing all entry points on the worker thread.
    std::atomic<FrameState> frame_state_ = FrameState::Idle;

    // Written by the worker thread in engine_service.initialize and engine_service.begin_frame to indicate
    // to the game thread that we want to begin queueing a batch of work. Written and read by the game thread
    // in beginFrame().
    std::atomic<bool> request_begin_frame_ = false;

    // Written by the worker thread in engine_service.initialize and engine_service.terminate to indicate to
    // the game thread that we want to terminate the application. Read by the worker thread in engine_service.begin_frame,
    // engine_service.execute_frame, and engine_service.end_frame. Read by the game thread in beginFrame()
    // and endFrame().
    std::atomic<bool> request_terminate_ = false;

    // Written by the worker thread in engine_service.initialize. Written by the worker thread and the game
    // thread in executeFuncInTryCatch(...) to indicate to the game thread that an error has occurred. Read
    // by the worker thread in engine_service.begin_frame, engine_service.execute_frame, and engine_service.end_frame.
    // Read by the game thread in beginFrame(), endFrame(), and the lambdas returned by the wrapFuncToExecuteInWorkQueue(...)
    // methods.
    std::atomic<bool> request_error_ = false;

    // These promises and futures are each initialized by the worker thread in engine_service.initialize, engine_service.begin_frame,
    // and engine_service.execute_frame. Subsequently, the promises are only ever set by the game thread in
    // beginFrame(), endFrame(), and terminate(). The futures are only ever read on the worker thread in engine_service.begin_frame
    // and engine_service.execute_frame. Each bool variable is set by the game thread whenever the corresponding
    // promise is set, and is read by the game thread in shutdown() to avoid deadlocks when shutting down the
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
